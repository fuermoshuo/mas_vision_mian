#include "calibrate_latency.hpp"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <string>
#include <thread>

#include <fmt/core.h>

#include "MvCameraControl.h"
#include "mas_log.hpp"

extern std::atomic<bool> g_shutdown;

namespace calibration
{
using Clock = std::chrono::steady_clock;

// ---------------------------------------------------------------- 统计

double min_of(const Step &s)
{
    return s.d.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::min_element(s.d.begin(), s.d.end());
}

// 不参与transfer_latency_ms的计算，仅打印日志看趋势，遇到偶数的bug先忽略
double med_of(const Step &s)
{
    if (s.d.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::vector<double> v   = s.d;
    const size_t        mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(mid), v.end());
    return v[mid];
}

// 分组必须按采集顺序：检验的是"这个 min 稳不稳定"，不是"数据散不散"
double spread3(const Step &s)
{
    if (s.d.size() < 3) return std::numeric_limits<double>::infinity();

    const size_t n = s.d.size() / 3;
    double       m[3];
    for (int g = 0; g < 3; ++g)
    {
        auto b = s.d.begin() + static_cast<long>(g * n);
        auto e = (g == 2) ? s.d.end() : b + static_cast<long>(n);
        m[g]   = *std::min_element(b, e);
    }
    return *std::max_element(m, m + 3) - *std::min_element(m, m + 3);
}

// 所有入库样本都满足 Δ > exposure_ms，所以这个值恒为正，不需要再校验
double latency(const Step &s) { return min_of(s) - s.exposure_ms; }

static double ok_rate(const Step &s) { return s.attempts ? double(s.d.size()) / s.attempts : 0.0; }
static double gap_rate(const Step &s) { return s.d.empty() ? 0.0 : double(s.gaps) / s.d.size(); }

bool gate_pass(const Step &s)
{
    return s.seconds >= kMinSeconds && int(s.d.size()) >= kMinSamples && spread3(s) < kMaxSpread3Ms &&
           ok_rate(s) >= kMinOkRate && gap_rate(s) <= kMaxGapRate;
}

// ------------ 终端
RawTerm::RawTerm()
{
    if (tcgetattr(STDIN_FILENO, &saved) != 0) return;
    ok = true;

    termios raw = saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

RawTerm::~RawTerm()
{
    if (ok) tcsetattr(STDIN_FILENO, TCSANOW, &saved);
}

char RawTerm::poll() const
{
    char c = 0;
    return (ok && ::read(STDIN_FILENO, &c, 1) == 1) ? c : 0;
}

// ------------- 相机

namespace
{

struct CamGuard
{
    void *h = nullptr;
    ~CamGuard()
    {
        if (!h) return;
        const int nRet = MV_CC_SetEnumValue(h, "TriggerMode", 0);
        MV_CC_StopGrabbing(h);
        MV_CC_CloseDevice(h);
        MV_CC_DestroyHandle(h);
        MV_CC_Finalize();

        if (nRet != MV_OK)
        {
            MAS_LOG_ERROR("TriggerMode 恢复失败! 请用 MVS Viewer 把 TriggerMode 关掉，并保存到 UserSet");
        }
    }
};

// 清除脏旧帧
bool flush(void *h)
{
    const int nRet = MV_CC_ClearImageBuffer(h);
    if (nRet != MV_OK)
    {
        MAS_LOG_ERROR("ClearImageBuffer fail! nRet 0x{0:x} —— 旧帧清不掉" ,nRet);
        return false;
    }
    return true;
}

// 软件触发一次相机、收取一帧
double trigger_once(void *h, uint32_t &frame_num)
{
    const auto t_trig = Clock::now();
    if (MV_CC_SetCommandValue(h, "TriggerSoftware") != MV_OK) return -1.0;

    MV_FRAME_OUT f      = {};
    const int    nRet   = MV_CC_GetImageBuffer(h, &f, kGrabTimeoutMs);
    const auto   t_recv = Clock::now();
    if (nRet != MV_OK) return -1.0;

    frame_num = f.stFrameInfo.nFrameNum;
    MV_CC_FreeImageBuffer(h, &f);
    return std::chrono::duration<double, std::milli>(t_recv - t_trig).count();
}

void log_step(const Step &s)
{
    if (s.d.empty())
    {
        MAS_LOG_WARN("[exp {:.2f}ms] 触发 {} 次无有效样本(陈旧帧 {})", s.exposure_ms, s.attempts, s.stale);
        return;
    }
    MAS_LOG_INFO("[exp {:.2f}ms] n={:<3} minΔ={:.2f} L={:.2f} med={:.2f} spread={:.2f} ok={:.0f}% "
                 "stale={} gap={} {}",
                 s.exposure_ms, s.d.size(), min_of(s), latency(s), med_of(s), spread3(s), ok_rate(s) * 100.0,
                 s.stale, s.gaps, gate_pass(s) ? "达标" : "");
}

// 设曝光并回读实际值。
bool set_exposure(void *h, float want_us, double &actual_ms)
{
    MVCC_FLOATVALUE range = {0};
    if (MV_CC_GetFloatValue(h, "ExposureTime", &range) != MV_OK) return false;

    if (want_us < range.fMin || want_us > range.fMax)
    {
        MAS_LOG_WARN("曝光 {:.0f}us 超出相机范围 [{:.0f}, {:.0f}]", want_us, range.fMin, range.fMax);
        return false;
    }
    if (MV_CC_SetFloatValue(h, "ExposureTime", want_us) != MV_OK) return false;

    MVCC_FLOATVALUE back = {0};
    if (MV_CC_GetFloatValue(h, "ExposureTime", &back) != MV_OK) return false;

    actual_ms = back.fCurValue / 1000.0;
    return true;
}

// 跑一档。返回 false 表示整体中止
bool run_step(void *h, float exposure_us, const RawTerm &term, Step &s)
{
    if (!set_exposure(h, exposure_us, s.exposure_ms))
    {
        MAS_LOG_WARN("曝光 {:.0f}us 设置或回读失败，标定终止", exposure_us);
        return false;
    }

    MAS_LOG_INFO("---- 曝光 {:.2f}ms 采集中：门槛 {:.0f}s / {}样本 / spread<{:.1f} / ok≥{:.0f}% / gap≤{:.0f}%"
                 ";s=提前结束 q=中止 ----",
                 s.exposure_ms, kMinSeconds, kMinSamples, kMaxSpread3Ms, kMinOkRate * 100.0, kMaxGapRate * 100.0);

    // 预热丢弃，不计入统计样本
    for (int i = 0; i < kWarmupDiscard && !g_shutdown; ++i)
    {
        uint32_t dummy = 0;
        trigger_once(h, dummy);
        std::this_thread::sleep_for(std::chrono::milliseconds(kTriggerIntervalMs));
    }

    const auto t0        = Clock::now();
    auto       t_next    = t0;
    auto       t_log     = t0;
    uint32_t   last_num  = 0;
    bool       abort_all = false;

    while (!g_shutdown && int(s.d.size()) < kMaxSamplesPoint && s.attempts < kMaxgrabFrame)
    {
        std::this_thread::sleep_until(t_next);
        t_next += std::chrono::milliseconds(kTriggerIntervalMs);

        uint32_t     num   = 0;
        const double delta = trigger_once(h, num);
        ++s.attempts;

        if (delta < 0.0)
        {
            if (!flush(h)) { abort_all = true; break; }
            last_num = 0;
        }
        else if (delta < s.exposure_ms)
        {
            // 小于曝光只能是陈旧帧，说明触发与取帧失步
            ++s.stale;
            if (!flush(h)) { abort_all = true; break; }
            last_num = 0;
        }
        else
        {
            s.d.push_back(delta);
            if (last_num != 0 && num != last_num + 1) ++s.gaps;
            last_num = num;
        }

        const auto now = Clock::now();
        if (t_next < now) t_next = now; // 超时拖慢后别补发一串
        s.seconds = std::chrono::duration<double>(now - t0).count();

        if (s.d.empty() && s.attempts >= kEarlyExitAttempts)
        {
            MAS_LOG_ERROR("连续 {} 次无有效样本（超时 {}、陈旧帧 {}），中止。"
                          "超时为主→该型号可能不支持软触发；陈旧帧为主→触发与取帧失步",
                          s.attempts, s.attempts - s.stale, s.stale);
            abort_all = true;
            break;
        }

        if (now - t_log >= std::chrono::milliseconds(kProgressIntervalMs))
        {
            log_step(s);
            t_log = now;
        }

        const char key = term.poll();
        if (key == 's' || key == 'S')
        {
            if (gate_pass(s)) break;
            MAS_LOG_WARN("[s] 未达标，继续采集");
        }
        else if (key == 'q' || key == 'Q')
        {
            abort_all = true;
            break;
        }
    }

    s.ok = gate_pass(s);
    return !g_shutdown && !abort_all;
}

// yaml-cpp 重写会丢掉文件里的注释，直接文本修改不会影响注释
bool write_yaml(double value, const std::string &note)
{
    std::vector<std::string> lines;
    {
        std::ifstream in(CONFIG_PATH);
        if (!in) return false;
        std::string l;
        while (std::getline(in, l)) lines.push_back(l);
    }

    int target = -1;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const size_t k = lines[i].find(LATENCY_YAML_KEY);
        const size_t h = lines[i].find('#');
        if (k != std::string::npos && (h == std::string::npos || h > k)) { target = int(i); break; }
    }
    if (target < 0) return false;

    const size_t      nonws  = lines[target].find_first_not_of(" \t");
    const std::string indent = (nonws == std::string::npos) ? "" : lines[target].substr(0, nonws);
    lines[target] = fmt::format("{}{}: {:.2f}   # 传输延迟(ms) [{}]", indent, LATENCY_YAML_KEY, value, note);

    std::ofstream out(CONFIG_PATH);
    if (!out) return false;
    for (const auto &l : lines) out << l << "\n";

    out.close();
    return out.good();
}

// 返回修改时间
std::string now_str()
{
    const std::time_t t = std::time(nullptr);
    std::tm           tm{};
    localtime_r(&t, &tm);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tm);
    return buf;
}

// 无论正常结束、按 q 还是 Ctrl-C，退出前都完整报备一次
void report(const std::vector<Step> &steps)
{
    MAS_LOG_INFO("==== transfer_latency calibration (software trigger) ====");

    std::vector<double> got;
    for (const auto &s : steps)
    {
        log_step(s);
        if (s.ok) got.push_back(latency(s));
    }

    if (got.empty())
    {
        MAS_LOG_WARN("无达标档位，不写入 yaml");
        return;
    }

    const double lo         = *std::min_element(got.begin(), got.end());
    const double spread     = *std::max_element(got.begin(), got.end()) - lo;
    const int    n          = int(got.size());
    const bool   consistent = spread < kMaxConsistencyMs;

    MAS_LOG_INFO("建议值 {:.2f} ms | 有效档 {}/{} | 一致性 {}（极差 {:.2f}ms,阈值 {:.1f})",
                 lo, n, kStepCount, consistent ? "OK" : "FAIL", spread, kMaxConsistencyMs);

    if (n < 2)
    {
        MAS_LOG_WARN("只有 1 档有效，无法做自洽性检验，**不写入** yaml。要用请手动填 {:.2f}", lo);
        return;
    }
    if (!consistent)
    {
        MAS_LOG_WARN("各档 L 极差 {:.2f}ms ≥ {:.1f}，模型不成立（读出与曝光重叠？触发延迟非零？），"
                     "**不写入** yaml", spread, kMaxConsistencyMs);
        return;
    }

    const std::string note =
        (n == kStepCount) ? fmt::format("calib {}, {} exp, spread {:.2f}ms", now_str(), n, spread)
                          : fmt::format("calib {}, PARTIAL {}/{} exp, spread {:.2f}ms", now_str(), n,
                                        kStepCount, spread);
    if (n != kStepCount) MAS_LOG_WARN("只有 {}/{} 档达标，按已有档位写入", n, kStepCount);

    if (write_yaml(lo, note))
        MAS_LOG_INFO("已写入 {}", CONFIG_PATH);
    else
        MAS_LOG_ERROR("写入 {} 失败，请手动填 {:.2f}", CONFIG_PATH, lo);
}

} // namespace

// ---------------------------------------------------------------- main

int calibrate_latency_main()
{
    CamGuard guard;

    if (MV_CC_Initialize() != MV_OK) { MAS_LOG_ERROR("Initialize SDK fail"); return 1; }

    MV_CC_DEVICE_INFO_LIST list;
    memset(&list, 0, sizeof(list));
    if (MV_CC_EnumDevices(MV_USB_DEVICE, &list) != MV_OK || list.nDeviceNum == 0)
    {
        MAS_LOG_ERROR("Find No Devices");
        MV_CC_Finalize();
        return 1;
    }

    void *h = nullptr;
    if (MV_CC_CreateHandle(&h, list.pDeviceInfo[0]) != MV_OK || MV_CC_OpenDevice(h) != MV_OK)
    {
        MAS_LOG_ERROR("Open Device fail");
        if (h) MV_CC_DestroyHandle(h);
        MV_CC_Finalize();
        return 1;
    }
    guard.h = h;

    MV_CC_SetEnumValue(h, "TriggerMode", 0); // 上次若异常退出，相机可能还停在触发模式
    if (MV_CC_SetEnumValue(h, "TriggerMode", 1) != MV_OK ||
        MV_CC_SetEnumValue(h, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE) != MV_OK)
    {
        MAS_LOG_ERROR("软触发不可用");
        return 1;
    }

    MV_CC_SetFloatValue(h, "TriggerDelay", 0.0f); // 有些相机默认非零
    {
        MVCC_FLOATVALUE td = {0};
        if (MV_CC_GetFloatValue(h, "TriggerDelay", &td) != MV_OK)
        {
            MAS_LOG_WARN("TriggerDelay 读写均失败，该型号可能无此节点；若结果一致偏大请手动确认");
        }
        else if (td.fCurValue > 1.0f) // >1us 才当回事：0.3ms 的门槛下 1us 可忽略
        {
            MAS_LOG_ERROR("TriggerDelay 置零失败，当前 {:.1f}us —— 它会让三档 L 同等偏大而五个门槛"
                          "全部通过，标定终止",
                          td.fCurValue);
            return 1;
        }
    }
    MV_CC_SetEnumValue(h, "ExposureAuto", 0);
    MV_CC_SetEnumValue(h, "GainAuto", 0);
    MV_CC_SetBoolValue(h, "AcquisitionFrameRateEnable", false);
    MV_CC_SetImageNodeNum(h, 1);
    MV_CC_SetGrabStrategy(h, MV_GrabStrategy_OneByOne);

    {
        MVCC_ENUMVALUE px = {0};
        // 读取相机硬件当前真实的像素格式参数
        if (MV_CC_GetEnumValue(h, "PixelFormat", &px) != MV_OK)
        {
            MAS_LOG_WARN("读 PixelFormat 失败，无法核对格式");
        }
        else if (px.nCurValue != PixelType_Gvsp_BayerRG8 && px.nCurValue != PixelType_Gvsp_BayerBG8 &&
                 px.nCurValue != PixelType_Gvsp_BayerGB8 && px.nCurValue != PixelType_Gvsp_BayerGR8)
        {
            MAS_LOG_ERROR("PixelFormat 0x{:x} 不是 Bayer8,标定终止。",px.nCurValue);
            return 1;
        }
        else
        {
            MAS_LOG_INFO("PixelFormat = 0x{:x} (Bayer8)", px.nCurValue);
        }
    }

    if (MV_CC_StartGrabbing(h) != MV_OK) { MAS_LOG_ERROR("Start Grabbing fail"); return 1; }

    RawTerm term;
    MAS_LOG_INFO("三档曝光依次采集,期间勿跑重负载(CPU 争用会抬高 min)");

    std::vector<Step> steps;
    steps.reserve(kStepCount);
    for (int i = 0; i < kStepCount; ++i)
    {
        steps.emplace_back();
        if (!run_step(h, kExposuresUs[i], term, steps.back())) break;
    }

    report(steps);
    return 0;
}

} // namespace calibration
