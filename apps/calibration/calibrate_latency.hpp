#ifndef _CALIBRATE_LATENCY_H_
#define _CALIBRATE_LATENCY_H_

#include <termios.h>

#include <vector>

namespace calibration
{

int calibrate_latency_main();

// ----配置
constexpr const char *CONFIG_PATH      = "config/hikcamera.yaml";
constexpr const char *LATENCY_YAML_KEY = "transfer_latency_ms";

// 三档曝光：6000 是生产值；另两档拉开跨度，让自洽性检验信号足够强(也可以改为5档，但采集时间需要拉长，可能拟合效果会不好)
constexpr float kExposuresUs[] = {2000.0f, 6000.0f, 10000.0f};
constexpr int   kStepCount     = 3;

// ---- 采集节奏 ---- 
constexpr int kTriggerIntervalMs  = 50;    // 触发间隔
constexpr int kMaxSamplesPoint    = 300;   // 每档采集点数
constexpr int kMaxgrabFrame       = 600;   // 每档最大抓帧次数（含失败）
constexpr int kEarlyExitAttempts  = 20;    // 开头连续这么多次没有效样本就放弃
constexpr int kWarmupDiscard      = 5;     // 开头丢帧不予计算
constexpr int kGrabTimeoutMs      = 500;   // 单帧抓帧最长时
constexpr int kProgressIntervalMs = 2000;  // 进度打印间隔

// ---- 标定门槛 ----
constexpr double kMinSeconds   = 8.0;  // 标定最短时间
constexpr int    kMinSamples   = 150;  // 标定最少样本数
constexpr double kMaxSpread3Ms = 0.3;  // 标定单档极差阈值
constexpr double kMinOkRate    = 0.95; // 标定最低配对成功率
constexpr double kMaxGapRate   = 0.05; // 标定最大 nFrameNum 跳号率

constexpr double kMaxConsistencyMs = 0.3; // 三档 L 的极差上限

// ---- 采集结果 ----
struct RawTerm
{

    /*
        RawTerm：终端信号处理对象。
        作用：监听终端键盘输入 q，捕获 Ctrl‑C(SIGINT)中断信号。
        实现中途可以终止采集，并且就算中途退出也能跑完report打印报告。
    */
    RawTerm();
    ~RawTerm();
    RawTerm(const RawTerm &)            = delete;
    RawTerm &operator=(const RawTerm &) = delete;

    char poll() const; // 无输入返回 0

    termios saved{};
    bool    ok = false;
};

// 单档曝光的采集结果
struct Step
{
    double              exposure_ms = 0.0; // 回读的实际值，不是配置值
    std::vector<double> d;                 // 有效样本 Δ(ms)，按采集顺序
    int                 attempts    = 0;
    int                 stale       = 0;   // Δ < 曝光，即陈旧帧，已丢弃
    int                 gaps        = 0;   // nFrameNum 跳号
    double              seconds     = 0.0;
    bool                ok          = false;
};

double min_of(const Step &s);
double med_of(const Step &s);
double spread3(const Step &s);   // 按采集顺序三等分，取三组 min 的极差
double latency(const Step &s);   // min_of − exposure_ms，恒为正
bool   gate_pass(const Step &s); // 是否达标

} // namespace calibration

#endif
