#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

#include <opencv2/core/mat.hpp>
#include <chrono>
#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "MvCameraControl.h"

#include "common_def.hpp"

namespace hikcamera
{
class HikCamera : public Base_Camera
{
  public:
    HikCamera(const std::string &config_path = "config/hikcamera.yaml");
    ~HikCamera();

    // 禁止误拷贝相机句柄
    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    bool openCamera() override;
    void closeCamera() override;

    /**
     * @brief 从内部帧队列中取出一帧图像（阻塞直至有数据或超时 kQueueWaitMs 毫秒）
     * @return CameraFrame，若超时仍无数据则返回空对象。
     */
    CameraFrame getImage() override;

    // 获取连接状态（无锁读取，供外部轮询展示用）
    bool isConnectedStatus() const override { return isConnected.load(std::memory_order_relaxed); }

  private:
    cv::Mat processFrame(MV_FRAME_OUT *pstFrame);

    void markDisconnected(std::chrono::steady_clock::time_point now_steady);     // 记录断线信号
    void logRecoveryLatencyIfPending();                                          // 记录重连恢复延迟的日志（仅在首帧到达时打印一次）
    void updateDropStats(uint32_t frame_num);                                    // 丢帧统计（captureLoop 在锁外独占调用）
    
    static bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);                // 辅助函数
    static void __stdcall ExceptionCallBack(unsigned int nMsgType, void *pUser); //SDK 异常回调注册

    // ---- 内部线程 ----
    void TryConnect(bool bypass_backoff = false);   // 重连：现在由内部守护线程(daemonLoop)周期性驱动，一般无需外部调用
    void captureLoop();                             // 采集线程：持续抓帧、转换、推入队列，更新心跳
    void daemonLoop();                              // 守护线程：每 kDaemonPollMs 检查一次连接状态与心跳，必要时重连
    void pushFrame(CameraFrame &&frame);
    void startThreads();                            // 在未运行时真正创建线程
    void stopThreads();                             // 设置退出标志并 join

    // ---- 内部状态 ----
    void              *handle;                          // 相机句柄
    std::atomic<bool> isConnected;                      // 连接状态
    char              serialNumber[64];                 // 设备序列号
    std::string       target_serial_;                   // 期望连接的设备序列号（空串表示不指定）

    bool  sdk_initialized_          = false;             // 全局 SDK 初始化状态标志
    bool  wedge_detected_           = false;             // 锁卡死标志位，防止相机阻塞使锁长时间被占用  
    bool  was_connected_            = true;
    bool  was_stalled_              = false;

    int   consecutive_sdk_errors_   = 0;                 // 相机启动连续错误计数器
    int   pool_index                = 0;
    int   wedge_consecutive_misses_ = 0;                 // 连续抢不到锁的次数（去抖计数器）
    int   reconnect_backoff_ms_     = kInitialBackoffMs; // 当前需要等待的退避间隔

    float exposure_actual_us_  = 0.0f;                  // 实际曝光时间（单位：微秒）
    float transfer_latency_ms_ = 1.5f;                  // 传输延迟
    float exposure_time_;                               // 曝光时间
    float gain_;                                        // 增益

    uint32_t last_frame_num_       = 0;                 // 上一帧的 nFrameNum；0 表示本次取流尚未收到首帧
    uint64_t dropped_total_        = 0;                 // 累计丢帧数
    uint64_t queue_dropped_total_  = 0;                 // 内部帧队列因满而主动丢弃的帧数
    uint64_t last_dropped_grab_    = 0;                 // 上次打日志时的 dropped_total_，用于算增量
    uint64_t last_dropped_queue_   = 0;                 // 上次打日志时的 queue_dropped_total_

    // ---- 线程 / 队列相关新增常量 ----
    static constexpr int    kInitialBackoffMs        = 200;   // TryConnect 初始退避 200ms
    static constexpr int    kMaxBackoffMs            = 500;   // TryConnect 最大退避 500ms
    static constexpr int    kFailLogThrottleMs       = 2000;  // TryConnect 失败日志节流间隔
    static constexpr int    kPoolCount               = 5;
    static constexpr int    kMaxConsecutiveSdkErrors = 5;     // captureLoop    错误计数累积阈值
    static constexpr int    kDaemonPollMs            = 100;   // daemonLoop     轮询间隔
    static constexpr int    kWatchdogTimeoutMs       = 1000;  // daemonLoop     超时阈值
    static constexpr int    kWedgeEscalateMs         = 10000; // daemonLoop     日志升级等待时长
    static constexpr int    kWedgeLogDebounceMisses  = 5;     // daemonLoop     抢锁失败日志触发阈值
    static constexpr int    kWedgeLogThrottleMs      = 2000;  // daemonLoop     抢锁失败日志节流间隔  
    static constexpr size_t kQueueCapacity           = 2;     // pushFrame      帧队列容量，超出丢弃最旧帧
    static constexpr int    kQueueWaitMs             = 100;   // getImage       等待队列的超时时间
    static constexpr int    kCloseTeardownWarnMs     = 150;   // closeCamera    相机关闭耗时警告阈值
    static constexpr int    kGetImageBufferTimeoutMs = 20;    // GetImageBuffer 单次调用的 SDK 内部超时

    static constexpr int    kCameraLockTimeoutMs     = 200;   // openCamera/closeCamera 等慢路径的等待上限
    static constexpr int    kCameraFastLockTimeoutMs = 30;    // captureLoop/daemonLoop 高频轮询的等待上限
    
    static constexpr int      kDropLogThrottleMs     = 5000;  // updateDropStats 丢帧日志的最小间隔
    static constexpr uint32_t kFrameGapSanityLimit   = 10000; // 帧号间隔超过此值视为异常（重连/乱序），不计入丢帧

    std::thread             capture_thread_;
    std::thread             daemon_thread_;

    std::mutex              queue_mutex_;                      // 保护相机帧队列
    std::timed_mutex        camera_mutex_;                     // 保护 handle / isConnected 及一切直接的 SDK 调用
    std::atomic<int64_t>    t_disconnect_signal_ms_{0};        // 断线信号触发时刻（SDK回调 / 首次判定为持续性错误）
    std::atomic<int64_t>    t_grab_started_ms_     {0};        // 相机真正取流的时间
    std::atomic<bool>       first_frame_pending_   {false};    // true 表示"重连完成等待首帧到达"
    std::atomic<bool>       stats_reset_pending_   {false};    // 丢帧统计需要重置的待处理标志。
    std::atomic<bool>       running_               {false};    // 采集、守护线程共用启停标记，true允许线程循环运行，false通知线程退出
    std::atomic<int64_t>    last_frame_tick_ms_    {0};        // 采集线程心跳

    std::condition_variable queue_cv_;                         // 用于 getImage() 阻塞等待队列非空(信号灯)
    std::queue<CameraFrame> frame_queue_;

    std::chrono::steady_clock::time_point last_reconnect_attempt_{};  //上一次重连的时间
    std::chrono::steady_clock::time_point wedge_since_           {};
    std::chrono::steady_clock::time_point last_wedge_log_time_   {};
    std::chrono::steady_clock::time_point last_fail_log_time_    {};  // 上一次打印"重连失败"日志的时间，
    std::chrono::steady_clock::time_point last_drop_log_time_    {};  // 上一次打印丢帧日志的时间（节流用）

  
    cv::Mat output_pool[kPoolCount];

};
} // namespace hikcamera

#endif // _HIKCAMERA_H_
