#include "hikcamera.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "mas_log.hpp"

// 守护线程：每 kDaemonPollMs 检查一次连接状态和采集心跳，
// 一旦发现掉线或采集卡死（长时间没有新帧），调用 TryConnect() 触发重连。
void hikcamera::HikCamera::daemonLoop()
{
    while (running_.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(kDaemonPollMs));

        if (!running_.load(std::memory_order_relaxed))
        {
            break;
        }

        // ---- 第一部分：尝试抢锁，仅用于"采集线程是否卡在阻塞SDK调用里"的诊断日志，同时降低日志更新频率
        {
            std::unique_lock<std::timed_mutex> lock(camera_mutex_, std::chrono::milliseconds(kCameraFastLockTimeoutMs));
            if (lock.owns_lock())
            {
                wedge_detected_           = false; // 抢到了锁，说明采集线程当前没有长期占锁，恢复正常，清除卡死计时
                wedge_consecutive_misses_ = 0;      // 同时清零去抖计数器
            }
            else
            {
                auto now_steady = std::chrono::steady_clock::now();
                ++wedge_consecutive_misses_;

                if (!wedge_detected_ && wedge_consecutive_misses_ >= kWedgeLogDebounceMisses)
                {
                    wedge_detected_ = true;
                    wedge_since_    = now_steady;

                    auto since_last_log = std::chrono::duration_cast<std::chrono::milliseconds>(
                                               now_steady - last_wedge_log_time_)
                                               .count();
                    if (last_wedge_log_time_.time_since_epoch().count() == 0 || since_last_log >= kWedgeLogThrottleMs)
                    {
                        MAS_LOG_WARN("daemonLoop: camera_mutex_ busy for {} consecutive poll(s) (~{} ms, "
                                     "diagnostic only, not treated as disconnect); capture thread likely "
                                     "busy fetching a frame",
                                     wedge_consecutive_misses_, wedge_consecutive_misses_ * kDaemonPollMs);
                        last_wedge_log_time_ = now_steady;
                    }
                }
                else if (wedge_detected_ &&
                         std::chrono::duration_cast<std::chrono::milliseconds>(now_steady - wedge_since_).count() > kWedgeEscalateMs)
                {
                    MAS_LOG_ERROR("daemonLoop: camera_mutex_ has been busy for over {} ms continuously; "
                                 "if this persists the capture thread may truly be wedged in a blocking SDK "
                                 "call and in-process recovery may not be possible, consider restarting the process",
                                 kWedgeEscalateMs);
                    wedge_since_ = now_steady; // 避免刷屏，重新计时下一次升级提醒
                }
            }
        }

        bool connected = isConnected.load(std::memory_order_relaxed);

        bool stalled = false;
        if (connected)
        {
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
            auto last_ms = last_frame_tick_ms_.load(std::memory_order_relaxed);
            stalled       = (now_ms - last_ms) > kWatchdogTimeoutMs;
        }

        if (!connected || stalled)
        {
            if (stalled)
            {
                MAS_LOG_WARN("Capture heartbeat stale for over {} ms, forcing reconnect",
                             kWatchdogTimeoutMs);
                // 心跳卡死本身就是一种"合法断线信号"，为分层延迟诊断记录信号触发时刻。
                markDisconnected(std::chrono::steady_clock::now());
                connected = false;
            }

            // 主要依靠的是上一次的连接状态(因为走到这一步说明connected和stalled都出现问题)
            bool fresh_disconnect = was_connected_ && !connected;
            bool fresh_stall      = stalled        && !was_stalled_;
            TryConnect(fresh_disconnect || fresh_stall);
        }

        was_connected_ = connected;
        was_stalled_   = stalled;
    }
}

// 重连：由 daemonLoop 周期性调用
void hikcamera::HikCamera::TryConnect(bool bypass_backoff)
{
    auto now         = std::chrono::steady_clock::now();
    auto connect_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_attempt_).count();

    // 已经短线很长时间但短线时长不超过reconnect_backoff_ms_
    if (!bypass_backoff && connect_time < reconnect_backoff_ms_)
    {
        return;
    }

    last_reconnect_attempt_ = now;
    closeCamera();
    bool success = openCamera();

    if (success)
    {
        reconnect_backoff_ms_ = kInitialBackoffMs;
        MAS_LOG_INFO("Camera reconnect successfully");
    }
    else
    {
        reconnect_backoff_ms_ = std::min(reconnect_backoff_ms_ * 2, kMaxBackoffMs);

        //限制日志打印频率，避免刷屏
        auto since_last_log = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fail_log_time_).count();
        if (last_fail_log_time_.time_since_epoch().count() == 0 || since_last_log >= kFailLogThrottleMs)
        {
            MAS_LOG_WARN("Camera reconnect failed, next attempt after {} ms", reconnect_backoff_ms_);
            last_fail_log_time_ = now;
        }
    }
}
