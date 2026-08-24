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
