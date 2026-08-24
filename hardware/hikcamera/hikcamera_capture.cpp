#include "hikcamera.hpp"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <thread>

#include "mas_log.hpp"

/*
    主机与 MCU 两套独立时钟，
    图像、姿态各自携带的时间戳存在传输延迟与随机抖动，
    静态误差均值抵消看着准，云台转动时抖动叠加导致瞄准偏移；·
    单纯姿态插值无法根治。需要要对齐两套时钟，或者在云台转动时使用图像时间戳作为姿态插值的参考时间戳。（目前仅手动减小误差）
    准确的方案需要搭配 strobe 并以MCU的始终为准
    当前方案实际公式  t_曝光中点 = t_recv − (实际曝光/2 + 传输延迟)

    transfer_latency_ms_为传输延迟 由 `./base calibrate_latency` 标定
*/

// 采集线程：持续抓帧、转换、推入队列，并更新心跳时间戳。
void hikcamera::HikCamera::captureLoop()
{
    while (running_.load(std::memory_order_relaxed))
    {
        CameraFrame data;
        bool     connected = false;
        bool     got_frame = false;
        uint32_t frame_num = 0;

        {
            std::unique_lock<std::timed_mutex> lock(camera_mutex_, std::chrono::milliseconds(kCameraFastLockTimeoutMs));
            if (lock.owns_lock())
            {
                connected = isConnected && handle != NULL;
                if (connected)
                {
                    MV_FRAME_OUT stOutFrame = {};
                    int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, kGetImageBufferTimeoutMs);
                    if (nRet == MV_OK)
                    {
                        auto t_recv    = std::chrono::steady_clock::now();
                        auto offset    = std::chrono::microseconds(static_cast<int64_t>(
                                                                exposure_actual_us_ / 2.0 + transfer_latency_ms_ * 1000.0));
                        data.t_recv    = t_recv;
                        data.timestamp = t_recv - offset;

                        frame_num = stOutFrame.stFrameInfo.nFrameNum;

                        data.frame     = processFrame(&stOutFrame);
                        MV_CC_FreeImageBuffer(handle, &stOutFrame);
                        got_frame = true;
                        consecutive_sdk_errors_ = 0;
                        logRecoveryLatencyIfPending();
                    }
                    else if (nRet == MV_E_NODATA)
                    {
                        // 单次无帧不做处理，统一由 daemonLoop 依据心跳判定是否断线
                    }
                    else
                    {
                        MAS_LOG_ERROR("GetImageBuffer fail! nRet 0x{0:x}", nRet);
                        if (++consecutive_sdk_errors_ >= kMaxConsecutiveSdkErrors)
                        {
                            MAS_LOG_WARN("GetImageBuffer failed {} consecutive times (non-NODATA); "
                                         "treating as a genuine disconnect signal",
                                         consecutive_sdk_errors_);
                            markDisconnected(std::chrono::steady_clock::now());
                        }
                    }
                }
            }
        }   

        if (!connected)
        {
            // 未连接：交由 daemonLoop 负责重连，这里只需避免忙等空转
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (got_frame)
        {
            last_frame_tick_ms_.store(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    data.t_recv.time_since_epoch()).count(),
                std::memory_order_relaxed);

            pushFrame(std::move(data));
            updateDropStats(frame_num);
        }
    }
}

// 丢帧统计
void hikcamera::HikCamera::updateDropStats(uint32_t frame_num)
{
    if (stats_reset_pending_.exchange(false, std::memory_order_relaxed))
    {
        last_frame_num_ = 0;
    }

    if (last_frame_num_ != 0)
    {
        const uint32_t gap = frame_num - last_frame_num_ - 1;
        if (gap > 0 && gap < kFrameGapSanityLimit)
        {
            dropped_total_ += gap;
        }
        else if (gap >= kFrameGapSanityLimit)
        {
            MAS_LOG_WARN("Implausible frame number step {} -> {}, ignored", last_frame_num_, frame_num);
        }
    }
    last_frame_num_ = frame_num;

    const uint64_t new_grab  = dropped_total_ - last_dropped_grab_;
    const uint64_t new_queue = queue_dropped_total_ - last_dropped_queue_;
    if (new_grab == 0 && new_queue == 0)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_drop_log_time_ < std::chrono::milliseconds(kDropLogThrottleMs))
    {
        return;
    }

    MAS_LOG_WARN("Frame drop: grab +{} (total {}), queue +{} (total {})",
                 new_grab, dropped_total_, new_queue, queue_dropped_total_);

    last_dropped_grab_    = dropped_total_;
    last_dropped_queue_   = queue_dropped_total_;
    last_drop_log_time_   = now;
}

void hikcamera::HikCamera::pushFrame(CameraFrame &&frame)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (frame_queue_.size() >= kQueueCapacity)
        {
            frame_queue_.pop(); // 丢弃最旧帧，保持"只要最新图像"的语义
            ++queue_dropped_total_;
        }
        frame_queue_.push(std::move(frame));
    }
    queue_cv_.notify_one();
}

// 依靠唤醒 queue_cv_ 来唤醒getImage函数
CameraFrame hikcamera::HikCamera::getImage()
{
    std::unique_lock<std::mutex> lock(queue_mutex_);
    bool has_data = queue_cv_.wait_for(lock, std::chrono::milliseconds(kQueueWaitMs), [this] {
        return !frame_queue_.empty() || !running_.load(std::memory_order_relaxed);
    });

    if (has_data && !frame_queue_.empty())
    {
        CameraFrame frame = std::move(frame_queue_.front());
        frame_queue_.pop();
        return frame;
    }

    return CameraFrame();
}

cv::Mat hikcamera::HikCamera::processFrame(MV_FRAME_OUT *stOutFrame)
{
    if (!stOutFrame || !stOutFrame->pBufAddr)
    {
        return cv::Mat();
    }

    int nRet = MV_OK;

    unsigned int width  = stOutFrame->stFrameInfo.nExtendWidth;
    unsigned int height = stOutFrame->stFrameInfo.nExtendHeight;

    // 针对彩色相机的像素处理逻辑
    if (stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 || stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8 ||
        stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8 || stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8)
    {
        // 使用 OpenCV Bayer 解码
        cv::Mat bayerMat(height, width, CV_8UC1, stOutFrame->pBufAddr);

        // 映射 SDK Bayer 格式到 OpenCV 颜色转换码（RG↔BG, GR↔GB）
        int cvt_code;
        switch (stOutFrame->stFrameInfo.enPixelType)
        {
        case PixelType_Gvsp_BayerRG8:
            cvt_code = cv::COLOR_BayerBG2BGR;
            break;
        case PixelType_Gvsp_BayerBG8:
            cvt_code = cv::COLOR_BayerRG2BGR;
            break;
        case PixelType_Gvsp_BayerGB8:
            cvt_code = cv::COLOR_BayerGR2BGR;
            break;
        case PixelType_Gvsp_BayerGR8:
            cvt_code = cv::COLOR_BayerGB2BGR;
            break;
        default:
            return cv::Mat();
        }

        int idx = pool_index;
        cv::Mat &output = output_pool[idx];

        if (!output.empty() && output.u != nullptr && output.u->refcount > 1)
        {
            output = cv::Mat();
        }
        
        output.create(height, width, CV_8UC3);
        cv::cvtColor(bayerMat, output, cvt_code);

        pool_index = (idx + 1) % kPoolCount;
        return output;    
    }
    else if (stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        cv::Mat rgbImg(height, width, CV_8UC3, stOutFrame->pBufAddr);
        cv::Mat outImg;
        cv::cvtColor(rgbImg, outImg, cv::COLOR_RGB2BGR);
        return outImg;
    }

    return cv::Mat();
}

void hikcamera::HikCamera::logRecoveryLatencyIfPending()
{
    if (!first_frame_pending_.exchange(false, std::memory_order_relaxed))
    {
        return;
    }

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();

    int64_t t_disconnect = t_disconnect_signal_ms_.load(std::memory_order_relaxed);
    int64_t t_grab       = t_grab_started_ms_.load(std::memory_order_relaxed);

    if (t_disconnect == 0 || t_grab == 0)
    {
        // 进程刚启动、首次成功打开的情况下没有"断线信号"时间戳，跳过，不打印误导性的负数/异常值
        return;
    }

    MAS_LOG_INFO(
        "Recovery latency breakdown: open+OpenGrab={}ms, firstFrameAfterGrab={}ms, total={}ms",
         (t_grab - t_disconnect), (now_ms - t_grab), (now_ms - t_disconnect));
}
