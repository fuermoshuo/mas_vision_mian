#include <atomic>
#include <csignal>
#include <cstddef>
#include <spdlog/common.h>
#include <thread>

#include "BS_thread_pool.hpp"
#include "SPSCQueue.h"
#include "display.hpp"
#include "mas_log.hpp"

#include "armor_thread.hpp"
#include "hikcamera_thread.hpp"
#include "serial_thread.hpp"
#include "usbcamera_config.hpp"
#include "usbcamera_thread.hpp"
#include "ros2_comm.hpp"
#include "ros2_thread.hpp"

#include "calibrate_camera.hpp"
#include "calibrate_handeye.hpp"
#include "calibrate_latency.hpp"

// 全局原子变量用于控制所有线程的生命周期
std::atomic<bool> g_shutdown{false};

// 信号处理函数
void signal_handler(int signum) { g_shutdown = true; }

int main(int argc, char *argv[])
{
    // 注册信号处理器
    std::signal(SIGINT, signal_handler);  // Ctrl+C
    std::signal(SIGTERM, signal_handler); // 终止信号

    // 初始化日志
    rm_utils::MasLog::init("logs/mas_vision.log", spdlog::level::debug, spdlog::level::info);

    if (argc > 1)
    {
        std::string command = argv[1];
        if (command == "calibrate_camera")
        {
            return calibrate_camera_main();
        }
        else if (command == "calibrate_handeye")
        {
            return calibration::calibrate_handeye_main();
        }
        else if (command == "calibrate_latency")
        {
            return calibration::calibrate_latency_main();
        }
        else
        {
            fmt::print("未知命令: {}\n", command);
            fmt::print("相机校准     : ./base calibrate_camera\n");
            fmt::print("手眼标定     : ./base calibrate_handeye\n");
            fmt::print("传输延迟标定 : ./base calibrate_latency\n");
            return 1;
        }
    }

    MAS_LOG_INFO("Mas Vision System Start");

    try
    {
        // 创建线程池
        BS::thread_pool pool(6);
        MAS_LOG_INFO("Thread pool created with {} threads", pool.get_thread_count());

        // 启动显示线程
        rm_utils::Display::getInstance().start(pool);

        // 加载 USB 相机配置文件
        if (!UsbCameraConfigManager::loadConfig("config/usbcamera_config.yaml"))
        {
            MAS_LOG_WARN("Using default USB camera configuration");
        }

        // 加载 ROS2 通信配置文件
        if (!Ros2_comm::loadConfig("config/ros2.yaml"))
        {
            MAS_LOG_WARN("Using default ROS2 communication configuration");
        }

        // 创建SPSCQueue，用于相机图像
        size_t camera_buffer_size = 2;
        auto   camera_buffer      = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);

        // 启动相机线程
        auto camera_future = pool.submit_task(
            [camera_buffer_size, camera_buffer]() { threads::hikcamera_thread_func("config/hikcamera.yaml", camera_buffer_size, camera_buffer); });

        // 启动USB相机线程
        std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> usb_buffer;
        std::shared_future<void>                         usb_camera_future;
        if (UsbCameraConfigManager::isEnabled())
        {
            usb_buffer = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);
            usb_camera_future =
                pool.submit_task([camera_buffer_size, usb_buffer]() { threads::usb_camera_thread_func(camera_buffer_size, usb_buffer); });
        }

        // 启动串口线程
        auto serial_future = pool.submit_task([]() { threads::serial_thread_func(); });

        // 启动装甲板处理线程
        auto armor_future = pool.submit_task([camera_buffer, usb_buffer]() { threads::armor_thread_func(camera_buffer, usb_buffer); });

        // 启动 ROS2 通信线程
        std::shared_future<void> ros2_future;
        
        if (Ros2_comm::isEnabled())
        {
            ros2_future = pool.submit_task([]() {
                threads::ros2_thread_func(UserSerial::getInstance());
            });
            
            MAS_LOG_INFO("ROS2 communication thread started");
        }
        else
        {
            MAS_LOG_INFO("ROS2 communication is disabled");
        }

        // 主线程等待关闭信号
        while (!g_shutdown)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        rm_utils::Display::getInstance().shutdown();

        // 等待所有线程退出
        if (camera_future.valid())
        {
            camera_future.get();
        }
        if (usb_camera_future.valid())
        {
            usb_camera_future.get();
        }
        if (serial_future.valid())
        {
            serial_future.get();
        }
        if (armor_future.valid())
        {
            armor_future.get();
        }
        if (ros2_future.valid())
        {
            ros2_future.get();
        }

        return 0;
    }
    catch (const std::exception &e)
    {
        g_shutdown = true;
        return 1;
    }
}