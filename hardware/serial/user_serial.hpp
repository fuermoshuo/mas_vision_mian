#ifndef _USER_SERIAL_H_
#define _USER_SERIAL_H_

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "SPSCQueue.h"
#include "serial.h"
#include "serial_types.hpp"

class UserSerial
{
  public:
    // 禁用拷贝构造函数和赋值运算符, 确保单例模式
    UserSerial(const UserSerial &)            = delete;
    UserSerial &operator=(const UserSerial &) = delete;

    static UserSerial &getInstance(const std::string &config_path = "config/serial_config.yaml")
    {
        static UserSerial instance(config_path);
        return instance;
    }

    ~UserSerial();

    /*
     * @brief 关闭串口
     */
    void closeSerial();
    /*
     * @brief 检查串口是否连接
     * @return true 已连接 false 未连接
     */
    bool isConnectedStatus() const { return isConnected; }
    /*
     * @brief 发送视觉数据
     * @param target_yaw 目标 yaw 角度
     * @param target_pitch 目标 pitch 角度
     * @param armor_found 目标是否被找到
     * @param fire_advice 开火建议
     */
    void sendVision(float target_yaw = 0.0f, float target_pitch = 0.0f, uint8_t found = 0, uint8_t fire_advice = 0);
    /** 禁用视觉跟随和开火，保留最后一次目标角度。 */
    void disableVision();
    /*
     * @brief 发送导航数据
     * @param vx x 方向速度
     * @param vy y 方向速度
     * @param nav_state 导航状态
     */
    void sendNav(float vx = 0.0f, float vy = 0.0f, uint8_t nav_state = 0);
    /*
     * @brief 发送数据包到串口，serial线程调用
     */
    void sendPacketToSerial();
    /*
     * @brief 接收串口数据，serial线程调用
     */
    void receiveSerial();
    /*
     * @brief 重新连接串口
     */
    void reconnect();
    /*
     * @brief 获取指定时间点的四元数
     * @param t 时间点
     * @return Eigen::Quaterniond 四元数
     */
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    /**
     * @brief 检查四元数数据是否有效（未超时）
     * @return true 数据有效，false 数据超时或无效
     */
    bool isQuaternionValid() const;

    /** 返回最近一次合法四元数的主机接收年龄，未收到时返回 -1。 */
    int64_t quaternionAgeMs() const;

    /**
     * @brief 获取当前模式
     * @return VisionMode 模式
     */
    VisionMode getVisionMode() const { return vision_mode_.load(); }

    bool hasReceivedData() const { return has_received_data_.load(); }

    /**
     * @brief 获取裁判系统数据
     * @return RefereePacket 裁判系统数据
     */
    RefereePacket getRefereeData() const
    {
        std::lock_guard<std::mutex> lock(referee_mutex_);
        return referee_data_;
    }

  private:
    UserSerial(const std::string &config_path = "config/serial_config.yaml");

    // 串口对象
    std::shared_ptr<serial::Serial>       serial_port;
    std::atomic<bool>                     isConnected{false};
    std::chrono::steady_clock::time_point last_reconnect_time_;
    std::atomic<int64_t>                  last_data_receive_ms_{0};

    // 发送数据包
    SendPacket send_packet_{};
    std::mutex send_packet_mutex_;

    // 配置参数
    std::string           port_{"/dev/mascdc"};
    uint32_t              baudrate_{115200};
    int                   timeout_{2};
    serial::bytesize_t    bytesize_{serial::eightbits};
    serial::parity_t      parity_{serial::parity_none};
    serial::stopbits_t    stopbits_{serial::stopbits_one};
    serial::flowcontrol_t flowcontrol_{serial::flowcontrol_none};
    bool                  debug_{false};
    int                   data_timeout_ms_{100};

    // 接收缓冲区
    std::vector<uint8_t> recv_buffer_;
    std::vector<uint8_t> temp_read_buf_;
    std::atomic<VisionMode> vision_mode_{VisionMode::AUTO_AIM_RED};
    RefereePacket          referee_data_{}; // 接收到的裁判系统数据
    mutable std::mutex     referee_mutex_;

    // 数据队列
    rigtorp::SPSCQueue<QuaternionWithTimestamp> data_queue_{5000};
    QuaternionWithTimestamp data_ahead_;
    QuaternionWithTimestamp data_behind_;
    
    // 标志位：是否收到过至少一次有效数据
    std::atomic<bool> has_received_data_{false};
};

#endif
