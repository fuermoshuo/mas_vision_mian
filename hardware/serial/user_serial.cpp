#include "user_serial.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <stdio.h>
#include <string.h>

#include "mas_log.hpp"
#include "serial_types.hpp"
#include <yaml-cpp/yaml.h>

extern std::atomic<bool> g_shutdown;

UserSerial::UserSerial(const std::string &config_path)
{
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_WARN("Config file not found: {}, using default values", config_path.c_str());
        }

        // 加载YAML文件
        YAML::Node config = YAML::LoadFile(config_path);

        if (!config["serial"])
        {
            MAS_LOG_WARN("No 'serial' section in config file");
        }

        if (config["serial"])
        {
            if (config["serial"]["debug"]) debug_ = config["serial"]["debug"].as<bool>(false);
            if (config["serial"]["port"]) port_ = config["serial"]["port"].as<std::string>();
            if (config["serial"]["baudrate"]) baudrate_ = config["serial"]["baudrate"].as<uint32_t>(115200);
            if (config["serial"]["timeout"]) timeout_ = config["serial"]["timeout"].as<int>(2);
            if (config["serial"]["data_timeout_ms"]) data_timeout_ms_ = config["serial"]["data_timeout_ms"].as<int>(100);
            if (config["serial"]["bytesize"])
            {
                int bs = config["serial"]["bytesize"].as<int>(8);
                switch (bs)
                {
                case 5:
                    bytesize_ = serial::fivebits;
                    break;
                case 6:
                    bytesize_ = serial::sixbits;
                    break;
                case 7:
                    bytesize_ = serial::sevenbits;
                    break;
                case 8:
                    bytesize_ = serial::eightbits;
                    break;
                default:
                    bytesize_ = serial::eightbits;
                    break;
                }
            }
            if (config["serial"]["parity"])
            {
                std::string p = config["serial"]["parity"].as<std::string>("none");
                if (p == "none")
                    parity_ = serial::parity_none;
                else if (p == "odd")
                    parity_ = serial::parity_odd;
                else if (p == "even")
                    parity_ = serial::parity_even;
                else
                    parity_ = serial::parity_none;
            }
            if (config["serial"]["stopbits"])
            {
                int sb = config["serial"]["stopbits"].as<int>(1);
                if (sb == 1)
                    stopbits_ = serial::stopbits_one;
                else if (sb == 2)
                    stopbits_ = serial::stopbits_two;
                else
                    stopbits_ = serial::stopbits_one;
            }
            if (config["serial"]["flowcontrol"])
            {
                std::string fc = config["serial"]["flowcontrol"].as<std::string>("none");
                if (fc == "none")
                    flowcontrol_ = serial::flowcontrol_none;
                else if (fc == "software")
                    flowcontrol_ = serial::flowcontrol_software;
                else if (fc == "hardware")
                    flowcontrol_ = serial::flowcontrol_hardware;
                else
                    flowcontrol_ = serial::flowcontrol_none;
            }
        }

        MAS_LOG_INFO("Config loaded successfully from: {}", config_path.c_str());
    }
    catch (const YAML::Exception &e)
    {
        MAS_LOG_ERROR("YAML parsing error: {}", e.what());
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Error loading config: {}", e.what());
    }

    recv_buffer_.reserve(1024);
    temp_read_buf_.resize(256);
    has_received_data_.store(false);
    // 初始化滑动窗口
    data_ahead_.quaternion  = Eigen::Quaterniond::Identity();
    data_behind_.quaternion = Eigen::Quaterniond::Identity();
    // 初始时间戳设为 epoch，这样第一次循环一定会尝试去取数据
    data_ahead_.timestamp  = std::chrono::steady_clock::time_point();
    data_behind_.timestamp = std::chrono::steady_clock::time_point();

    try
    {
        serial_port =
            std::make_unique<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(timeout_), bytesize_, parity_, stopbits_, flowcontrol_);
        if (serial_port && serial_port->isOpen())
        {
            isConnected = true;
        }
        else
        {
            isConnected = false;
            return;
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while opening serial port: {}", e.what());
        isConnected = false;
        return;
    }
}

UserSerial::~UserSerial() {}

void UserSerial::closeSerial()
{
    if (serial_port)
    {
        if (serial_port->isOpen())
        {
            serial_port->close();
        }
        serial_port.reset();
    }
    isConnected = false;
    MAS_LOG_INFO("Serial port closed");
}

void UserSerial::sendVision(float target_yaw, float target_pitch, uint8_t found, uint8_t fire_advice)
{
    std::lock_guard<std::mutex> lock(send_packet_mutex_);
    if (found)
    {
        send_packet_.target_yaw   = target_yaw;
        send_packet_.target_pitch = target_pitch;
    }
    send_packet_.found        = found;
    send_packet_.fire_advice  = found ? fire_advice : 0;
}

void UserSerial::disableVision()
{
    std::lock_guard<std::mutex> lock(send_packet_mutex_);
    send_packet_.found       = 0;
    send_packet_.fire_advice = 0;
}

void UserSerial::sendNav(float vx, float vy, uint8_t nav_state)
{
    std::lock_guard<std::mutex> lock(send_packet_mutex_);
    send_packet_.vx        = vx;
    send_packet_.vy        = vy;
    send_packet_.nav_state = nav_state;
}

void UserSerial::sendPacketToSerial()
{
    while (!g_shutdown && (!isConnected || !serial_port || !serial_port->isOpen()))
    {
        reconnect();
    }
    if (g_shutdown) return;

    try
    {
        SendPacket packet;
        {
            std::lock_guard<std::mutex> lock(send_packet_mutex_);
            packet = send_packet_;
        }
        serial_port->write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while sending data: {}", e.what());
        isConnected = false;
    }
}

void UserSerial::receiveSerial()
{
    while (!g_shutdown && (!isConnected || !serial_port || !serial_port->isOpen()))
    {
        reconnect();
    }
    if (g_shutdown) return;

    try
    {
        size_t bytes_available = serial_port->available();
        if (bytes_available > 0)
        {
            size_t bytes_to_read = std::min(bytes_available, temp_read_buf_.size());
            size_t bytes_read    = serial_port->read(temp_read_buf_.data(), bytes_to_read);

            if (bytes_read > 0)
            {
                recv_buffer_.insert(recv_buffer_.end(), temp_read_buf_.begin(), temp_read_buf_.begin() + bytes_read);

                if (debug_)
                {
                    std::string hex_str;
                    for (size_t i = 0; i < bytes_read; ++i)
                    {
                        char buf[4];
                        snprintf(buf, sizeof(buf), "%02X ", temp_read_buf_[i]);
                        hex_str += buf;
                    }
                    MAS_LOG_INFO("Received {} bytes (HEX): {}", bytes_read, hex_str);
                }
            }

            while (recv_buffer_.size() >= sizeof(ReceivePacket))
            {
                auto header_it = std::find(recv_buffer_.begin(), recv_buffer_.end(), 0xAA);
                if (header_it == recv_buffer_.end())
                {
                    recv_buffer_.clear();
                    break;
                }
                if (header_it != recv_buffer_.begin())
                {
                    recv_buffer_.erase(recv_buffer_.begin(), header_it);
                    header_it = recv_buffer_.begin();
                }

                if (recv_buffer_.size() < sizeof(ReceivePacket))
                {
                    break;
                }

                ReceivePacket packet;
                std::memcpy(&packet, recv_buffer_.data(), sizeof(ReceivePacket));

                if (packet.tail == 0x5A)
                {
                    Eigen::Quaterniond q(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
                    const double norm = q.norm();
                    if (std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) && norm > 1e-6)
                    {
                        q.normalize();
                        const auto received_at = std::chrono::steady_clock::now();
                        if (data_queue_.try_push({q, received_at}))
                        {
                            switch (packet.mode)
                            {
                            case 0: vision_mode_.store(VisionMode::AUTO_AIM_RED); break;
                            case 1: vision_mode_.store(VisionMode::AUTO_AIM_BLUE); break;
                            case 2: vision_mode_.store(VisionMode::SMALL_RUNE_RED); break;
                            case 3: vision_mode_.store(VisionMode::SMALL_RUNE_BLUE); break;
                            case 4: vision_mode_.store(VisionMode::BIG_RUNE_RED); break;
                            case 5: vision_mode_.store(VisionMode::BIG_RUNE_BLUE); break;
                            default:
                                vision_mode_.store(VisionMode::AUTO_AIM_RED);
                                MAS_LOG_WARN("[UserSerial] Invalid mode: {}", packet.mode);
                                break;
                            }

#ifdef SENTRY
                            {
                                std::lock_guard<std::mutex> lock(referee_mutex_);
                                referee_data_ = packet.referee_data;
                            }
#endif
                            const auto received_ms = std::chrono::duration_cast<std::chrono::milliseconds>(received_at.time_since_epoch()).count();
                            last_data_receive_ms_.store(received_ms);
                            has_received_data_.store(true);
                        }
                    }
                    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + sizeof(ReceivePacket));
                }
                else
                {
                    recv_buffer_.erase(recv_buffer_.begin());
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while receiving data: {}", e.what());
        isConnected = false;
    }
}

void UserSerial::reconnect()
{
    if (g_shutdown)
    {
        return;
    }
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_time_).count() < 5000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }
    last_reconnect_time_ = now;

    MAS_LOG_INFO("Attempting to reconnect to serial port...");

    if (serial_port)
    {
        if (serial_port->isOpen())
        {
            serial_port->close();
        }
        serial_port.reset();
    }
    isConnected = false;

    disableVision();
    recv_buffer_.clear();

    // data_queue_ 是单生产者/单消费者队列，重连线程不能从生产者侧 pop。上层靠标志位拒绝使用旧数据
    has_received_data_.store(false);
    last_data_receive_ms_.store(0);

    try
    {
        serial_port =
            std::make_shared<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(timeout_), bytesize_, parity_, stopbits_, flowcontrol_);
        if (serial_port && serial_port->isOpen())
        {
            isConnected = true;
            MAS_LOG_INFO("Reconnected to {}", port_.c_str());
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while reconnecting to {}: {}", port_.c_str(), e.what());
        isConnected = false;
    }
}

bool UserSerial::isQuaternionValid() const
{
    const auto age_ms = quaternionAgeMs();
    return age_ms >= 0 && age_ms <= data_timeout_ms_;
}

int64_t UserSerial::quaternionAgeMs() const
{
    if (!has_received_data_.load()) return -1;

    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    return now_ms - last_data_receive_ms_.load();
}

Eigen::Quaterniond UserSerial::q(std::chrono::steady_clock::time_point t)
{
    // 1. 尝试从 SPSCQueue 搬运数据到本地窗口
    // 逻辑：如果目标时间 t 比当前窗口的尾部还新，说明我们需要更新窗口
    while (data_behind_.timestamp < t)
    {
        // 把旧的尾部变成新的头部
        data_ahead_ = data_behind_;

        // 尝试从队列里取新数据
        QuaternionWithTimestamp *ptr = data_queue_.front();
        if (ptr == nullptr)
        {
            // 队列里没有新数据了，无法更新窗口
            // 此时只能基于现有的 data_behind_ 进行外推/返回
            break;
        }

        // 取出新数据
        data_behind_ = *ptr;
        data_queue_.pop();
    }

    // 2. 边界条件：如果窗口还未初始化（比如刚开始）
    // 如果 data_ahead_ 是默认构造的（时间戳为 epoch），且 data_behind_ 也是旧的
    // 这里我们做一个简单的检查，如果没有有效的两个点，直接返回最新的
    if (data_ahead_.timestamp == std::chrono::steady_clock::time_point())
    {
        return data_behind_.quaternion;
    }

    // 3. 准备插值数据
    Eigen::Quaterniond q_a = data_ahead_.quaternion;
    Eigen::Quaterniond q_b = data_behind_.quaternion;
    auto               t_a = data_ahead_.timestamp;
    auto               t_b = data_behind_.timestamp;

    // 4. 时间差校验
    std::chrono::duration<double> t_ab = t_b - t_a;
    if (t_ab.count() <= 1e-9)
    {
        // 时间戳异常，直接返回最新的
        return q_b.normalized();
    }

    // 5. 计算插值系数 k
    std::chrono::duration<double> t_ac = t - t_a;
    double                        k    = t_ac / t_ab;

    // 限制 k 在 [0, 1] 之间，防止外推导致的抖动
    // 如果 t < t_a，返回 A；如果 t > t_b，返回 B
    k = std::max(0.0, std::min(1.0, k));

    // 6. 四元数符号一致性检查 (关键补充，参考代码里没有这个)
    // 确保插值走最短路径
    double dot_product = q_a.dot(q_b);
    if (dot_product < 0.0)
    {
        // 翻转其中一个四元数的符号
        q_b.coeffs() = -q_b.coeffs();
    }

    // 7. 执行球面线性插值并归一化
    return q_a.slerp(k, q_b).normalized();
}
