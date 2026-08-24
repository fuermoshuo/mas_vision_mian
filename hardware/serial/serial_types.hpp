#ifndef _SERIAL_TYPES_H_
#define _SERIAL_TYPES_H_

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>

// #define SENTRY 

// 接收结构体

struct __attribute__((packed)) RefereePacket
{
    uint16_t projectile_allowance_17mm;       // 剩余发弹量
    uint8_t  power_management_shooter_output; // 功率管理 shooter 输出
    uint16_t current_hp;                      // 机器人当前血量
    uint16_t outpost_HP;                      // 前哨站血量
    uint16_t base_HP;                         // 基地血量
    uint8_t  game_progess;
};

#ifdef  SENTRY
struct __attribute__((packed)) ReceivePacket
{
    uint8_t header = 0xAA;
    uint8_t mode;
    float   q[4]; // 四元数 wxyz 顺序
    RefereePacket referee_data;
    uint8_t tail = 0X5A;
};
#else
struct __attribute__((packed)) ReceivePacket
{
    uint8_t header = 0xAA;
    uint8_t mode;
    float   q[4]; // 四元数 wxyz 顺序
    uint8_t tail = 0X5A;
};
#endif

// 发送结构体
struct __attribute__((packed)) SendPacket
{
    uint8_t header = 0XBB;
    uint8_t found;
    uint8_t fire_advice;
    float   target_yaw;
    float   target_pitch;
    float   vx;
    float   vy;
    uint8_t nav_state;
    uint8_t tail = 0X5B;
};

// 发送到队列的数据消息
struct ReceivedDataMsg
{
    Eigen::Quaterniond                    quaternion;
    uint8_t                               mode;
    std::chrono::steady_clock::time_point timestamp;
};

// 带时间戳的四元数数据
struct QuaternionWithTimestamp
{
    Eigen::Quaterniond                    quaternion;
    std::chrono::steady_clock::time_point timestamp;
};

// 视觉模式枚举
enum VisionMode
{
    AUTO_AIM_RED    = 0,
    AUTO_AIM_BLUE   = 1,
    SMALL_RUNE_RED  = 2,
    SMALL_RUNE_BLUE = 3,
    BIG_RUNE_RED    = 4,
    BIG_RUNE_BLUE   = 5,
};

#endif // _SERIAL_TYPES_H_