#ifndef _ARMOR_SHOOT_H_
#define _ARMOR_SHOOT_H_

#include "armor_target.hpp"
#include "plotter.hpp"
#include "serial_types.hpp"
#include "trajectory.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace auto_aim
{
/**
 * @brief 瞄准模式枚举
 */
enum class AimMode
{
    TRACK,      ///< 跟踪模式：低速时锁定并跟踪预测装甲板
    HERO_PHASE, ///< 英雄模式：瞄准预测装甲板，仅在正面相位开火
    COMING      ///< 来板模式：选择正在靠近的预测装甲板
};

/**
 * @brief 瞄准点结构
 */
struct AimPoint
{
    bool            valid;        ///< 是否有效
    Eigen::Vector4d xyza;         ///< 瞄准点坐标 [x, y, z, yaw]
    bool            fire_allowed; ///< 是否允许开火
    int             armor_id;     ///< 目标装甲板 ID
    AimMode         mode;         ///< 使用的瞄准模式
    double          phase_angle;  ///< 预测命中时刻装甲板相对车中心视线的相位角（rad）
};

class ArmorShoot
{
  public:
    /**
     * @brief 构造函数
     * @param config_path YAML 配置文件路径
     */
    explicit ArmorShoot(const std::string &config_path = "config/auto_aim.yaml");

    ~ArmorShoot();

    /**
     * @brief 主解算函数：计算目标角度和开火建议
     * @param target 目标对象（可选）
     * @param timestamp 当前时间戳
     * @param R_gimbal2world 云台到世界坐标系的旋转矩阵
     * @param bgr_img BGR 图像（用于调试显示）
     * @param window_name 调试窗口名称
     * @param quaternion_age_ms 最近一次串口姿态的主机接收年龄
     * @param tracking_confirmed 当前帧是否有可信的跟踪更新
     * @return SendPacket 包含开火建议和角度的数据包
     */
    SendPacket shoot(const std::optional<Target> &target, std::chrono::steady_clock::time_point timestamp, const Eigen::Matrix3d &R_gimbal2world,
                     const cv::Mat &bgr_img = cv::Mat(), const std::string &window_name = "", int64_t quaternion_age_ms = -1,
                     bool tracking_confirmed = true);

    AimPoint debug_aim_point; ///< 调试用的瞄准点信息

  private:
    /**
     * @brief 选择瞄准点
     * @param target 目标对象
     * @param lock_id 当前锁定的装甲板 ID
     * @param coming_mode 当前是否处于来板模式
     * @return AimPoint 选择的瞄准点
     */
    AimPoint chooseAimPoint(const Target &target, int &lock_id, bool &coming_mode) const;

    /**
     * @brief 由瞄准点算出连续化的目标 yaw，并更新 last_target_yaw_
     * @details yaw 只依赖几何（atan2），不依赖弹道解算。所以弹道无解时它仍然有效，
     *          云台可以继续对准目标方向。调用一次即把 has_yaw_ 置真。
     */
    double continuousYaw(const Eigen::Vector3d &xyz, double gimbal_yaw);

    /**
     * @brief 临时丢失目标 (temp_lost) 的时候发送的 “维持包”
     * @details 视觉暂时看不到装甲板，但是不能直接撒手不管云台。
     *          yaw/pitch 冻结在上一次实际发送值。开机后尚无有效值时交还当前云台姿态。
     *          found=1（保持视觉控制）、fire_advice=0（不开火）。
     */
    SendPacket holdPacket(double gimbal_yaw, double gimbal_pitch, std::chrono::steady_clock::time_point now);

    /**
    * @brief 平滑指令：对目标角度进行平滑处理
    * @param yaw 目标 yaw 角度
    * @param pitch 目标 pitch 角度
    * @param gimbal_yaw 云台当前 yaw 角度
    * @param gimbal_pitch 云台当前 pitch 角度
    * @param now 当前时间点
    * @return std::pair<double, double> 平滑后的 yaw 和 pitch 角度
    */
    std::pair<double, double> smoothCommand(double yaw, double pitch, double gimbal_yaw, double gimbal_pitch,
                                             std::chrono::steady_clock::time_point now);
    
    /**
     * @brief 对单个轴进行平滑处理
     * @param target 目标角度
     * @param target_speed 目标角速度
     * @param dt 时间间隔
     * @param max_speed 最大角速度
     * @param max_acc 最大角加速度
     * @param position 当前角度
     * @param velocity 当前角速度
     * @details 该函数会根据目标角度和速度，结合最大速度和加速度限制，更新当前角度和角速度，实现平滑过渡。
     */
    static void stepAxis(double target, double target_speed, double dt, double max_speed, double max_acc,
                         double &position, double &velocity);

    /**
     * @brief 绘制诊断信息
     * @param status 当前状态描述
     * @param target 当前目标对象（可选）
     * @param aim_point 当前瞄准点信息
     * @param packet 当前发送的数据包
     * @param gimbal_euler 云台欧拉角
     * @param quaternion_age_ms 最近一次串口姿态的主机接收年龄
     * @param process_delay_ms 处理延迟（毫秒）
     * @param iteration_converged 迭代求解是否收敛
     * @details 该函数会将当前的诊断信息以 JSON 格式发送到 Plotter，用于可视化和调试。
     */
    void plotDiagnostics(const char *status, const std::optional<Target> &target, const AimPoint &aim_point,
                         const SendPacket &packet, const Eigen::Vector3d &gimbal_euler, int64_t quaternion_age_ms,
                         double process_delay_ms, bool iteration_converged) const;

    void showDebug(const std::optional<Target> &target, const AimPoint &aim_point, double gimbal_yaw, double gimbal_pitch, float target_yaw,
                   float target_pitch, uint8_t fire, const cv::Mat &bgr_img, const std::string &window_name) const noexcept;

    // 配置参数
    double yaw_offset_;           ///< yaw 偏移量（弧度）
    double pitch_offset_;         ///< pitch 偏移量（弧度）
    double comming_angle_;        ///< 来板角度阈值（弧度）
    double leaving_angle_;        ///< 去板角度阈值（弧度）
    double fire_delay_time_;      ///< 发弹延迟时间（秒）
    double yaw_tolerance_near_;   ///< 近距离 yaw 容差（弧度）
    double yaw_tolerance_far_;    ///< 远距离 yaw 容差（弧度）
    double pitch_tolerance_near_; ///< 近距离 pitch 容差（弧度）
    double pitch_tolerance_far_;  ///< 远距离 pitch 容差（弧度）
    double bullet_speed_;         ///< 子弹初速度（m/s）
    bool   debug_;                ///< 调试模式开关
    bool   plotter_enable_;       ///< plotter 输出开关

    // 小陀螺相关参数
    bool   hero_mode_;                         ///< 英雄模式开关
    double spinning_threshold_low_;            ///< 低转速阈值（rad/s）
    double fire_phase_angle_    = 20.0 / 57.3; ///< 装甲板允许开火的最大正面相位角（rad）
    double recovery_done_error_ = 0.5 / 57.3;  ///< 重捕平滑结束的最大指令误差（rad）
    double max_yaw_speed_   = 12.0;            ///< 重捕过渡最大 yaw 速度（rad/s）
    double max_pitch_speed_ = 8.0;             ///< 重捕过渡最大 pitch 速度（rad/s）
    double max_yaw_acc_     = 50.0;            ///< 重捕过渡最大 yaw 加速度（rad/s²）
    double max_pitch_acc_   = 100.0;           ///< 重捕过渡最大 pitch 加速度（rad/s²）
    int    switch_fire_hold_frames_ = 3;       ///< 切换装甲板后的禁火帧数

    int  lock_id_;                    ///< 锁定的装甲板 ID
    bool coming_mode_ = false;        ///< 是否保持在来板/英雄相位模式，用于转速阈值迟滞
    int  last_aim_armor_id_ = -1;     ///< 上一帧最终选择的装甲板 ID
    int  switch_hold_remaining_ = 0;  ///< 当前剩余切板禁火帧数

    std::unique_ptr<rm_utils::Plotter> plotter_; ///< plotter 实例

    double last_target_yaw_ = 0.0;   // 连续化后的目标 yaw（跨帧累加，可超出 ±π）
    double command_pitch_   = 0.0;   // 重捕过渡中实际发出的 pitch
    double last_raw_pitch_  = 0.0;   // 上一次弹道解算得到的 pitch
    double command_yaw_     = 0.0;   // 重捕过渡中实际发出的 yaw
    double ref_yaw_         = 0.0;   // 上一帧原始 yaw，用于估计目标指令速度
    double ref_pitch_       = 0.0;   // 上一帧原始 pitch
    double yaw_speed_       = 0.0;   // 重捕过渡 yaw 速度
    double pitch_speed_     = 0.0;   // 重捕过渡 pitch 速度
    bool   has_yaw_         = false; // last_target_yaw_ 是否已被有效赋值过
    bool   has_command_     = false; // 是否有可保持的实际发送角度
    bool   has_ref_         = false; // 是否有上一帧原始角可供差分
    bool   recovering_      = true;  // 丢失/保持后是否正在平滑追赶新目标
    std::chrono::steady_clock::time_point last_command_time_{};

    mutable std::map<std::string, double>                                fps_map_;       ///< FPS 统计
    mutable std::map<std::string, int>                                   count_map_;     ///< 帧计数
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_; ///< 时间统计
};

} // namespace auto_aim

#endif // _ARMOR_SHOOT_H_
