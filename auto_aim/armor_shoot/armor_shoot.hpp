#ifndef _ARMOR_SHOOT_H_
#define _ARMOR_SHOOT_H_

#include "armor_target.hpp"
#include "plotter.hpp"
#include "serial_types.hpp"
#include "trajectory.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <optional>
#include <string>

namespace auto_aim
{
/**
 * @brief 瞄准模式枚举
 */
enum class AimMode
{
    TRACK,       ///< 跟踪模式：低速时锁定并跟踪可见装甲板
    HERO_CENTER, ///< 英雄模式：高速或 hero_mode 启用时瞄准旋转中心
    COMING       ///< 来板模式：中速时选择正在靠近的装甲板
};

/**
 * @brief 瞄准点结构
 */
struct AimPoint
{
    bool            valid;        ///< 是否有效
    Eigen::Vector4d xyza;         ///< 瞄准点坐标 [x, y, z, yaw]
    bool            center_aim;   ///< 是否为中心瞄准（英雄模式）
    bool            fire_allowed; ///< 是否允许开火
    int             armor_id;     ///< 目标装甲板 ID
    AimMode         mode;         ///< 使用的瞄准模式
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
     * @return SendPacket 包含开火建议和角度的数据包
     */
    SendPacket shoot(const std::optional<Target> &target, std::chrono::steady_clock::time_point timestamp, const Eigen::Matrix3d &R_gimbal2world,
                     const cv::Mat &bgr_img = cv::Mat(), const std::string &window_name = "");

    AimPoint debug_aim_point; ///< 调试用的瞄准点信息

  private:
    /**
     * @brief 选择瞄准点
     * @param target 目标对象
     * @return AimPoint 选择的瞄准点
     */
    AimPoint chooseAimPoint(const Target &target);

    /**
     * @brief 由瞄准点算出连续化的目标 yaw，并更新 last_target_yaw_
     * @details yaw 只依赖几何（atan2），不依赖弹道解算。所以弹道无解时它仍然有效，
     *          云台可以继续对准目标方向。调用一次即把 has_yaw_ 置真。
     */
    double continuousYaw(const Eigen::Vector3d &xyz, double gimbal_yaw);

    /**
     * @brief 保持指令：识别到装甲板但这一帧算不出完整指令时使用
     * @details yaw 用 last_target_yaw_（若本帧能算，调用前先调 continuousYaw 更新它）；
     *          pitch 冻结在上一次有效值。开机后尚无有效值时交还当前云台姿态（原地不动）。
     *          found=1（确实识别到了）、fire_advice=0（不开火）。
     */
    SendPacket holdPacket(double gimbal_yaw, double gimbal_pitch) const;

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
    bool   hero_mode_;               ///< 英雄模式开关
    double spinning_threshold_low_;  ///< 低转速阈值（rad/s）
    double spinning_threshold_high_; ///< 高转速阈值（rad/s）

    int lock_id_; ///< 锁定的装甲板 ID

    std::unique_ptr<rm_utils::Plotter> plotter_; ///< plotter 实例

    double last_target_yaw_ = 0.0;   // 连续化后的目标 yaw（跨帧累加，可超出 ±π）
    double last_sent_pitch_ = 0.0;   // 上一次弹道有解时发出的 pitch，保持状态下冻结用
    bool   has_yaw_         = false; // last_target_yaw_ 是否已被有效赋值过
    bool   has_pitch_       = false; // last_sent_pitch_ 是否已被有效赋值过

    mutable std::map<std::string, double>                                fps_map_;       ///< FPS 统计
    mutable std::map<std::string, int>                                   count_map_;     ///< 帧计数
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_; ///< 时间统计
};

} // namespace auto_aim

#endif // _ARMOR_SHOOT_H_
