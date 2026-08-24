#ifndef _ARMOR_TRACK_H_
#define _ARMOR_TRACK_H_

#include "armor_track/armor_pose.hpp"
#include "armor_track/armor_target.hpp"
#include <Eigen/Dense>
#include <optional>
#include <opencv2/core/mat.hpp>
#include <string>

#include "plotter.hpp"

namespace auto_aim
{

class ArmorTrack
{
  public:
    /**
     * @brief 构造函数
     * @param track_config_path 跟踪配置文件路径
     * @param pose_config_path 位姿配置文件路径
     */
    ArmorTrack(const std::string &track_config_path = "config/auto_aim.yaml", const std::string &pose_config_path = "config/hikcamera.yaml");
    ~ArmorTrack();

    /**
     * @brief 获取位姿解算器
     * @return 位姿解算器引用
     */
    ArmorPose &armor_pose() noexcept { return armor_pose_; }

    /**
     * @brief 获取当前跟踪状态
     * @return 状态字符串 (lost/detecting/tracking/temp_lost)
     */
    std::string state() const { return state_; }

    /**
     * @brief 主跟踪函数
     * @param armors 检测到的装甲板列表
     * @param t 当前时间戳
     * @param window_name 窗口名称（用于debug显示）
     * @param bgr_img 图像（用于debug显示）
     * @return 当前锁定目标，tracking/temp_lost 状态下有值，否则 nullopt
     */
    std::optional<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t, std::string window_name = "",
                                cv::Mat bgr_img = cv::Mat()) noexcept;

  private:
    /**
     * @brief 显示跟踪结果（debug模式）
     */
    void showResult(const std::vector<Armor> &armors, const cv::Mat &bgr_img, std::string window_name) const noexcept;

    // 装甲板姿态解算器
    ArmorPose armor_pose_;
    // 当前跟踪目标
    std::optional<Target> target_;

    // YAML 配置参数
    bool                                  debug_;                          // debug开关
    bool                                  plotter_enable_;                 // UDP诊断记录开关
    int                                   min_detect_count_;               // 最小检测次数
    int                                   max_temp_lost_count_;            // 最大临时丢失次数
    int                                   detect_count_;                   // 当前检测次数
    int                                   temp_lost_count_;                // 当前临时丢失次数
    int                                   outpost_max_temp_lost_count_;    // 前哨站最大丢失次数
    int                                   normal_temp_lost_count_;         // 普通目标最大丢失次数
    double                                max_z_innovation_        = 0.25; // PnP 与预测装甲板的最大高度差（m）
    double                                max_position_innovation_ = 0.8;  // PnP 与预测装甲板的最大三维距离（m）
    std::string                           state_, pre_state_;              // 当前/上一状态
    std::chrono::steady_clock::time_point last_timestamp_;                 // 上一帧时间戳

    Eigen::Vector3d                       raw_pnp_xyz_          = Eigen::Vector3d::Zero();   // 保存本帧原始 PnP 解算出来的世界坐标
    bool                                  raw_pnp_valid_        = false;                     // 本帧原始 PnP 解算是否有效
    bool                                  measurement_accepted_ = false;                     // 本帧 PnP 解算是否被 EKF 接受
    int                                   matching_armor_count_ = 0;                         // 本帧匹配到目标编号和类型的装甲板数量
    int                                   pnp_attempt_count_    = 0;                         // 本帧尝试 PnP 解算的次数
    int                                   pnp_success_count_    = 0;                         // 本帧成功 PnP 解算的次数
    double                                innovation_z_         = 0.0;                       // 本帧 PnP 与 EKF 预测的 z 方向新息
    double                                innovation_norm_      = 0.0;                       // 本帧 PnP 与 EKF 预测的位移新息
    std::string                           reject_reason_        = "none";                    // 本帧 PnP 解算被拒绝的原因
    std::string                           reset_reason_         = "none";                    // 本帧 EKF 重置的原因

    /**
     * @brief 状态机处理
     * @param found 是否找到目标
     */
    void state_machine(bool found);

    /**
     * @brief 设置新目标
     * @param armor 初始装甲板
     * @param t 时间戳
     * @return 是否成功
     */
    bool set_target(const Armor &armor, std::chrono::steady_clock::time_point t);

    /**
     * @brief 更新目标
     * @param armors 检测到的装甲板列表
     * @param t 时间戳
     * @return 是否成功
     */
    bool update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

    void plotDiagnostics(const std::vector<Armor> &armors, bool found, double dt, const std::string &state_before);

    // Debug相关
    mutable std::map<std::string, double>                                fps_map_;       // FPS计算
    mutable std::map<std::string, int>                                   count_map_;     // 帧计数
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_; // 上次时间

    // Plotter for debug
    rm_utils::Plotter plotter_;
};
} // namespace auto_aim
#endif // _ARMOR_TRACK_H_
