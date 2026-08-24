#ifndef _ARMOR_TARGET_H_
#define _ARMOR_TARGET_H_

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor_types.hpp"
#include "extended_kalman_filter.hpp"

namespace auto_aim
{

/**
 * @brief 跟踪目标类
 * @details 封装目标状态、EKF滤波器和预测/更新方法
 */
class Target
{
  public:
    enum class UpdateRejectReason
    {
        NONE,
        NONFINITE,
        Z_INNOVATION,
        POSITION_INNOVATION
    };

    std::string name;         // 目标编号
    ArmorType   armor_type;   // 装甲板类型
    int         priority;     // 优先级
    bool        jumped;       // 是否发生跳变
    int         last_id;      // 上一次跟踪的装甲板ID
    int         armor_num;    // 装甲板数量
    int         update_count; // 更新次数
    bool        is_switch;    // 是否切换目标
    int         switch_count; // 切换次数

    Target() = default;

    /**
     * @brief 构造函数
     * @param armor 初始装甲板
     * @param t 时间戳
     * @param radius 旋转半径
     * @param armor_num 装甲板数量
     * @param P0_dig 初始协方差对角线
     */
    Target(const Armor &armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, const Eigen::VectorXd &P0_dig,
           double max_z_innovation, double max_position_innovation);

    /**
     * @brief 预测目标在未来时刻的状态
     * @param t 未来时间戳
     */
    void predict(std::chrono::steady_clock::time_point t);

    /**
     * @brief 更新EKF状态
     * @param armor 观测到的装甲板
     * @return 观测是否通过离群检查并更新了 EKF
     */
    bool update(const Armor &armor);

    // 记录跟踪器更新被拒绝的原因
    UpdateRejectReason last_update_reject_reason() const { return last_update_reject_reason_; }

    double last_innovation_z() const { return last_innovation_z_; }
    double last_innovation_norm() const { return last_innovation_norm_; }

    /**
     * @brief 获取 EKF 状态向量
     */
    Eigen::VectorXd ekf_x() const { return ekf_.x; }

    /**
     * @brief 获取 EKF 滤波器
     */
    const rm_utils::ExtendedKalmanFilter &ekf() const { return ekf_; }

    /**
     * @brief 获取所有装甲板的位置和角度列表
     */
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    /**
     * @brief 检查目标是否发散
     */
    bool diverged() const;

    /**
     * @brief 检查目标是否收敛
     */
    bool converged() const;

    /**
     * @brief 获取时间戳
     */
    std::chrono::steady_clock::time_point timestamp() const { return timestamp_; }

  private:
    rm_utils::ExtendedKalmanFilter        ekf_;          // EKF滤波器
    std::chrono::steady_clock::time_point timestamp_;    // 时间戳
    bool                                  is_converged_; // 是否收敛
    double                                max_z_innovation_          = 0.25;      // Z 方向新息最大允许阈值
    double                                max_position_innovation_   = 0.8;       // 位置新息最大允许阈值
    UpdateRejectReason                    last_update_reject_reason_ = UpdateRejectReason::NONE;
    double                                last_innovation_z_         = 0.0;       // 上一帧更新的 z 方向新息
    double                                last_innovation_norm_      = 0.0;       // 上一帧更新的位移新息

    /**
     * @brief 更新EKF（yaw, pitch, distance, angle）
     */
    void update_ypda(const Armor &armor, int id);

    /**
     * @brief 观测函数：计算装甲板在世界坐标系下的位置
     */
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd &x, int id) const;

    /**
     * @brief 观测函数雅可比矩阵
     */
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd &x, int id) const;

    /**
     * @brief 约束状态向量
     */
    void constrainState();
};

} // namespace auto_aim

#endif // _ARMOR_TARGET_H_
