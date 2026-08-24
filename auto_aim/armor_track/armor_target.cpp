#include "armor_target.hpp"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <utility>

#include "math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr bool radiusModelValid(double r, double l)
{
    return r > 0.05 && r <= 0.45 && r + l > 0.05 && r + l <= 0.5;
}

static_assert(radiusModelValid(0.25, 0.25));
} // namespace

/**
 * @brief 构造函数
 * @details 根据装甲板初始化EKF状态向量和协方差矩阵
 */
Target::Target(const Armor &armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, const Eigen::VectorXd &P0_dig,
               double max_z_innovation, double max_position_innovation)
    : jumped(false), last_id(0), armor_num(armor_num), update_count(0), is_switch(false), switch_count(0), timestamp_(t), is_converged_(false),
      max_z_innovation_(max_z_innovation), max_position_innovation_(max_position_innovation)
{
    name       = armor.number;
    armor_type = armor.type;

    const Eigen::Vector3d &xyz = armor.xyz_in_world;
    const Eigen::Vector3d &ypr = armor.ypr_in_world;

    // 根据装甲板位置反推旋转中心
    auto center_x = xyz[0] + radius * std::cos(ypr[0]);
    auto center_y = xyz[1] + radius * std::sin(ypr[0]);
    auto center_z = xyz[2];

    // x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, r, l, h]
    Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius, 0, 0}};
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();

    // 状态向量加法（处理角度环绕）
    auto x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a + b;
        c[6]              = rm_utils::limit_rad(c[6]);
        return c;
    };

    ekf_ = rm_utils::ExtendedKalmanFilter(x0, P0, x_add);
}

/**
 * @brief 预测目标在未来时刻的状态
 * @details 使用EKF进行状态预测，构建状态转移矩阵和过程噪声矩阵
 */
void Target::predict(std::chrono::steady_clock::time_point t)
{
    // 计算时间间隔
    auto dt    = rm_utils::delta_time(t, timestamp_);
    timestamp_ = t;

    // 状态转移矩阵 F (匀速模型)
    // x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, r, l, h]
    Eigen::MatrixXd F{{1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

    // 过程噪声参数：前哨站运动较慢，使用较小的噪声
    double v1 = (name == "outpost") ? 10 : 100;
    double v2 = (name == "outpost") ? 0.1 : 400;

    // 计算过程噪声矩阵Q的系数
    auto   a   = dt * dt * dt * dt / 4;
    auto   b   = dt * dt * dt / 2;
    auto   c   = dt * dt;
    double q_h = (name == "outpost") ? 0.0 : 1e-6;

    // 过程噪声矩阵 Q
    Eigen::MatrixXd Q{
        {a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, a * v1, b * v1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, b * v1, c * v1, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, a * v1, b * v1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, b * v1, c * v1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, a * v2, b * v2, 0, 0, 0}, {0, 0, 0, 0, 0, 0, b * v2, c * v2, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_h}};

    // 非线性状态转移函数（处理角度环绕）
    auto f = [&](const Eigen::VectorXd &x) -> Eigen::VectorXd {
        Eigen::VectorXd x_prior = F * x;
        x_prior[6]              = rm_utils::limit_rad(x_prior[6]);
        return x_prior;
    };

    // 前哨站转速特判：收敛后限制角速度为理论值
    if (converged() && name == "outpost" && std::abs(ekf_.x[7]) > 2) ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;

    // 执行EKF预测
    ekf_.predict(F, Q, f);
}

/**
 * @brief 更新 EKF 状态
 * @details 匹配装甲板并更新 EKF
 */
bool Target::update(const Armor &armor)
{
    // 装甲板匹配：按距离和角度综合评分，只匹配最近的 3 个
    int         best_id         = 0;
    double      min_score       = 1e10;
    const auto &xyza_list       = armor_xyza_list();

    // 构建带索引的装甲板列表
    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num && i < (int)xyza_list.size(); i++)
    {
        xyza_i_list.push_back({xyza_list[i], i});
    }

    // 按距离从小到大排序（距离最近的优先匹配）
    std::sort(xyza_i_list.begin(), xyza_i_list.end(), [](const std::pair<Eigen::Vector4d, int> &a, const std::pair<Eigen::Vector4d, int> &b) {
        Eigen::Vector3d ypd1 = rm_utils::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = rm_utils::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
    });

    // 只取前 3 个最近的装甲板进行匹配，背面装甲板不可能匹配
    int match_count = std::min(3, (int)xyza_i_list.size());
    for (int i = 0; i < match_count; i++)
    {
        const auto &xyza = xyza_i_list[i].first;
        const int   id   = xyza_i_list[i].second;

        const Eigen::Vector3d innovation =
            armor.xyz_in_world - xyza.head<3>();

        if (!innovation.allFinite())
        {
            continue;
        }
        const double xy_error =innovation.head<2>().norm() / max_position_innovation_;
        const double z_error =std::abs(innovation.z()) / max_z_innovation_;
        const double yaw_error = std::abs(rm_utils::limit_rad(armor.ypr_in_world[0] - xyza[3]));
        const double angle_weight =(name != "outpost" && std::abs(ekf_.x[7]) > 2.0)? 2.0: 1.0;
        const double score =xy_error + z_error + angle_weight * yaw_error / CV_PI;

        if (score < min_score)
        {
            min_score = score;
            best_id   = id;
        }
    }

    const Eigen::Vector3d predicted_xyz = xyza_list[best_id].head<3>();
    const Eigen::Vector3d innovation    = armor.xyz_in_world - predicted_xyz;
    last_innovation_z_                  = innovation.z();
    last_innovation_norm_               = innovation.norm();
    last_update_reject_reason_          = UpdateRejectReason::NONE;

    if (!armor.xyz_in_world.allFinite() || !predicted_xyz.allFinite() || !innovation.allFinite())
    {
        last_update_reject_reason_ = UpdateRejectReason::NONFINITE;
        return false;
    }
    if (std::abs(last_innovation_z_) > max_z_innovation_)
    {
        last_update_reject_reason_ = UpdateRejectReason::Z_INNOVATION;
        return false;
    }
    if (last_innovation_norm_ > max_position_innovation_)
    {
        last_update_reject_reason_ = UpdateRejectReason::POSITION_INNOVATION;
        return false;
    }

    // 更新目标状态
    if (best_id != 0) jumped = true;
    is_switch = (best_id != last_id);
    if (is_switch) switch_count++;
    last_id = best_id;
    update_count++;

    // EKF 更新
    update_ypda(armor, best_id);

    //约束状态向量
    constrainState();
    return true;
}

/**
 * @brief 更新EKF（yaw, pitch, distance, angle）
 */
void Target::update_ypda(const Armor &armor, int id)
{
    // 计算观测雅可比矩阵
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    // 自适应观测噪声：根据装甲板角度和距离调整
    auto            center_yaw  = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
    auto            delta_angle = rm_utils::limit_rad(armor.ypr_in_world[0] - center_yaw);
    Eigen::VectorXd R_dig{{4e-3, 4e-3, std::log(std::abs(delta_angle) + 1) + 1, std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

    Eigen::MatrixXd R = R_dig.asDiagonal();

    // 观测函数：从状态向量计算观测值
    auto h = [&](const Eigen::VectorXd &x) -> Eigen::Vector4d {
        Eigen::VectorXd xyz   = h_armor_xyz(x, id);
        Eigen::VectorXd ypd   = rm_utils::xyz2ypd(xyz);
        auto            angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
        return {ypd[0], ypd[1], ypd[2], angle};
    };

    // 观测值减法（处理角度环绕）
    auto z_subtract = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a - b;
        c[0]              = rm_utils::limit_rad(c[0]);
        c[1]              = rm_utils::limit_rad(c[1]);
        c[3]              = rm_utils::limit_rad(c[3]);
        return c;
    };

    // 实际观测值 z = [yaw, pitch, distance, armor_yaw]
    const Eigen::Vector3d &ypd = armor.ypd_in_world;
    const Eigen::Vector3d &ypr = armor.ypr_in_world;
    Eigen::VectorXd        z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

    // 执行EKF更新
    ekf_.update(z, H, R, h, z_subtract);
}

/**
 * @brief 观测函数：计算装甲板在世界坐标系下的位置
 */
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd &x, int id) const
{
    // 前哨站特殊处理：三层高度，每层一个装甲板
    if (name == "outpost")
    {
        auto angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / 3);
        auto r     = x[8];
        auto ax    = x[0] - r * std::cos(angle);
        auto ay    = x[2] - r * std::sin(angle);
        auto az    = x[4] + (id - 1) * 0.102;
        return {ax, ay, az};
    }

    // 普通目标：4个装甲板，id=0,2同高度同半径r，id=1,3半径r+l、高度差h
    auto   angle   = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
    auto   use_lh  = (armor_num == 4) && (id == 1 || id == 3);
    auto   r       = use_lh ? x[8] + x[9] : x[8];
    auto   armor_x = x[0] - r * std::cos(angle);
    auto   armor_y = x[2] - r * std::sin(angle);
    double dz      = use_lh ? x[10] : 0;
    auto   armor_z = x[4] + dz;

    return {armor_x, armor_y, armor_z};
}

/**
 * @brief 观测函数雅可比矩阵
 */
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd &x, int id) const
{
    // 前哨站特殊处理
    if (name == "outpost")
    {
        auto angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / 3);
        auto r     = x[8];

        auto dx_da = r * std::sin(angle);
        auto dy_da = -r * std::cos(angle);
        auto dx_dr = -std::cos(angle);
        auto dy_dr = -std::sin(angle);

        Eigen::MatrixXd H_armor_xyza{{1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, 0, 0},
                                     {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, 0, 0},
                                     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}};

        Eigen::VectorXd armor_xyz   = h_armor_xyz(x, id);
        Eigen::MatrixXd H_armor_ypd = rm_utils::xyz2ypd_jacobian(armor_xyz);

        Eigen::MatrixXd H_armor_ypda{{H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
                                     {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
                                     {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
                                     {0, 0, 0, 1}};

        return H_armor_ypda * H_armor_xyza;
    }

    // 普通目标：4个装甲板，id=0,2同半径r，id=1,3半径r+l、高度差h
    auto angle  = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
    auto use_lh = (armor_num == 4) && (id == 1 || id == 3);
    auto r      = use_lh ? x[8] + x[9] : x[8];

    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);
    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    auto dx_dl = use_lh ? -std::cos(angle) : 0.0;
    auto dy_dl = use_lh ? -std::sin(angle) : 0.0;
    auto dz_dh = use_lh ? 1.0 : 0.0;

    Eigen::MatrixXd H_armor_xyza{{1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl, 0},
                                 {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl, 0},
                                 {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh},
                                 {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}};

    Eigen::VectorXd armor_xyz   = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = rm_utils::xyz2ypd_jacobian(armor_xyz);

    Eigen::MatrixXd H_armor_ypda{{H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
                                 {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
                                 {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
                                 {0, 0, 0, 1}};

    return H_armor_ypda * H_armor_xyza;
}

/**
 * @brief 获取所有装甲板的位置和角度列表
 */
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
    std::vector<Eigen::Vector4d> list;

    for (int i = 0; i < armor_num; i++)
    {
        auto            angle = rm_utils::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num);
        Eigen::Vector3d xyz   = h_armor_xyz(ekf_.x, i);
        list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return list;
}

/**
 * @brief 检查目标是否发散
 */
bool Target::diverged() const
{
    return !radiusModelValid(ekf_.x[8], ekf_.x[9]);
}

/**
 * @brief 检查目标是否收敛
 */
bool Target::converged() const
{
    // 普通目标：更新3次以上且未发散
    if (name != "outpost" && update_count > 3 && !diverged()) return true;

    // 前哨站：更新10次以上且未发散（转速较慢，需要更多数据）
    if (name == "outpost" && update_count > 10 && !diverged()) return true;

    return false;
}

void Target::constrainState()
{
    //速度约束：线速度 v <= ω * r
    double omega = ekf_.x[7];
    double r = ekf_.x[8];
    double max_linear_vel = r * std::abs(omega);

    // 设置最小兜底速度，防止 omega 为 0 时完全锁死平移
    max_linear_vel = std::max(max_linear_vel, 0.5); 

    double vx = ekf_.x[1];
    double vy = ekf_.x[3];
    double current_vel = std::hypot(vx, vy);

    if (current_vel > max_linear_vel && current_vel > 1e-6)
    {
        double scale = max_linear_vel / current_vel;
        ekf_.x[1] *= scale; // 修正 vx
        ekf_.x[3] *= scale; // 修正 vy
    }

    // 几何约束：限制旋转半径 r 的物理范围，机器人中心到装甲板的半径大概在 0.1m 到 0.4m 之间
    ekf_.x[8] = std::clamp(ekf_.x[8], 0.08, 0.45);

    // 约束加长半径 (r + l)，防止大装甲板飘太远
    if (armor_num == 4)
    {
        double r_plus_l = ekf_.x[8] + ekf_.x[9];
        r_plus_l = std::clamp(r_plus_l, 0.08, 0.5);
        ekf_.x[9] = r_plus_l - ekf_.x[8]; // 回写 l
    }

    //高度约束：约束高度差 h (x[10]) 的绝对值
    if (armor_num == 4 && name != "outpost")
    {
        const double MAX_HEIGHT_DIFF = 0.1; // 物理最大高差，m

        // 如果 EKF 算出的高差超过了物理极限，强制拉回到边界
        if (std::abs(ekf_.x[10]) > MAX_HEIGHT_DIFF)
        {
            ekf_.x[10] = (ekf_.x[10] > 0) ? MAX_HEIGHT_DIFF : -MAX_HEIGHT_DIFF;
        }
    }
}

} // namespace auto_aim
