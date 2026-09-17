#include "extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace rm_utils
{
namespace
{
/**
 * @brief 卡方检验阈值（置信水平95%）按实际非零观测行计算，用于卡尔曼滤波器中的 NIS 一致性检验
 *              屏蔽副板 yaw 时不是 8 自由度。
 * @param observation_dimension 观测维度
 * @return 卡方检验阈值
 */
constexpr double nisThreshold(int observation_dimension)
{
    switch (observation_dimension)
    {
    case 1: return 3.841;
    case 2: return 5.991;
    case 3: return 7.815;
    case 4: return 9.488;
    case 5: return 11.070;
    case 6: return 12.592;
    case 7: return 14.067;
    default: return 15.507;
    }
}

/**
 * @brief 计算观测矩阵 H 的有效观测维度（非零行数），至少为 1。
 * @param H 观测矩阵
 * @return 有效观测维度(非零行代表实际有效的观测分量)
 */
int activeObservationDimension(const Eigen::MatrixXd &H)
{
    int active_rows = 0;
    for (Eigen::Index row = 0; row < H.rows(); ++row)
        if (H.row(row).squaredNorm() > 1e-18) active_rows++;
    return std::max(active_rows, 1);
}

} // namespace

// 装甲跟踪使用：x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, radius_02, radius_13, h]
ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
                                           std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
    : x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
    data["residual_yaw"]        = 0.0;
    data["residual_pitch"]      = 0.0;
    data["residual_distance"]   = 0.0;
    data["residual_angle"]      = 0.0;
    data["nis"]                 = 0.0;
    data["nees"]                = 0.0;
    data["nis_fail"]            = 0.0;
    data["nees_fail"]           = 0.0;
    data["nis_valid"]           = 1.0;
    data["nees_valid"]          = 0.0;
    data["update_applied"]      = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)
{
    return predict(F, Q, [&](const Eigen::VectorXd &x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
                                              std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                                             std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    return update(z, H, R, [&](const Eigen::VectorXd &x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                                             std::function<Eigen::VectorXd(const Eigen::VectorXd &)>                          h,
                                             std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    last_update_applied = false;

    data["update_applied"] = 0.0; 

    const Eigen::VectorXd x_prior = x;
    const Eigen::MatrixXd P_prior = P;
    // 残差
    const Eigen::VectorXd residual = z_subtract(z, h(x_prior));         

    // 创新协方差矩阵 S
    Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
    // 对 S 做 LDLT 分解，替代直接求逆
    S = 0.5 * (S + S.transpose());
    // 采用 LDLT 分解求解线性方程组，避免直接求逆，提高数值稳定性
    const Eigen::LDLT<Eigen::MatrixXd> innovation_ldlt(S);

    data["nis_fail"]   = 0.0;
    data["nees_fail"]  = 0.0;
    data["nis_valid"]  = 0.0;
    data["nees_valid"] = 0.0;

    // 记录最近 N 次 NIS 检验失败的比例
    const auto publish_residuals = [&] {
        data["residual_yaw"]      = residual.size() > 0 ? residual[0] : 0.0;
        data["residual_pitch"]    = residual.size() > 1 ? residual[1] : 0.0;
        data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
        data["residual_angle"]    = residual.size() > 3 ? residual[3] : 0.0;
    };
    // 记录最近 N 次 NIS 检验失败的比例
    const auto record_nis_result = [&](bool failed) {
        recent_nis_failures.push_back(failed ? 1 : 0);
        const std::size_t effective_window = std::max<std::size_t>(window_size, 1);
        while (recent_nis_failures.size() > effective_window) recent_nis_failures.pop_front();
        const int recent_failures =
            std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
        data["recent_nis_failures"] = recent_nis_failures.empty()
                                              ? 0.0
                                              : static_cast<double>(recent_failures) /
                                                    static_cast<double>(recent_nis_failures.size());
    };
    // 数值失败时保持先验不变，并让上层知道这次观测没有真正生效。
    const auto reject_numeric_update = [&]() -> Eigen::VectorXd {
        x = x_prior;
        P = P_prior;
        data["nis_fail"] = 1.0;
        data["nis"]      = 0.0;
        data["nees"]     = 0.0;
        last_nis         = 0.0;
        ++nis_count_;
        ++total_count_;
        publish_residuals();
        record_nis_result(true);
        return x;
    };

    if (innovation_ldlt.info() != Eigen::Success || !innovation_ldlt.vectorD().allFinite() ||
        innovation_ldlt.vectorD().minCoeff() <= 1e-12 || !residual.allFinite())
        return reject_numeric_update();

    // 卡尔曼增益 K
    const Eigen::MatrixXd K = innovation_ldlt.solve(H * P_prior).transpose();
    if (!K.allFinite())
        return reject_numeric_update();

    // 计算 NIS（Normalized Innovation Squared）用于一致性检验
    const Eigen::VectorXd innovation_solution = innovation_ldlt.solve(residual);
    const double nis = residual.dot(innovation_solution);
    if (!innovation_solution.allFinite() || !std::isfinite(nis) || nis < 0.0)
        return reject_numeric_update();

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());

    x = x_add(x_prior, K * residual);

    // 数值失败时保持先验不变，并让上层知道这次观测没有真正生效。
    if (!x.allFinite() || !P.allFinite() || P.diagonal().minCoeff() < -1e-12)
        return reject_numeric_update();
    const Eigen::LDLT<Eigen::MatrixXd> posterior_ldlt(P);
    if (posterior_ldlt.info() != Eigen::Success || !posterior_ldlt.vectorD().allFinite() ||
        posterior_ldlt.vectorD().minCoeff() < -1e-9)
        return reject_numeric_update();
    last_update_applied = true;
    data["update_applied"] = 1.0;

    /// 卡方检验
    double nees = 0.0;
    bool nees_valid = false;
    if (posterior_ldlt.info() == Eigen::Success && posterior_ldlt.vectorD().allFinite() &&
        posterior_ldlt.vectorD().minCoeff() > 1e-12)
    {
        const Eigen::VectorXd state_delta = x - x_prior;
        const Eigen::VectorXd state_solution = posterior_ldlt.solve(state_delta);
        nees       = state_delta.dot(state_solution);
        nees_valid = std::isfinite(nees) && nees >= 0.0;
    }

    // 卡方检验阈值（置信水平95%）按实际非零观测行计算；屏蔽副板 yaw 时不是 8 自由度。
    const double     nis_threshold  = nisThreshold(activeObservationDimension(H));
    constexpr double nees_threshold = 19.68;

    const bool nis_valid   = std::isfinite(nis) && nis >= 0.0;
    const bool nis_failed  = !nis_valid || nis > nis_threshold;
    const bool nees_failed = nees_valid && nees > nees_threshold;

    if (nis_failed) nis_count_++, data["nis_fail"] = 1;
    if (nees_failed) nees_count_++, data["nees_fail"] = 1;

    total_count_++;
    last_nis = nis_valid ? nis : 0.0;
    
    data["nis_valid"]  = nis_valid ? 1.0 : 0.0;
    data["nees_valid"] = nees_valid ? 1.0 : 0.0;
    data["nis"]        = nis_valid ? nis : 0.0;
    data["nees"]       = nees_valid ? nees : 0.0;
    publish_residuals();
    record_nis_result(nis_failed);

    return x;
}

} // namespace rm_utils
