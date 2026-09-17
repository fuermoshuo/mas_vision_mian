#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace rm_utils
{
class ExtendedKalmanFilter
{
  public:
    Eigen::VectorXd x;      // 状态向量
    Eigen::MatrixXd P;      // 协方差矩阵

    ExtendedKalmanFilter() = default;

    /**
     * @brief 构造函数
     * @param x0 初始状态向量
     * @param P0 初始协方差矩阵
     * @param x_add 状态向量相加函数，默认为普通的向量加法
     */
    ExtendedKalmanFilter(
        const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add = [](const Eigen::VectorXd &a,
                                                                                                    const Eigen::VectorXd &b) { return a + b; });
    /**
     * @brief 预测步骤，使用线性状态转移函数
     * @param F 状态转移矩阵
     * @param Q 过程噪声协方差矩阵
     * @return 预测后的状态向量
     */
    Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

    /**
     * @brief 预测步骤，使用非线性状态转移函数
     * @param F 状态转移矩阵
     * @param Q 过程噪声协方差矩阵
     * @param f 非线性状态转移函数
     * @return 预测后的状态向量
     */
    Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);
    
    /**
     * @brief 更新步骤，使用【线性观测函数】 h(x)=H·x
     * @param z 观测向量
     * @param H 观测矩阵（线性观测的观测矩阵，也是EKF的观测雅可比）
     * @param R 观测噪声协方差矩阵
     * @param z_subtract 观测向量相减函数；处理角度环绕(如yaw ±π)，默认普通向量减法 a-b
     * @return 更新后的状态向量
    */
    Eigen::VectorXd update(
        const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract = [](const Eigen::VectorXd &a,
                                                                                                         const Eigen::VectorXd &b) { return a - b; });
    /**
     * @brief 更新步骤，使用【非线性观测函数】EKF标准更新
     * @param z 观测向量
     * @param H 观测雅可比矩阵 H = ∂h(x)/∂x 在x_prior处求值
     * @param R 观测噪声协方差矩阵
     * @param h 非线性观测映射 h(x)：状态x → 预测观测向量
     * @param z_subtract 观测向量相减函数；处理角度环绕（yaw角度差不能直接a‑b），默认普通向量减法 a‑b
     * @return 更新后的状态向量
     */
    Eigen::VectorXd update(
        const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract = [](const Eigen::VectorXd &a,
                                                                                                         const Eigen::VectorXd &b) { return a - b; });

    std::map<std::string, double> data; // 卡方检验数据
    std::deque<int>               recent_nis_failures{};
    std::size_t                   window_size = 100;
    double                        last_nis = 0.0;
    bool                          last_update_applied = false;    // 上一次更新是否被应用

  private:
    Eigen::MatrixXd                                                                  I;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

    int nees_count_  = 0;         // NEES 计数器
    int nis_count_   = 0;         // NIS 计数器
    int total_count_ = 0;         // 总计数器 
};

} // namespace rm_utils

#endif // TOOLS__EXTENDED_KALMAN_FILTER_HPP
