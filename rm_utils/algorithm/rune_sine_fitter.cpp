#include "rune_sine_fitter.hpp"

#include <Eigen/Dense>
#include <deque>
#include <utility>
#include <cmath>
#include <limits>

namespace rm_utils
{   

rune_sine_fitter::rune_sine_fitter(int max_iterations, double threshold, double ridge_penalty, double num_segments)
    : max_iterations(max_iterations), threshold_(threshold), ridge_penalty_(ridge_penalty), num_segments_(num_segments)
{
}

void rune_sine_fitter::add_data(double t, double v)
{
    if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();
    fit_data_.emplace_back(std::make_pair(t, v));
}

void rune_sine_fitter::fit()
{

    if (fit_data_.size() < 3) return;

    double omega_step = (max_omega - min_omega) / num_segments_;

    Eigen::MatrixXd X(fit_data_.size(),3);
    Eigen::VectorXd Y(fit_data_.size());

    for(double omega = min_omega; omega <= max_omega; omega += omega_step)
    {

        for(size_t i = 0; i < fit_data_.size(); ++i)
        {
            double t = fit_data_[i].first;
            double v = fit_data_[i].second;

            X(i,0) = std::sin(omega * t);
            X(i,1) = std::cos(omega * t);
            X(i,2) = 1.0;
            Y(i)   = v;

            // 进行Huber IRLS鲁棒最小二乘拟合
            Eigen::VectorXd theta(3);
            double cost;

            if(!Huber_loss_calculate(X , Y, theta, cost))
            {
                continue;
            }

            if(cost < best_cost)
            {
                best_cost            = cost;
                best_result_.A       = std::sqrt(theta(0) * theta(0) + theta(1) * theta(1));
                best_result_.omega   = omega;
                best_result_.phi     = std::atan2(theta(1), theta(0));
                best_result_.C       = theta(2); 
                best_result_.inliers = fit_data_.size(); // 这里假设所有点都是内点
            }
        }

    }

}

bool rune_sine_fitter::Huber_loss_calculate(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y, Eigen::VectorXd &theta, double &cost)
{
    theta = Eigen::VectorXd::Zero(X.cols());
    cost  = std::numeric_limits<double>::max();

    for(int iter = 0; iter < max_iterations; ++iter)
    {
        Eigen::VectorXd residuals = Y - X * theta;
        Eigen::VectorXd weights(residuals.size());

        for(int i = 0; i < residuals.size(); ++i)
        {
            weights(i) = Huber_Weight(residuals(i), threshold_);
        }

        // 构造权重对角矩阵
        Eigen::MatrixXd W    = weights.asDiagonal();
        Eigen::MatrixXd XTWX = X.transpose() * W * X + ridge_penalty_ * Eigen::MatrixXd::Identity(X.cols(), X.cols());
        Eigen::VectorXd XTWY = X.transpose() * W * Y;
        theta = XTWX.ldlt().solve(XTWY);

        if (!theta.allFinite())
        {
            return false;
        }

        Eigen::VectorXd r = Y - X * theta;
        double new_cost = (r.transpose() * W * r).value();

        if(std::abs(new_cost - cost) < 1e-6)
            break;

        cost = new_cost;
    }

    return true;
}

double rune_sine_fitter::Huber_Weight(double residuals, double threshold_)
{
    if(abs(residuals) < threshold_)
    {
        return 1.0;
    }
    else
    {
        return threshold_ / abs(residuals);
    }
}

}
