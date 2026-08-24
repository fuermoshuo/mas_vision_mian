#include "rune_sine_fitter.hpp"

#include <Eigen/Dense>
#include <deque>
#include <utility>
#include <cmath>
#include <limits>

namespace rm_utils
{   

rune_sine_fitter::rune_sine_fitter(int max_iterations, double threshold, double ridge_penalty, int num_segments)
    : max_iterations(max_iterations), threshold_(threshold), ridge_penalty_(ridge_penalty), num_segments_(num_segments)
{
}

void rune_sine_fitter::add_data(double t, double v)
{
    if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) 
    {
        fit_data_.clear();
    }
    while (!fit_data_.empty() && (t - fit_data_.front().first > 5))
    {
        fit_data_.pop_front();
    }
    while (!fit_data_.empty() && (fit_data_.size() >= 300))
    {
        fit_data_.pop_front();
    }
    fit_data_.emplace_back(std::make_pair(t, v));
}

void rune_sine_fitter::fit()
{
    Result temp_result_    = Result(); 
    double best_huber_loss = std::numeric_limits<double>::max();

    if (fit_data_.size() < 3) 
    {
        best_result_.is_valid = false;
        return;
    }
    double t0         = fit_data_.front().first;
    double omega_step = (max_omega - min_omega) / num_segments_;

    Eigen::MatrixXd X(fit_data_.size(),3);
    Eigen::VectorXd Y(fit_data_.size());

    for(int k = 0; k <= num_segments_; k++)
    {
        double omega = k * omega_step + min_omega;

        for(size_t i = 0; i < fit_data_.size(); ++i)
        {
            double t   = fit_data_[i].first;
            double tau = t - t0;
            double v   = fit_data_[i].second;

            X(i,0) = std::sin(omega * tau);
            X(i,1) = std::cos(omega * tau);
            X(i,2) = 1.0;
            Y(i)   = v;
        }

        // 进行Huber IRLS鲁棒最小二乘拟合
        Eigen::VectorXd theta(3);
        double cost;

        if(!Huber_loss_calculate(X , Y, theta, cost))
        {
            continue;
        }

        // 收敛后计算标准Huber损失，用于择优
        double total_loss = 0.0;
        for(size_t i = 0; i < fit_data_.size(); ++i)
        {
            double r = Y(i) - X.row(i) * theta;
            double ar = std::fabs(r);
            if (ar <= threshold_)
            {
                total_loss += 0.5 * r * r;
            }
            else
            {
                total_loss += threshold_ * (ar - 0.5 * threshold_);
            }
        }

        if(total_loss < best_huber_loss)
        {
            best_huber_loss       = total_loss;
            temp_result_.A        = std::sqrt(theta(0) * theta(0) + theta(1) * theta(1));
            temp_result_.omega    = omega;
            temp_result_.phi      = std::atan2(theta(1), theta(0));
            temp_result_.C        = theta(2); 
            temp_result_.inliers  = fit_data_.size(); // 这里假设所有点都是内点
            temp_result_.is_valid = true;
        }
    }
        if(temp_result_.is_valid)
        {
            best_result_ = temp_result_;
        }

}

bool rune_sine_fitter::Huber_loss_calculate(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y, Eigen::VectorXd &theta, double &cost)
{
    Eigen::Vector3d reg(ridge_penalty_, ridge_penalty_, 0.0);       // 

    Eigen::VectorXd ones = Eigen::VectorXd::Ones(Y.size());
    Eigen::MatrixXd W0 = ones.asDiagonal();
    Eigen::MatrixXd initXTWX = X.transpose() * W0 * X + reg.asDiagonal().toDenseMatrix();
    theta = initXTWX.ldlt().solve(X.transpose() * W0 * Y);
    cost = std::numeric_limits<double>::max();

    for(int iter = 0; iter < max_iterations; ++iter)
    {
        Eigen::VectorXd residuals = Y - X * theta;
        Eigen::VectorXd weights(residuals.size());

        for(int i = 0; i < residuals.size(); ++i)
        {
            weights(i) = Huber_Weight(residuals(i), threshold_);
        }

        Eigen::MatrixXd W    = weights.asDiagonal();                    // 构造权重对角矩阵

        auto            XTWX = X.transpose() * W * X + reg.asDiagonal().toDenseMatrix();        
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
    if(std::fabs(residuals) <= threshold_)
    {
        return 1.0;
    }
    else
    {
        return threshold_ / std::fabs(residuals);
    }
}

}
