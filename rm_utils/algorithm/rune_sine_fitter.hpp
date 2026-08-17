/******************************************************************************
 *                                                                            
 *        	      /\     代码安全            |\_/|                                  
 *               /  \    代码无BUG           |^_^|                                  
 *       	    /||||\    永不宕机          /     \                                 
 *      	   _||||||_   法力无边        _/       \_                               
 *                                                                            
 *                       _oo0oo_                                             
 *                      o8888888o                                            
 *                      88" . "88                                            
 *                      (| -_- |)                                            
 *                      0\  =  /0                                            
 *                    ___/`---'\___                                          
 *                  .' \\|     |// '.                                        
 *                 / \\|||  :  |||// \                                       
 *                / _||||| -:- |||||- \                                      
 *               |   | \\\  -  /// |   |                                     
 *               | \_|  ''\---/''  |_/ |                                     
 *               \  .-\__  '-'  ___/-. /                                     
 *             ___'. .'  /--.--\  `. .'___                                   
 *          ."" '<  `.___\_<|>_/___.' >' "".                                 
 *         | | :  `- \`.;`\ _ /`;.`/ - ` : | |                               
 *         \  \ `_.   \_ __\ /__ _/   .-` /  /                               
 *     =====`-.____`.___ \_____/___.-`___.-'=====                            
 *                       `=---='                                             
 *                                                                            
 *     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                           
 *         如来保佑     永无BUG        法力无边                           
 *                                                                            
 *****************************************************************************/

/* To do
        1. 让每个固定频率的 Huber-IRLS 真正收敛
        2. 判断数据是否值得信任
*/

#pragma once

#include <Eigen/Dense>
#include <deque>
#include <utility>
#include <cmath>
#include <limits>

namespace rm_utils
{

/**
 * @brief 能量机关正弦轨迹拟合器
 * 主场景：大能量机关(Large Power Rune/大符)周期性运动参数估计,预测大符未来位置
 * 算法：一维ω网格遍历 + Huber IRLS鲁棒最小二乘
 * 背景：ransac_sine_fitter.cpp不适用于自瞄装甲板识别，而大符因其相对有迹可循的转速，相比装甲板更加有迹可循，装甲板建议直接优化卡尔曼滤波
 * 模型：v(t) = A*sin(ωt+φ) + C
 */
class rune_sine_fitter
{

public:
    struct Result
    {
        double A       = 0.0;
        double omega   = 0.0;
        double phi     = 0.0;
        double C       = 0.0;
        int    inliers = 0;
    };
    Result best_result_;

    rune_sine_fitter(int max_iterations, double threshold, double ridge_penalty, double num_segments);

    void add_data(double t, double v);

    void fit();

    /**
     * @brief 固定ω，执行Huber IRLS鲁棒最小二乘
     * @param X 设计矩阵 N×3 [sinωt, cosωt, 1]
     * @param Y 观测向量 N×1
     * @param theta [a,b,C]输出参数
     * @param cost 本轮加权残差代价输出
     * @return 求解成功true
     */
    bool Huber_loss_calculate(const Eigen::MatrixXd &X, const Eigen::VectorXd &Y, Eigen::VectorXd &theta, double &cost);

    double Huber_Weight(double residuals, double threshold_);

    const Result& get_best_result() const { return best_result_; }

private:
    int    max_iterations = 3;                                      // Huber IRLS 最大迭代次数
    double num_segments_  = 50;                                     // ω 网格分段数量
    double best_cost      = std::numeric_limits<double>::max();
    double threshold_;                                              // Huber 阈值 
    double ridge_penalty_;                                          // 岭正则系数 

    // 根据大符的运动特性，ω的范围是[1.884, 2.0]，因此在拟合时可以限制ω的搜索范围
    static constexpr double min_omega = 1.884;
    static constexpr double max_omega = 2.0;

    std::deque<std::pair<double, double>> fit_data_;

}; 

}

    
