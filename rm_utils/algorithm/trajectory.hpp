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

#ifndef RM_UTILS__TRAJECTORY_HPP
#define RM_UTILS__TRAJECTORY_HPP

#include <cmath>

namespace rm_utils
{
struct Trajectory
{
    bool   unsolvable = true;
    double fly_time   = 0.0;
    double pitch      = 0.0; // 抬头为正

    /**
     * @brief 构造函数
     * @param v0 子弹初速度大小，单位：m/s
     * @param d 目标水平距离，单位：m
     * @param h 目标竖直高度，单位：m
     * @param gravity 重力加速度，单位：m/s²
     * @param air_resistance 二次空气阻力系数，单位：1/m
     */
    Trajectory(double v0, double d, double h, double gravity = 9.81, double air_resistance = 0.003);
};

} // namespace rm_utils

#endif // RM_UTILS__TRAJECTORY_HPP
