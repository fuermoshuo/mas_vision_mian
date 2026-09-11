#include "trajectory.hpp"

#include <cmath>
#include <optional>
#include <utility>

// 具体算法解释在 ..\..\doc\learn_doc\rm_utils\algorithm\trajectory.md

namespace rm_utils
{
namespace
{
constexpr   int    MAX_ITERATIONS = 10;                 // 最大迭代次数
constexpr   double MAX_PITCH      = 40.0 / 57.3;        // 最大抬头角度，单位：rad
constexpr   double STEP_TIME      = 0.005;              // 积分步长，单位：s
constexpr   double HEIGHT_ERROR   = 0.001;              // 允许的高度误差，单位：m
constexpr   double MAX_FLY_TIME   = 4.0;                // 最大飞行时间，单位：s
constexpr   double MIN_VELOCITY_X = 0.1;                // 最小水平速度，单位：m/s

/**
 * @brief 估算飞行时间和高度
 * @param v0 子弹初速度大小，单位：m/s
 * @param pitch 发射角度，单位：rad
 * @param distance 目标水平距离，单位：m
 * @param gravity 重力加速度，单位：m/s²
 * @param air_resistance 二次空气阻力系数，单位：1/m
 * @return 如果无法估算，则返回 std::nullopt；否则返回 std::pair<double, double>，
 *              其中 first 为飞行时间，second 为飞行高度
 */
std::optional<std::pair<double, double>> estimate(double v0, double pitch, double distance,
                                                   double gravity, double air_resistance)
{
    double x = 0.0;
    double y = 0.0;
    double t = 0.0;
    double vx = v0 * std::cos(pitch);
    double vy = v0 * std::sin(pitch);
    double previous_x = 0.0;
    double previous_y = 0.0;
    double previous_t = 0.0;

    while (x < distance && t <= MAX_FLY_TIME && vx > MIN_VELOCITY_X)
    {
        previous_x = x;
        previous_y = y;
        previous_t = t;

        const double speed = std::hypot(vx, vy);

        vx -= air_resistance * speed * vx * STEP_TIME;
        vy -= (gravity + air_resistance * speed * vy) * STEP_TIME;
        x  += vx * STEP_TIME;
        y  += vy * STEP_TIME;
        t  += STEP_TIME;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(t)) return std::nullopt;
    }

    if (x < distance || x <= previous_x) return std::nullopt;
    const double ratio = (distance - previous_x) / (x - previous_x);
    return std::pair<double, double>{previous_y + ratio * (y - previous_y),
                                     previous_t + ratio * (t - previous_t)};
}
} // namespace

Trajectory::Trajectory(double v0, double d, double h, double gravity, double air_resistance)
{
    if (!(v0 > 0.0) || !(d > 0.0) || !(gravity > 0.0) || air_resistance < 0.0 ||
        !std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) ||
        !std::isfinite(gravity) || !std::isfinite(air_resistance))
        return;

    double launch_pitch = std::atan2(h, d);
    for (int i = 0; i < MAX_ITERATIONS; ++i)
    {
        const auto estimated = estimate(v0, launch_pitch, d, gravity, air_resistance);
        if (!estimated) return;

        const auto [actual_h, time] = *estimated;
        const double error = h - actual_h;
        if (std::abs(error) < HEIGHT_ERROR)
        {
            unsolvable = false;
            fly_time   = time;
            pitch      = launch_pitch;
            return;
        }

        launch_pitch += std::atan2(error, d);
        if (std::abs(launch_pitch) > MAX_PITCH) return;
    }
}

} // namespace rm_utils
