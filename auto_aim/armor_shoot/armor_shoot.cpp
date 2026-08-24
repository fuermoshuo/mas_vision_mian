#include "armor_shoot.hpp"

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <vector>

#include "display.hpp"
#include "serial_types.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "trajectory.hpp"
#include "plotter.hpp"

#include <nlohmann/json.hpp>

namespace auto_aim
{

ArmorShoot::ArmorShoot(const std::string &config_path) : lock_id_(-1), plotter_(nullptr)
{
    try
    {
        YAML::Node config     = YAML::LoadFile(config_path);
        auto       node       = config["auto_aim"]["armor_shoot"];
        yaw_offset_           = node["yaw_offset"].as<double>(0.0) / 57.3;
        pitch_offset_         = node["pitch_offset"].as<double>(0.0) / 57.3;
        comming_angle_        = node["comming_angle"].as<double>(60.0) / 57.3;
        leaving_angle_        = node["leaving_angle"].as<double>(30.0) / 57.3;
        fire_delay_time_      = node["fire_delay_time"].as<double>(0.02);
        yaw_tolerance_near_   = node["yaw_tolerance_near"].as<double>(1.0) / 57.3;
        yaw_tolerance_far_    = node["yaw_tolerance_far"].as<double>(2.0) / 57.3;
        pitch_tolerance_near_ = node["pitch_tolerance_near"].as<double>(1.0) / 57.3;
        pitch_tolerance_far_  = node["pitch_tolerance_far"].as<double>(2.0) / 57.3;
        bullet_speed_         = node["bullet_speed"].as<double>(25.0);
        debug_                = node["debug"].as<bool>(false);

        hero_mode_               = node["hero_mode"].as<bool>(false);
        spinning_threshold_low_  = node["spinning_threshold_low"].as<double>(2.0);
        fire_phase_angle_        = node["fire_phase_angle"].as<double>(20.0) / 57.3;
        recovery_done_error_     = node["recovery_done_error"].as<double>(0.5) / 57.3;
        max_yaw_speed_           = node["max_yaw_speed"].as<double>(12.0);
        max_pitch_speed_         = node["max_pitch_speed"].as<double>(8.0);
        max_yaw_acc_             = node["max_yaw_acc"].as<double>(50.0);
        max_pitch_acc_           = node["max_pitch_acc"].as<double>(100.0);
        switch_fire_hold_frames_ = node["switch_fire_hold_frames"].as<int>(3);

        // plotter 配置
        plotter_enable_ = node["plotter_enable"].as<bool>(false);
        if (plotter_enable_)
        {
            plotter_ = std::make_unique<rm_utils::Plotter>();
        }

        MAS_LOG_INFO("armor_shoot yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("armor_shoot: Failed to load config, using defaults: {}", e.what());
    }
}

ArmorShoot::~ArmorShoot() {}

double ArmorShoot::continuousYaw(const Eigen::Vector3d &xyz, double gimbal_yaw)
{
    const double yaw_raw = std::atan2(xyz.y(), xyz.x()) + yaw_offset_;

    // 首次调用或 yaw 跳变过大时，会出现云台割裂性跳变，重置 gimbal_yaw 的同时云台的割裂性旋转(待修)
    if (!has_yaw_ || std::abs(last_target_yaw_ - gimbal_yaw) > 3.0)
    {
        last_target_yaw_ = gimbal_yaw;
    }

    // 差值归一化到 [-π, π] 后累加，得到连续的目标角度（可超出 ±π）
    last_target_yaw_ += rm_utils::limit_rad(yaw_raw - last_target_yaw_);
    has_yaw_ = true;
    return last_target_yaw_;
}

void ArmorShoot::stepAxis(double target, double target_speed, double dt, double max_speed, double max_acc,
                          double &position, double &velocity)
{
    const double error = target - position;

    constexpr double RESPONSE_RATE = 30.0;      // 响应增益系数，越大追赶越快，越大越容易抖

    // 0.5 * max_acc * dt 是离散欧拉积分的一阶补偿项，提前刹车，
    // 抵消离散迭代带来的超调倾向(根因前面求的 v 是预测，而实际的v每帧都会变化)
    const double brake_speed   = std::max(0.0, std::sqrt(2.0 * max_acc * std::abs(error)) - 0.5 * max_acc * dt);
    const double correction    = std::copysign(std::min(brake_speed, RESPONSE_RATE * std::abs(error)), error);
    const double desired_speed = std::clamp(target_speed + correction, -max_speed, max_speed);
    const double speed_change  = std::clamp(desired_speed - velocity, -max_acc * dt, max_acc * dt);
    velocity                  += speed_change;
    position                  += velocity * dt;
}

std::pair<double, double> ArmorShoot::smoothCommand(double yaw, double pitch, double gimbal_yaw, double gimbal_pitch,
                                                     std::chrono::steady_clock::time_point now)
{
    // ==========分支1：不在重捕获模式 recovering_ = false：直通模式==========
    if (!recovering_)
    {
        command_yaw_       = yaw;
        command_pitch_     = pitch;
        yaw_speed_         = 0.0;
        pitch_speed_       = 0.0;
        has_command_       = true;
        ref_yaw_           = yaw;
        ref_pitch_         = pitch;
        has_ref_           = true;
        last_command_time_ = now;
        return {yaw, pitch};
    }

    // ==========分支2：进入重捕获模式 recovering_ = true ==========
    // 如果平滑器还没有初始化（第一次进入重捕获）
    if (!has_command_)
    {
        command_yaw_       = gimbal_yaw;
        command_pitch_     = gimbal_pitch;
        yaw_speed_         = 0.0;
        pitch_speed_       = 0.0;
        has_command_       = true;
        last_command_time_ = now;
    }

    const double elapsed = std::chrono::duration<double>(now - last_command_time_).count();
    const double dt      = std::clamp(elapsed, 0.001, 0.02);
    last_command_time_   = now;

    // ----------------计算 target_speed（前馈速度）----------------
    const double yaw_target_speed = has_ref_ ? std::clamp((yaw - ref_yaw_) / dt, -max_yaw_speed_, max_yaw_speed_) : 0.0;
    const double pitch_target_speed = has_ref_ ? std::clamp((pitch - ref_pitch_) / dt, -max_pitch_speed_, max_pitch_speed_) : 0.0;
    ref_yaw_   = yaw;
    ref_pitch_ = pitch;
    has_ref_   = true;

    // ----------------判断是否结束重捕获，退出平滑模式----------------
    // 进入可射击角度范围前结束恢复状态；不再要求持续运动目标追到 0.1° 且速度完全一致。
    if (std::abs(command_yaw_ - yaw) < recovery_done_error_ &&
        std::abs(command_pitch_ - pitch) < recovery_done_error_)
    {
        command_yaw_     = yaw;
        command_pitch_   = pitch;
        yaw_speed_       = 0.0;
        pitch_speed_     = 0.0;
        recovering_      = false;
        return {yaw, pitch};
    }

    stepAxis(yaw, yaw_target_speed, dt, max_yaw_speed_, max_yaw_acc_, command_yaw_, yaw_speed_);
    stepAxis(pitch, pitch_target_speed, dt, max_pitch_speed_, max_pitch_acc_, command_pitch_, pitch_speed_);

    return {command_yaw_, command_pitch_};
}

SendPacket ArmorShoot::holdPacket(double gimbal_yaw, double gimbal_pitch, std::chrono::steady_clock::time_point now)
{
    recovering_       = true;
    yaw_speed_         = 0.0;
    pitch_speed_       = 0.0;
    has_ref_           = false;
    last_command_time_ = now;

    SendPacket p{};    // header / tail 走默认成员初始化器，其余清零
    p.found       = 1; // 保持视觉控制，避免云台因无目标包回零
    p.fire_advice = 0; // 但这一帧不给开火建议

    // 开机后还没有过有效值时，交还当前云台姿态 —— 等价于"原地不动"，而不是回零位
    p.target_yaw   = static_cast<float>(has_command_ ? command_yaw_ : gimbal_yaw);
    p.target_pitch = static_cast<float>(has_command_ ? command_pitch_ : gimbal_pitch);
    return p;
}

SendPacket ArmorShoot::shoot(const std::optional<Target> &target, std::chrono::steady_clock::time_point timestamp,
                              const Eigen::Matrix3d &R_gimbal2world, const cv::Mat &bgr_img, const std::string &window_name,
                              int64_t quaternion_age_ms, bool tracking_confirmed)
{
    const auto command_time = std::chrono::steady_clock::now();

    // 从旋转矩阵提取云台欧拉角（yaw, pitch）
    Eigen::Vector3d gimbal_euler = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    double          gimbal_yaw   = gimbal_euler[0];
    double          gimbal_pitch = gimbal_euler[1];

    if (!target.has_value())
    {
        recovering_            = true;
        has_command_           = false;
        yaw_speed_             = 0.0;
        pitch_speed_           = 0.0;
        has_ref_               = false;
        last_aim_armor_id_     = -1;
        switch_hold_remaining_ = 0;
        last_command_time_     = command_time;

        if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch, 0.0f, 0.0f, 0, bgr_img, window_name);
        const SendPacket packet{};
        
        if (plotter_enable_)
        {
            plotDiagnostics("no_target", target, {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK, 0.0}, packet,
                            gimbal_euler, quaternion_age_ms, 0.0, false);
        }
        return packet;
    }

    // temp_lost 时保留内部预测用于重新关联，但不再用旧目标驱动云台或给出开火建议。
    if (!tracking_confirmed)
    {
        const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
        if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch,
                              hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
        if (plotter_enable_)
        {
            plotDiagnostics("temp_lost_hold", target, {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK, 0.0}, hold,
                            gimbal_euler, quaternion_age_ms, 0.0, false);
        }
        return hold;
    }

    auto ekf_x             = target->ekf_x();
    int  frame_lock_id     = lock_id_;
    bool frame_coming_mode = coming_mode_;

    // 总延迟 = 处理延迟 + 发弹延迟
    double process_delay = rm_utils::delta_time(std::chrono::steady_clock::now(), timestamp);
    double dt            = process_delay + fire_delay_time_;

    auto future = timestamp + std::chrono::microseconds(static_cast<int>(dt * 1e6));

    Target predicted_target = *target;
    predicted_target.predict(future);

    // 选择初始瞄准点
    auto aim_point  = chooseAimPoint(predicted_target, frame_lock_id, frame_coming_mode);
    debug_aim_point = aim_point;

    if (!aim_point.valid)
    {
        // 连瞄准点都选不出来，yaw 无从更新，全部沿用上一次
        const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
        if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch,
                                            hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
        if (plotter_enable_)
        {
            plotDiagnostics("aim_point_invalid", target, aim_point, hold, gimbal_euler, quaternion_age_ms, process_delay * 1000.0, false);
        }   
        return hold;
    }

    // 计算初始弹道
    Eigen::Vector3d      xyz0 = aim_point.xyza.head(3);
    double               d0   = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    rm_utils::Trajectory trajectory0(bullet_speed_, d0, xyz0[2]);

    if (trajectory0.unsolvable)
    {
        // 瞄准点有效，yaw 可以继续跟；只有 pitch 因弹道无解而冻结
        continuousYaw(xyz0, gimbal_yaw);
        debug_aim_point.valid = false;
        const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
        if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch,
                         hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
        if (plotter_enable_)
        {
            plotDiagnostics("initial_trajectory_unsolvable", target, aim_point, hold, gimbal_euler, quaternion_age_ms,
                            process_delay * 1000.0, false);
        }
        return hold;
    }

    // 迭代求解飞行时间 (最多 10 次，收敛条件：相邻两次 fly_time 差 < 0.001)
    bool                converged     = false;
    double              prev_fly_time = trajectory0.fly_time;
    std::vector<Target> iteration_target(10, *target); // 创建 10 个目标副本用于迭代预测

    for (int iter = 0; iter < 10; ++iter)
    {
        // 预测目标在 future + prev_fly_time 时刻的位置
        auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
        iteration_target[iter].predict(predict_time);

        // 计算瞄准点
        auto aim_point_iter = chooseAimPoint(iteration_target[iter], frame_lock_id, frame_coming_mode);
        debug_aim_point     = aim_point_iter;

        if (!aim_point_iter.valid)
        {
            // 迭代中选不出瞄准点，同上：yaw 无从更新，全部沿用
            const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
            if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch, 
                                    hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
            if (plotter_enable_)
            {
                plotDiagnostics("iteration_aim_invalid", target, aim_point_iter, hold, gimbal_euler, quaternion_age_ms,
                                process_delay * 1000.0, false);
            }
            return hold;
        }

        // 计算新弹道
        Eigen::Vector3d      xyz = aim_point_iter.xyza.head(3);
        double               d   = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
        rm_utils::Trajectory current_traj(bullet_speed_, d, xyz.z());

        // 检查弹道是否可解
        if (current_traj.unsolvable)
        {
            MAS_LOG_DEBUG("Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1, bullet_speed_, d, xyz.z());
            // 瞄准点有效，yaw 继续跟；pitch 冻结
            continuousYaw(xyz, gimbal_yaw);
            debug_aim_point.valid = false;
            const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
            if (debug_) showDebug(target, {false, {}}, gimbal_yaw, gimbal_pitch,
                                     hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
            if (plotter_enable_)
            {
                plotDiagnostics("iteration_trajectory_unsolvable", target, aim_point_iter, hold, gimbal_euler, quaternion_age_ms,
                                process_delay * 1000.0, false);
            }
            return hold;
        }

        // 检查收敛条件
        if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001)
        {
            converged = true;
            break;
        }
        prev_fly_time = current_traj.fly_time;
    }

    // 计算最终角度。yaw 只依赖几何，先算出来 —— 即使下面弹道无解它也是有效的
    Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
    const double    yaw       = continuousYaw(final_xyz, gimbal_yaw);

    const rm_utils::Trajectory final_trajectory(bullet_speed_, final_xyz.head<2>().norm(), final_xyz.z());
    if (final_trajectory.unsolvable)
    {
        // yaw 已经跟上了，pitch 冻结在上一次有效值
        const SendPacket hold = holdPacket(gimbal_yaw, gimbal_pitch, command_time);
        if (debug_) showDebug(target, debug_aim_point, gimbal_yaw, gimbal_pitch,
                                 hold.target_yaw, hold.target_pitch, 0, bgr_img, window_name);
        if (plotter_enable_)
        {
            plotDiagnostics("final_trajectory_unsolvable", target, debug_aim_point, hold, gimbal_euler, quaternion_age_ms,
                            process_delay * 1000.0, converged);
        }
        return hold;
    }
    const double pitch = -(final_trajectory.pitch + pitch_offset_);
    last_raw_pitch_    = pitch;
    lock_id_           = frame_lock_id;
    coming_mode_       = frame_coming_mode;

    const bool was_recovering = recovering_;
    const auto command        = smoothCommand(yaw, pitch, gimbal_yaw, gimbal_pitch, command_time);

    if (debug_aim_point.armor_id >= 0)
    {
        if (last_aim_armor_id_ >= 0 && debug_aim_point.armor_id != last_aim_armor_id_)
        {
            switch_hold_remaining_ = std::max(0, switch_fire_hold_frames_);
        }
        last_aim_armor_id_ = debug_aim_point.armor_id;
    }
    const bool switch_ready = switch_hold_remaining_ == 0;

    float dist = static_cast<float>(final_xyz.head<2>().norm());

    // 射击决策，只有解算成功且在阈值内才允许开火
    double yaw_tolerance   = dist < 3.0 ? yaw_tolerance_near_ : yaw_tolerance_far_;
    double pitch_tolerance = dist < 3.0 ? pitch_tolerance_near_ : pitch_tolerance_far_;

    const bool yaw_ok   = std::abs(rm_utils::limit_rad(gimbal_yaw - yaw)) < yaw_tolerance;
    const bool pitch_ok = std::abs(gimbal_pitch - pitch) < pitch_tolerance;
    uint8_t    fire     = 0;
    if (!was_recovering && converged && switch_ready && debug_aim_point.valid && debug_aim_point.fire_allowed)
    {
        if (yaw_ok && pitch_ok) fire = 1;
    }

    const char *shoot_status = fire                         ? "fire_ready"
                               : was_recovering             ? "recovery_smoothing"
                               : !converged                 ? "iteration_not_converged"
                               : !switch_ready              ? "armor_switch_hold"
                               : !debug_aim_point.fire_allowed ? "phase_blocked"
                               : (!yaw_ok || !pitch_ok)      ? "aim_error"
                                                            : "fire_blocked";

    if (debug_)
    {
        showDebug(target, debug_aim_point, gimbal_yaw, gimbal_pitch, command.first, command.second, fire, bgr_img, window_name);
    }

    // 返回结果
    SendPacket result;
    result.found        = true;
    result.target_yaw   = command.first;
    result.target_pitch = command.second;
    result.fire_advice  = fire;
    if(plotter_enable_)
    {
        plotDiagnostics(shoot_status, target, debug_aim_point, result, gimbal_euler,
                            quaternion_age_ms, process_delay * 1000.0, converged);
    }
    if (switch_hold_remaining_ > 0) --switch_hold_remaining_;
    return result;
}

void ArmorShoot::plotDiagnostics(const char *status, const std::optional<Target> &target, const AimPoint &aim_point,
                                 const SendPacket &packet, const Eigen::Vector3d &gimbal_euler, int64_t quaternion_age_ms,
                                 double process_delay_ms, bool iteration_converged) const
{
    if (!plotter_enable_ || !plotter_) return;

    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    if (aim_point.valid) xyz = aim_point.xyza.head<3>();
    nlohmann::json data;
    data["shoot"]["status"]                   = status;
    data["shoot"]["target_valid"]             = target.has_value();
    data["shoot"]["process_delay_ms"]         = process_delay_ms;
    data["shoot"]["iteration_converged"]      = iteration_converged;
    data["send_packet"]["found"]              = packet.found;
    data["send_packet"]["fire_advice"]        = packet.fire_advice;
    data["send_packet"]["target_yaw"]         = packet.target_yaw * 57.3;
    data["send_packet"]["target_pitch"]       = packet.target_pitch * 57.3;
    data["gimbal"]["yaw"]                     = gimbal_euler[0] * 57.3;
    data["gimbal"]["pitch"]                   = gimbal_euler[1] * 57.3;
    data["gimbal"]["roll"]                    = gimbal_euler[2] * 57.3;
    data["serial"]["quaternion_age_ms"]       = quaternion_age_ms;
    data["distance"]                          = xyz.head<2>().norm();
    data["aim_point"]["valid"]                = aim_point.valid;
    data["aim_point"]["x"]                    = xyz.x();
    data["aim_point"]["y"]                    = xyz.y();
    data["aim_point"]["z"]                    = xyz.z();
    data["aim"]["mode"]                       = static_cast<int>(aim_point.mode);
    data["aim"]["armor_id"]                   = aim_point.armor_id;
    data["aim"]["raw_yaw"]                    = last_target_yaw_ * 57.3;
    data["aim"]["raw_pitch"]                  = last_raw_pitch_ * 57.3;
    data["aim"]["recovery_smoothing"]         = recovering_;
    data["aim"]["phase_deg"]                  = aim_point.phase_angle * 57.3;
    data["aim"]["fire_allowed"]               = aim_point.fire_allowed;
    data["aim"]["switch_hold_remaining"]      = switch_hold_remaining_;

    plotter_->plot(data);
}

AimPoint ArmorShoot::chooseAimPoint(const Target &target, int &lock_id, bool &coming_mode) const
{
    // 数据准备
    auto   ekf_x      = target.ekf_x();
    auto   armors     = target.armor_xyza_list();
    double omega      = std::abs(ekf_x[7]);
    double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    // 在低速阈值两侧保留 0.4 rad/s 迟滞，避免 TRACK/COMING 来回切换。
    constexpr double MODE_HYSTERESIS = 0.4;
    if (coming_mode)
    {
        if (omega < spinning_threshold_low_ - MODE_HYSTERESIS) coming_mode = false;
    }
    else if (omega > spinning_threshold_low_ + MODE_HYSTERESIS)
    {
        coming_mode = true;
    }

    // 异常情况：无装甲板数据
    if (armors.empty())
    {
        lock_id = -1; // 重置锁定的 ID
        return {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK, 0.0};
    }

    // 验证 lock_id_ 是否有效，防止锁定到不存在的装甲板
    if (lock_id != -1 && (lock_id < 0 || lock_id >= static_cast<int>(armors.size())))
    {
        MAS_LOG_DEBUG("Invalid lock_id {} reset to -1 (armor_num: {})", lock_id, armors.size());
        lock_id = -1;
    }

    // 默认打击装甲板：列表中的第一个装甲板
    int default_id = 0;

    // 预计算所有装甲板相对中心的夹角
    std::vector<double> delta_angles;
    for (const auto &armor : armors)
    {
        delta_angles.emplace_back(rm_utils::limit_rad(armor[3] - center_yaw));
    }

    // 决策逻辑

    // 英雄相位模式：仍瞄预测命中时刻的真实装甲板，只在装甲板接近正面时允许开火。
    if (hero_mode_ && coming_mode)
    {
        int best_id = 0;
        for (size_t i = 1; i < armors.size(); ++i)
        {
            if (std::abs(delta_angles[i]) < std::abs(delta_angles[best_id]))
            {
                best_id = static_cast<int>(i);
            }
        }

        lock_id = best_id;
        const double phase = delta_angles[best_id];
        return {true, armors[best_id], std::abs(phase) < fire_phase_angle_, best_id, AimMode::HERO_PHASE, phase};
    }

    // 跟踪模式：低速时选择最正面的装甲板，并保留少量切板迟滞
    else if (!coming_mode)
    {
        int best_id = 0;
        for (size_t i = 1; i < armors.size(); ++i)
        {
            if (std::abs(delta_angles[i]) < std::abs(delta_angles[best_id]))
            {
                best_id = static_cast<int>(i);
            }
        }

        // 切换迟滞阈值 5°，防止两块装甲角度差不多时来回振荡
        constexpr double SWITCH_HYSTERESIS = 5.0 / 57.3;
        if (lock_id == -1 ||
            std::abs(delta_angles[best_id]) + SWITCH_HYSTERESIS < std::abs(delta_angles[lock_id]))
        {
            lock_id = best_id;
        }

        const double phase = delta_angles[lock_id];
        return {true, armors[lock_id], std::abs(phase) < fire_phase_angle_, lock_id, AimMode::TRACK, phase};
    }

    // 非英雄来板模式：转速越过低阈值后，选择正在靠近的装甲板打击。
    else
    {
        double coming_ang  = (target.name == "outpost") ? (70.0 / 57.3) : comming_angle_;
        double leaving_ang = (target.name == "outpost") ? (30.0 / 57.3) : leaving_angle_;
        int    selected_id = -1;

        // 尝试寻找"正在靠近"的板
        for (size_t i = 0; i < armors.size(); ++i)
        {
            if (std::abs(delta_angles[i]) > coming_ang) continue;
            if (ekf_x[7] > 0 && delta_angles[i] < leaving_ang)
            {
                selected_id = i;
                break;
            }
            if (ekf_x[7] < 0 && delta_angles[i] > -leaving_ang)
            {
                selected_id = i;
                break;
            }
        }

        // 防抖动优化：如果找不到合适的板或者当前有锁定的板，优先保持连续性
        if (selected_id == -1)
        {
            // 如果没有找到"正在靠近"的板，检查是否有视野内的板
            constexpr double VISIBLE_ANGLE_THRESHOLD = 60.0 / 57.3;
            std::vector<int> visible_ids;
            for (size_t i = 0; i < armors.size(); ++i)
            {
                if (std::abs(delta_angles[i]) < VISIBLE_ANGLE_THRESHOLD)
                {
                    visible_ids.push_back(static_cast<int>(i));
                }
            }

            // 优先选择之前锁定的板（如果有）
            if (!visible_ids.empty())
            {
                if (lock_id != -1 && std::find(visible_ids.begin(), visible_ids.end(), lock_id) != visible_ids.end())
                {
                    selected_id = lock_id;
                }
                else
                {
                    // 选择夹角最小的
                    int    best_id   = visible_ids[0];
                    double min_angle = std::abs(delta_angles[best_id]);
                    for (int id : visible_ids)
                    {
                        double angle = std::abs(delta_angles[id]);
                        if (angle < min_angle)
                        {
                            min_angle = angle;
                            best_id   = id;
                        }
                    }
                    selected_id = best_id;
                }
            }
            else
            {
                // 视野内没有板，使用默认的第一个板
                selected_id = default_id;
            }
        }
        else
        {
            // 找到了正在靠近的板，更新 lock_id_
            lock_id = selected_id;
        }

        const double phase = delta_angles[selected_id];
        return {true, armors[selected_id], std::abs(phase) < fire_phase_angle_, selected_id, AimMode::COMING, phase};
    }
}

void ArmorShoot::showDebug(const std::optional<Target> &target, const AimPoint &aim_point, double gimbal_yaw, double gimbal_pitch, float target_yaw,
                           float target_pitch, uint8_t fire, const cv::Mat &bgr_img, const std::string &window_name) const noexcept
{
    if (bgr_img.empty() || window_name.empty()) return;

    auto &display   = rm_utils::Display::getInstance();
    auto &fps       = fps_map_[window_name];
    auto &count     = count_map_[window_name];
    auto &last_time = last_time_map_[window_name];

    // FPS 统计
    count++;
    auto now      = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (duration >= 1000)
    {
        fps       = count * 1000.0 / duration;
        count     = 0;
        last_time = now;
    }

    std::vector<rm_utils::DisplayText>  texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine>  lines;

    // 1. FPS 信息
    rm_utils::DisplayText t_fps;
    t_fps.content = "FPS: " + std::to_string(static_cast<int>(fps));
    t_fps.x       = 10;
    t_fps.y       = 10;
    t_fps.size    = 22;
    t_fps.color   = {255, 255, 0, 255};
    texts.push_back(t_fps);

    // 云台当前角度
    rm_utils::DisplayText t_gimbal;
    t_gimbal.content =
        "Gimbal  yaw:" + std::to_string(gimbal_yaw * 57.3f).substr(0, 6) + "  pitch:" + std::to_string(gimbal_pitch * 57.3f).substr(0, 6) + " deg";
    t_gimbal.x     = 10;
    t_gimbal.y     = 36;
    t_gimbal.size  = 32;
    t_gimbal.color = {0, 255, 255, 255};
    texts.push_back(t_gimbal);

    // 无目标情况
    if (!target.has_value())
    {
        rm_utils::DisplayText t_no;
        t_no.content = "No Target";
        t_no.x       = 10;
        t_no.y       = 36;
        t_no.size    = 32;
        t_no.color   = {200, 200, 200, 255};
        texts.push_back(t_no);
        display.display_add(window_name, bgr_img, texts, points, lines);
        return;
    }

    const Eigen::VectorXd &x = target->ekf().x;

    // 目标角度（解算结果）
    rm_utils::DisplayText t_target;
    t_target.content =
        "Target  yaw:" + std::to_string(target_yaw * 57.3f).substr(0, 6) + "  pitch:" + std::to_string(target_pitch * 57.3f).substr(0, 6) + " deg";
    t_target.x     = 10;
    t_target.y     = 70;
    t_target.size  = 32;
    t_target.color = {0, 255, 0, 255};
    texts.push_back(t_target);

    // 角度偏差
    rm_utils::DisplayText t_diff;
    double                diff_yaw   = (target_yaw - gimbal_yaw) * 57.3;
    double                diff_pitch = (target_pitch - gimbal_pitch) * 57.3;
    t_diff.content                   = "Diff_yaw:" + std::to_string(diff_yaw).substr(0, std::to_string(diff_yaw).find('.') + 3) +
                     "Diff_pitch:" + std::to_string(diff_pitch).substr(0, std::to_string(diff_pitch).find('.') + 3) + " deg";
    t_diff.x     = 10;
    t_diff.y     = 104;
    t_diff.size  = 32;
    t_diff.color = {255, 200, 100, 255};
    texts.push_back(t_diff);

    // 瞄准点模式和信息
    std::string mode_str;
    switch (aim_point.mode)
    {
    case AimMode::TRACK:
        mode_str = "TRACK";
        break;
    case AimMode::HERO_PHASE:
        mode_str = "HERO_PHASE";
        break;
    case AimMode::COMING:
        mode_str = "COMING";
        break;
    }

    rm_utils::DisplayText t_info;
    t_info.content = "Name:" + target->name + "  armor_num:" + std::to_string(target->armor_num) + "  omega:" + std::to_string(x[7]).substr(0, 5) +
                     "  fire:" + std::to_string(fire) + "  " + mode_str + "  hero_mode:" + (hero_mode_ ? "1" : "0");
    t_info.x     = 10;
    t_info.y     = 144;
    t_info.size  = 32;
    t_info.color = {255, 150, 0, 255};
    texts.push_back(t_info);

    // 距离和弹道信息
    double                dist = std::sqrt(aim_point.xyza[0] * aim_point.xyza[0] + aim_point.xyza[1] * aim_point.xyza[1]);
    rm_utils::DisplayText t_dist;
    t_dist.content = "Dist:" + std::to_string(dist).substr(0, 5) + "m  " + "Valid:" + std::string(aim_point.valid ? "YES" : "NO") + "  " +
                     "ArmorID:" + std::to_string(aim_point.armor_id);
    t_dist.x     = 10;
    t_dist.y     = 170;
    t_dist.size  = 28;
    t_dist.color = aim_point.valid ? SDL_Color{0, 255, 0, 255} : SDL_Color{0, 0, 255, 255};
    texts.push_back(t_dist);

    display.display_add(window_name, bgr_img, texts, points, lines);
}

} // namespace auto_aim
