#include "armor_track.hpp"

#include <chrono>
#include <exception>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include "armor_types.hpp"
#include "display.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "yaml-cpp/yaml.h"

namespace auto_aim
{

/**
 * @brief 构造函数
 * @details 初始化位姿解算器，加载跟踪参数配置
 * @param track_config_path 跟踪配置文件路径
 * @param pose_config_path 位姿配置文件路径
 */
ArmorTrack::ArmorTrack(const std::string &track_config_path, const std::string &pose_config_path)
    : armor_pose_(pose_config_path), detect_count_(0), temp_lost_count_(0), state_{"lost"}, pre_state_{"lost"},
      last_timestamp_(std::chrono::steady_clock::now())
{
    debug_ = false;
    plotter_enable_ = false;

    try
    {
        // 加载YAML配置文件
        YAML::Node config            = YAML::LoadFile(track_config_path);
        debug_                       = config["auto_aim"]["armor_track"]["armor_track_debug"].as<bool>(false);
        plotter_enable_               = config["auto_aim"]["armor_track"]["plotter_enable"].as<bool>(false);
        min_detect_count_            = config["auto_aim"]["armor_track"]["min_detect_count"].as<int>(5);
        max_temp_lost_count_         = config["auto_aim"]["armor_track"]["max_temp_lost_count"].as<int>(3);
        outpost_max_temp_lost_count_ = config["auto_aim"]["armor_track"]["outpost_max_temp_lost_count"].as<int>(5);
        normal_temp_lost_count_      = max_temp_lost_count_;
        max_z_innovation_            = config["auto_aim"]["armor_track"]["max_z_innovation"].as<double>(0.25);
        max_position_innovation_     = config["auto_aim"]["armor_track"]["max_position_innovation"].as<double>(0.8);

        MAS_LOG_INFO("armor_track yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

ArmorTrack::~ArmorTrack() {}

std::optional<Target> ArmorTrack::track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t, std::string window_name,
                                        cv::Mat bgr_img) noexcept
{
    raw_pnp_xyz_.setZero();
    raw_pnp_valid_        = false;
    measurement_accepted_ = false;
    matching_armor_count_ = 0;
    pnp_attempt_count_    = 0;
    pnp_success_count_    = 0;
    innovation_z_         = 0.0;
    innovation_norm_      = 0.0;
    reject_reason_        = "none";
    reset_reason_         = "none";
    const std::string state_before = state_;
    bool plot_emitted = false;

    // 计算时间间隔
    auto dt = rm_utils::delta_time(t, last_timestamp_);

    // 预防时间间隔为负或零的情况，可能是时间戳异常或系统时钟调整
    if (dt <= 0.0)
    {
        reject_reason_ = "invalid_timestamp";
        plotDiagnostics(armors, false, dt, state_before);
        if ((state_ == "tracking" || state_ == "temp_lost") && target_.has_value()) return target_;
        return std::nullopt;
    }

    last_timestamp_ = t;

    // 时间间隔过长，说明可能发生了相机离线，重置状态
    if (state_ != "lost" && dt > 0.1)
    {
        MAS_LOG_WARN("Large dt: {}s", dt);
        state_           = "lost";
        detect_count_    = 0;
        temp_lost_count_ = 0;
        target_.reset();
        reset_reason_ = "large_dt";
    }

    // 先按优先级排序，同优先级按距离图像中心排序
    if (!armors.empty())
    {
        const cv::Point2f              img_center(bgr_img.cols / 2.0f, bgr_img.rows / 2.0f);
        const std::vector<std::string> priority_order = {"1", "2", "3", "4", "5", "sentry", "outpost", "base"};

        // 获取装甲板优先级分数（越小优先级越高）
        auto getPriorityScore = [&](const Armor &armor) -> int {
            auto it = std::find(priority_order.begin(), priority_order.end(), armor.number);
            if (it != priority_order.end()) return std::distance(priority_order.begin(), it);
            return 100; // 未知目标最低优先级
        };

        // 排序：先按优先级，同优先级按距离图像中心
        std::sort(armors.begin(), armors.end(), [&](const Armor &a, const Armor &b) {
            int prio_a = getPriorityScore(a);
            int prio_b = getPriorityScore(b);
            if (prio_a != prio_b) return prio_a < prio_b;
            double dist_a = cv::norm(a.center - img_center);
            double dist_b = cv::norm(b.center - img_center);
            return dist_a < dist_b;
        });
    }

    // 状态机处理
    bool found = false;
    if (state_ == "lost")
    {
        // 丢失状态下，选择第一个装甲板作为新目标
        if (!armors.empty())
        {
            // 只解算需要用的装甲板
            pnp_attempt_count_ = 1;
            if (armor_pose_.GetArmorPose(armors[0]))
            {
                pnp_success_count_     = 1;
                raw_pnp_xyz_          = armors[0].xyz_in_world;
                raw_pnp_valid_        = true;
                measurement_accepted_ = true;
                found                 = set_target(armors[0], t);
                reject_reason_        = "accepted";
            }
            else reject_reason_ = "pnp_failed";
        }
        else reject_reason_ = "no_detection";
    }
    else
    {
        // 非丢失状态下，更新已有目标
        found = update_target(armors, t);
    }

    state_machine(found);

    if (state_before == "detecting" && state_ == "lost" && !found)
    {
        reset_reason_ = "detecting_miss";
    }
    else if (state_before == "temp_lost" && state_ == "lost" && !found)
    {
        reset_reason_ = "temp_lost_timeout";
    }

    // 发散检测：目标参数异常时重置
    if (state_ != "lost" && target_.has_value() && target_->diverged())
    {
        MAS_LOG_INFO("Target diverged!");
        state_ = "lost";
        reset_reason_ = "radius_diverged";
        plotDiagnostics(armors, found, dt, state_before);
        plot_emitted = true;
        target_.reset();
    }

    // 收敛效果检测：
    if (target_.has_value() && std::accumulate(target_->ekf().recent_nis_failures.begin(), target_->ekf().recent_nis_failures.end(), 0) >=
                                   (0.4 * target_->ekf().window_size))
    {
        MAS_LOG_INFO("Bad Converge Found!");
        state_ = "lost";
        reset_reason_ = "nis_bad_converge";
        plotDiagnostics(armors, found, dt, state_before);
        plot_emitted = true;
        target_.reset();
    }

    // Debug显示
    if (debug_)
    {
        showResult(armors, bgr_img, window_name);
    }

    if (!plot_emitted) plotDiagnostics(armors, found, dt, state_before);

    // 仅 tracking / temp_lost 状态下返回目标
    if ((state_ == "tracking" || state_ == "temp_lost") && target_.has_value()) return target_;
    return std::nullopt;
}

/**
 * @brief 状态机处理
 * @details 管理跟踪状态转换：lost -> detecting -> tracking <-> temp_lost
 * @param found 是否找到目标
 */
void ArmorTrack::state_machine(bool found)
{
    if (state_ == "lost")
    {
        // 丢失状态：找到目标后进入检测状态
        if (!found) return;
        state_        = "detecting";
        detect_count_ = 1;
    }
    else if (state_ == "detecting")
    {
        // 检测状态：连续检测到足够次数后进入跟踪状态
        if (found)
        {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = "tracking";
        }
        else
        {
            // 检测失败，回到丢失状态
            detect_count_ = 0;
            state_        = "lost";
            target_.reset();
        }
    }
    else if (state_ == "tracking")
    {
        // 跟踪状态：丢失目标后进入临时丢失状态
        if (found) return;
        temp_lost_count_ = 1;
        state_           = "temp_lost";
    }
    else if (state_ == "temp_lost")
    {
        // 临时丢失状态：重新找到目标则恢复跟踪，否则计数
        if (found)
        {
            state_ = "tracking";
        }
        else
        {
            temp_lost_count_++;
            // 前哨站允许更长的丢失时间
            int max_lost = (target_.has_value() && target_->name == "outpost") ? outpost_max_temp_lost_count_ : normal_temp_lost_count_;
            if (temp_lost_count_ > max_lost)
            {
                state_ = "lost";
                target_.reset();
            }
        }
    }
}

/**
 * @brief 设置新目标
 * @details 根据装甲板类型初始化EKF参数，创建跟踪目标
 * @param armor 初始装甲板
 * @param t 时间戳
 * @return 是否成功
 */
bool ArmorTrack::set_target(const Armor &armor, std::chrono::steady_clock::time_point t)
{
    double          radius;    // 旋转半径
    int             armor_num; // 装甲板数量
    Eigen::VectorXd P0_dig;    // 初始协方差矩阵对角线

    // 根据目标类型设置EKF参数
    if (armor.number == "outpost")
    {
        // 前哨站：3块装甲板，半径0.2765m
        radius    = 0.2765;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    }
    else if (armor.number == "base")
    {
        // 基地：3块装甲板，半径0.3205m
        radius    = 0.3205;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    }
    else
    {
        // 普通车辆：4块装甲板，半径0.2m
        radius    = 0.2;
        armor_num = 4;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    }

    // 使用 Target 构造函数创建目标
    target_ = Target(armor, t, radius, armor_num, P0_dig, max_z_innovation_, max_position_innovation_);
    return true;
}

/**
 * @brief 更新目标
 * @details 预测目标位置，匹配最佳装甲板，更新EKF状态
 * @param armors 检测到的装甲板列表
 * @param t 时间戳
 * @return 是否成功
 */
bool ArmorTrack::update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    if (!target_.has_value())
    {
        reject_reason_ = "no_target";
        return false;
    }

    // EKF预测
    target_->predict(t);

    // 统计匹配目标编号和类型的装甲板数量
    int found_count = 0;
    for (const auto &armor : armors)
    {
        if (armor.number != target_->name || armor.type != target_->armor_type) continue;
        found_count++;
    }
    matching_armor_count_ = found_count;

    if (found_count == 0)
    {
        reject_reason_ = armors.empty() ? "no_detection" : "target_mismatch";
        return false;
    }

    // 防止在 armors 旋转时出现多块装甲板，检测一块接近的装甲板即可 
    for (auto &armor : armors)
    {
        if (armor.number != target_->name || armor.type != target_->armor_type) continue;

        pnp_attempt_count_++;
        if (!armor_pose_.GetArmorPose(armor)) continue;
        pnp_success_count_++;

        raw_pnp_xyz_   = armor.xyz_in_world;
        raw_pnp_valid_ = true;
        if (target_->update(armor))
        {
            innovation_z_         = target_->last_innovation_z();
            innovation_norm_      = target_->last_innovation_norm();
            measurement_accepted_ = true;
            reject_reason_        = "accepted";
            return true;
        }

        innovation_z_    = target_->last_innovation_z();
        innovation_norm_ = target_->last_innovation_norm();
        switch (target_->last_update_reject_reason())
        {
        case Target::UpdateRejectReason::NONFINITE:
            reject_reason_ = "innovation_nonfinite";
            break;
        case Target::UpdateRejectReason::Z_INNOVATION:
            reject_reason_ = "z_innovation";
            break;
        case Target::UpdateRejectReason::POSITION_INNOVATION:
            reject_reason_ = "position_innovation";
            break;
        case Target::UpdateRejectReason::NONE:
            break;
        }
    }

    if (pnp_success_count_ == 0) reject_reason_ = "pnp_failed";
    return false;
}

void ArmorTrack::plotDiagnostics(const std::vector<Armor> &armors, bool found, double dt, const std::string &state_before)
{
    if (!plotter_enable_) return;

    nlohmann::json data;
    data["state"]                    = state_;
    data["state_before"]             = state_before;
    data["reject_reason"]            = reject_reason_;
    data["reset_reason"]             = reset_reason_;
    data["dt_ms"]                    = dt * 1000.0;
    data["target_valid"]             = target_.has_value();
    data["detected_armor_count"]     = armors.size();
    data["matching_armor_count"]     = matching_armor_count_;
    data["pnp_attempt_count"]        = pnp_attempt_count_;
    data["pnp_success_count"]        = pnp_success_count_;
    data["association_found"]        = found;
    data["innovation_z"]             = innovation_z_;
    data["innovation_norm"]          = innovation_norm_;
    data["raw_pnp"]["valid"]       = raw_pnp_valid_;
    data["raw_pnp"]["accepted"]    = measurement_accepted_;
    data["raw_pnp"]["x"]           = raw_pnp_xyz_.x();
    data["raw_pnp"]["y"]           = raw_pnp_xyz_.y();
    data["raw_pnp"]["z"]           = raw_pnp_xyz_.z();

    data["target_name"] = target_.has_value() ? target_->name : "none";
    data["target_id"]   = target_.has_value() ? target_->last_id : -1;
    data["armor_type"]  = target_.has_value() ? ((target_->armor_type == ArmorType::BIG) ? "BIG" : "SMALL") : "NONE";
    data["update_count"] = target_.has_value() ? target_->update_count : 0;
    data["jumped"]       = target_.has_value() && target_->jumped;
    data["is_switch"]    = target_.has_value() && target_->is_switch;
    data["armor_num"]    = target_.has_value() ? target_->armor_num : 0;

    Eigen::VectorXd x;
    if (target_.has_value()) x = target_->ekf_x();
    data["x"]   = x.size() >= 11 ? x[0] : 0.0;
    data["vx"]  = x.size() >= 11 ? x[1] : 0.0;
    data["y"]   = x.size() >= 11 ? x[2] : 0.0;
    data["vy"]  = x.size() >= 11 ? x[3] : 0.0;
    data["z"]   = x.size() >= 11 ? x[4] : 0.0;
    data["vz"]  = x.size() >= 11 ? x[5] : 0.0;
    data["yaw"] = x.size() >= 11 ? x[6] : 0.0;
    data["w"]   = x.size() >= 11 ? x[7] : 0.0;
    data["r"]   = x.size() >= 11 ? x[8] : 0.0;
    data["l"]   = x.size() >= 11 ? x[9] : 0.0;
    data["h"]   = x.size() >= 11 ? x[10] : 0.0;

    plotter_.plot(data);
}

void ArmorTrack::showResult(const std::vector<Armor> &armors, const cv::Mat &bgr_img, std::string window_name) const noexcept
{
    if (!debug_) return;

    // 检查图像有效性
    if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0)
    {
        MAS_LOG_WARN("Invalid input image for showResult");
        return;
    }

    // 调用位姿解算器的debug显示
    if (armor_pose_.isDebug())
    {
        armor_pose_.showPoseDebug(armors, bgr_img, window_name + "_pose");
    }

    auto &display = rm_utils::Display::getInstance();

    // 计算FPS
    auto &fps       = fps_map_[window_name];
    auto &count     = count_map_[window_name];
    auto &last_time = last_time_map_[window_name];

    count++;
    auto now      = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (duration >= 1000)
    {
        fps       = count * 1000.0 / duration;
        count     = 0;
        last_time = now;
    }

    // 初始化显示元素容器
    std::vector<rm_utils::DisplayPoint> display_points;
    std::vector<rm_utils::DisplayLine>  display_lines;
    std::vector<rm_utils::DisplayText>  display_texts;

    // FPS和状态显示
    rm_utils::DisplayText fps_text;
    fps_text.content   = "FPS: " + std::to_string(static_cast<int>(fps)) + " Armors: " + std::to_string(armors.size()) + " State: " + state_;
    fps_text.x         = 20;
    fps_text.y         = 20;
    fps_text.size      = 24;
    fps_text.color     = {255, 255, 0, 255};
    fps_text.thickness = 2;
    display_texts.push_back(fps_text);

    // 显示跟踪目标参数
    if (target_.has_value())
    {
        // 目标基本信息
        rm_utils::DisplayText target_text;
        target_text.content =
            "Target: " + target_->name + " ID: " + std::to_string(target_->last_id) + " Updates: " + std::to_string(target_->update_count);
        target_text.x         = 20;
        target_text.y         = 50;
        target_text.size      = 20;
        target_text.color     = {0, 255, 0, 255};
        target_text.thickness = 1;
        display_texts.push_back(target_text);

        // 目标类型和状态
        rm_utils::DisplayText target_info;
        target_info.content = "type: " + std::string(target_->armor_type == ArmorType::BIG ? "BIG" : "SMALL") +
                              " jumped: " + std::to_string(target_->jumped) + " switch: " + std::to_string(target_->is_switch) +
                              " armor_num: " + std::to_string(target_->armor_num);
        target_info.x         = 20;
        target_info.y         = 75;
        target_info.size      = 18;
        target_info.color     = {255, 200, 0, 255};
        target_info.thickness = 1;
        display_texts.push_back(target_info);

        // EKF状态显示
        if (target_->ekf().x.size() >= 11)
        {
            // 位置
            rm_utils::DisplayText ekf_text1;
            ekf_text1.content = "x: " + std::to_string(target_->ekf().x[0]).substr(0, 4) + " y: " + std::to_string(target_->ekf().x[2]).substr(0, 4) +
                                " z: " + std::to_string(target_->ekf().x[4]).substr(0, 4);
            ekf_text1.x         = 20;
            ekf_text1.y         = 100;
            ekf_text1.size      = 18;
            ekf_text1.color     = {0, 255, 255, 255};
            ekf_text1.thickness = 1;
            display_texts.push_back(ekf_text1);

            // 速度
            rm_utils::DisplayText ekf_text2;
            ekf_text2.content = "vx: " + std::to_string(target_->ekf().x[1]).substr(0, 4) +
                                " vy: " + std::to_string(target_->ekf().x[3]).substr(0, 4) +
                                " vz: " + std::to_string(target_->ekf().x[5]).substr(0, 4);
            ekf_text2.x         = 20;
            ekf_text2.y         = 125;
            ekf_text2.size      = 18;
            ekf_text2.color     = {0, 255, 255, 255};
            ekf_text2.thickness = 1;
            display_texts.push_back(ekf_text2);

            // 角度和角速度
            rm_utils::DisplayText ekf_text3;
            ekf_text3.content =
                "yaw: " + std::to_string(target_->ekf().x[6]).substr(0, 4) + " w: " + std::to_string(target_->ekf().x[7]).substr(0, 4);
            ekf_text3.x         = 20;
            ekf_text3.y         = 150;
            ekf_text3.size      = 18;
            ekf_text3.color     = {0, 255, 255, 255};
            ekf_text3.thickness = 1;
            display_texts.push_back(ekf_text3);

            // 半径参数
            rm_utils::DisplayText ekf_text4;
            ekf_text4.content = "r: " + std::to_string(target_->ekf().x[8]).substr(0, 4) + " l: " + std::to_string(target_->ekf().x[9]).substr(0, 4) +
                                " h: " + std::to_string(target_->ekf().x[10]).substr(0, 4);
            ekf_text4.x         = 20;
            ekf_text4.y         = 175;
            ekf_text4.size      = 18;
            ekf_text4.color     = {0, 255, 255, 255};
            ekf_text4.thickness = 1;
            display_texts.push_back(ekf_text4);
        }
    }

    for (const auto &armor : armors)
    {
        // 显示装甲板编号和置信度
        rm_utils::DisplayText armor_info_text;
        armor_info_text.content   = "[" + armor.number + "]: [" + std::to_string(armor.confidence).substr(0, 5) + "]";
        armor_info_text.x         = static_cast<int>(armor.center.x) - static_cast<int>(armor_info_text.size * armor_info_text.content.size() / 2);
        armor_info_text.y         = static_cast<int>(armor.center.y);
        armor_info_text.size      = 32;
        armor_info_text.color     = {255, 255, 255, 255};
        armor_info_text.thickness = 1;
        display_texts.push_back(armor_info_text);
    }

    if (target_.has_value() && target_->ekf().x.size() >= 11)
    {
        // 绘制预测的装甲板位置
        auto xyza_list = target_->armor_xyza_list();
        for (int i = 0; i < (int)xyza_list.size() && i < target_->armor_num; i++)
        {
            const auto     &xyza = xyza_list[i];
            Eigen::Vector3d armor_xyz_world(xyza[0], xyza[1], xyza[2]);
            double          yaw = target_->ekf().x[6] + i * (2 * CV_PI / target_->armor_num);

            // 投影到图像平面
            auto image_points = armor_pose_.reproject_armor(armor_xyz_world, yaw, target_->armor_type, target_->name);

            if (image_points.empty()) continue;

            // 当前跟踪的装甲板用绿色，其他用红色
            bool      is_tracking = (i == target_->last_id);
            SDL_Color color       = is_tracking ? SDL_Color{0, 255, 0, 255} : SDL_Color{255, 0, 0, 255};

            // 绘制矩形框
            for (int j = 0; j < 4; j++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(image_points[j].x);
                line.y1        = static_cast<int>(image_points[j].y);
                line.x2        = static_cast<int>(image_points[(j + 1) % 4].x);
                line.y2        = static_cast<int>(image_points[(j + 1) % 4].y);
                line.thickness = is_tracking ? 3 : 2;
                line.color     = color;
                display_lines.push_back(line);
            }

            // 当前跟踪的装甲板绘制X型对角线
            if (is_tracking)
            {
                rm_utils::DisplayLine diag1;
                diag1.x1        = static_cast<int>(image_points[0].x);
                diag1.y1        = static_cast<int>(image_points[0].y);
                diag1.x2        = static_cast<int>(image_points[2].x);
                diag1.y2        = static_cast<int>(image_points[2].y);
                diag1.thickness = 2;
                diag1.color     = color;
                display_lines.push_back(diag1);

                rm_utils::DisplayLine diag2;
                diag2.x1        = static_cast<int>(image_points[1].x);
                diag2.y1        = static_cast<int>(image_points[1].y);
                diag2.x2        = static_cast<int>(image_points[3].x);
                diag2.y2        = static_cast<int>(image_points[3].y);
                diag2.thickness = 2;
                diag2.color     = color;
                display_lines.push_back(diag2);
            }

            // 绘制装甲板ID
            rm_utils::DisplayText id_text;
            id_text.content   = std::to_string(i);
            id_text.x         = static_cast<int>(image_points[0].x);
            id_text.y         = static_cast<int>(image_points[0].y) - 5;
            id_text.size      = 32;
            id_text.color     = {255, 255, 0, 255};
            id_text.thickness = 1;
            display_texts.push_back(id_text);
        }

        // 绘制旋转中心点
        Eigen::Vector3d          center_xyz(target_->ekf().x[0], target_->ekf().x[2], target_->ekf().x[4]);
        std::vector<cv::Point3f> center_world = {cv::Point3f(center_xyz[0], center_xyz[1], center_xyz[2])};
        auto                     center_pixel = armor_pose_.world2pixel(center_world);
        if (!center_pixel.empty())
        {
            rm_utils::DisplayPoint center_point;
            center_point.x     = static_cast<int>(center_pixel[0].x);
            center_point.y     = static_cast<int>(center_pixel[0].y);
            center_point.size  = 15;
            center_point.color = {255, 0, 255, 255};
            display_points.push_back(center_point);
        }
    }

    // 提交显示
    display.display_add(window_name, bgr_img, display_texts, display_points, display_lines);
}

} // namespace auto_aim
