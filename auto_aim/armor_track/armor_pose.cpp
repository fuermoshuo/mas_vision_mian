#include "armor_pose.hpp"

#include <cmath>
#include <exception>
#include <fstream>

#include "armor_types.hpp"
#include "display.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "yaml-cpp/yaml.h"

namespace auto_aim
{
ArmorPose::ArmorPose(const std::string &config_path, const std::string &track_config_path)
    : R_gimbal2world_(Eigen::Matrix3d::Identity()), debug_(false)
{
    // yaml 读取参数
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_ERROR("Config file not found: {}, using default values", config_path.c_str());
            return;
        }

        YAML::Node config = YAML::LoadFile(config_path);
        // 读取参数
        auto R_gimbal2imubody_data = config["handeye_calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
        auto R_camera2gimbal_data  = config["handeye_calibration"]["R_camera2gimbal"].as<std::vector<double>>();
        auto t_camera2gimbal_data  = config["handeye_calibration"]["t_camera2gimbal"].as<std::vector<double>>();
        R_gimbal2imubody_          = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
        R_camera2gimbal_           = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
        t_camera2gimbal_           = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

        auto                                         camera_matrix_data  = config["calibration"]["camera_matrix"].as<std::vector<double>>();
        auto                                         distort_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
        Eigen::Matrix<double, 1, 5>                  distort_coeffs(distort_coeffs_data.data());
        cv::eigen2cv(camera_matrix, camera_matrix_);
        cv::eigen2cv(distort_coeffs, distort_coeffs_);

        MAS_LOG_INFO("armor_pose yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }

    // 读取debug配置
    try
    {
        std::ifstream track_file(track_config_path);
        if (track_file.good())
        {
            YAML::Node track_config = YAML::LoadFile(track_config_path);
            if (track_config["auto_aim"] && track_config["auto_aim"]["armor_track"])
            {
                debug_ = track_config["auto_aim"]["armor_track"]["armor_pose_debug"].as<bool>(false);
            }
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load armor_pose_debug config: {}", e.what());
    }
}

ArmorPose::~ArmorPose() = default;


void ArmorPose::set_R_gimbal2world(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R_imubody2world = q.toRotationMatrix();
    R_gimbal2world_                 = R_gimbal2imubody_.transpose() * R_imubody2world * R_gimbal2imubody_;
}

bool ArmorPose::GetArmorPose(Armor &armor) const
{
    const auto &object_points = (armor.type == ArmorType::BIG) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    // 使用 SOLVEPNP求解
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE) ||
        !std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) || tvec[2] <= 0.0)
    {
        return false;
    }

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

    if (!armor.xyz_in_gimbal.allFinite() || !armor.xyz_in_world.allFinite()) return false;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);

    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal            = rm_utils::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world             = rm_utils::eulers(R_armor2world, 2, 1, 0);
    armor.ypd_in_world             = rm_utils::xyz2ypd(armor.xyz_in_world);

    if (!armor.ypr_in_gimbal.allFinite() || !armor.ypr_in_world.allFinite() || !armor.ypd_in_world.allFinite()) return false;

    // 平衡不做yaw优化，因为pitch假设不成立
    auto is_balance = (armor.type == ArmorType::BIG) && (armor.number == "3" || armor.number == "4" || armor.number == "5");
    if (is_balance) return true;

    optimize_yaw(armor);
    return armor.ypr_in_world.allFinite();

}

void ArmorPose::optimize_yaw(Armor &armor) const
{
    Eigen::Vector3d gimbal_ypr = rm_utils::eulers(R_gimbal2world_, 2, 1, 0);

    constexpr double SEARCH_RANGE = 140; // degree
    auto             yaw0         = rm_utils::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

    auto min_error = 1e10;
    auto best_yaw  = armor.ypr_in_world[0];

    for (int i = 0; i < SEARCH_RANGE; i++)
    {
        double yaw   = rm_utils::limit_rad(yaw0 + i * CV_PI / 180.0);
        auto   error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

        if (error < min_error)
        {
            min_error = error;
            best_yaw  = yaw;
        }
    }

    armor.yaw_raw         = armor.ypr_in_world[0];
    armor.ypr_in_world[0] = best_yaw;
}

std::vector<cv::Point2f> ArmorPose::reproject_armor(const Eigen::Vector3d &xyz_in_world, double yaw, ArmorType type, std::string name) const
{
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch     = (name == "outpost") ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d R_armor2world {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
        {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d &t_armor2world  = xyz_in_world;
    Eigen::Matrix3d        R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d        t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d rvec;
    cv::Mat   R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    const auto              &object_points = (type == ArmorType::BIG) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}

double ArmorPose::outpost_reprojection_error(Armor armor, const double &pitch)
{
    // solve
    const auto &object_points = (armor.type == ArmorType::BIG) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::Vec3d rvec, tvec;
    cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal            = rm_utils::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world             = rm_utils::eulers(R_armor2world, 2, 1, 0);

    armor.ypd_in_world = rm_utils::xyz2ypd(armor.xyz_in_world);

    auto yaw          = armor.ypr_in_world[0];
    auto xyz_in_world = armor.xyz_in_world;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d _R_armor2world {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
        {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d &t_armor2world   = xyz_in_world;
    Eigen::Matrix3d        _R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
    Eigen::Vector3d        t_armor2camera  = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d _rvec;
    cv::Mat   R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    return error;
}

double ArmorPose::armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const
{
    auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.number);
    auto error        = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    // auto error = SJTU_cost(image_points, armor.points, inclined);

    return error;
}

std::vector<cv::Point2f> ArmorPose::world2pixel(const std::vector<cv::Point3f> &worldPoints) const
{
    Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    cv::Mat rvec;
    cv::Mat tvec;
    cv::eigen2cv(R_world2camera, rvec);
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point3f> valid_world_points;
    for (const auto &world_point : worldPoints)
    {
        Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
        Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

        if (camera_point.z() > 0)
        {
            valid_world_points.push_back(world_point);
        }
    }
    // 如果没有有效点，返回空vector
    if (valid_world_points.empty())
    {
        return std::vector<cv::Point2f>();
    }
    std::vector<cv::Point2f> pixelPoints;
    cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
    return pixelPoints;
}

void ArmorPose::showPoseDebug(const std::vector<Armor> &armors, const cv::Mat &bgr_img, const std::string &window_name) const
{
    if (!debug_) return;

    if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0)
    {
        MAS_LOG_WARN("Invalid input image for showPoseDebug");
        return;
    }

    auto &display = rm_utils::Display::getInstance();

    std::vector<rm_utils::DisplayPoint> display_points;
    std::vector<rm_utils::DisplayLine>  display_lines;
    std::vector<rm_utils::DisplayText>  display_texts;

    int text_offset = 50;

    // 显示IMU位姿信息
    Eigen::Matrix3d       R_gimbal2world = Get_R_gimbal2world();
    Eigen::Vector3d       imu_ypr        = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    rm_utils::DisplayText imu_text;
    imu_text.content = "  IMU YPR: [" + std::to_string(imu_ypr.x() * 57.3f).substr(0, 5) + ", " + std::to_string(imu_ypr.y() * 57.3f).substr(0, 5) +
                       ", " + std::to_string(imu_ypr.z() * 57.3f).substr(0, 5) + "] deg";
    imu_text.x         = 20;
    imu_text.y         = text_offset;
    imu_text.size      = 32;
    imu_text.color     = {255, 255, 0, 0};
    imu_text.thickness = 1;
    display_texts.push_back(imu_text);
    text_offset += imu_text.size;

    // 仅显示第一个装甲板的信息
    if (!armors.empty())
    {
        const auto           &armor = armors[0];
        rm_utils::DisplayText gimbal_xyz_text;
        gimbal_xyz_text.content = "  Gimbal XYZ: [" + std::to_string(armor.xyz_in_gimbal.x()).substr(0, 5) + ", " +
                                  std::to_string(armor.xyz_in_gimbal.y()).substr(0, 5) + ", " + std::to_string(armor.xyz_in_gimbal.z()).substr(0, 5) +
                                  "] m";
        gimbal_xyz_text.x         = 20;
        gimbal_xyz_text.y         = text_offset;
        gimbal_xyz_text.size      = 32;
        gimbal_xyz_text.color     = {255, 255, 255, 255};
        gimbal_xyz_text.thickness = 1;
        display_texts.push_back(gimbal_xyz_text);
        text_offset += gimbal_xyz_text.size;

        rm_utils::DisplayText gimbal_ypr_text;
        gimbal_ypr_text.content = "  Gimbal YPR: [" + std::to_string(armor.ypr_in_gimbal.x() * 57.3f).substr(0, 5) + ", " +
                                  std::to_string(armor.ypr_in_gimbal.y() * 57.3f).substr(0, 5) + ", " +
                                  std::to_string(armor.ypr_in_gimbal.z() * 57.3f).substr(0, 5) + "] deg";
        gimbal_ypr_text.x         = 20;
        gimbal_ypr_text.y         = text_offset;
        gimbal_ypr_text.size      = 32;
        gimbal_ypr_text.color     = {255, 255, 255, 255};
        gimbal_ypr_text.thickness = 1;
        display_texts.push_back(gimbal_ypr_text);
        text_offset += gimbal_ypr_text.size;

        rm_utils::DisplayText world_xyz_text;
        world_xyz_text.content = "  World XYZ: [" + std::to_string(armor.xyz_in_world.x()).substr(0, 5) + ", " +
                                 std::to_string(armor.xyz_in_world.y()).substr(0, 5) + ", " + std::to_string(armor.xyz_in_world.z()).substr(0, 5) +
                                 "] m";
        world_xyz_text.x         = 20;
        world_xyz_text.y         = text_offset;
        world_xyz_text.size      = 32;
        world_xyz_text.color     = {255, 255, 255, 255};
        world_xyz_text.thickness = 1;
        display_texts.push_back(world_xyz_text);
        text_offset += world_xyz_text.size;

        rm_utils::DisplayText world_ypr_text;
        world_ypr_text.content = "  World YPR: [" + std::to_string(armor.ypr_in_world.x() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypr_in_world.y() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypr_in_world.z() * 57.3f).substr(0, 5) + "] deg";
        world_ypr_text.x         = 20;
        world_ypr_text.y         = text_offset;
        world_ypr_text.size      = 32;
        world_ypr_text.color     = {255, 255, 255, 255};
        world_ypr_text.thickness = 1;
        display_texts.push_back(world_ypr_text);
        text_offset += world_ypr_text.size;

        rm_utils::DisplayText world_ypd_text;
        world_ypd_text.content = "  World YPD: [" + std::to_string(armor.ypd_in_world.x() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypd_in_world.y() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypd_in_world.z()).substr(0, 5) + "] deg";
        world_ypd_text.x         = 20;
        world_ypd_text.y         = text_offset;
        world_ypd_text.size      = 32;
        world_ypd_text.color     = {255, 255, 255, 255};
        world_ypd_text.thickness = 1;
        display_texts.push_back(world_ypd_text);
        text_offset += world_ypd_text.size;

        if (armor.points.size() == 4)
        {
            // 绘制检测到的装甲板轮廓（绿色）
            for (int j = 0; j < 4; j++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(armor.points[j].x);
                line.y1        = static_cast<int>(armor.points[j].y);
                line.x2        = static_cast<int>(armor.points[(j + 1) % 4].x);
                line.y2        = static_cast<int>(armor.points[(j + 1) % 4].y);
                line.color     = {0, 255, 0, 255}; // 绿色：检测结果
                line.thickness = 3;
                display_lines.push_back(line);
            }

            const auto &object_points = (armor.type == ArmorType::BIG) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

            // 根据 ypr_in_world 计算旋转矩阵 R_world2armor
            double yaw   = armor.ypr_in_world.x();
            double pitch = armor.ypr_in_world.y();
            double roll  = armor.ypr_in_world.z();

            Eigen::Matrix3d R_world2armor;
            R_world2armor = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            // 将物体坐标系的点转换到世界坐标系
            std::vector<cv::Point3f> world_corners;
            for (const auto &p_local : object_points)
            {
                // 转换为 Eigen 向量
                Eigen::Vector3d p_local_eigen(p_local.x, p_local.y, p_local.z);

                // 注意顺序：先旋转，后平移
                Eigen::Vector3d p_world_eigen = R_world2armor * p_local_eigen + armor.xyz_in_world;

                world_corners.push_back(cv::Point3f(p_world_eigen.x(), p_world_eigen.y(), p_world_eigen.z()));
            }

            std::vector<cv::Point2f> projected_points = world2pixel(world_corners);

            // 绘制投影点
            if (!projected_points.empty())
            {
                for (int j = 0; j < 4; j++)
                {
                    rm_utils::DisplayLine line;
                    line.x1        = static_cast<int>(projected_points[j].x);
                    line.y1        = static_cast<int>(projected_points[j].y);
                    line.x2        = static_cast<int>(projected_points[(j + 1) % 4].x);
                    line.y2        = static_cast<int>(projected_points[(j + 1) % 4].y);
                    line.color     = {0, 0, 255, 255}; // world2pixel投影结果
                    line.thickness = 2;
                    display_lines.push_back(line);
                }

                // 计算重投影误差
                double reproj_error = 0;
                for (int j = 0; j < 4; j++)
                {
                    float dx = projected_points[j].x - armor.points[j].x;
                    float dy = projected_points[j].y - armor.points[j].y;
                    reproj_error += std::sqrt(dx * dx + dy * dy);
                }
                reproj_error /= 4.0;

                rm_utils::DisplayText error_text;
                error_text.content   = "  Reproj Error: " + std::to_string(reproj_error).substr(0, 5) + " px";
                error_text.x         = 20;
                error_text.y         = text_offset;
                error_text.size      = 32;
                error_text.color     = (reproj_error < 5.0) ? SDL_Color{0, 255, 0, 255} : SDL_Color{0, 0, 255, 255};
                error_text.thickness = 1;
                display_texts.push_back(error_text);
                text_offset += error_text.size;
            }
        }
    }

    display.display_add(window_name, bgr_img, display_texts, display_points, display_lines);
}

} // namespace auto_aim
