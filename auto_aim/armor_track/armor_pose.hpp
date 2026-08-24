#ifndef _ARMOR_POSE_H_
#define _ARMOR_POSE_H_

#include "armor_types.hpp"
#include <Eigen/Dense> // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

namespace auto_aim
{

class ArmorPose
{
  public:
    /**
     * @brief 构造函数
     * @param config_path 相机配置文件路径
     * @param track_config_path 跟踪配置文件路径
     */
    ArmorPose(const std::string &config_path = "config/hikcamera.yaml", const std::string &track_config_path = "config/auto_aim.yaml");
    ~ArmorPose();

    /**
     * @brief 获取云台坐标系到世界坐标系的旋转矩阵
     * @return Eigen::Matrix3d 云台坐标系到世界坐标系的旋转矩阵
     */
    Eigen::Matrix3d Get_R_gimbal2world() const { return R_gimbal2world_; }

    /**
     * @brief 设置云台坐标系到世界坐标系的旋转矩阵
     * @param q 接收到的imu姿态的四元数
     */
    void set_R_gimbal2world(const Eigen::Quaterniond &q);

    /**
     * @brief 获取装甲板的位姿
     * @param armor 装甲板对象
     * @return PnP 解算和坐标转换是否成功
     */
    bool GetArmorPose(Armor &armor) const;

    /**
     * @brief 获取debug状态
     * @return bool debug状态
     */
    bool isDebug() const { return debug_; }

    /**
     * @brief 显示位姿debug信息
     * @param armors 装甲板列表
     * @param bgr_img 图像
     * @param window_name 窗口名称
     */
    void showPoseDebug(const std::vector<Armor> &armors, const cv::Mat &bgr_img, const std::string &window_name) const;

    /**
     * @brief 世界坐标系到像素坐标系的转换，用于debug时绘制
     * @param worldPoints 世界坐标系中的点集
     * @return std::vector<cv::Point2f> 像素坐标系中的点集
     */
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> &worldPoints) const;

    /**
     * @brief 重投影装甲板到图像平面
     * @param xyz_in_world 装甲板中心世界坐标
     * @param yaw 装甲板yaw角
     * @param type 装甲板类型
     * @param name 目标名称
     * @return 图像平面上的四个角点
     */
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &xyz_in_world, double yaw, ArmorType type, std::string name) const;

  private:
    /**
     * @brief 优化装甲板yaw角度（搜索最优yaw）
     */
    void optimize_yaw(Armor &armor) const;
    /**
     * @brief 计算装甲板重投影误差
     */
    double armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const;
    /**
     * @brief 计算outpost重投影误差
     */
    double outpost_reprojection_error(Armor armor, const double &pitch);
    /**
     * @brief 计算重投影误差
     */
    double calculateReprojectionError(const std::vector<cv::Point2f> &image_points, const cv::Mat &rvec, const cv::Mat &tvec,
                                      const std::vector<cv::Point3f> &object_points) const;

    cv::Mat         camera_matrix_;    // 相机内参矩阵
    cv::Mat         distort_coeffs_;   // 畸变系数
    Eigen::Matrix3d R_gimbal2imubody_; // 云台到IMU本体坐标系的旋转
    Eigen::Matrix3d R_camera2gimbal_;  // 相机到云台坐标系的旋转
    Eigen::Vector3d t_camera2gimbal_;  // 相机到云台坐标系的平移
    Eigen::Matrix3d R_gimbal2world_;   // 云台到世界坐标系的旋转

    bool debug_ = false; // debug开关
};
} // namespace auto_aim

#endif // _ARMOR_POSE_H_
