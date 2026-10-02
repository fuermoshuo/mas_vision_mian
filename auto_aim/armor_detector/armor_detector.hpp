#ifndef _ARMOR_DETECTOR_H_
#define _ARMOR_DETECTOR_H_

#include <chrono>
#include <cstddef>
#include <map>
#include <opencv2/core/mat.hpp>
#include <string>

#include "armor_types.hpp"
#include "light_corner_corrector.hpp"
#include "number_classifier.hpp"
#include "plotter.hpp"

namespace auto_aim
{
class ArmorDetector
{
  public:
    ArmorDetector(const std::string &config_path = "config/hikcamera.yaml");
    /**
     * @brief 识别装甲板
     * @param bgr_img 输入图像
     * @param window_name 窗口名称(仅用于debug时显示窗口名称)
     * @return 识别到的装甲板
     */
    std::vector<Armor> ArmorDetect(const cv::Mat &bgr_img, std::string window_name = "") noexcept;

    void ArmorDetector_Set_Color(EnemyColor color) { detect_color_ = color; }

  private:
    /**
     * @brief 寻找灯条
     * @param bin_img 二值化图像
     * @param src_img 输入图像
     * @return std::vector<LightBar>，找到的灯条
     */
    std::vector<LightBar> findLights(const cv::Mat &bin_img, const cv::Mat &src_img) noexcept;
    /**
     * @brief 根据灯条匹配装甲板
     * @param lights 找到的灯条
     * @return std::vector<Armor>，匹配到的装甲板
     */
    std::vector<Armor> findArmors(const std::vector<LightBar> &lights, const cv::Mat &src_img) noexcept;
    /**
     * @brief 显示识别结果，仅在debug下有效
     * @param bgr_img 输入图像
     * @param window_name 窗口名称
     */
    void showResult(const cv::Mat &bgr_img, const cv::Mat &bin_img, std::string window_name = "") const;

    /**
     * @brief 获取所有识别到的数字图像拼接后的图像
     * @return 拼接后的数字图像，如果没有识别到则返回黑色图像
     */
    cv::Mat getAllNumbersImage() const noexcept;

    /**
     * @brief 检查灯条是否存在共用灯条的情况
     * @param i 灯条1
     * @param j 灯条2
     * @param lights 所有灯条
     * @return true 灯条1和灯条2共用灯条
     * @return false 灯条1和灯条2不共用灯条
     */
    bool containLight(const int i, const int j, const std::vector<LightBar> &lights) noexcept;

    std::vector<LightBar> lights_;
    std::vector<Armor>    armors_;

    // 统计detect信息
    struct DetectionStats
    {
        std::size_t contour_count                 = 0;
        std::size_t contour_too_small             = 0;
        std::size_t light_reject_area             = 0;
        std::size_t light_reject_angle            = 0;
        std::size_t light_reject_ratio            = 0;
        std::size_t light_reject_length           = 0;
        std::size_t light_reject_color            = 0;
        std::size_t geometry_light_count          = 0;
        std::size_t color_light_count             = 0;
        std::size_t armor_candidate_count         = 0;
        std::size_t classifier_pass_count         = 0;
        std::size_t classifier_model_error        = 0;
        std::size_t classifier_low_confidence     = 0;
        std::size_t classifier_negative_class     = 0;
        std::size_t classifier_type_mismatch      = 0;
        std::size_t classifier_evaluated_count    = 0;
        std::size_t pair_reject_contain_light     = 0;
        std::size_t pair_reject_ratio             = 0;
        std::size_t pair_reject_side_ratio        = 0;
        std::size_t pair_reject_rectangular_error = 0;
        std::size_t corner_sample_count           = 0;

        bool        reject_candidate_valid             = false;
        bool        reject_candidate_classifier        = false;
        bool        reject_candidate_contain_light     = false;
        int         reject_candidate_failed_gates      = 0;
        int         reject_candidate_left_source_id    = -1;
        int         reject_candidate_right_source_id   = -1;
        double      reject_candidate_distance          = 0.0;
        double      reject_candidate_ratio             = 0.0;
        double      reject_candidate_side_ratio        = 0.0;
        double      reject_candidate_rectangular_error = 0.0;
        double      reject_candidate_confidence        = 0.0;

        std::string reject_candidate_reason     = "none";
        std::string reject_candidate_class_name = "";

        double      classifier_confidence_sum               = 0.0;
        double      classifier_confidence_max               = 0.0;
        double      corner_raw_lightbar_length_sum          = 0.0;
        double      corner_corrected_lightbar_length_sum    = 0.0;
        double      corner_length_scale_sum                 = 0.0;
        double      corner_raw_width_height_ratio_sum       = 0.0;
        double      corner_corrected_width_height_ratio_sum = 0.0;
    } stats_;

    // 缓存图像，避免重复申请内存
    cv::Mat gray_;
    cv::Mat binary_;

    // yaml参数
    bool       debug_;
    bool       plotter_enable_ = false;
    int        binary_thres_;

    EnemyColor detect_color_  = EnemyColor::RED;

    double min_lightbar_ratio_;
    double max_lightbar_ratio_;
    double min_lightbar_length_;
    double max_angle_error_;
    double max_lightbar_area_;
    double min_armor_ratio_;
    double max_armor_ratio_;
    double max_side_ratio_;
    double max_rectangular_error_;

    // 数字识别器
    std::unique_ptr<NumberClassifier> classifier;

    rm_utils::Plotter plotter_;

    // 灯条角点优化器
    LightCornerCorrector light_corner_corrector_;

    // debug下的fps计算参数
    mutable std::map<std::string, double>                                fps_map_;
    mutable std::map<std::string, int>                                   count_map_;
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_;
};

} // namespace auto_aim

#endif // _ARMOR_DETECTOR_H_
