#include "armor_detector.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <fmt/format.h>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_types.hpp"
#include "display.hpp"
#include "mas_log.hpp"
#include "yaml-cpp/yaml.h"

namespace auto_aim
{
namespace
{
// 单像素颜色差异门限
constexpr int TARGET_COLOR_MARGIN = 20;

enum class ColorMatch
{
    OPPOSITE,           // 我方 
    NEUTRAL,            // 中性
    TARGET,             // 敌方   
};

constexpr ColorMatch classifyTargetColor(int diff_sum, int valid_count)
{
    if (valid_count <= 0) return ColorMatch::NEUTRAL;
    if (diff_sum < -TARGET_COLOR_MARGIN * valid_count) return ColorMatch::OPPOSITE;
    if (diff_sum > TARGET_COLOR_MARGIN * valid_count) return ColorMatch::TARGET;
    return ColorMatch::NEUTRAL;
}

// 断言提前编译
static_assert(classifyTargetColor(0, 0) == ColorMatch::NEUTRAL);
static_assert(classifyTargetColor(-21 * 27, 27) == ColorMatch::OPPOSITE);
static_assert(classifyTargetColor(0, 27) == ColorMatch::NEUTRAL);
static_assert(classifyTargetColor(21 * 27, 27) == ColorMatch::TARGET);
} // namespace

ArmorDetector::ArmorDetector(const std::string &config_path)
{
    // 禁用 OpenCV 多线程，避免在处理小任务时产生过多的线程切换和 yielding 开销
    cv::setNumThreads(1);

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
        if (config["auto_aim"] && config["auto_aim"]["armor_detector"])
        {
            YAML::Node armor_detector = config["auto_aim"]["armor_detector"];
            // 读取基础参数
            debug_ = armor_detector["debug"].as<bool>(false);
            plotter_enable_ = armor_detector["plotter_enable"].as<bool>(false);
            // 读取二值化参数
            binary_thres_ = armor_detector["binary_thres"].as<int>(90);
            // 读取颜色参数
            std::string color_str = armor_detector["detect_color"].as<std::string>("RED");
            if (color_str == "blue" || color_str == "BLUE")
            {
                detect_color_ = EnemyColor::BLUE;
            }
            else
            {
                detect_color_ = EnemyColor::RED;
            }
            // 读取参数
            YAML::Node lights_params = armor_detector["lights_params"];
            min_lightbar_ratio_      = lights_params["min_lightbar_ratio"].as<double>(0.1);
            max_lightbar_ratio_      = lights_params["max_lightbar_ratio"].as<double>(0.5);
            min_lightbar_length_     = lights_params["min_lightbar_length"].as<double>(8.0);
            max_angle_error_         = lights_params["max_angle_error"].as<double>(20.0) / 57.3; // rad
            max_lightbar_area_       = lights_params["max_lightbar_area"].as<double>(10000.0);

            YAML::Node armors_params = armor_detector["armors_params"];
            min_armor_ratio_         = armors_params["min_armor_ratio"].as<double>(0.7);
            max_armor_ratio_         = armors_params["max_armor_ratio"].as<double>(0.8);
            max_side_ratio_          = armors_params["max_side_ratio"].as<double>(3.2);
            max_rectangular_error_   = armors_params["max_rectangular_error"].as<double>(3.2) / 57.3; // rad

            // 读取数字识别参数
            if (armor_detector["number_params"])
            {
                YAML::Node  number_params = armor_detector["number_params"];
                std::string model_path    = number_params["model_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/lenet.onnx");
                std::string label_path    = number_params["label_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/label.txt");
                double      confidence    = number_params["classifier_threshold"].as<double>(0.7);

                std::vector<std::string> ignore_classes;
                // 读取忽略类别
                if (number_params["ignore_classes"])
                {
                    ignore_classes.clear();
                    for (const auto &cls : number_params["ignore_classes"])
                    {
                        ignore_classes.push_back(cls.as<std::string>());
                    }
                }
                // 创建数字识别器
                classifier = std::make_unique<NumberClassifier>(model_path, label_path, confidence, ignore_classes);
            }
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

std::vector<Armor> ArmorDetector::ArmorDetect(const cv::Mat &bgr_img, std::string window_name) noexcept
{
    stats_ = {};

    if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0)
    {
        MAS_LOG_WARN("Invalid input image");
        return armors_;
    }

    if (bgr_img.channels() == 3)
    {
        // 使用 create 复用已分配的内存
        gray_.create(bgr_img.size(), CV_8UC1);
        cv::cvtColor(bgr_img, gray_, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray_ = bgr_img;
    }
    // 使用 create 复用已分配的内存
    binary_.create(gray_.size(), CV_8UC1);
    cv::threshold(gray_, binary_, binary_thres_, 255, cv::THRESH_BINARY);

    // 查找灯条
    lights_ = findLights(binary_, bgr_img);

    // 查找装甲板
    armors_ = findArmors(lights_, bgr_img);

    if (plotter_enable_)
    {
        const char *stage = "detected";
        if (stats_.contour_count == 0)              stage = "no_contour";
        else if (stats_.geometry_light_count == 0)  stage = "light_geometry";
        else if (stats_.color_light_count < 2)      stage = "light_color_or_count";
        else if (stats_.armor_candidate_count == 0) stage = "armor_pair";
        else if (stats_.classifier_pass_count == 0) stage = "classifier";

        nlohmann::json data;
        data["detector"]["stage"]                      = stage;
        data["detector"]["contour_count"]              = stats_.contour_count;
        data["detector"]["contour_too_small"]          = stats_.contour_too_small;
        data["detector"]["light_reject_area"]          = stats_.light_reject_area;
        data["detector"]["light_reject_angle"]         = stats_.light_reject_angle;
        data["detector"]["light_reject_ratio"]         = stats_.light_reject_ratio;
        data["detector"]["light_reject_length"]        = stats_.light_reject_length;
        data["detector"]["light_reject_color"]         = stats_.light_reject_color;
        data["detector"]["geometry_light_count"]       = stats_.geometry_light_count;
        data["detector"]["color_light_count"]          = stats_.color_light_count;
        data["detector"]["armor_candidate_count"]      = stats_.armor_candidate_count;
        data["detector"]["classifier_pass_count"]      = stats_.classifier_pass_count;
        data["detector"]["classifier_model_error"]     = stats_.classifier_model_error;
        data["detector"]["classifier_low_confidence"]  = stats_.classifier_low_confidence;
        data["detector"]["classifier_negative_class"]  = stats_.classifier_negative_class;
        data["detector"]["classifier_type_mismatch"]   = stats_.classifier_type_mismatch;
        data["detector"]["classifier_confidence_mean"] = stats_.classifier_evaluated_count == 0
                                                               ? 0.0
                                                               : stats_.classifier_confidence_sum / stats_.classifier_evaluated_count;
        data["detector"]["classifier_confidence_max"]     = stats_.classifier_confidence_max;
        data["detector"]["pair_reject_contain_light"]     = stats_.pair_reject_contain_light;
        data["detector"]["pair_reject_ratio"]             = stats_.pair_reject_ratio;
        data["detector"]["pair_reject_side_ratio"]        = stats_.pair_reject_side_ratio;
        data["detector"]["pair_reject_rectangular_error"] = stats_.pair_reject_rectangular_error;
        plotter_.plot(data);
    }

    // 显示结果
    if (debug_)
    {
        showResult(bgr_img, binary_, window_name);
    }

    return armors_;
}

std::vector<LightBar> ArmorDetector::findLights(const cv::Mat &bin_img, const cv::Mat &src_img) noexcept
{
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    stats_.contour_count = contours.size();

    std::vector<LightBar> lightbars;
    lightbars.reserve(20);

    for (const auto &contour : contours)
    {
        // 初步过滤
        if (contour.size() < 4) 
        {
            stats_.contour_too_small++;
            continue;
        }
        // 计算轮廓面积
        double contour_area = cv::contourArea(contour);

        // 面积过滤
        if (contour_area > max_lightbar_area_)
        {
            stats_.light_reject_area++;
            continue;
        } 

        // 旋转矩形
        auto r_rect = cv::minAreaRect(contour);

        // 初步创建灯条
        auto lightbar = LightBar(r_rect);

        // 根据几何条件过滤
        if (lightbar.angle_error > max_angle_error_)
        {
            stats_.light_reject_angle++;
            continue;
        }
        if (lightbar.ratio < min_lightbar_ratio_ || lightbar.ratio > max_lightbar_ratio_)
        {
            stats_.light_reject_ratio++;
            continue;
        }
        if (lightbar.length < min_lightbar_length_)
        {
            stats_.light_reject_length++;
            continue;
        }
        stats_.geometry_light_count++;

        // 颜色识别：在灯条区域内采样，避免单条中心线被反光或运动模糊误判
        if (!src_img.empty() && src_img.channels() == 3)
        {
            const int      cols      = src_img.cols;
            const int      rows      = src_img.rows;
            const uint8_t *pixel_ptr = src_img.data;
            const size_t   stride    = src_img.step;
            const bool     is_red    = (detect_color_ == EnemyColor::RED);

            constexpr int  N_LENGTH_SAMPLES = 10;
            constexpr int  N_WIDTH_SAMPLES  = 5;
            cv::Point2f    axis             = lightbar.bottom - lightbar.top;
            const float    axis_length      = cv::norm(axis);
            if (axis_length < 1e-3f)
            {
                continue;
            }
            axis *= 1.0f / axis_length;
            const cv::Point2f perpendicular(-axis.y, axis.x); // axis 的垂直方向单位向量
            int               diff_sum    = 0;
            int               valid_count = 0;

            for (int i = 1; i < N_LENGTH_SAMPLES; i++)
            {
                const float length_ratio = static_cast<float>(i) / N_LENGTH_SAMPLES;
                const cv::Point2f center = lightbar.top + (lightbar.bottom - lightbar.top) * length_ratio;
                for (int j = 0; j < N_WIDTH_SAMPLES; ++j)
                {
                    // 覆盖灯条核心及两侧光晕，避免只采到过曝白芯。
                    const float width_ratio = (static_cast<float>(j) - 2.0f) / 2.0f;
                    const cv::Point2f pt = center + perpendicular * static_cast<float>(lightbar.width * width_ratio * 0.75);
                    const int px = static_cast<int>(pt.x);
                    const int py = static_cast<int>(pt.y);
                    if (px < 0 || px >= cols || py < 0 || py >= rows) continue;

                    const uint8_t *pixel = pixel_ptr + py * stride + px * 3;
                    const int brightness = std::max({static_cast<int>(pixel[0]), static_cast<int>(pixel[1]), static_cast<int>(pixel[2])});
                    const int color_diff = is_red ? (pixel[2] - pixel[0]) : (pixel[0] - pixel[2]);
                    // 暗部噪声和白色饱和点都不携带可靠红蓝信息。
                    if (brightness < 40 || (color_diff >= -TARGET_COLOR_MARGIN && color_diff <= TARGET_COLOR_MARGIN)) continue;
                    diff_sum += color_diff;
                    valid_count++;
                }
            }

            const ColorMatch color_match = classifyTargetColor(diff_sum, valid_count);
            if (color_match == ColorMatch::OPPOSITE)
            {
                stats_.light_reject_color++;
                continue;
            }
            lightbar.target_color_confirmed = color_match == ColorMatch::TARGET;
            lightbar.color = is_red ? EnemyColor::RED : EnemyColor::BLUE;
        }

        stats_.color_light_count++;
        lightbars.emplace_back(lightbar);
    }

    // 排序
    std::sort(lightbars.begin(), lightbars.end(), [](const LightBar &a, const LightBar &b) { return a.center.x < b.center.x; });

    return lightbars;
}

std::vector<Armor> ArmorDetector::findArmors(const std::vector<LightBar> &lights, const cv::Mat &src_img) noexcept
{
    std::vector<Armor> armors;
    std::vector<Armor> candidates; // 预筛选的候选装甲板

    // 几何特征筛选，收集候选装甲板
    for (auto left = lights.begin(); left != lights.end(); left++)
    {
        for (auto right = left + 1; right != lights.end(); right++)
        {
            if (left->color != right->color) continue;
            // 中性灯条可以补全一根过曝灯条，但两根都无法确认颜色时不能组成装甲板。
            if (!left->target_color_confirmed && !right->target_color_confirmed)
            {
                continue;
            }
            // 检查是否存在共用灯条的情况
            if (containLight(left - lights.begin(), right - lights.begin(), lights))
            {
                stats_.pair_reject_contain_light++;
                continue;
            }

            auto armor = Armor(*left, *right);

            // 几何特征判断
            if (armor.ratio < min_armor_ratio_ || armor.ratio > max_armor_ratio_)
            {
                stats_.pair_reject_ratio++;
                continue;
            }
            if (armor.side_ratio > max_side_ratio_)
            {
                stats_.pair_reject_side_ratio++;
                continue;
            }
            if (armor.rectangular_error > max_rectangular_error_)
            {
                stats_.pair_reject_rectangular_error++;
                continue;
            }

            // 装甲板类型判断
            if (armor.ratio > 3.0)
            {
                armor.type = ArmorType::BIG;
            }
            else
            {
                armor.type = ArmorType::SMALL; // 默认小装甲板
            }

            candidates.emplace_back(armor);
        }
    }

    stats_.armor_candidate_count = candidates.size();

    for (auto &armor : candidates)
    {
        // 数字识别
        const auto reject_reason = classifier->classify(src_img, armor);
        if (reject_reason != NumberClassifier::RejectReason::MODEL_NOT_LOADED)
        {
            stats_.classifier_evaluated_count++;
            stats_.classifier_confidence_sum += armor.confidence;
            stats_.classifier_confidence_max = std::max(stats_.classifier_confidence_max, static_cast<double>(armor.confidence));
        }
        switch (reject_reason)
        {
        case NumberClassifier::RejectReason::MODEL_NOT_LOADED:
            stats_.classifier_model_error++;
            break;
        case NumberClassifier::RejectReason::LOW_CONFIDENCE:
            stats_.classifier_low_confidence++;
            break;
        case NumberClassifier::RejectReason::NEGATIVE_CLASS:
            stats_.classifier_negative_class++;
            break;
        case NumberClassifier::RejectReason::TYPE_MISMATCH:
            stats_.classifier_type_mismatch++;
            break;
        case NumberClassifier::RejectReason::PASS:
            break;
        }
        if (reject_reason != NumberClassifier::RejectReason::PASS) continue;

        // 角点优化
        light_corner_corrector_.correctCorners(armor, gray_);

        armors.emplace_back(armor);
    }
    stats_.classifier_pass_count = armors.size();

    return armors;
}

bool ArmorDetector::containLight(const int i, const int j, const std::vector<LightBar> &lights) noexcept
{
    const LightBar &light_1       = lights[i], light_2 = lights[j];
    double          avg_length    = (light_1.length + light_2.length) / 2.0;
    double          avg_width     = (light_1.width + light_2.width) / 2.0;

    const std::vector<cv::Point2f> armor_polygon{light_1.top, light_2.top, light_2.bottom, light_1.bottom};
    // 仅检查这两个灯条之间的灯条
    for (int k = i + 1; k < j; k++)
    {
        const LightBar &test_light = lights[k];

        // 防止数字干扰
        if (test_light.width > 2 * avg_width)
        {
            continue;
        }
        // 防止红点准星或弹丸干扰
        if (test_light.length < 0.5 * avg_length)
        {
            continue;
        }

        if (cv::pointPolygonTest(armor_polygon, test_light.top, false) >= 0 ||
            cv::pointPolygonTest(armor_polygon, test_light.bottom, false) >= 0 ||
            cv::pointPolygonTest(armor_polygon, test_light.center, false) >= 0)
        {
            return true;
        }
    }
    return false;
}

void ArmorDetector::showResult(const cv::Mat &bgr_img, const cv::Mat &bin_img, std::string window_name) const
{
    if (!debug_) return;

    // 检查输入图像是否有效
    if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0)
    {
        MAS_LOG_WARN("Invalid input input image for showResult");
        return;
    }

    // 获取显示图像指针
    auto &display = rm_utils::Display::getInstance();

    // 计算 FPS
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

    std::vector<rm_utils::DisplayPoint> display_points;
    std::vector<rm_utils::DisplayLine>  display_lines;
    std::vector<rm_utils::DisplayText>  display_texts;

    // 绘制灯条等信息
    for (const auto &light : lights_)
    {
        // 灯条边框
        cv::Point2f vertices[4];
        light.rotated_rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            rm_utils::DisplayLine line;
            line.x1        = static_cast<int>(vertices[i].x);
            line.y1        = static_cast<int>(vertices[i].y);
            line.x2        = static_cast<int>(vertices[(i + 1) % 4].x);
            line.y2        = static_cast<int>(vertices[(i + 1) % 4].y);
            line.color     = {0, 255, 0, 255}; // 绿色 (RGBA)
            line.thickness = 2;
            display_lines.push_back(line);
        }

        // 灯条中心点
        rm_utils::DisplayPoint center_p;
        center_p.x     = static_cast<int>(light.center.x);
        center_p.y     = static_cast<int>(light.center.y);
        center_p.color = {255, 0, 0, 255}; // 红色 (RGBA)
        center_p.size  = 2;
        display_points.push_back(center_p);
    }

    // 绘制装甲板
    for (const auto &armor : armors_)
    {
        // 绘制左灯条角点
        rm_utils::DisplayPoint left_top;
        left_top.x     = static_cast<int>(armor.points[0].x);
        left_top.y     = static_cast<int>(armor.points[0].y);
        left_top.color = {0, 255, 255, 255}; // 青色 (RGBA)
        left_top.size  = 10;
        display_points.push_back(left_top);

        rm_utils::DisplayPoint left_bottom;
        left_bottom.x     = static_cast<int>(armor.points[3].x);
        left_bottom.y     = static_cast<int>(armor.points[3].y);
        left_bottom.color = {0, 255, 255, 255}; // 青色 (RGBA)
        left_bottom.size  = 10;
        display_points.push_back(left_bottom);

        // 绘制右灯条角点
        rm_utils::DisplayPoint right_top;
        right_top.x     = static_cast<int>(armor.points[1].x);
        right_top.y     = static_cast<int>(armor.points[1].y);
        right_top.color = {0, 255, 255, 255}; // 青色 (RGBA)
        right_top.size  = 10;
        display_points.push_back(right_top);

        rm_utils::DisplayPoint right_bottom;
        right_bottom.x     = static_cast<int>(armor.points[2].x);
        right_bottom.y     = static_cast<int>(armor.points[2].y);
        right_bottom.color = {0, 255, 255, 255}; // 青色 (RGBA)
        right_bottom.size  = 10;
        display_points.push_back(right_bottom);

        // 装甲板四边形（使用四个角点）
        if (armor.points.size() == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(armor.points[i].x);
                line.y1        = static_cast<int>(armor.points[i].y);
                line.x2        = static_cast<int>(armor.points[(i + 1) % 4].x);
                line.y2        = static_cast<int>(armor.points[(i + 1) % 4].y);
                line.color     = {255, 255, 0, 255}; // 黄色 (RGBA)
                line.thickness = 3;
                display_lines.push_back(line);
            }
        }

        // 装甲板中心点
        rm_utils::DisplayPoint center_p;
        center_p.x     = static_cast<int>(armor.center.x);
        center_p.y     = static_cast<int>(armor.center.y);
        center_p.color = {0, 255, 255, 255}; // 青色 (RGBA)
        center_p.size  = 20;
        display_points.push_back(center_p);

        // 装甲板调试信息文本
        rm_utils::DisplayText armor_info;
        armor_info.content   = fmt::format("{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio, armor.rectangular_error * 57.3,
                                           armor.confidence, armor.number, ARMOR_TYPES[armor.type]);
        armor_info.x         = static_cast<int>(armor.center.x) + 25;
        armor_info.y         = static_cast<int>(armor.center.y);
        armor_info.size      = 24;
        armor_info.color     = {255, 255, 0, 255};
        armor_info.thickness = 1;
        display_texts.push_back(armor_info);
    }

    // 文本显示
    rm_utils::DisplayText stats;
    stats.content =
        "FPS: " + std::to_string(static_cast<int>(fps)) + " Armors: " + std::to_string(armors_.size()) + " Lights: " + std::to_string(lights_.size());
    stats.x         = 20;
    stats.y         = 20;
    stats.size      = 24;
    stats.color     = {255, 255, 0, 255}; // 黄色 (RGBA)
    stats.thickness = 2;
    display_texts.push_back(stats);

    // 图像显示
    std::string bin_window_name = window_name + "_binary";
    display.display_add(bin_window_name, bin_img);

    // 显示拼接后的数字图像
    cv::Mat all_numbers_img = getAllNumbersImage();
    if (!all_numbers_img.empty())
    {
        cv::Mat numbers_display;
        cv::resize(all_numbers_img, numbers_display, cv::Size(140, all_numbers_img.rows * 5), 0, 0, cv::INTER_NEAREST);
        std::string numbers_window_name = window_name + "_numbers";
        display.display_add(numbers_window_name, numbers_display);
    }

    rm_utils::Display::getInstance().display_add(window_name, bgr_img, display_texts, display_points, display_lines);
}

cv::Mat ArmorDetector::getAllNumbersImage() const noexcept
{
    if (armors_.empty())
    {
        return cv::Mat(cv::Size(20, 28), CV_8UC1, cv::Scalar(0));
    }
    else
    {
        std::vector<cv::Mat> number_imgs;
        number_imgs.reserve(armors_.size());

        for (const auto &armor : armors_)
        {
            if (!armor.number_img.empty())
            {
                number_imgs.emplace_back(armor.number_img);
            }
        }

        if (number_imgs.empty())
        {
            return cv::Mat(cv::Size(20, 28), CV_8UC1, cv::Scalar(0));
        }

        cv::Mat all_num_img;
        cv::vconcat(number_imgs, all_num_img);

        return all_num_img;
    }
}
} // namespace auto_aim
