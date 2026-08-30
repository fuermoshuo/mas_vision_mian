#include "number_classifier.hpp"
#include <fstream>

#include "armor_types.hpp"
#include "mas_log.hpp"

namespace auto_aim
{

NumberClassifier::NumberClassifier(const std::string &model_path, const std::string &label_path, const double thre,
                                   const std::vector<std::string> &ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes), model_loaded_(false)
{

    net_ = cv::dnn::readNetFromONNX(model_path); // 加载 ONNX 模型
    if (net_.empty())
    {
        MAS_LOG_ERROR("Failed to load model: {}", model_path);
        model_loaded_ = false;
    }
    else
    {
        MAS_LOG_INFO("Model loaded successfully: {}", model_path);
        model_loaded_ = true;
    }

    std::ifstream label_file(label_path); // 打开标签文件
    if (!label_file.is_open())
    {
        MAS_LOG_ERROR("Failed to open label file: {}", label_path);
        return;
    }

    std::string line;
    while (std::getline(label_file, line)) // 逐行读取标签
    {
        class_names_.push_back(line);
    }
    MAS_LOG_INFO("Loaded {} class names from: {}", class_names_.size(), label_path);
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat &src, Armor &armor) const
{
    // 目标图像尺寸参数
    static const int input_height = 28; // 模型输入高度
    static const int input_width  = 28; // 模型输入宽度

    // 中间处理时的图像尺寸 (稍微大一点，方便裁剪中间数字区域)
    static const int warp_height       = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;

    // 数字 ROI 在中间图像中的尺寸
    static const cv::Size roi_size(20, 28);

    // 使用几何推算获取更稳定的角点，利用灯条中心 + 向量延伸
    // 系数 1.125 来源于物理尺寸：装甲板高度的一半 / 灯条长度 ≈ 0.5 * 126mm / 56mm
    constexpr float k_extend_ratio = 1.125f;

    // 计算左灯条的上下角点
    cv::Point2f tl = armor.left.center - armor.left.top2bottom * k_extend_ratio;
    cv::Point2f bl = armor.left.center + armor.left.top2bottom * k_extend_ratio;

    // 计算右灯条的上下角点
    cv::Point2f tr = armor.right.center - armor.right.top2bottom * k_extend_ratio;
    cv::Point2f br = armor.right.center + armor.right.top2bottom * k_extend_ratio;

    // 3. 构造透视变换的源点和目标点，左下、左上、右上、右下
    cv::Point2f src_vertices[4] = {bl, tl, tr, br};

    // 目标图像的尺寸确定
    int warp_width = (armor.type == ArmorType::SMALL) ? small_armor_width : large_armor_width;

    // 目标点：将推算出的装甲板区域映射到 warp_width x warp_height 的图上
    // 映射后，装甲板上下边缘将贴合图像上下边缘
    cv::Point2f dst_vertices[4] = {
        cv::Point(0, warp_height - 1),             // 左下
        cv::Point(0, 0),                           // 左上
        cv::Point(warp_width - 1, 0),              // 右上
        cv::Point(warp_width - 1, warp_height - 1) // 右下
    };

    // 透视变换
    cv::Mat number_image;
    cv::Mat rotation_matrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 提取中间数字 ROI，变换后的图像包含了左右灯条和中间的数字，我们只需要中间的数字部分
    int roi_x    = (warp_width - roi_size.width) / 2;
    number_image = number_image(cv::Rect(cv::Point(roi_x, 0), roi_size));

    // 预处理：灰度化 + 二值化 + 尺寸调整
    cv::cvtColor(number_image, number_image, cv::COLOR_BGR2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 最终调整为模型输入尺寸 (例如 28x28)
    cv::resize(number_image, number_image, cv::Size(input_width, input_height));

    return number_image;
}

NumberClassifier::RejectReason NumberClassifier::classify(const cv::Mat &src, Armor &armor)
{
    if (!model_loaded_)
    {
        MAS_LOG_WARN("Model not loaded, cannot classify");
        armor.number = "negative";
        return RejectReason::MODEL_NOT_LOADED;
    }

    cv::Mat number_img = extractNumber(src, armor);
    armor.number_img   = number_img;

    // 归一化：使用 convertTo 直接除以 255
    cv::Mat input;
    armor.number_img.convertTo(input, CV_32F, 1.0 / 255.0);
    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);

    cv::Mat outputs = net_.forward();

    // Decode the output
    double    confidence;
    cv::Point class_id_point;
    minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    if (label_id < 0 || label_id >= static_cast<int>(class_names_.size()))
    {
        armor.number = "negative";
        return RejectReason::MODEL_NOT_LOADED;
    }
    armor.number = class_names_[label_id];

    if (armor.confidence < threshold)
    {
        armor.number = "negative";
        return RejectReason::LOW_CONFIDENCE;
    }

    for (const auto &ignore_class : ignore_classes_)
    {
        if (armor.number == ignore_class)
        {
            armor.number = "negative";
            return RejectReason::NEGATIVE_CLASS;
        }
    }

    // 1号为大装甲板，其余数字为小装甲板
    if ((armor.type == ArmorType::SMALL && armor.number == "1") ||
        (armor.type == ArmorType::BIG && (armor.number == "2" || armor.number == "3" || armor.number == "4" ||
                                          armor.number == "5" || armor.number == "sentry" || armor.number == "outpost")))
    {
        armor.number = "negative";
        return RejectReason::TYPE_MISMATCH;
    }

    return RejectReason::PASS;
}

} // namespace auto_aim
