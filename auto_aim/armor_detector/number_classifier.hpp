#ifndef ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor_types.hpp"

namespace auto_aim
{

class NumberClassifier
{
  public:
    enum class RejectReason
    {
        PASS,
        MODEL_NOT_LOADED,
        LOW_CONFIDENCE,
        NEGATIVE_CLASS,
        TYPE_MISMATCH
    };

    NumberClassifier(const std::string &model_path, const std::string &label_path, const double threshold,
                     const std::vector<std::string> &ignore_classes = {});

    // 数字识别接口
    RejectReason classify(const cv::Mat &src, Armor &armor);

  private:
    cv::Mat extractNumber(const cv::Mat &src,
                          Armor         &armor) const;       // 从源图像中提取装甲板数字的 ROI 图像
    double                   threshold;       // 置信度阈值
    cv::dnn::Net             net_;            // ONNX 神经网络模型
    std::vector<std::string> class_names_;    // 类别名称列表
    std::vector<std::string> ignore_classes_; // 需要忽略的类别列表
    bool                     model_loaded_;   // 模型是否加载成功
};

} // namespace auto_aim

#endif // ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
