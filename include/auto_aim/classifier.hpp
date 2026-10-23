#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "auto_aim/armor.hpp"

namespace auto_aim
{
// 数字识别分类器：ROI(灰度 + 缩放 32x32) -> ONNX(cv::dnn) -> softmax -> argmax 绑定 armor.name
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

private:
  cv::dnn::Net net_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
