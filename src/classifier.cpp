#include "auto_aim/classifier.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace auto_aim
{
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto model = yaml["classify_model"].as<std::string>();

  // 相对路径按 config 文件所在目录解析（ros2 launch 时 cwd 不一定是包目录）
  if (!model.empty() && model[0] != '/') {
    auto slash = config_path.find_last_of('/');
    auto dir = (slash == std::string::npos) ? std::string(".") : config_path.substr(0, slash);
    model = dir + "/" + model;
  }

  net_ = cv::dnn::readNetFromONNX(model);
}

void Classifier::classify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  // ① BGR -> 灰度
  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // ② 等比缩放到 32x32（短边贴齐，不足部分补 0）
  cv::Mat input(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = 32.0 / gray.cols;
  auto y_scale = 32.0 / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);
  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }
  cv::resize(gray, input(cv::Rect(0, 0, w, h)), {w, h});

  // ③ 归一化到 [0,1] + 推理
  auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());
  net_.setInput(blob);
  cv::Mat outputs = net_.forward();  // [1, N] logits

  // ④ softmax 归一化
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  // ⑤ argmax 取最大类别 -> 绑定编号
  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace auto_aim
