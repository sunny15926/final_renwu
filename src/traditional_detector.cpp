#include "armor_track_ros2/traditional_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace traditional_armor
{

TraditionalDetector::TraditionalDetector(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  // 敌方颜色
  auto enemy_color_str = yaml["enemy_color"].as<std::string>();
  enemy_color_ = enemy_color_str == "red" ? Color::red : Color::blue;

  // 红色 HSV 阈值：红色在色相环上跨越 0°/180° 两端，用两段区间取并集
  red_hue_low_[0] = yaml["red_hue_low"][0].as<int>();
  red_hue_high_[0] = yaml["red_hue_high"][0].as<int>();
  red_hue_low_[1] = yaml["red_hue_low"][1].as<int>();
  red_hue_high_[1] = yaml["red_hue_high"][1].as<int>();
  // 蓝色 HSV 阈值：只用一段区间
  blue_hue_low_ = yaml["blue_hue_low"].as<int>();
  blue_hue_high_ = yaml["blue_hue_high"].as<int>();

  saturation_low_ = yaml["saturation_low"].as<int>();
  saturation_high_ = yaml["saturation_high"].as<int>();
  value_low_ = yaml["value_low"].as<int>();
  value_high_ = yaml["value_high"].as<int>();
  morph_kernel_size_ = yaml["morph_kernel_size"].as<int>();

  // 灯条筛选阈值（角度由 degree 转 rad）
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();

  // 装甲配对判断阈值
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;

  debug_ = yaml["debug"].as<bool>();

  // Perspective warp start
  perspective_debug_ = yaml["perspective_debug"].as<bool>(false);
  perspective_height_ = yaml["perspective_height"].as<int>(100);
  digit_match_threshold_ = yaml["digit_match_threshold"].as<double>(0.6);
  enable_digit_filter_ = yaml["enable_digit_filter"].as<bool>(false);  // 数字过滤总开关

  // CNN 数字识别（OpenCV dnn + onnx）
  digit_model_path_ = yaml["digit_model_path"].as<std::string>("");
  digit_model_input_size_ = yaml["digit_model_input_size"].as<int>(28);
  digit_class_num_ = yaml["digit_class_num"].as<int>(6);
  load_digit_model();
  // Perspective warp end
}

std::vector<Armor> TraditionalDetector::detect(const cv::Mat & bgr_img)
{
  // 1. 图像预处理 + HSV 颜色阈值分割 + 形态学，得到二值图
  auto binary_img = preprocess(bgr_img);
  if (debug_) cv::imshow("binary", binary_img);

  // 2. findContours 提取轮廓 + RotatedRect 筛选灯条
  auto lightbars = extract_lightbars(binary_img);

  // 3. 灯条两两配对组成装甲板
  std::vector<Armor> armors;
  for (std::size_t i = 0; i < lightbars.size(); i++) {
    for (std::size_t j = i + 1; j < lightbars.size(); j++) {
      auto & left = lightbars[i];
      auto & right = lightbars[j];

      // 只配合同色灯条（本模块仅分割敌方颜色，此处为防御性判断）
      if (left.color != right.color) continue;

      auto armor = Armor(left, right);
      if (!check_armor(armor)) continue;

      armors.emplace_back(armor);
    }
  }

  // 4. 去重：两块装甲共用同一灯条时，保留 rectangular_error 更小（更接近矩形）者。
  std::vector<bool> duplicated(armors.size(), false);
  for (std::size_t i = 0; i < armors.size(); i++) {
    for (std::size_t j = i + 1; j < armors.size(); j++) {
      auto & a = armors[i];
      auto & b = armors[j];

      auto share = a.left.id == b.left.id || a.left.id == b.right.id ||
                   a.right.id == b.left.id || a.right.id == b.right.id;
      if (!share) continue;

      if (a.rectangular_error <= b.rectangular_error)
        duplicated[j] = true;
      else
        duplicated[i] = true;
    }
  }

  std::vector<Armor> result;
  for (std::size_t i = 0; i < armors.size(); i++) {
    if (!duplicated[i]) result.emplace_back(armors[i]);
  }

  // 5. CNN 数字识别校验（由总开关 enable_digit_filter_ 控制）
  rejected_armors_.clear();
  if (!enable_digit_filter_) return result;

  std::vector<Armor> valid_result;
  valid_result.reserve(result.size());
  double total_infer_ms = 0.0;
  for (auto & armor : result) {
    cv::Mat warped = warp_armor(bgr_img, armor);  // 透视矫正，得到正对装甲板小图
    double confidence = 0.0;
    std::string digit = "?";
    if (!warped.empty()) {
      auto t0 = std::chrono::steady_clock::now();
      digit = classify_digit(warped, &confidence, &armor.digit_box);
      auto t1 = std::chrono::steady_clock::now();
      total_infer_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    armor.digit = digit;
    armor.match_score = confidence;

    if (digit != "?") {  // 识别到有效数字才保留；"?" 表示失败或置信度不足
      valid_result.emplace_back(armor);
    } else {
      rejected_armors_.emplace_back(armor);
    }
  }

  last_inference_ms_ = total_infer_ms;
  return valid_result;
}

cv::Mat TraditionalDetector::preprocess(const cv::Mat & bgr_img)
{
  // 1. BGR 转 HSV
  cv::cvtColor(bgr_img, hsv_img_, cv::COLOR_BGR2HSV);

  // 2. 颜色阈值分割：只保留敌方颜色
  if (enemy_color_ == Color::red) {
    // 红色跨越色相环 0°/180° 两端，用两段区间取并集
    cv::inRange(
      hsv_img_, cv::Scalar(red_hue_low_[0], saturation_low_, value_low_),
      cv::Scalar(red_hue_high_[0], saturation_high_, value_high_), mask1_);
    cv::inRange(
      hsv_img_, cv::Scalar(red_hue_low_[1], saturation_low_, value_low_),
      cv::Scalar(red_hue_high_[1], saturation_high_, value_high_), mask2_);
    cv::bitwise_or(mask1_, mask2_, mask_);
  } else {
    // 蓝色只用一段区间
    cv::inRange(
      hsv_img_, cv::Scalar(blue_hue_low_, saturation_low_, value_low_),
      cv::Scalar(blue_hue_high_, saturation_high_, value_high_), mask_);
  }

  // 3. 形态学闭运算：先膨胀再腐蚀，填补灯条内部空洞、连接细小断裂
  auto kernel =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size_, morph_kernel_size_));
  cv::dilate(mask_, mask_, kernel);
  cv::erode(mask_, mask_, kernel);

  return mask_;
}

std::vector<Lightbar> TraditionalDetector::extract_lightbars(const cv::Mat & binary_img)
{
  // 提取外轮廓（只取最外层，灯条是实心亮块）
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 每个轮廓求最小外接旋转矩形，并按几何条件筛选灯条
  std::size_t lightbar_id = 0;
  std::vector<Lightbar> lightbars;
  lightbars.reserve(contours.size());
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_lightbar(lightbar)) continue;

    lightbar.color = enemy_color_;
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  // 灯条按中心 x 坐标从左到右排序，便于两两配对
  std::sort(
    lightbars.begin(), lightbars.end(),
    [](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  return lightbars;
}

bool TraditionalDetector::check_lightbar(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

bool TraditionalDetector::check_armor(const Armor & armor) const
{
  auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
  auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
  auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
  return ratio_ok && side_ratio_ok && rectangular_error_ok;
}

// Perspective warp start
cv::Mat TraditionalDetector::warp_armor(const cv::Mat & img, const Armor & armor) const
{
  // 源四角点：armor.points 顺序为 左上、右上、右下、左下
  std::vector<cv::Point2f> src = armor.points;

  float width = static_cast<float>(cv::norm(armor.right.center - armor.left.center));
  float height = std::max(armor.left.length, armor.right.length);
  if (width <= 0.0f || height <= 0.0f) return cv::Mat();

  float scale = static_cast<float>(perspective_height_) / height;
  int dst_w = std::max(1, cvRound(width * scale));
  int dst_h = perspective_height_;

  std::vector<cv::Point2f> dst = {
    cv::Point2f(0.0f, 0.0f),
    cv::Point2f(static_cast<float>(dst_w - 1), 0.0f),
    cv::Point2f(static_cast<float>(dst_w - 1), static_cast<float>(dst_h - 1)),
    cv::Point2f(0.0f, static_cast<float>(dst_h - 1))};

  cv::Mat transform = cv::getPerspectiveTransform(src, dst);
  cv::Mat warped;
  cv::warpPerspective(img, warped, transform, cv::Size(dst_w, dst_h));
  return warped;
}

std::vector<cv::Point2f> TraditionalDetector::map_warped_to_original(
  const Armor & armor, const std::vector<cv::Point2f> & pts) const
{
  std::vector<cv::Point2f> src = armor.points;
  float width = static_cast<float>(cv::norm(armor.right.center - armor.left.center));
  float height = std::max(armor.left.length, armor.right.length);
  if (width <= 0.0f || height <= 0.0f) return pts;

  float scale = static_cast<float>(perspective_height_) / height;
  int dst_w = std::max(1, cvRound(width * scale));
  int dst_h = perspective_height_;

  std::vector<cv::Point2f> dst = {
    cv::Point2f(0.0f, 0.0f),
    cv::Point2f(static_cast<float>(dst_w - 1), 0.0f),
    cv::Point2f(static_cast<float>(dst_w - 1), static_cast<float>(dst_h - 1)),
    cv::Point2f(0.0f, static_cast<float>(dst_h - 1))};

  cv::Mat inv = cv::getPerspectiveTransform(dst, src);
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(pts, out, inv);
  return out;
}

void TraditionalDetector::load_digit_model()
{
  if (digit_model_path_.empty()) {
    std::cerr << "[traditional_detector] 未配置 CNN 数字模型路径（digit_model_path），"
                 "数字识别将全部返回 ?" << std::endl;
    return;
  }
  digit_net_ = cv::dnn::readNetFromONNX(digit_model_path_);
  if (digit_net_.empty()) {
    std::cerr << "[traditional_detector] 加载 CNN 数字模型失败: " << digit_model_path_
              << std::endl;
    return;
  }
  std::cerr << "[traditional_detector] CNN 数字模型加载成功: " << digit_model_path_
            << "（输入 " << digit_model_input_size_ << "x" << digit_model_input_size_
            << " 灰度单通道，" << digit_class_num_ << " 类）" << std::endl;
}

std::string TraditionalDetector::classify_digit(const cv::Mat & warped, double * confidence,
                                                cv::Rect * digit_box_out)
{
  if (confidence) *confidence = 0.0;
  if (digit_box_out) *digit_box_out = cv::Rect();
  if (warped.empty() || digit_net_.empty()) return "?";

  // 1. 裁剪中间数字区域：左右两侧约各 1/4 宽为灯条，数字印刷在中间
  int roi_x = warped.cols / 4;
  int roi_w = warped.cols / 2;
  if (roi_w <= 0) return "?";
  cv::Rect roi(roi_x, 0, roi_w, warped.rows);
  cv::Mat center = warped(roi);

  if (digit_box_out) *digit_box_out = roi;

  // 2. 转灰度
  cv::Mat gray;
  cv::cvtColor(center, gray, cv::COLOR_BGR2GRAY);

  // 3. 归一化到 [0,1] + 缩放到模型输入尺寸，转成 NCHW 单通道 blob
  cv::Mat blob = cv::dnn::blobFromImage(
    gray, 1.0 / 255.0, cv::Size(digit_model_input_size_, digit_model_input_size_),
    cv::Scalar(), false, false);

  // 4. 前向推理
  digit_net_.setInput(blob);
  cv::Mat output = digit_net_.forward();
  if (output.empty()) return "?";

  // 5. 展平成一行，取最大概率对应的类别作为预测数字
  cv::Mat prob = output.reshape(1, 1);
  cv::Point class_id_point;
  double max_prob = 0.0;
  cv::minMaxLoc(prob, nullptr, &max_prob, nullptr, &class_id_point);
  int class_id = class_id_point.x;
  if (confidence) *confidence = max_prob;

  // 6. 置信度不足兜底显示 ?
  if (class_id < 0 || class_id >= digit_class_num_ || max_prob < digit_match_threshold_) {
    return "?";
  }
  return std::to_string(class_id);
}
// Perspective warp end

void TraditionalDetector::draw(cv::Mat & img, const std::vector<Armor> & armors)
{
  for (const auto & armor : armors) {
    cv::line(img, armor.left.top, armor.left.bottom, cv::Scalar(0, 255, 255), 2);
    cv::line(img, armor.right.top, armor.right.bottom, cv::Scalar(0, 255, 255), 2);

    for (int i = 0; i < 4; i++) {
      cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }

    cv::circle(img, armor.center, 3, cv::Scalar(0, 0, 255), -1);
  }
}

}  // namespace traditional_armor
