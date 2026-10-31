#include "armor_track_ros2/traditional_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <sstream>

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
}

std::vector<Armor> TraditionalDetector::detect(const cv::Mat & bgr_img)
{
  // 1. 图像预处理 + HSV 颜色阈值分割 + 形态学，得到二值图
  auto binary_img = preprocess(bgr_img);

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

  // 5. 调试可视化：在原图副本上绘制装甲框 + 叠加 overlay_text_，再把显示副本缩小后 imshow。
  //    仅缩放这份显示副本；送入算法的 bgr_img 保持原始分辨率不变。
  if (debug_) {
    cv::Mat viz = bgr_img.clone();
    draw(viz, result);
    cv::flip(viz, viz, 0);  // 倒置：上下翻转图像内容（灯条/装甲框随图翻转）
    cv::Mat display;
    cv::resize(viz, display, cv::Size(), 0.5, 0.5, cv::INTER_AREA);  // 显示放大到 0.5 倍
    // 叠加数据：绘制在翻转+缩放之后的显示图上，保持正立、固定于左上角。
    if (!overlay_text_.empty()) {
      int base_y = 24;
      const int line_h = 22;
      std::istringstream iss(overlay_text_);
      std::string line;
      while (std::getline(iss, line)) {
        cv::putText(display, line, cv::Point(10, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        base_y += line_h;
      }
    }
    cv::imshow("armor", display);
    cv::waitKey(1);
  }

  return result;
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
