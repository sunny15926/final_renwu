#include "armor_track_ros2/traditional_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>

#include "DigitalRecognition.h"

namespace traditional_armor
{

namespace {
// 把 img 等比缩放放入固定尺寸画布（黑底居中），使 imshow 窗口尺寸固定、不随内容跳动。
cv::Mat letterbox(const cv::Mat & img, int canvas_w, int canvas_h)
{
  if (img.empty()) return cv::Mat();
  const double scale = std::min(
    static_cast<double>(canvas_w) / img.cols, static_cast<double>(canvas_h) / img.rows);
  cv::Mat resized;
  cv::resize(img, resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
  cv::Mat canvas(canvas_h, canvas_w, img.type(), cv::Scalar::all(0));
  const int x = (canvas_w - resized.cols) / 2;
  const int y = (canvas_h - resized.rows) / 2;
  resized.copyTo(canvas(cv::Rect(x, y, resized.cols, resized.rows)));
  return canvas;
}

// 固定画布尺寸（像素）
constexpr int kVizWarpSize = 480;
constexpr int kVizDigitSize = 280;
}  // namespace

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
  digit_viz_stable_ = yaml["digit_viz_stable"].as<bool>(true);  // 稳定可视化：固定画布 + 主目标锁定 + 上一帧保持
  perspective_height_ = yaml["perspective_height"].as<int>(100);
  digit_extend_ratio_ = yaml["digit_extend_ratio"].as<double>(0.5);  // 数字裁剪沿灯条轴线向两头各延伸 1/2 个灯条长度
  digit_match_threshold_ = yaml["digit_match_threshold"].as<double>(0.6);
  enable_digit_filter_ = yaml["enable_digit_filter"].as<bool>(false);  // 数字过滤总开关
  digit_collect_dir_ = yaml["digit_collect_dir"].as<std::string>("");  // 数字样本采集目录（空=不采集）

  // 数字识别（ONNX Runtime）
  digit_model_path_ = yaml["digit_model_path"].as<std::string>("");
  digit_model_input_size_ = yaml["digit_model_input_size"].as<int>(28);
  digit_class_num_ = yaml["digit_class_num"].as<int>(6);
  load_digit_model();
  // Perspective warp end
}

// 析构函数在 .cpp 中定义：此处 DigitalRecognition 已完整，可正确析构 unique_ptr
TraditionalDetector::~TraditionalDetector() = default;

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

  // 5. 数字识别：透视矫正 -> ONNX Runtime 推理，结果打印并写入 armor 供可视化
  rejected_armors_.clear();
  std::vector<Armor> valid_result;
  valid_result.reserve(result.size());
  double total_infer_ms = 0.0;

  // 可视化目标选择：稳定模式优先延续上一帧锁存目标，否则取面积最大者；传统模式取第一块。
  Armor * viz_target = nullptr;
  cv::Mat viz_warp, viz_digit;
  double viz_target_area = -1.0;
  double viz_best_dist = std::numeric_limits<double>::max();

  for (auto & armor : result) {
    cv::Mat warped = warp_armor(bgr_img, armor);  // 透视矫正，得到正对装甲板小图
    double confidence = 0.0;
    std::string digit = "?";
    cv::Mat digit_out;
    if (!warped.empty() && digit_recognizer_) {
      auto t0 = std::chrono::steady_clock::now();
      digit = classify_digit(warped, &confidence, &armor.digit_box, &digit_out);
      auto t1 = std::chrono::steady_clock::now();
      total_infer_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

      // 识别结果只打印 + 画图可视化，不参与串口发包
      std::cout << "[traditional_detector] 装甲数字: " << digit
                << " (置信度 " << confidence << ")" << std::endl;

      // 面积近似 = 灯条间距 × 灯条长（像素），用于「主目标」选择
      const double area = cv::norm(armor.right.center - armor.left.center) *
                          std::max(armor.left.length, armor.right.length);
      const double dist = viz_valid_ ? cv::norm(armor.center - viz_center_) : 0.0;
      const bool same = viz_valid_ && dist <= viz_radius_;

      if (digit_viz_stable_) {
        // 稳定模式：优先延续锁存目标（取中心距离最近者），否则取面积最大者
        if (same && dist < viz_best_dist) {
          viz_best_dist = dist;
          viz_target = &armor;
          viz_warp = warped;
          viz_digit = digit_out;
        } else if (!same && area > viz_target_area) {
          viz_target_area = area;
          viz_target = &armor;
          viz_warp = warped;
          viz_digit = digit_out;
        }
      } else if (viz_target == nullptr) {
        viz_target = &armor;
        viz_warp = warped;
        viz_digit = digit_out;
      }
    }
    armor.digit = digit;
    armor.match_score = confidence;

    // 仅当 enable_digit_filter_ 开启时才按数字过滤；否则保留全部候选（数字仅用于可视化）
    if (!enable_digit_filter_ || digit != "?") {
      valid_result.emplace_back(armor);
    } else {
      rejected_armors_.emplace_back(armor);
    }
  }

  // 更新/保持可视化锁存状态
  if (digit_viz_stable_) {
    if (viz_target != nullptr) {
      viz_center_ = viz_target->center;
      viz_radius_ =
        std::max(150.0, cv::norm(viz_target->right.center - viz_target->left.center) * 1.5);
      viz_valid_ = true;
      if (!viz_warp.empty()) last_warp_ = viz_warp;
      if (!viz_digit.empty()) last_digit_ = viz_digit;  // 数字未识别出时保持上一帧，避免跳变
    }
    // 无目标：保持 last_warp_ / last_digit_ 不变（上一帧保持）
  } else {
    last_warp_ = viz_warp;
    last_digit_ = viz_digit;
  }

  // 调试可视化：把装甲框与数字画到原图并显示（perspective_debug_ 开关）
  if (perspective_debug_) {
    cv::Mat viz = bgr_img.clone();
    draw(viz, valid_result);
    // 左上角叠加文本（如 EKF 估计结果，由外层 auto_aim_node 每帧写入）
    if (!overlay_text_.empty()) {
      int base_y = 24;
      const int line_h = 22;
      std::istringstream iss(overlay_text_);
      std::string line;
      while (std::getline(iss, line)) {
        cv::putText(viz, line, cv::Point(10, base_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        base_y += line_h;
      }
    }
    cv::imshow("armor_digit", viz);

    // 额外显示「透视矫正后的装甲板」与「模型实际看到的二值化数字」，便于判断数字是否可见
    if (!last_warp_.empty()) {
      cv::Mat w;
      cv::cvtColor(last_warp_, w, cv::COLOR_BGR2GRAY);
      if (digit_viz_stable_) {
        w = letterbox(w, kVizWarpSize, kVizWarpSize);  // 固定画布，窗口尺寸不跳
      } else {
        cv::resize(w, w, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST);
      }
      cv::imshow("armor_warp", w);
    }
    if (!last_digit_.empty()) {
      cv::Mat d;
      if (digit_viz_stable_) {
        d = letterbox(last_digit_, kVizDigitSize, kVizDigitSize);  // 固定画布
      } else {
        cv::resize(last_digit_, d, cv::Size(), 10.0, 10.0, cv::INTER_NEAREST);
      }
      cv::imshow("armor_digit_crop", d);
    }

    // 按键：'0'~'5' 保存当前数字、's' 保存哨兵（需在 yaml 配置 digit_collect_dir）
    int key = cv::waitKey(1) & 0xFF;  // 刷新 HighGUI 窗口，否则画面不更新
    handle_digit_collect(key, last_digit_);
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
  // 源四角点：armor.points 顺序为 左上、右上、右下、左下（紧贴灯条两端）。
  // 沿灯条轴线向两头各延伸 digit_extend_ratio_ 个灯条长度，扩大裁剪范围，
  // 使数字印刷区略超出灯条两端时仍能被完整纳入。
  std::vector<cv::Point2f> src = armor.points;
  if (digit_extend_ratio_ > 0.0) {
    const cv::Point2f ext_l = armor.left.top2bottom * digit_extend_ratio_;
    const cv::Point2f ext_r = armor.right.top2bottom * digit_extend_ratio_;
    src[0] -= ext_l;  // 左上，向上延伸
    src[3] += ext_l;  // 左下，向下延伸
    src[1] -= ext_r;  // 右上，向上延伸
    src[2] += ext_r;  // 右下，向下延伸
  }

  float width = static_cast<float>(cv::norm(armor.right.center - armor.left.center));
  float height = std::max(armor.left.length, armor.right.length);
  if (width <= 0.0f || height <= 0.0f) return cv::Mat();

  float scale = static_cast<float>(perspective_height_) / height;
  int dst_w = std::max(1, cvRound(width * scale));
  // 目标高度按延伸比例同步放大，保持灯条仍映射为 perspective_height_ 像素（数字不被压缩）
  int dst_h = std::max(1, cvRound(perspective_height_ * (1.0 + 2.0 * digit_extend_ratio_)));

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
    std::cerr << "[traditional_detector] 未配置数字模型路径（digit_model_path），"
                 "数字识别将全部返回 ?" << std::endl;
    return;
  }
  try {
    digit_recognizer_ = std::make_unique<DigitalRecognition>(digit_model_path_);
    std::cerr << "[traditional_detector] ONNX Runtime 数字模型加载成功: " << digit_model_path_
              << "（输入 " << digit_model_input_size_ << "x" << digit_model_input_size_
              << " 灰度单通道，" << digit_class_num_ << " 类 + 哨兵）" << std::endl;
  } catch (const std::exception & e) {
    digit_recognizer_.reset();
    std::cerr << "[traditional_detector] 加载 ONNX Runtime 数字模型失败: " << e.what()
              << std::endl;
  }
}

std::string TraditionalDetector::classify_digit(const cv::Mat & warped, double * confidence,
                                                cv::Rect * digit_box_out,
                                                cv::Mat * digit_out)
{
  if (confidence) *confidence = 0.0;
  if (digit_box_out) *digit_box_out = cv::Rect();
  if (warped.empty() || !digit_recognizer_) return "?";

  // 对齐训练集预处理：模型期望「白字黑底、数字紧贴框」的 28x28 单个数字。
  cv::Mat gray;
  if (warped.channels() == 3) cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
  else if (warped.channels() == 4) cv::cvtColor(warped, gray, cv::COLOR_BGRA2GRAY);
  else gray = warped;

  // 1) 先取中间区域（两侧各去掉 25% 排除灯条），数字印刷在中间
  int cx0 = gray.cols * 25 / 100;
  int cx1 = gray.cols - cx0;
  if (cx1 - cx0 < 8) { cx0 = 0; cx1 = gray.cols; }
  cv::Mat center = gray(cv::Rect(cx0, 0, cx1 - cx0, gray.rows));

  // 2) Otsu 二值化 + 极性自动判断：数字是少数类，统一成白字(255)黑底(0)
  cv::Mat bin;
  cv::threshold(center, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  int bright = cv::countNonZero(bin);
  int total = center.rows * center.cols;
  if (bright > total - bright) cv::bitwise_not(bin, bin);  // 亮是多数 => 数字是暗的 => 取反

  // 3) 连通域：剔除贴左右边缘/满高的灯条，取中间最大者=数字
  cv::Mat labels, stats, centroids;
  int n = cv::connectedComponentsWithStats(bin, labels, stats, centroids, 8);
  int best = -1, best_area = 0;
  for (int i = 1; i < n; ++i) {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (x <= 2 || x + w >= center.cols - 2) continue;  // 贴边 = 灯条
    if (h > center.rows * 0.8) continue;               // 满高亮条 = 灯条
    if (area > best_area) { best_area = area; best = i; }
  }

  double conf = 0.0;
  int label = -1;
  cv::Mat digit;
  cv::Rect roi;
  if (best >= 0 && best_area >= 4) {
    int bx = stats.at<int>(best, cv::CC_STAT_LEFT);
    int by = stats.at<int>(best, cv::CC_STAT_TOP);
    int bw = stats.at<int>(best, cv::CC_STAT_WIDTH);
    int bh = stats.at<int>(best, cv::CC_STAT_HEIGHT);
    int pad = std::max(2, std::max(bw, bh) / 6);
    int x0 = std::max(0, bx - pad);
    int y0 = std::max(0, by - pad);
    int x1 = std::min(center.cols, bx + bw + pad);
    int y1 = std::min(center.rows, by + bh + pad);
    digit = bin(cv::Rect(x0, y0, x1 - x0, y1 - y0)).clone();  // 白字黑底
    roi = cv::Rect(cx0 + x0, y0, x1 - x0, y1 - y0);           // 映射回 warp 坐标
    if (digit_box_out) *digit_box_out = roi;
    if (digit_out) *digit_out = digit;
    label = digit_recognizer_->recognize(digit, &conf);
  }

  // 哨兵(无有效数字) / 越界 / 置信度不足 -> 失败
  if (label < 0 || label >= digit_class_num_ || conf < digit_match_threshold_) {
    return "?";
  }
  if (confidence) *confidence = conf;
  return std::to_string(label);
}
// Perspective warp end

void TraditionalDetector::handle_digit_collect(int key, const cv::Mat & digit)
{
  if (digit_collect_dir_.empty() || digit.empty()) return;

  int label = -1;
  if (key >= '0' && key <= '5') label = key - '0';
  else if (key == 's' || key == 'S') label = 6;  // 哨兵（无数字）
  if (label < 0) return;

  std::filesystem::path dir = std::filesystem::path(digit_collect_dir_) / std::to_string(label);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::cerr << "[traditional_detector] 创建采集目录失败: " << dir << " (" << ec.message()
              << ")" << std::endl;
    return;
  }

  char name[64];
  std::snprintf(name, sizeof(name), "%06d.png", collect_index_++);
  std::string path = (dir / name).string();
  if (cv::imwrite(path, digit)) {
    std::cout << "[traditional_detector] 已采集样本 -> " << path << " (label=" << label << ")"
              << std::endl;
  } else {
    std::cerr << "[traditional_detector] 保存样本失败: " << path << std::endl;
  }
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

    // 数字识别结果可视化：在装甲中心上方标注识别数字
    if (!armor.digit.empty() && armor.digit != "?") {
      cv::putText(img, armor.digit,
                  cv::Point(static_cast<int>(armor.center.x),
                            static_cast<int>(armor.center.y - 10)),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }
  }
}

}  // namespace traditional_armor
