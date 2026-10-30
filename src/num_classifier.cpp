/**
 * @file    num_classifier.cpp
 * @brief   装甲板数字分类器实现（ONNX Runtime CPU 推理）
 *
 * 预处理规则对齐 Reborn-Vision-2024-armor-Inference 原版 number_classifier.cpp。
 */

#include "num_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace
{
// 与 Reborn 原版 class_names_ 顺序一致
constexpr const char * kClassNames[9] = {
    "1", "2", "3", "4", "5", "outpost", "guard", "base", "negative"};
}  // namespace

NumClassifier::NumClassifier(const std::string & model_path)
: env_(ORT_LOGGING_LEVEL_WARNING, "NumClassifier")
{
  try {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);  // 数字 ROI 极小，单线程足够且避免线程抖动
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

    input_buffer_.resize(kRoiHeight * kRoiWidth);  // 1*1*28*20
  } catch (const std::exception & e) {
    session_.reset();
    std::cerr << "[NumClassifier] 加载 ONNX 模型失败: " << e.what() << std::endl;
  }
}

cv::Mat NumClassifier::preprocess(
  const cv::Mat & bgr_img, const std::vector<cv::Point2f> & points) const
{
  if (bgr_img.empty() || points.size() != 4) return cv::Mat();

  // 透视变换参数（与 Reborn 原版一致）
  const int top_light_y = (kWarpHeight - kLightLength) / 2 - 1;  // 7
  const int bottom_light_y = top_light_y + kLightLength;         // 19
  const int warp_width = kSmallArmorWidth;                       // 32

  // 源四角点按 [左下, 左上, 右上, 右下] 排列，对齐 Reborn 原版 lights_vertices 顺序；
  // 用户传入顺序为 [左上, 右上, 右下, 左下]。
  cv::Point2f src[4] = {
    points[3],                                  // 左下
    points[0],                                  // 左上
    points[1],                                  // 右上
    points[2],                                  // 右下
  };
  cv::Point2f dst[4] = {
    cv::Point2f(0, bottom_light_y),                // 左下
    cv::Point2f(0, top_light_y),                   // 左上
    cv::Point2f(warp_width - 1, top_light_y),      // 右上
    cv::Point2f(warp_width - 1, bottom_light_y),   // 右下
  };

  cv::Mat transform = cv::getPerspectiveTransform(src, dst);
  cv::Mat warped;
  cv::warpPerspective(bgr_img, warped, transform, cv::Size(warp_width, kWarpHeight));

  // 取中间数字 ROI：宽 20 高 28（两侧各去掉 6 像素避开灯条，全高）
  cv::Mat roi = warped(
    cv::Rect((warp_width - kRoiWidth) / 2, 0, kRoiWidth, kRoiHeight));

  // 灰度（原版用 COLOR_RGB2GRAY；输入为 BGR 时 R/B 权重互换，但 OTSU 二值化不敏感）
  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_RGB2GRAY);

  // OTSU 二值化
  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // 归一化到 [0,1]（原版 image / 255.0，无 mean/std）
  cv::Mat normalized;
  binary.convertTo(normalized, CV_32FC1, 1.0f / 255.0f);
  return normalized;
}

NumClassifier::Result NumClassifier::classify(
  const cv::Mat & bgr_img, const std::vector<cv::Point2f> & points)
{
  Result result;
  if (!session_) return result;  // id 保持 -1

  try {
    cv::Mat normalized = preprocess(bgr_img, points);
    if (normalized.empty() || normalized.rows != kRoiHeight || normalized.cols != kRoiWidth) {
      return result;
    }

    // 拷贝到输入 buffer：单通道 NCHW [1,1,28,20]，内存与 28x20 连续一致
    std::memcpy(input_buffer_.data(), normalized.data, input_buffer_.size() * sizeof(float));

    std::array<int64_t, 4> input_shape{1, 1, kRoiHeight, kRoiWidth};
    Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, input_buffer_.data(), input_buffer_.size(),
      input_shape.data(), input_shape.size());

    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    auto outputs = session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    const float * logits = outputs[0].GetTensorMutableData<float>();

    // Softmax（减最大值保证数值稳定），同时取最大概率类别
    float max_logit = *std::max_element(logits, logits + kNumClasses);
    float sum_exp = 0.0f;
    float exp_val[kNumClasses];
    for (int i = 0; i < kNumClasses; ++i) {
      exp_val[i] = std::exp(logits[i] - max_logit);
      sum_exp += exp_val[i];
    }
    int best = 0;
    float best_prob = 0.0f;
    for (int i = 0; i < kNumClasses; ++i) {
      const float p = exp_val[i] / sum_exp;
      if (p > best_prob) {
        best_prob = p;
        best = i;
      }
    }

    result.label = kClassNames[best];
    result.confidence = best_prob;
    result.id = map_to_id(best);
  } catch (const std::exception & e) {
    std::cerr << "[NumClassifier] 推理失败: " << e.what() << std::endl;
    return Result();
  }
  return result;
}

int NumClassifier::map_to_id(int class_index)
{
  // 0~4 对应 "1"~"5"；5~8 为 outpost/guard/base/negative，统一映射为 0（非编号目标）
  if (class_index >= 0 && class_index < 5) return class_index + 1;
  return 0;
}
