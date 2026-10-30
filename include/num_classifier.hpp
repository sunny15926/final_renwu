/**
 * @file    num_classifier.hpp
 * @brief   装甲板数字分类器（北科大 Reborn 数字识别模型，ONNX Runtime CPU 推理）
 *
 * 完全独立、插拔式：只依赖 OpenCV4 + ONNX Runtime，不依赖 detector / PnP / 串口等任何模块。
 *
 * 输入是装甲板【四个角点】cv::Point2f（不是灯条），内部完成 Reborn 原版整条预处理：
 *   透视矫正(warpPerspective) -> 裁剪中间数字区 -> 灰度 -> OTSU 二值化 -> /255 归一化
 * 然后送入网络，输出装甲编号 0~5。
 *
 * 预处理规则逐条对齐 Reborn-Vision-2024-armor-Inference 原版 src/number_classifier.cpp：
 *   - 透视目标尺寸 32x28，灯条段 y=[7,19]，数字 ROI = Rect(6,0,20,28)（宽 20 高 28）
 *   - 灰度用 COLOR_RGB2GRAY（原版如此；输入为 BGR 时 R/B 权重互换，但 OTSU 二值化对其不敏感）
 *   - OTSU 二值化 threshold(...,0,255,THRESH_BINARY|THRESH_OTSU)
 *   - 归一化仅除以 255，无 mean/std
 *   - 网络输入 NCHW [1,1,28,20]（H=28 高, W=20 宽，单通道 float32），即竖直数字
 *
 * ⚠️ 模型说明：原版 number_classifier.onnx 里声明的输入是 [1,1,20,28]（H/W 颠倒的导出笔误），
 *    但原版实际用 OpenCV dnn 以 [1,1,28,20] 喂入且能正常识别（OpenCV dnn 不校验形状）。
 *    ONNX Runtime 会严格按声明校验，所以必须使用已修正为 [1,1,28,20] 的模型
 *    （本仓库 assets/number_classifier.onnx 即修正版）。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

class NumClassifier
{
public:
  // 分类结果
  struct Result
  {
    int id = -1;              // 装甲编号 0~5；0=非编号目标(outpost/guard/base/negative)；-1=失败
    std::string label;        // 原始类别名："1"~"5" / "outpost" / "guard" / "base" / "negative"
    float confidence = 0.0f;  // softmax 最大概率（0~1）
  };

  // 加载 ONNX 模型；失败时 is_loaded() == false，classify 恒返回 id=-1
  explicit NumClassifier(const std::string & model_path);

  // 推理入口：BGR 原图 + 装甲板四个角点（顺序：左上、右上、右下、左下）
  Result classify(const cv::Mat & bgr_img, const std::vector<cv::Point2f> & points);

  // 预处理（不含推理）：透视矫正 -> 裁剪中间数字区 -> 灰度 -> OTSU 二值化 -> /255
  // 返回 28 高 x 20 宽 CV_32F 单通道图，与 Reborn 原版 number_img 完全一致（调试/可视化用）
  cv::Mat preprocess(const cv::Mat & bgr_img, const std::vector<cv::Point2f> & points) const;

  bool is_loaded() const noexcept { return session_ != nullptr; }

private:
  // 与 Reborn 原版完全一致的透视 / 裁剪参数
  static constexpr int kLightLength = 12;       // 灯条在透视图中的长度
  static constexpr int kWarpHeight = 28;        // 透视变换后高度
  static constexpr int kSmallArmorWidth = 32;   // 小装甲板宽度（全按小装甲板处理）
  static constexpr int kRoiWidth = 20;          // 数字 ROI 宽
  static constexpr int kRoiHeight = 28;         // 数字 ROI 高（= 透视高度）
  static constexpr int kNumClasses = 9;         // 1~5 + outpost/guard/base/negative

  // 把 argmax 类别索引映射为装甲编号 0~5：0~4 -> 1~5，其余 -> 0
  static int map_to_id(int class_index);

  Ort::Env env_;                              // 必须先于 session 构造
  std::unique_ptr<Ort::Session> session_;     // 推理会话（空 = 未加载/加载失败）
  std::string input_name_;                    // 模型输入节点名
  std::string output_name_;                   // 模型输出节点名
  std::vector<float> input_buffer_;           // 1*1*28*20 归一化输入（NCHW，单通道连续）
};
