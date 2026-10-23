#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include "armor_track_ros2/traditional_armor.hpp"

namespace traditional_armor
{

// 传统装甲识别类：输入 BGR 图像，输出装甲目标列表。
// 流水线：图像预处理 → HSV 颜色阈值分割 → 形态学操作 → findContours 提取轮廓
//         → RotatedRect 筛选灯条 → 灯条两两配对组成装甲板
//         → 透视矫正 + CNN 数字识别校验（仅保留识别出有效数字 0~5 的装甲）。
class TraditionalDetector
{
public:
  explicit TraditionalDetector(const std::string & config_path);

  // 识别入口：输入一帧 BGR 图像，返回通过数字校验的装甲板
  std::vector<Armor> detect(const cv::Mat & bgr_img);

  // 在原图上绘制灯条与装甲框
  void draw(cv::Mat & img, const std::vector<Armor> & armors);

  // 最近一帧全部候选装甲的 CNN 推理总耗时（毫秒），供主循环帧率日志打印
  double last_inference_ms() const { return last_inference_ms_; }

private:
  // ---- 颜色 ----
  Color enemy_color_;                    // 敌方颜色（仅分割该颜色）
  int red_hue_low_[2], red_hue_high_[2];  // 红色跨越 0°/180° 两端，用两段阈值
  int blue_hue_low_, blue_hue_high_;      // 蓝色用一段阈值
  int saturation_low_, saturation_high_;  // 饱和度上下限
  int value_low_, value_high_;            // 明度上下限
  int morph_kernel_size_;                 // 形态学核大小

  // 预处理复用缓冲区，避免每帧堆分配
  cv::Mat hsv_img_;         // BGR->HSV 中间图
  cv::Mat mask1_, mask2_;   // 红色两段阈值中间结果
  cv::Mat mask_;            // 最终二值图

  // ---- 灯条筛选阈值 ----
  double max_angle_error_;                         // 灯条偏离竖直方向的最大角度（rad）
  double min_lightbar_ratio_, max_lightbar_ratio_;  // 灯条长宽比范围
  double min_lightbar_length_;                     // 灯条最小长度（像素）

  // ---- 装甲配对判断阈值 ----
  double min_armor_ratio_, max_armor_ratio_;  // 装甲板宽高比范围
  double max_side_ratio_;                     // 左右灯条最大长度比
  double max_rectangular_error_;              // 灯条与连线垂直误差（rad）

  bool debug_;  // 是否显示中间二值图

  // Perspective warp start
  bool perspective_debug_;              // 调试开关：透视矫正 + 数字识别 + 额外可视化
  int perspective_height_;              // 矫正后目标高度（像素）
  double digit_match_threshold_;        // 数字识别置信度阈值（softmax 概率，低于该值判定为无效装甲并剔除）
  bool enable_digit_filter_;            // 数字过滤总开关：false 时跳过数字识别，直接用候选装甲
  std::vector<Armor> rejected_armors_;  // 被数字识别剔除的候选装甲（仅调试绘制灰框用）

  // ---- CNN 数字识别（OpenCV dnn + onnx） ----
  std::string digit_model_path_;        // CNN 模型文件路径（onnx）；空字符串则数字识别全部返回 ?
  int digit_model_input_size_;          // 模型输入尺寸（宽=高，灰度单通道），需与训练时一致
  int digit_class_num_;                 // 分类类别数（装甲数字 0~5，共 6 类）
  cv::dnn::Net digit_net_;              // 已加载的 CNN 网络
  double last_inference_ms_ = 0.0;      // 最近一帧全部候选装甲的 CNN 推理总耗时（毫秒）
  // Perspective warp end

  // 预处理 + HSV 阈值分割 + 形态学，输出二值图
  cv::Mat preprocess(const cv::Mat & bgr_img);

  // 从二值图提取并筛选灯条
  std::vector<Lightbar> extract_lightbars(const cv::Mat & binary_img);

  // 灯条几何筛选条件
  bool check_lightbar(const Lightbar & lightbar) const;

  // 装甲板配对几何判断条件
  bool check_armor(const Armor & armor) const;

  // Perspective warp start
  // 透视矫正：把装甲板 4 角点映射到正对矩形，返回矫正后的装甲板小图
  cv::Mat warp_armor(const cv::Mat & img, const Armor & armor) const;

  // 加载 CNN 数字模型（onnx）：读取失败时 digit_net_ 为空，数字识别全部返回 ?
  void load_digit_model();

  // CNN 数字识别：裁剪中间数字区域 -> 灰度 -> 归一化 -> 缩放到模型输入尺寸 ->
  // 送入轻量 CNN 分类，返回预测的数字字符串（0~5）；
  // 识别失败或置信度不足返回 "?"；confidence 输出最大 softmax 概率（可选），
  // digit_box 输出数字区域外接框（矫正图坐标，可选，供调试绘制）
  std::string classify_digit(const cv::Mat & warped, double * confidence = nullptr,
                             cv::Rect * digit_box = nullptr);

  // 把矫正图坐标点映射回原图坐标（用于在原图上绘制数字框）
  std::vector<cv::Point2f> map_warped_to_original(
    const Armor & armor, const std::vector<cv::Point2f> & pts) const;
  // Perspective warp end
};

}  // namespace traditional_armor
