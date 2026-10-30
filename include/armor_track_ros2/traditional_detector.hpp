#pragma once

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

#include "armor_track_ros2/traditional_armor.hpp"

// 前向声明：ONNX Runtime 数字识别类（定义见 Digital-recognition-master/include/DigitalRecognition.h）
class DigitalRecognition;

namespace traditional_armor
{

// 传统装甲识别类：输入 BGR 图像，输出装甲目标列表。
// 流水线：图像预处理 → HSV 颜色阈值分割 → 形态学操作 → findContours 提取轮廓
//         → RotatedRect 筛选灯条 → 灯条两两配对组成装甲板
//         → 透视矫正 + ONNX Runtime 数字识别（结果打印/可视化，可按开关过滤）。
class TraditionalDetector
{
public:
  explicit TraditionalDetector(const std::string & config_path);

  // 析构函数在 .cpp 中定义（DigitalRecognition 为前向声明，需完整类型才能析构 unique_ptr）
  ~TraditionalDetector();

  // 识别入口：输入一帧 BGR 图像，返回通过数字校验的装甲板
  std::vector<Armor> detect(const cv::Mat & bgr_img);

  // 在原图上绘制灯条与装甲框
  void draw(cv::Mat & img, const std::vector<Armor> & armors);

  // 设置叠加到可视化窗口左上角的文本（多行，'\n' 分隔）；空串清除。由外层（auto_aim_node）每帧写入。
  void set_overlay_text(const std::string & text) { overlay_text_ = text; }

  // 最近一帧全部候选装甲的数字识别推理总耗时（毫秒），供主循环帧率日志打印
  double last_inference_ms() const { return last_inference_ms_; }

  // 最近一帧第一块装甲的透视矫正图 / 二值化数字（曝光扫描、离线采集分析用）
  const cv::Mat & last_warp() const { return last_warp_; }
  const cv::Mat & last_digit() const { return last_digit_; }

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
  bool digit_viz_stable_;               // 稳定可视化：固定画布 + 主目标锁定 + 上一帧保持
  int perspective_height_;              // 矫正后目标高度（像素）
  double digit_extend_ratio_;           // 数字裁剪沿灯条轴线向两头延伸比例（单位=灯条长度）
  double digit_match_threshold_;        // 数字识别置信度阈值（softmax 概率，低于该值判定为无效装甲并剔除）
  bool enable_digit_filter_;            // 数字过滤开关：true 时剔除未识别出有效数字的候选；false 仅打印/可视化不过滤
  std::vector<Armor> rejected_armors_;  // 被数字识别剔除的候选装甲（仅调试绘制灰框用）

  // ---- 数字识别（ONNX Runtime） ----
  std::string digit_model_path_;                          // 数字模型文件路径（onnx）；空字符串则数字识别全部返回 ?
  int digit_model_input_size_;                            // 模型输入尺寸（宽=高，灰度单通道），需与训练时一致
  int digit_class_num_;                                   // 分类类别数（装甲数字 0~5，共 6 类）
  std::unique_ptr<DigitalRecognition> digit_recognizer_;  // 已加载的 ONNX Runtime 数字识别器
  double last_inference_ms_ = 0.0;                        // 最近一帧全部候选装甲的数字推理总耗时（毫秒）

  // 数字样本采集：可视化窗口按键 '0'~'5' 存对应数字、's' 存哨兵，保存到 digit_collect_dir_/<label>/
  std::string digit_collect_dir_;                         // 采集输出目录（空=不采集）
  int collect_index_ = 0;                                 // 采集样本自增序号
  cv::Mat last_warp_;    // 最近一帧用于显示的透视矫正图（稳定模式=锁存主目标）
  cv::Mat last_digit_;   // 最近一帧用于显示的二值化数字（白字黑底，稳定模式=锁存）
  cv::Point2f viz_center_;   // 稳定模式锁存目标中心（原图坐标）
  double viz_radius_ = 0.0;  // 稳定模式锁存目标身份匹配半径（像素）
  bool viz_valid_ = false;   // 稳定模式是否已有锁存目标
  // Perspective warp end

  std::string overlay_text_;  // 叠加到可视化窗口左上角的文本（由外层 set_overlay_text 设置）

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

  // 加载数字模型（onnx，ONNX Runtime）：读取失败时 digit_recognizer_ 为空，数字识别全部返回 ?
  void load_digit_model();

  // 数字识别：裁剪中间数字区域后调用 ONNX Runtime 推理（内部完成灰度/缩放/归一化），
  // 返回预测的数字字符串（0~5）；
  // 识别失败或置信度不足返回 "?"；confidence 输出 softmax 概率（可选），
  // digit_box 输出数字区域外接框（矫正图坐标，可选，供调试绘制）
  std::string classify_digit(const cv::Mat & warped, double * confidence = nullptr,
                             cv::Rect * digit_box = nullptr,
                             cv::Mat * digit_out = nullptr);

  // 把矫正图坐标点映射回原图坐标（用于在原图上绘制数字框）
  std::vector<cv::Point2f> map_warped_to_original(
    const Armor & armor, const std::vector<cv::Point2f> & pts) const;

  // 键盘采集样本：key 为 '0'~'5' 存对应数字、's' 存哨兵；digit 为已二值化的白字黑底数字裁剪
  void handle_digit_collect(int key, const cv::Mat & digit);
  // Perspective warp end
};

}  // namespace traditional_armor
