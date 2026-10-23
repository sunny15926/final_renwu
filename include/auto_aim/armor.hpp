#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_aim
{
enum ArmorType
{
  big,
  small
};

// 数字类别（与分类模型输出下标一一对应）
// 下标 0..8 依次为：1、2、3、4、5、哨兵、前哨站、基地、非装甲
enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};

// 数字名，用于日志打印
constexpr const char * ARMOR_NAMES[] = {"one",    "two",     "three", "four",     "five",
                                        "sentry", "outpost", "base",  "not_armor"};

struct Armor
{
  std::vector<cv::Point2f> points;  // 四角点 [左上, 右上, 右下, 左下]（PnP 顺序）
  cv::Point2f center;
  cv::Mat pattern;                          // 数字识别 ROI（BGR 裁剪图）
  ArmorName name = ArmorName::not_armor;    // 数字识别绑定的编号
  double confidence = 0.0;                  // 数字识别置信度
  ArmorType type = ArmorType::small;        // 大小装甲（PnP 选 3D 模型尺寸）

  // 坐标链：相机系 -> 云台系 -> 世界/基座系
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();  // [yaw, pitch, distance]
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
