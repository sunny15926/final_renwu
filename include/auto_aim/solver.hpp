#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <string>

#include "auto_aim/armor.hpp"

namespace auto_aim
{
// PnP 解算 + 坐标变换：
//   像素四点 -> solvePnP(IPPE) 得相机系 tvec -> 云台系(R_camera2gimbal) -> 世界/基座系(R_gimbal2world)
//   -> 球坐标 [yaw, pitch, distance]
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  // 内参/畸变从 ROS2 CameraInfo 注入（替代 yaml 读取）
  void set_camera(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs);

  // 云台 -> 世界旋转；固定基座时保持单位阵，基座可动时由下位机姿态四元数更新
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  double small_armor_width_;  // m
  double big_armor_width_;    // m
  double lightbar_length_;    // m
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
