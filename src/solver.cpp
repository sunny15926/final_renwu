#include "auto_aim/solver.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/core/eigen.hpp>
#include <vector>

#include "tools/math_tools.hpp"

namespace auto_aim
{
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  small_armor_width_ = yaml["small_armor_width"].as<double>();
  big_armor_width_ = yaml["big_armor_width"].as<double>();
  lightbar_length_ = yaml["lightbar_length"].as<double>();
}

void Solver::set_camera(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs)
{
  camera_matrix_ = camera_matrix;
  distort_coeffs_ = distort_coeffs;
}

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  R_gimbal2world_ = q.toRotationMatrix();
}

void Solver::solve(Armor & armor)
{
  // 装甲板 3D 模型角点（单位 m）。板面在 x=0 平面，法向 +x。
  // 顺序 [左上, 右上, 右下, 左下]，必须与 armor.points 的排序一致。
  double W = (armor.type == ArmorType::big) ? big_armor_width_ : small_armor_width_;
  double L = lightbar_length_;
  double hw = W / 2.0;
  double hl = L / 2.0;
  std::vector<cv::Point3f> object_points{
    cv::Point3f(0.0f, hw, hl), cv::Point3f(0.0f, -hw, hl),
    cv::Point3f(0.0f, -hw, -hl), cv::Point3f(0.0f, hw, -hl)};

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  // 相机系 -> 云台系
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

  // 云台系 -> 世界/基座系
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  // 直角 -> 球坐标（跟随角 + 距离）
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
}

}  // namespace auto_aim
