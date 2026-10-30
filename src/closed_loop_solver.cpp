#include "auto_aim/closed_loop_solver.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>

namespace auto_aim
{
namespace
{
constexpr double PI = 3.14159265358979323846;

double clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

ClosedLoopSolver::ClosedLoopSolver(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  small_armor_width_ = yaml["small_armor_width"].as<double>();
  big_armor_width_ = yaml["big_armor_width"].as<double>();
  lightbar_length_ = yaml["lightbar_length"].as<double>();

  alpha_ = yaml["closed_loop_alpha"].as<double>(0.3);
  angle_limit_ = yaml["closed_loop_angle_limit"].as<double>(0.3);
  debug_ = yaml["closed_loop_debug"].as<bool>(false);
}

void ClosedLoopSolver::set_camera(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs)
{
  camera_matrix_ = camera_matrix;
  distort_coeffs_ = distort_coeffs;
  camera_ready_ = !camera_matrix_.empty() && !distort_coeffs_.empty();
}

ClosedLoopSolver::Result ClosedLoopSolver::aim(
  bool armor_valid, bool imu_valid,
  const std::vector<cv::Point2f> & points, ArmorType type,
  double gimbal_yaw, double gimbal_pitch)
{
  Result r;

  // 保护：装甲丢失 / IMU 失效 / 相机未标定 -> 复位滤波并输出 0，停止跟随。
  if (!armor_valid || !imu_valid || !camera_ready_ || points.size() != 4) {
    filter_initialized_ = false;
    yaw_filt_ = 0.0;
    pitch_filt_ = 0.0;
    return r;  // valid=false, control=false, 角度全 0
  }

  // ① solvePnP：装甲 3D 模型 -> 相机系 tvec。
  double W = (type == ArmorType::big) ? big_armor_width_ : small_armor_width_;
  double L = lightbar_length_;
  double hw = W / 2.0;
  double hl = L / 2.0;
  // 对象系：x 右、y 下、z 面朝相机（法向 +z）；角点顺序与 points [左上,右上,右下,左下] 一致。
  std::vector<cv::Point3f> object_points{
    cv::Point3f(-hw, -hl, 0.0f),  // 左上
    cv::Point3f(+hw, -hl, 0.0f),  // 右上
    cv::Point3f(+hw, +hl, 0.0f),  // 右下
    cv::Point3f(-hw, +hl, 0.0f)}; // 左下

  cv::Vec3d rvec, tvec;
  cv::solvePnP(object_points, points, camera_matrix_, distort_coeffs_,
               rvec, tvec, false, cv::SOLVEPNP_IPPE);

  r.tvec = tvec;
  Eigen::Vector3d p_cam(tvec[0], tvec[1], tvec[2]);  // (Xc 右, Yc 下, Zc 前)

  // ② 相机系 -> 世界系（闭环：用云台当前 yaw/pitch 构造旋转矩阵）。
  //    R0：零位时相机轴到世界轴的固定排列（相机 X右 -> W Y右，Y下 -> W -Z，Z前 -> W X）。
  Eigen::Matrix3d R0;
  R0 << 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0;

  //    R_C2W = R_z(-yaw) * R_y(+pitch) * R0。
  //    注意：IMU 四元数给出的是 world->camera 姿态，其 Z-Y-X 欧拉角 yaw/pitch 与
  //    云台命令角（yaw 右正 / pitch 上正）互为相反数，故这里取反后再构造 camera->world。
  double cy = std::cos(-gimbal_yaw), sy = std::sin(-gimbal_yaw);
  double cp = std::cos(gimbal_pitch), sp = std::sin(gimbal_pitch);
  Eigen::Matrix3d Rz;
  Rz << cy, -sy, 0.0,
        sy,  cy, 0.0,
        0.0, 0.0, 1.0;
  Eigen::Matrix3d Ry;
  Ry << cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp;

  Eigen::Matrix3d R_C2W = Rz * Ry * R0;
  r.xyz_in_world = R_C2W * p_cam;

  // ③ 世界系下 atan2 目标角（yaw 右正、pitch 上正）。
  double xw = r.xyz_in_world[0], yw = r.xyz_in_world[1], zw = r.xyz_in_world[2];
  r.yaw_raw = std::atan2(yw, xw);
  r.pitch_raw = std::atan2(zw, std::hypot(xw, yw));

  // ④ 一阶低通滤波（存未限幅的滤波值，避免饱和；限幅只作用于输出）。
  if (!filter_initialized_) {
    yaw_filt_ = r.yaw_raw;
    pitch_filt_ = r.pitch_raw;
    filter_initialized_ = true;
  } else {
    yaw_filt_ = alpha_ * r.yaw_raw + (1.0 - alpha_) * yaw_filt_;
    pitch_filt_ = alpha_ * r.pitch_raw + (1.0 - alpha_) * pitch_filt_;
  }

  // ⑤ 限幅。
  r.yaw = clamp(yaw_filt_, -angle_limit_, angle_limit_);
  r.pitch = clamp(pitch_filt_, -angle_limit_, angle_limit_);

  r.valid = true;
  r.control = true;

  if (debug_) {
    std::printf(
      "[closed_loop] tvec=(%.3f,%.3f,%.3f) gimbal=(%.3f,%.3f) world=(%.3f,%.3f,%.3f) "
      "raw=(%.3f,%.3f) -> filt=(%.3f,%.3f)\n",
      tvec[0], tvec[1], tvec[2], gimbal_yaw, gimbal_pitch,
      xw, yw, zw, r.yaw_raw, r.pitch_raw, r.yaw, r.pitch);
  }

  return r;
}

}  // namespace auto_aim
