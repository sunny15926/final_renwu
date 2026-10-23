#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Dense>
#include <chrono>

namespace tools
{
// 将弧度限制在 (-pi, pi]
double limit_rad(double angle);

// 直角坐标 -> 球坐标，ypd = [yaw, pitch, distance]（单位 rad, rad, m）
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// xyz2ypd 对 xyz 的雅可比矩阵（3x3）
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 时间差 a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP
