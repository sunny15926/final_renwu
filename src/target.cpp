#include "auto_aim/target.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr int kDim = 6;  // [x, vx, y, vy, z, vz]
constexpr double kQ = 0.001;  // 过程噪声强度（加速度方差）
constexpr double kR = 0.01;   // 观测噪声基础方差
}  // namespace

void Target::init(const Eigen::Vector3d & xyz, double dt)
{
  Eigen::VectorXd x0(kDim);
  x0 << xyz[0], 0.0, xyz[1], 0.0, xyz[2], 0.0;

  // 初始位置不确定度大、速度不确定度更大
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(kDim, kDim);
  P0.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 1.0;

  ekf_ = tools::ExtendedKalmanFilter(x0, P0);
  initialized_ = true;

  // 用 dt 先把过程噪声矩阵算好（状态不变，只初始化协方差到合理量级）
  (void)dt;
}

void Target::predict(double dt)
{
  if (!initialized_) return;

  // F: 恒速度转移矩阵
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kDim, kDim);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;

  // Q: 每轴 2x2 [dt^4/4, dt^3/2; dt^3/2, dt^2] * q
  double dt2 = dt * dt;
  Eigen::Matrix2d q_axis;
  q_axis << dt2 * dt2 / 4.0, dt2 * dt / 2.0, dt2 * dt / 2.0, dt2;
  q_axis *= kQ;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kDim, kDim);
  Q.block<2, 2>(0, 0) = q_axis;
  Q.block<2, 2>(2, 2) = q_axis;
  Q.block<2, 2>(4, 4) = q_axis;

  ekf_.predict(F, Q);
}

void Target::update(const Eigen::Vector3d & ypd, double dt)
{
  if (!initialized_) return;

  predict(dt);

  Eigen::Vector3d xyz = position();
  auto H3 = tools::xyz2ypd_jacobian(xyz);

  // H: 观测对状态求导，位置维映射到 0/2/4 列
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kDim);
  H.block<3, 1>(0, 0) = H3.col(0);
  H.block<3, 1>(0, 2) = H3.col(1);
  H.block<3, 1>(0, 4) = H3.col(2);

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * kR;

  // 非线性观测 h: 状态 -> [yaw, pitch, distance]
  auto h = [](const Eigen::VectorXd & x) {
    return tools::xyz2ypd(Eigen::Vector3d(x[0], x[2], x[4]));
  };

  // 角度差用 limit_rad 卷到 (-pi, pi]
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::Vector3d d;
    d << tools::limit_rad(a[0] - b[0]), tools::limit_rad(a[1] - b[1]), a[2] - b[2];
    return d;
  };

  ekf_.update(ypd, H, R, h, z_subtract);
}

Eigen::Vector3d Target::position() const
{
  return Eigen::Vector3d(ekf_.x[0], ekf_.x[2], ekf_.x[4]);
}

Eigen::Vector3d Target::linear_velocity() const
{
  return Eigen::Vector3d(ekf_.x[1], ekf_.x[3], ekf_.x[5]);
}

Eigen::Vector2d Target::angular_velocity() const
{
  auto x = ekf_.x[0], vx = ekf_.x[1];
  auto y = ekf_.x[2], vy = ekf_.x[3];
  auto z = ekf_.x[4], vz = ekf_.x[5];

  auto r2 = x * x + y * y;
  auto yaw_rate = (r2 > 1e-9) ? (x * vy - y * vx) / r2 : 0.0;

  // pitch = atan2(z, r), 对其求导：
  //   pitch_rate = (r * vz - z * r_dot) / (r^2 + z^2)
  auto r = std::sqrt(r2);
  auto r_dot = (r > 1e-9) ? (x * vx + y * vy) / r : 0.0;
  auto denom = r2 + z * z;
  auto pitch_rate = (denom > 1e-9) ? (r * vz - z * r_dot) / denom : 0.0;

  return Eigen::Vector2d(yaw_rate, pitch_rate);
}

}  // namespace auto_aim
