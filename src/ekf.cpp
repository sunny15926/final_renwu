#include "auto_aim/ekf.hpp"

namespace auto_aim
{

EKF::EKF() : EKF(Config()) {}

EKF::EKF(const Config & cfg) : cfg_(cfg)
{
  x_.setZero();
  P_.setZero();

  // 量测矩阵：只观测位置 [x, y, z]。
  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 2) = 1.0;
  H_(2, 4) = 1.0;

  build_R();
}

void EKF::set_config(const Config & cfg)
{
  cfg_ = cfg;
  build_R();
}

void EKF::build_R()
{
  R_.setZero();
  R_(0, 0) = cfg_.sigma_x * cfg_.sigma_x;
  R_(1, 1) = cfg_.sigma_y * cfg_.sigma_y;
  R_(2, 2) = cfg_.sigma_z * cfg_.sigma_z;
}

void EKF::init(const Eigen::Vector3d & pos)
{
  x_.setZero();
  x_(0) = pos[0];
  x_(2) = pos[1];
  x_(4) = pos[2];

  P_.setZero();
  P_(0, 0) = cfg_.init_p_pos;
  P_(2, 2) = cfg_.init_p_pos;
  P_(4, 4) = cfg_.init_p_pos;
  P_(1, 1) = cfg_.init_p_vel;
  P_(3, 3) = cfg_.init_p_vel;
  P_(5, 5) = cfg_.init_p_vel;

  initialized_ = true;
}

void EKF::predict(double dt)
{
  if (!initialized_) return;

  const double dt2 = dt * dt;

  // 状态转移 F：X_{k+1} = F X_k。
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;

  // 过程噪声 Q：白噪声加速度离散模型，三个轴独立。
  const double q = cfg_.sigma_acc * cfg_.sigma_acc;
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  // x 轴
  Q(0, 0) = dt2 * dt2 / 4.0 * q;
  Q(0, 1) = dt2 * dt / 2.0 * q;
  Q(1, 0) = Q(0, 1);
  Q(1, 1) = dt2 * q;
  // y 轴
  Q(2, 2) = dt2 * dt2 / 4.0 * q;
  Q(2, 3) = dt2 * dt / 2.0 * q;
  Q(3, 2) = Q(2, 3);
  Q(3, 3) = dt2 * q;
  // z 轴
  Q(4, 4) = dt2 * dt2 / 4.0 * q;
  Q(4, 5) = dt2 * dt / 2.0 * q;
  Q(5, 4) = Q(4, 5);
  Q(5, 5) = dt2 * q;

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void EKF::update(const Eigen::Vector3d & pos)
{
  if (!initialized_) return;

  const Eigen::Vector3d z(pos[0], pos[1], pos[2]);
  const Eigen::Vector3d innov = z - H_ * x_;
  const Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
  const Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * innov;

  const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
  P_ = (I - K * H_) * P_;
}

void EKF::reset()
{
  initialized_ = false;
  x_.setZero();
  P_.setZero();
}

}  // namespace auto_aim
