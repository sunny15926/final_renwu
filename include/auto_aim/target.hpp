#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>

#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
// 6 维常速度模型 EKF，状态 x = [x, vx, y, vy, z, vz]
//   观测 z = xyz2ypd([x, y, z]) = [yaw, pitch, distance]
// 用于估计手持装甲板（缓慢匀速）的线速度与视线角速度。
class Target
{
public:
  Target() = default;

  // 用首次观测初始化状态与协方差
  void init(const Eigen::Vector3d & xyz, double dt);

  // 仅预测一步（无观测时推进状态）
  void predict(double dt);

  // 预测 + 更新，ypd = [yaw, pitch, distance]
  void update(const Eigen::Vector3d & ypd, double dt);

  bool initialized() const { return initialized_; }

  // 位置 [x, y, z]
  Eigen::Vector3d position() const;
  // 线速度 [vx, vy, vz]
  Eigen::Vector3d linear_velocity() const;
  // 视线角速度 [yaw_rate, pitch_rate]（rad/s）
  Eigen::Vector2d angular_velocity() const;

private:
  tools::ExtendedKalmanFilter ekf_;
  bool initialized_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
