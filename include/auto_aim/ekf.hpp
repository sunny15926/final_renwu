#ifndef AUTO_AIM__EKF_HPP
#define AUTO_AIM__EKF_HPP

#include <Eigen/Dense>

namespace auto_aim
{

// 6 维扩展卡尔曼滤波：状态 X = [x, vx, y, vy, z, vz]^T。
//   世界系坐标约定（对齐闭环解算）：X 前、Y 右、Z 上；单位 m / m/s。
// 运动模型为匀速直线（CV），量测为位置 [x, y, z]（由 PnP + 坐标变换得到）。
// 因模型与量测均线性，实际退化为标准卡尔曼滤波，这里统一按 EKF 的 predict/update 结构实现。
class EKF
{
public:
  struct Config
  {
    double sigma_acc = 2.0;     // 过程噪声：加速度扰动标准差 (m/s^2)，越小越平滑、越不信任量测
    double sigma_x = 0.02;      // 量测噪声标准差 x (m)
    double sigma_y = 0.02;      // 量测噪声标准差 y (m)
    double sigma_z = 0.02;      // 量测噪声标准差 z (m)
    double init_p_pos = 1.0;    // 初始位置方差 (m^2)
    double init_p_vel = 100.0;  // 初始速度方差 (m/s)^2
  };

  EKF();
  explicit EKF(const Config & cfg);

  void set_config(const Config & cfg);

  // 首次量测：初始化状态（速度置 0）与协方差。
  void init(const Eigen::Vector3d & pos);
  // 时间推进 dt 秒（匀速直线模型 + 白噪声加速度过程噪声）。
  void predict(double dt);
  // 位置量测更新。
  void update(const Eigen::Vector3d & pos);
  // 复位（目标丢失时调用）。
  void reset();

  bool initialized() const { return initialized_; }
  const Eigen::Matrix<double, 6, 1> & x() const { return x_; }

private:
  void build_R();

  Config cfg_;
  Eigen::Matrix<double, 6, 1> x_;
  Eigen::Matrix<double, 6, 6> P_;
  Eigen::Matrix<double, 3, 6> H_;
  Eigen::Matrix<double, 3, 3> R_;
  bool initialized_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__EKF_HPP
