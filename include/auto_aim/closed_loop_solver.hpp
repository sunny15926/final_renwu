#ifndef AUTO_AIM__CLOSED_LOOP_SOLVER_HPP
#define AUTO_AIM__CLOSED_LOOP_SOLVER_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <string>
#include <vector>

#include "auto_aim/ekf.hpp"

namespace auto_aim
{

// 大小装甲类型（PnP 据此选择 3D 模型尺寸）。
enum class ArmorType { small, big };

// 闭环自瞄解算（Eye-in-Hand）：相机装在云台上，用云台当前 yaw/pitch 反馈构造旋转矩阵，
// 把 solvePnP 得到的相机系坐标变换到世界/惯性系，再 atan2 求目标角。
//
// 坐标约定（对齐同济 TJURM）：
//   相机系 C：光心原点，Z 前、X 右、Y 下（OpenCV）。
//   世界系 W：云台零位即基座，X 前、Y 右、Z 上。
//   云台 yaw 向右为正、pitch 向上为正（rad）。
// 变换：P_world = R_z(yaw) * R_y(-pitch) * R0 * P_cam。
//
// 只做：PnP + 坐标变换 + 目标角计算 + 一阶低通滤波 + 限幅 + 保护。
// 不做：EKF、运动预测、手眼标定。数字筛选/大小装甲判定放在外层（本类只接收四点与类型）。
class ClosedLoopSolver
{
public:
  struct Result
  {
    bool valid = false;              // 本帧是否有效（装甲 + IMU + 相机标定均有效）
    bool control = false;            // 是否下发跟随（= valid）
    cv::Vec3d tvec;                  // 相机系 tvec (Xc 右, Yc 下, Zc 前)，单位 m
    Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();  // 世界系 (X 前, Y 右, Z 上)，单位 m
    double yaw_raw = 0.0;            // 滤波前目标 yaw（rad）
    double pitch_raw = 0.0;          // 滤波前目标 pitch（rad）
    double yaw = 0.0;                // 滤波后 + 限幅的目标 yaw（rad）
    double pitch = 0.0;              // 滤波后 + 限幅的目标 pitch（rad）

    // ---- EKF 估计输出（世界系） ----
    Eigen::Matrix<double, 6, 1> ekf_state = Eigen::Matrix<double, 6, 1>::Zero();  // [x,vx,y,vy,z,vz]
    double linear_speed = 0.0;       // 线速度模长 (m/s)
    double yaw_rate = 0.0;           // 视线角速度 yaw（rad/s，右为正）
    double pitch_rate = 0.0;         // 视线角速度 pitch（rad/s，上为正）
  };

  explicit ClosedLoopSolver(const std::string & config_path);

  void set_camera(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs);

  // 单帧解算。armor_valid=false（本帧无装甲）或 imu_valid=false（IMU 失效）时复位并输出 0。
  Result aim(bool armor_valid, bool imu_valid,
             const std::vector<cv::Point2f> & points, ArmorType type,
             double gimbal_yaw, double gimbal_pitch);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  bool camera_ready_ = false;

  double small_armor_width_;
  double big_armor_width_;
  double lightbar_length_;

  double alpha_;        // 一阶低通滤波系数 (0,1]，越小越平滑
  double yaw_limit_;    // yaw 输出限幅（rad）
  double pitch_limit_;  // pitch 输出限幅（rad）

  bool filter_initialized_ = false;
  double yaw_filt_ = 0.0;
  double pitch_filt_ = 0.0;

  EKF ekf_;                                  // 6 维 EKF：估计 [x,vx,y,vy,z,vz]
  std::chrono::steady_clock::time_point last_ekf_t_;  // 上一有效帧时刻（算 dt）
  bool ekf_time_valid_ = false;              // 是否已有上一有效帧时刻
  double ekf_max_dt_ = 0.5;                  // dt 上限（s），防止帧间隔异常导致 Q 发散

  bool debug_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLOSED_LOOP_SOLVER_HPP
