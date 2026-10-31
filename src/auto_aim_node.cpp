#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <vector>

#include "armor_track_ros2/msg/gimbal_cmd.hpp"
#include "armor_track_ros2/msg/gimbal_feedback.hpp"
#include "armor_track_ros2/traditional_detector.hpp"
#include "auto_aim/closed_loop_solver.hpp"

using namespace std::chrono_literals;

namespace
{
cv::Mat toCvMat33(const std::array<double, 9> & k)
{
  cv::Mat m(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) m.at<double>(i / 3, i % 3) = k[i];
  return m;
}

cv::Mat toCvMatD(const std::vector<double> & d)
{
  cv::Mat m(d.size(), 1, CV_64F);
  for (size_t i = 0; i < d.size(); ++i) m.at<double>(i) = d[i];
  return m;
}
}  // namespace

namespace armor_track_ros2
{
using GimbalCmd = armor_track_ros2::msg::GimbalCmd;
using GimbalFeedback = armor_track_ros2::msg::GimbalFeedback;

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const std::string & config_path)
  : Node("armor_track_node"),
    detector_(config_path),
    closed_loop_(config_path)
  {
    auto yaml = YAML::LoadFile(config_path);
    double publish_rate = yaml["publish_rate"].as<double>(30.0);
    imu_timeout_ms_ = yaml["closed_loop_imu_timeout_ms"].as<int>(100);

    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::imageCallback, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::infoCallback, this, std::placeholders::_1));
    feedback_sub_ = create_subscription<GimbalFeedback>(
      "gimbal_feedback", 10,
      std::bind(&AutoAimNode::feedbackCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<GimbalCmd>("gimbal_cmd", 10);

    auto period = publish_rate > 0.0
                    ? std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::duration<double>(1.0 / publish_rate))
                    : std::chrono::milliseconds(33);
    timer_ = create_wall_timer(period, std::bind(&AutoAimNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "armor_track_node (closed-loop) started, config: %s", config_path.c_str());
  }

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_matrix_ = toCvMat33(msg->k);
    distort_coeffs_ = toCvMatD(msg->d);
    closed_loop_.set_camera(camera_matrix_, distort_coeffs_);
    info_received_ = true;
  }

  void feedbackCallback(const GimbalFeedback::SharedPtr msg)
  {
    gimbal_yaw_ = msg->yaw;
    gimbal_pitch_ = msg->pitch;
    imu_received_ = true;
    last_imu_ms_ = nowMs();
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for camera_info ...");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    // ① 传统装甲识别（灯条配对直接给出装甲四点，这里只取第一个目标）。
    auto armors = detector_.detect(cv_ptr->image);
    bool armor_valid = !armors.empty();

    std::vector<cv::Point2f> points;
    auto_aim::ArmorType type = auto_aim::ArmorType::small;  // 大小装甲判定放外层，先按小装甲
    if (armor_valid) {
      points = armors.front().points;
    }

    // ② IMU 新鲜度判定（超时视为失效）。
    bool imu_valid = imu_received_ && (nowMs() - last_imu_ms_ < imu_timeout_ms_);

    // ③ 闭环解算（PnP + 坐标变换 + 滤波 + 限幅 + 保护）。
    auto res = closed_loop_.aim(armor_valid, imu_valid, points, type, gimbal_yaw_, gimbal_pitch_);

    latest_cmd_.control = res.control;
    latest_cmd_.yaw = res.yaw;
    latest_cmd_.pitch = res.pitch;
    latest_cmd_.distance = res.xyz_in_world.norm();
    // 6 维 EKF 估计的目标线速度与视线角速度（世界系）。
    latest_cmd_.vx = res.ekf_state[1];
    latest_cmd_.vy = res.ekf_state[3];
    latest_cmd_.vz = res.ekf_state[5];
    latest_cmd_.yaw_rate = res.yaw_rate;
    latest_cmd_.pitch_rate = res.pitch_rate;

    // 屏幕左上角叠加显示：PnP 测距 + EKF 估计的高低差(z)、线速度、视线角速度。
    // 注意：OpenCV Hershey 字体不支持中文，故标签用英文。
    if (res.valid) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
        "PnP dist   : %6.3f m\n"
        "EKF [x,vx,y,vy,z,vz]\n"
        " height z   : %+6.3f m\n"
        " linear vel : %6.3f m/s\n"
        " LOS rate   : yaw %+6.3f  pitch %+6.3f rad/s",
        cv::norm(res.tvec), res.ekf_state[4], res.linear_speed, res.yaw_rate, res.pitch_rate);
      last_overlay_text_ = buf;
    }
    // 持续显示：目标短暂丢失时保留上一帧有效数据，避免文字断断续续闪烁。
    detector_.set_overlay_text(last_overlay_text_);
  }

  void timerCallback()
  {
    cmd_pub_->publish(latest_cmd_);
  }

  static int64_t nowMs()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<GimbalFeedback>::SharedPtr feedback_sub_;
  rclcpp::Publisher<GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  traditional_armor::TraditionalDetector detector_;
  auto_aim::ClosedLoopSolver closed_loop_;

  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  bool info_received_ = false;

  double gimbal_yaw_ = 0.0;
  double gimbal_pitch_ = 0.0;
  bool imu_received_ = false;
  int64_t last_imu_ms_ = 0;
  int imu_timeout_ms_ = 100;

  std::string last_overlay_text_;  // 最近一次有效帧的叠加文本，用于持续显示

  GimbalCmd latest_cmd_;
};

}  // namespace armor_track_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(rclcpp::get_logger("armor_track_node"),
                 "用法: armor_track_node <config.yaml>");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(std::make_shared<armor_track_ros2::AutoAimNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
