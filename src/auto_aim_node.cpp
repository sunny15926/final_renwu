#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include "armor_track_ros2/msg/gimbal_cmd.hpp"
#include "armor_track_ros2/traditional_detector.hpp"
#include "auto_aim/armor.hpp"
#include "auto_aim/solver.hpp"
#include "auto_aim/target.hpp"
#include "tools/math_tools.hpp"

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

// 传统检测器数字（"0"~"5" / "?"）映射到 auto_aim 的 ArmorName。
// 传统模型仅 6 类（数字 0~5），0 通常对应基地/前哨；"?" 表示识别失败。
auto_aim::ArmorName nameFromDigit(const std::string & digit)
{
  if (digit == "1") return auto_aim::one;
  if (digit == "2") return auto_aim::two;
  if (digit == "3") return auto_aim::three;
  if (digit == "4") return auto_aim::four;
  if (digit == "5") return auto_aim::five;
  if (digit == "0") return auto_aim::base;
  return auto_aim::not_armor;
}
}  // namespace

namespace armor_track_ros2
{
using GimbalCmd = armor_track_ros2::msg::GimbalCmd;

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const std::string & config_path)
  : Node("armor_track_node"),
    detector_(config_path),
    solver_(config_path)
  {
    auto yaml = YAML::LoadFile(config_path);
    latency_ms_ = yaml["latency_ms"].as<double>();
    yaw_offset_ = yaml["yaw_offset"].as<double>();
    pitch_offset_ = yaml["pitch_offset"].as<double>();
    max_lost_count_ = yaml["max_lost_count"].as<int>();
    min_lock_count_ = yaml["min_lock_count"].as<int>();
    double publish_rate = yaml["publish_rate"].as<double>();

    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::imageCallback, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::infoCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<GimbalCmd>("gimbal_cmd", 10);

    auto period = publish_rate > 0.0
                    ? std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::duration<double>(1.0 / publish_rate))
                    : std::chrono::milliseconds(33);
    timer_ = create_wall_timer(period, std::bind(&AutoAimNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "armor_track_node started, config: %s", config_path.c_str());
  }

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_matrix_ = toCvMat33(msg->k);
    distort_coeffs_ = toCvMatD(msg->d);
    solver_.set_camera(camera_matrix_, distort_coeffs_);
    info_received_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "waiting for camera_info ...");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    const cv::Mat & bgr = cv_ptr->image;

    // ① 传统装甲识别：HSV 分割 -> 灯条配对 -> 装甲四点 + 数字
    auto armors = detector_.detect(bgr);
    if (armors.empty()) {
      lost_count_++;
      lock_count_ = 0;
      return;
    }

    // 取第一个目标（灯条已按 x 排序；多目标时这里可改为选置信度最高/最靠近中心者）
    const auto & best = armors.front();

    // ② 映射到 auto_aim::Armor（points 已是 [左上, 右上, 右下, 左下] 的 PnP 顺序）
    auto_aim::Armor armor;
    armor.points = best.points;
    armor.center = best.center;
    armor.name = nameFromDigit(best.digit);
    armor.confidence = best.match_score;
    armor.type = auto_aim::ArmorType::small;  // 手持板为小装甲，如需大装甲按尺寸判定

    // ③ PnP + 坐标变换 -> ypd_in_world
    solver_.solve(armor);
    auto ypd = armor.ypd_in_world;

    // ④ 6 维 EKF 更新
    auto now = std::chrono::steady_clock::now();
    if (!target_.initialized()) {
      target_.init(armor.xyz_in_world, 0.0);
      last_time_ = now;
    } else {
      double dt = tools::delta_time(now, last_time_);
      last_time_ = now;
      target_.update(ypd, dt);
    }

    // ⑤ 延迟补偿：用速度外推到下发时刻
    double dt_lat = latency_ms_ / 1000.0;
    Eigen::Vector3d compensated = target_.position() + target_.linear_velocity() * dt_lat;
    auto ypd_cmd = tools::xyz2ypd(compensated);

    lock_count_++;
    lost_count_ = 0;

    // 组装下发指令
    latest_cmd_.control = (lock_count_ >= min_lock_count_);
    latest_cmd_.yaw = ypd_cmd[0] + yaw_offset_;
    latest_cmd_.pitch = ypd_cmd[1] + pitch_offset_;
    latest_cmd_.distance = ypd_cmd[2];
    auto v = target_.linear_velocity();
    latest_cmd_.vx = v[0];
    latest_cmd_.vy = v[1];
    latest_cmd_.vz = v[2];
    auto w = target_.angular_velocity();
    latest_cmd_.yaw_rate = w[0];
    latest_cmd_.pitch_rate = w[1];
  }

  void timerCallback()
  {
    if (lost_count_ >= max_lost_count_) latest_cmd_.control = false;
    cmd_pub_->publish(latest_cmd_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  traditional_armor::TraditionalDetector detector_;
  auto_aim::Solver solver_;
  auto_aim::Target target_;

  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  bool info_received_ = false;

  double latency_ms_;
  double yaw_offset_;
  double pitch_offset_;
  int max_lost_count_;
  int min_lock_count_;
  int lock_count_ = 0;
  int lost_count_ = 0;

  std::chrono::steady_clock::time_point last_time_;
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
