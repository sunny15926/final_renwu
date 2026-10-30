#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "armor_track_ros2/msg/gimbal_cmd.hpp"
#include "armor_track_ros2/msg/gimbal_feedback.hpp"
#include "armor_track_ros2/serial_driver.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double PI = 3.14159265358979323846;

// ===== 角度符号开关（真机方向反了，把对应项改成 -1 即可）=====
// 约定：yaw 向右为正、pitch 向上为正（与电控一致）。
constexpr double IMU_YAW_SIGN = 1.0;    // 电控回传 yaw 的符号（四元数 -> yaw 后乘）
constexpr double IMU_PITCH_SIGN = 1.0;  // 电控回传 pitch 的符号
constexpr double CMD_YAW_SIGN = 1.0;    // 下发 yaw 的符号
constexpr double CMD_PITCH_SIGN = -1.0;  // 下发 pitch 的符号（上下跟随方向反了，取反）

// 把 GimbalCmd（rad）装填进串口发送帧。
// 符号与电控约定一致：Pitch/YawRelativeAngle = 期望角（rad），正 yaw 向右、正 pitch 向上。
serial_driver::AimbotFrame makeSendPacket(
  const armor_track_ros2::msg::GimbalCmd & cmd)
{
  serial_driver::AimbotFrame pkt{};
  pkt._SOF = serial_driver::SERIAL_HEADER;
  pkt.ID = serial_driver::AIMBOT_ID;
  pkt.PitchRelativeAngle = static_cast<float>(CMD_PITCH_SIGN * cmd.pitch);
  pkt.YawRelativeAngle = static_cast<float>(CMD_YAW_SIGN * cmd.yaw);
  pkt._EOF = serial_driver::SERIAL_END;
  return pkt;
}

// 四元数 (q0=qw, q1=qx, q2=qy, q3=qz) -> Z-Y-X 欧拉角，返回 {yaw, pitch}（rad）。
// 与 scripts/gimbal_state.py 的 quat_to_ypr 一致。yaw 向右为正、pitch 向上为正。
std::pair<double, double> quat_to_yaw_pitch(const serial_driver::GimbalImuFrame & f)
{
  double w = f.q0, x = f.q1, y = f.q2, z = f.q3;
  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  double sinp = 2.0 * (w * y - z * x);
  double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(PI / 2.0, sinp) : std::asin(sinp);
  return {IMU_YAW_SIGN * yaw, IMU_PITCH_SIGN * pitch};
}
}  // namespace

class SerialNode : public rclcpp::Node
{
public:
  explicit SerialNode(const std::string & config_path)
  : Node("serial_node")
  {
    auto yaml = YAML::LoadFile(config_path);
    std::string port = yaml["serial_port"].as<std::string>("/dev/ttyACM0");
    int baud = yaml["serial_baud"].as<int>(460800);

    if (!driver_.open(port, baud)) {
      RCLCPP_ERROR(get_logger(), "无法打开串口 %s @ %d，将不断重试", port.c_str(), baud);
    } else {
      RCLCPP_INFO(get_logger(), "串口已打开 %s @ %d (8N1)", port.c_str(), baud);
    }

    cmd_sub_ = create_subscription<armor_track_ros2::msg::GimbalCmd>(
      "gimbal_cmd", 10,
      std::bind(&SerialNode::cmdCallback, this, std::placeholders::_1));

    feedback_pub_ = create_publisher<armor_track_ros2::msg::GimbalFeedback>(
      "gimbal_feedback", 10);

    // 接收线程：持续读串口，解析 IMU 帧并发布 yaw/pitch。
    reader_ = std::thread([this]() { readLoop(); });

    // 断连检测定时器（每 1s 检查一次）。
    check_timer_ = create_wall_timer(1s, std::bind(&SerialNode::checkTimer, this));
  }

  ~SerialNode() override
  {
    stop_ = true;
    if (reader_.joinable()) reader_.join();
    driver_.close();
  }

private:
  void cmdCallback(const armor_track_ros2::msg::GimbalCmd::SharedPtr msg)
  {
    if (!driver_.isOpen()) {
      return;  // 未打开串口时静默丢弃（打开失败会在构造时告警）
    }

    // 跟随规则：control=true 才持续下发；false 停发 = 不跟随。
    if (!msg->control) {
      return;
    }

    auto pkt = makeSendPacket(*msg);
    if (!driver_.send(pkt)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口发送失败");
    }
  }

  void readLoop()
  {
    while (!stop_) {
      if (driver_.isOpen() && driver_.read(rx_buf_)) {
        last_rx_ms_ = nowMs();
        parseFrames();
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  // 从 rx_buf_ 前部解析完整 IMU 帧并发布 yaw/pitch；解析一帧删一帧。
  void parseFrames()
  {
    const size_t frame_size = sizeof(serial_driver::GimbalImuFrame);
    while (rx_buf_.size() >= frame_size) {
      if (rx_buf_[0] == serial_driver::SERIAL_HEADER &&
          rx_buf_[1] == serial_driver::GIMBAL_IMU_ID &&
          rx_buf_[frame_size - 1] == serial_driver::SERIAL_END) {
        serial_driver::GimbalImuFrame frame;
        std::memcpy(&frame, rx_buf_.data(), frame_size);
        auto [yaw, pitch] = quat_to_yaw_pitch(frame);
        armor_track_ros2::msg::GimbalFeedback msg;
        msg.yaw = yaw;
        msg.pitch = pitch;
        feedback_pub_->publish(msg);
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + frame_size);
      } else {
        rx_buf_.erase(rx_buf_.begin());  // 错位，丢一字节重新对齐
      }
    }
  }

  void checkTimer()
  {
    if (!driver_.isOpen()) return;
    if (nowMs() - last_rx_ms_ > 2000) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "2s 未收到电控数据，检查串口连接");
    }
  }

  static int64_t nowMs()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  serial_driver::SerialDriver driver_;
  rclcpp::Subscription<armor_track_ros2::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<armor_track_ros2::msg::GimbalFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::thread reader_;
  std::vector<uint8_t> rx_buf_;
  std::atomic<bool> stop_{false};
  std::atomic<int64_t> last_rx_ms_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_node"),
                 "用法: serial_node <config.yaml>");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(std::make_shared<SerialNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
