#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "armor_track_ros2/msg/gimbal_cmd.hpp"
#include "armor_track_ros2/serial_driver.hpp"

using namespace std::chrono_literals;

namespace
{
// 把 GimbalCmd（rad）装填进串口发送帧。
// 注意：符号与单位需与电控固件约定一致，此处沿用 serial_driver 源码的
//   Pitch/YawAbsoluteAngle = -期望角（rad） 的写法；如电控侧相反，改符号即可。
serial_driver::SendPacket makeSendPacket(
  const armor_track_ros2::msg::GimbalCmd & cmd, uint32_t now_ms)
{
  serial_driver::SendPacket pkt{};
  pkt._SOF = serial_driver::SERIAL_HEADER;
  pkt.ID = serial_driver::RMOS_SEND_ID;
  pkt.AimbotState = cmd.control ? 0x01 : 0x00;  // bit0 = 锁定（capture_cmd）
  pkt.AimbotTarget = 0;                         // 目标编号，暂不区分
  pkt.PitchAbsoluteAngle = -static_cast<float>(cmd.pitch);
  pkt.YawAbsoluteAngle = -static_cast<float>(cmd.yaw);
  pkt.distance = static_cast<float>(cmd.distance);
  pkt.TargetPitchSpeed = static_cast<float>(cmd.pitch_rate);
  pkt.TargetYawSpeed = static_cast<float>(cmd.yaw_rate);
  pkt.SystemTimer = now_ms;
  pkt._EOF = serial_driver::SERIAL_END;
  return pkt;
}
}  // namespace

class SerialNode : public rclcpp::Node
{
public:
  explicit SerialNode(const std::string & config_path)
  : Node("serial_node"),
    start_time_(std::chrono::steady_clock::now())
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

    // 接收线程：持续读串口防止缓冲积满，并据此判定连接状态
    reader_ = std::thread([this]() { readLoop(); });

    // 断连检测定时器（每 1s 检查一次）
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

    auto now_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count());

    auto pkt = makeSendPacket(*msg, now_ms);
    if (!driver_.send(pkt)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口发送失败");
    }
  }

  void readLoop()
  {
    while (!stop_) {
      if (driver_.isOpen() && driver_.poll()) {
        last_rx_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  void checkTimer()
  {
    if (!driver_.isOpen()) return;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now_ms - last_rx_ms_ > 2000) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "2s 未收到电控数据，检查串口连接");
    }
  }

  serial_driver::SerialDriver driver_;
  rclcpp::Subscription<armor_track_ros2::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::thread reader_;
  std::atomic<bool> stop_{false};
  std::atomic<int64_t> last_rx_ms_{0};
  std::chrono::steady_clock::time_point start_time_;
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
