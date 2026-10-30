#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace serial_driver
{

// 帧协议常量（与电控协议一致：0x55 帧头 + 0xFF 帧尾，无 CRC）
constexpr uint8_t SERIAL_HEADER = 0x55;
constexpr uint8_t SERIAL_END = 0xFF;
constexpr uint8_t GIMBAL_IMU_ID = 0x01;  // 电控 -> 视觉（云台 IMU 四元数）
constexpr uint8_t AIMBOT_ID = 0x02;      // 视觉 -> 电控（云台期望角）

// 视觉端 -> 电控：云台期望角（相对云台零位，rad）。
// 跟随规则：持续下发 = 跟随，停止下发 = 不跟随（见 serial_node.cpp）。
struct AimbotFrame
{
  uint8_t _SOF;                // 帧头 0x55
  uint8_t ID;                  // 0x02
  float PitchRelativeAngle;    // pitch 相对角（rad，正 pitch 向上）
  float YawRelativeAngle;      // yaw 相对角（rad，正 yaw 向右）
  uint8_t _EOF;                // 帧尾 0xFF
} __attribute__((packed));

// 电控 -> 视觉：云台 IMU 四元数（当前仅作帧对齐/连接检测，内容未解析）
struct GimbalImuFrame
{
  uint8_t _SOF;                // 帧头 0x55
  uint8_t ID;                  // 0x01
  float q0, q1, q2, q3;        // 四元数（w x y z）
  uint8_t _EOF;                // 帧尾 0xFF
} __attribute__((packed));

// 轻量串口驱动：只依赖 POSIX（termios/ioctl），无 ROS、无 utils 框架依赖。
class SerialDriver
{
public:
  SerialDriver() = default;
  ~SerialDriver();

  // 打开并配置串口（8N1）；失败返回 false
  bool open(const std::string & port, int baud = 460800);

  void close();

  bool isOpen() const { return fd_ >= 0; }

  // 发送一帧；失败返回 false
  bool send(const AimbotFrame & packet);

  // 非阻塞读取：把当前可读字节追加到 out；读到数据返回 true。由接收线程周期性调用。
  bool read(std::vector<uint8_t> & out);

private:
  int openPort(const char * port);
  int configurePort(int fd, int baud);

  int fd_ = -1;
};

}  // namespace serial_driver
