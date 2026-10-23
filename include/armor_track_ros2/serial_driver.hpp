#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace serial_driver
{

// 帧协议常量（与 serial_driver 源码一致：0x55 帧头 + 0xFF 帧尾，无 CRC）
constexpr uint8_t SERIAL_HEADER = 0x55;
constexpr uint8_t SERIAL_END = 0xFF;
constexpr uint8_t RMOS_RECEIVE_ID = 0x01;  // 电控 -> 视觉
constexpr uint8_t RMOS_SEND_ID = 0x02;     // 视觉 -> 电控

// 视觉端 -> 电控 发送帧（字段与 serial_driver 源码 SendPacket 保持一致）
struct SendPacket
{
  uint8_t _SOF;                // 帧头 0x55
  uint8_t ID;                  // 0x02
  uint8_t AimbotState;         // bit0=锁定(capture_cmd)，bit3=开火(is_fire)
  uint8_t AimbotTarget;        // 目标编号位图（bit7/bit1/... 对应不同装甲编号）
  float PitchAbsoluteAngle;    // pitch 期望角（rad）
  float YawAbsoluteAngle;      // yaw 期望角（rad）
  float distance;              // 目标距离（m）
  float TargetPitchSpeed;      // 目标 pitch 角速度（rad/s）
  float TargetYawSpeed;        // 目标 yaw 角速度（rad/s）
  uint32_t SystemTimer;        // 系统时间戳（ms）
  uint8_t _EOF;                // 帧尾 0xFF
} __attribute__((packed));

// 电控 -> 视觉 接收帧（惯导四元数 + 机器人信息），当前仅作帧对齐/连接检测用
struct ReceivePacket
{
  uint8_t _SOF;                // 帧头 0x55
  uint8_t ID;                  // 0x01
  uint32_t TimeStamp;          // 电控时间戳（ms）
  float q0, q1, q2, q3;        // 云台四元数（w x y z）
  uint8_t robot_id;            // 机器人 ID
  uint8_t mode;                // 自瞄模式位
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
  bool send(const SendPacket & packet);

  // 非阻塞读取并丢弃接收字节（防止接收缓冲积满），有数据返回 true。
  // 由接收线程周期性调用。
  bool poll();

private:
  int openPort(const char * port);
  int configurePort(int fd, int baud);

  int fd_ = -1;
  std::vector<uint8_t> rx_buf_;  // 接收临时缓冲
};

}  // namespace serial_driver
