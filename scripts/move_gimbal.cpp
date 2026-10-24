// C++ 版云台最小发包测试（等价于 scripts/move_gimbal.py），用同一套 serial_driver 代码。
// 只发 AimbotFrame：0x55 头 / ID=0x02 / pitch,yaw 两个 float（rad，取负）/ 0xFF 尾，共 11 字节小端。
// 跟随规则：持续发包 = 跟随；停止发包 = 不跟随，所以连发 duration 秒后停发。
//
// 编译（不依赖 ROS，仅 POSIX + 本包头文件）：
//   g++ -std=c++17 -I include scripts/move_gimbal.cpp src/serial_driver.cpp -o build/move_gimbal
// 用法：
//   ./build/move_gimbal                          # 默认 yaw=0.1 rad 动一下
//   ./build/move_gimbal /dev/ttyACM0 0.1 0.0 100 2   # port yaw pitch hz duration

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>

#include "armor_track_ros2/serial_driver.hpp"

using serial_driver::SerialDriver;
using serial_driver::AimbotFrame;

int main(int argc, char ** argv)
{
  const char * port = argc > 1 ? argv[1] : "/dev/ttyACM0";
  float yaw = argc > 2 ? static_cast<float>(atof(argv[2])) : 0.1f;
  float pitch = argc > 3 ? static_cast<float>(atof(argv[3])) : 0.0f;
  double hz = argc > 4 ? atof(argv[4]) : 100.0;
  double duration = argc > 5 ? atof(argv[5]) : 2.0;

  SerialDriver driver;
  if (!driver.open(port, 460800)) {
    printf("打开串口失败: %s\n", port);
    return 1;
  }

  AimbotFrame pkt{};
  pkt._SOF = serial_driver::SERIAL_HEADER;   // 0x55
  pkt.ID = serial_driver::AIMBOT_ID;         // 0x02
  pkt.PitchRelativeAngle = -pitch;           // 与 Python 一致，取负
  pkt.YawRelativeAngle = -yaw;
  pkt._EOF = serial_driver::SERIAL_END;      // 0xFF

  printf("向 %s@460800 下发 AimbotFrame：pitch=%.3frad yaw=%.3frad，%.0fHz 持续 %.0fs\n",
         port, pitch, yaw, hz, duration);

  auto period = std::chrono::microseconds(static_cast<long long>(1e6 / hz));
  auto end = std::chrono::steady_clock::now() +
             std::chrono::microseconds(static_cast<long long>(duration * 1e6));
  long n = 0;
  while (std::chrono::steady_clock::now() < end) {
    if (!driver.send(pkt)) {
      printf("警告: 发送失败\n");
    }
    ++n;
    std::this_thread::sleep_for(period);
  }

  driver.close();
  printf("完成：共下发 %ld 帧（已停发 = 不跟随）\n", n);
  return 0;
}
