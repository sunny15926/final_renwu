// 独立相机测试：不依赖 ROS / camera_info_manager，直接用 DahengCamera 取几帧存成 PNG。
// 编译见下方命令；运行后把一帧图存到 /tmp/daheng_frame.png。
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <string>

#include "armor_track_ros2/daheng_camera.hpp"

int main(int argc, char ** argv)
{
  using namespace armor_track_ros2;
  DahengCamera cam;

  if (!cam.initLib()) {
    printf("initLib 失败: %s\n", cam.lastError().c_str());
    return 1;
  }

  auto devs = cam.enumerate();
  if (devs.empty()) {
    printf("未找到相机: %s\n", cam.lastError().c_str());
    return 1;
  }
  for (auto & d : devs) {
    printf("设备: 型号=%s 序列号=%s\n", d.model.c_str(), d.serial.c_str());
  }

  std::string serial = (argc > 1) ? argv[1] : devs.front().serial;
  if (!cam.open(serial)) {
    printf("打开失败: %s\n", cam.lastError().c_str());
    return 1;
  }
  printf("打开成功 %dx%d\n", cam.width(), cam.height());

  if (!cam.startStream()) {
    printf("取流失败: %s\n", cam.lastError().c_str());
    return 1;
  }

  // 固定曝光/增益（与 config/daheng_camera.yaml 一致），否则默认偏暗
  cam.setExposure(3000.0);
  cam.setGain(8.0);

  // 连续取几帧，跳过曝光未稳定的前几帧
  cv::Mat bgr;
  for (int i = 0; i < 5; ++i) {
    if (!cam.grab(bgr, 1000)) {
      printf("取帧 %d 失败: %s\n", i, cam.lastError().c_str());
      return 1;
    }
  }

  std::string out = argc > 2 ? argv[2] : "/tmp/daheng_frame.png";
  cv::imwrite(out, bgr);
  printf("已保存一帧到 %s (%dx%d)\n", out.c_str(), bgr.cols, bgr.rows);

  cam.stopStream();
  cam.close();
  return 0;
}
