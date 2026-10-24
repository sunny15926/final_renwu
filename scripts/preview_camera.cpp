// 实时预览相机画面（独立于 ROS），在你桌面弹窗显示。
// 按键：q / ESC 退出；+ / - 调曝光。
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

  std::string serial = argc > 1 ? argv[1] : devs.front().serial;
  if (!cam.open(serial)) {
    printf("打开失败: %s\n", cam.lastError().c_str());
    return 1;
  }
  if (!cam.startStream()) {
    printf("取流失败: %s\n", cam.lastError().c_str());
    return 1;
  }

  double exposure = 3000.0;  // us，与 config 一致
  double gain = 8.0;         // dB
  cam.setExposure(exposure);
  cam.setGain(gain);
  printf("实时预览中… q/ESC 退出，+/- 调曝光，初始曝光 %.0fus 增益 %.1fdB\n", exposure, gain);

  cv::Mat bgr;
  double fps = 0.0;
  while (true) {
    double t0 = static_cast<double>(cv::getTickCount());
    if (!cam.grab(bgr, 1000)) {
      printf("取帧失败: %s\n", cam.lastError().c_str());
      break;
    }

    char txt[128];
    snprintf(txt, sizeof(txt), "exp %.0fus gain %.1fdB mean %.0f fps %.1f",
             exposure, gain, cv::mean(bgr)[0], fps);
    cv::putText(bgr, txt, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                cv::Scalar(0, 255, 0), 2);
    cv::imshow("daheng_camera", bgr);

    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    if (key == '+' || key == '=') { exposure *= 1.5; cam.setExposure(exposure); }
    if (key == '-' || key == '_') { exposure /= 1.5; cam.setExposure(exposure); }

    double t1 = static_cast<double>(cv::getTickCount());
    fps = cv::getTickFrequency() / (t1 - t0);
  }

  cam.stopStream();
  cam.close();
  printf("预览结束\n");
  return 0;
}
