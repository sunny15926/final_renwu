// 曝光/增益扫描：跑完整检测流水线，把每档 (曝光, 增益) 下的矫正图与二值化数字存盘，
// 用于离线判断「数字可见」的最佳曝光。
//
// 编译（在仓库根目录）：
//   g++ -std=c++17 -O2 -I include -I Digital-recognition-master/include \
//       -I /home/sunny/onnxruntime-linux-x64-1.23.2/include \
//       $(pkg-config --cflags opencv4) \
//       scripts/exposure_sweep.cpp src/daheng_camera.cpp src/traditional_detector.cpp \
//       Digital-recognition-master/src/DigitalRecognition.cpp \
//       -o build/exposure_sweep \
//       $(pkg-config --libs opencv4) -lyaml-cpp -lgxiapi \
//       -L /home/sunny/onnxruntime-linux-x64-1.23.2/lib -lonnxruntime \
//       -Wl,-rpath,/home/sunny/onnxruntime-linux-x64-1.23.2/lib
#include <opencv2/opencv.hpp>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "armor_track_ros2/daheng_camera.hpp"
#include "armor_track_ros2/traditional_detector.hpp"

using namespace armor_track_ros2;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    printf("用法: exposure_sweep <serial> <config.yaml> [out_dir]\n");
    return 1;
  }
  std::string serial = argv[1];
  std::string config = argv[2];
  std::string out_dir = (argc > 3) ? argv[3] : "/tmp/exposure_sweep";

  DahengCamera cam;
  if (!cam.initLib()) { printf("initLib 失败: %s\n", cam.lastError().c_str()); return 1; }
  if (!cam.open(serial)) { printf("打开失败: %s\n", cam.lastError().c_str()); return 1; }
  if (!cam.startStream()) { printf("取流失败: %s\n", cam.lastError().c_str()); return 1; }
  printf("打开成功 %dx%d\n", cam.width(), cam.height());

  traditional_armor::TraditionalDetector detector(config);

  std::filesystem::create_directories(out_dir);

  struct Setting { double exp, gain; };
  std::vector<Setting> grid;
  for (double e : {2000.0, 3000.0, 4000.0, 6000.0, 8000.0, 10000.0})
    for (double g : {8.0, 12.0, 16.0, 20.0})
      grid.push_back({e, g});

  cv::Mat bgr;
  for (auto & s : grid) {
    cam.setExposure(s.exp);
    cam.setGain(s.gain);
    for (int i = 0; i < 3; ++i) cam.grab(bgr, 1000);  // 等曝光稳定
    if (!cam.grab(bgr, 1000)) {
      printf("取帧失败 exp=%.0f gain=%.1f\n", s.exp, s.gain);
      continue;
    }

    detector.detect(bgr);
    const cv::Mat & warp = detector.last_warp();
    const cv::Mat & digit = detector.last_digit();

    char tag[64];
    snprintf(tag, sizeof(tag), "e%05.0f_g%04.1f", s.exp, s.gain);
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    double mean = cv::mean(gray)[0];
    int digit_area = digit.empty() ? -1 : cv::countNonZero(digit);
    printf("%s  full_mean=%.0f  warp=%dx%d  digit_area=%d\n",
           tag, mean, warp.cols, warp.rows, digit_area);

    auto p = std::filesystem::path(out_dir);
    cv::imwrite((p / (std::string(tag) + "_full.png")).string(), bgr);
    if (!warp.empty()) cv::imwrite((p / (std::string(tag) + "_warp.png")).string(), warp);
    if (!digit.empty()) cv::imwrite((p / (std::string(tag) + "_digit.png")).string(), digit);
  }

  cam.stopStream();
  cam.close();
  printf("扫描完成，输出目录: %s\n", out_dir.c_str());
  return 0;
}
