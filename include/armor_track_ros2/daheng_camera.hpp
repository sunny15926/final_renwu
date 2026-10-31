#pragma once

// 大恒（Galaxy）工业相机封装（从 dahengshibie-main 移植，去掉 rm_log/spdlog 依赖）。
// 只负责：枚举设备 / 按序列号打开 / 配置参数 / 取流转 cv::Mat（Bayer -> BGR）/ 释放。
// 出错信息通过 lastError() 抛出，由调用方（ROS 节点）决定怎么打印。
//
// 典型流程：
//   DahengCamera cam;
//   cam.initLib();               // 1. 全局库初始化（最先）
//   cam.enumerate();             // 2. 枚举，拿序列号（可选）
//   cam.open("<序列号>");         // 3. 打开并配置相机
//   cam.startStream();           // 4. 开始取流
//   cam.grab(mat);               // 5. 循环取帧（内部已做 Bayer -> BGR）
//   cam.close();                 // 6. 停流 + 关设备 + 关库

#include <opencv2/opencv.hpp>
#include <sdk/GxIAPI.h>       // 大恒 Galaxy SDK 主接口（头文件内置在 include/sdk/）
#include <sdk/DxImageProc.h>  // DxRaw8toRGB24Ex 等

#include <cstdint>
#include <string>
#include <vector>

namespace armor_track_ros2
{

struct DahengDeviceInfo
{
  std::string serial;
  std::string model;
};

class DahengCamera
{
public:
  DahengCamera() = default;
  ~DahengCamera();

  // ---- 生命周期 ----
  bool initLib();                                   // 全局库初始化，与 close() 配对
  std::vector<DahengDeviceInfo> enumerate();        // 枚举，返回 (序列号, 型号)
  bool open(const std::string & serial);            // 按序列号打开并配置相机
  bool startStream();                               // 开始取流
  bool grab(cv::Mat & image, int timeout_ms = 1000);  // 取一帧（Bayer -> BGR）
  void stopStream();                                // 停流（释放转换缓冲）
  void close();                                     // 关相机 + 关库

  // ---- 属性 ----
  int width() const { return width_; }
  int height() const { return height_; }
  const std::string & lastError() const { return last_error_; }

  // ---- 运行时调参 ----
  bool setExposure(double us);  // 固定曝光（短曝光避免锁死帧率，按现场光照标定）
  bool setGain(double db);      // 固定增益

private:
  void setError(const std::string & what);

  GX_DEV_HANDLE device_ = nullptr;         // 设备句柄
  GX_DS_HANDLE stream_handle_ = nullptr;   // 数据流句柄
  int width_ = 0;
  int height_ = 0;
  int64_t color_filter_ = GX_COLOR_FILTER_NONE;  // Bayer 排列（RG/GB/GR/BG）

  // 双缓冲：交替写两块 RGB 输出，返回帧在下次取帧前有效，避免每帧 clone
  unsigned char * rgb_buffer_[2] = {nullptr, nullptr};
  int rgb_buffer_index_ = 0;

  bool lib_initialized_ = false;
  bool device_opened_ = false;
  bool streaming_ = false;

  std::string last_error_;
};

}  // namespace armor_track_ros2
