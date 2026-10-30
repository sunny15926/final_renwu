#include "armor_track_ros2/daheng_camera.hpp"

#include <cstdlib>
#include <cstring>

namespace
{

// 大恒错误码 -> 人类可读描述（SDK 提供 GXGetLastError，无需自己查表）
std::string getErrorString(GX_STATUS status)
{
  char * error_info = nullptr;
  size_t size = 0;
  // 第一次调用只取字符串长度
  if (GXGetLastError(&status, nullptr, &size) != GX_STATUS_SUCCESS) {
    return "<无法获取 GXGetLastError 描述>";
  }
  error_info = new char[size];
  // 第二次调用取内容
  GX_STATUS s = GXGetLastError(&status, error_info, &size);
  std::string msg = error_info != nullptr ? error_info : "";
  delete[] error_info;
  return s == GX_STATUS_SUCCESS ? msg : "<无法获取 GXGetLastError 描述>";
}

}  // namespace

namespace armor_track_ros2
{

DahengCamera::~DahengCamera() { close(); }

void DahengCamera::setError(const std::string & what)
{
  last_error_ = what;
}

bool DahengCamera::initLib()
{
  GX_STATUS status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    setError("GXInitLib 失败: " + getErrorString(status));
    return false;
  }
  lib_initialized_ = true;
  return true;
}

std::vector<DahengDeviceInfo> DahengCamera::enumerate()
{
  std::vector<DahengDeviceInfo> devices;

  uint32_t device_num = 0;
  // GigE 相机响应慢，枚举超时官方建议至少 1000ms
  GX_STATUS status = GXUpdateAllDeviceList(&device_num, 1000);
  if (status != GX_STATUS_SUCCESS || device_num == 0) {
    setError("枚举失败或未找到设备: " + getErrorString(status));
    return devices;
  }

  for (uint32_t i = 1; i <= device_num; ++i) {  // 大恒设备序号从 1 开始
    GX_DEVICE_INFO info;
    memset(&info, 0, sizeof(info));
    if (GXGetDeviceInfo(i, &info) != GX_STATUS_SUCCESS) {
      continue;
    }
    // 只支持 USB3 相机（U3V）
    if (info.emDevType == GX_DEVICE_CLASS_U3V) {
      auto & u3v = info.DevInfo.stU3VDevInfo;
      std::string model = reinterpret_cast<const char *>(u3v.chModelName);
      std::string serial = reinterpret_cast<const char *>(u3v.chSerialNumber);
      devices.push_back({serial, model});
    }
  }
  return devices;
}

bool DahengCamera::open(const std::string & serial)
{
  // 枚举后按序列号定位 1 开始的大恒设备序号
  auto devices = enumerate();
  int index = -1;
  for (size_t i = 0; i < devices.size(); ++i) {
    if (devices[i].serial == serial) {
      index = static_cast<int>(i) + 1;
      break;
    }
  }
  if (index < 0) {
    setError("序列号 '" + serial + "' 不在枚举列表中");
    return false;
  }

  GX_STATUS status = GXOpenDeviceByIndex(index, &device_);
  if (status != GX_STATUS_SUCCESS) {
    setError("GXOpenDeviceByIndex: " + getErrorString(status));
    return false;
  }
  device_opened_ = true;

  // 1. 自动挡全关（调参前必须关）
  GXSetEnumValueByString(device_, "ExposureAuto", "Off");
  GXSetEnumValueByString(device_, "GainAuto", "Off");
  GXSetEnumValueByString(device_, "BalanceWhiteAuto", "Continuous");

  // 2. 连续采集 + 关触发
  GXSetEnumValueByString(device_, "AcquisitionMode", "Continuous");
  GXSetEnumValueByString(device_, "TriggerMode", "Off");

  // 3. 像素格式：传感器原始 Bayer RG8
  GXSetEnumValue(device_, "PixelFormat", GX_PIXEL_FORMAT_BAYER_RG8);

  // 4. USB3 相机出厂限速，把 DeviceLinkThroughputLimit 拉到最大，否则帧率上不去
  GX_INT_VALUE limit_node;
  memset(&limit_node, 0, sizeof(limit_node));
  if (GXGetIntValue(device_, "DeviceLinkThroughputLimit", &limit_node) == GX_STATUS_SUCCESS) {
    GXSetIntValue(device_, "DeviceLinkThroughputLimit", limit_node.nMax);
  }

  // 5. 读分辨率（设置 PixelFormat 之后再读）
  GX_INT_VALUE width_node, height_node;
  memset(&width_node, 0, sizeof(width_node));
  memset(&height_node, 0, sizeof(height_node));
  GXGetIntValue(device_, "Width", &width_node);
  GXGetIntValue(device_, "Height", &height_node);
  width_ = static_cast<int>(width_node.nCurValue);
  height_ = static_cast<int>(height_node.nCurValue);

  // 6. Bayer 转换前查滤镜排列（RG/GB/GR/BG），传错会导致颜色错乱
  GX_ENUM_VALUE filter_value;
  memset(&filter_value, 0, sizeof(filter_value));
  if (GXGetEnumValue(device_, "PixelColorFilter", &filter_value) == GX_STATUS_SUCCESS) {
    color_filter_ = filter_value.stCurValue.nCurValue;
  }

  return true;
}

bool DahengCamera::startStream()
{
  // 大恒的"流"概念：payload（单帧字节数）从数据流句柄查询，本项目只取 1 号流
  uint32_t stream_num = 0;
  if (GXGetDataStreamNumFromDev(device_, &stream_num) != GX_STATUS_SUCCESS || stream_num < 1) {
    setError("获取数据流失败");
    return false;
  }
  // SDK 内部取流缓存 5 个，并分配 Bayer -> RGB 转换输出缓冲
  GXSetAcqusitionBufferNumber(device_, 5);
  rgb_buffer_[0] = static_cast<unsigned char *>(
    malloc(sizeof(unsigned char) * width_ * height_ * 3));
  rgb_buffer_[1] = static_cast<unsigned char *>(
    malloc(sizeof(unsigned char) * width_ * height_ * 3));
  rgb_buffer_index_ = 0;

  GX_STATUS status = GXStreamOn(device_);
  if (status != GX_STATUS_SUCCESS) {
    setError("GXStreamOn: " + getErrorString(status));
    free(rgb_buffer_[0]);
    free(rgb_buffer_[1]);
    rgb_buffer_[0] = nullptr;
    rgb_buffer_[1] = nullptr;
    return false;
  }
  streaming_ = true;
  return true;
}

bool DahengCamera::grab(cv::Mat & image, int timeout_ms)
{
  PGX_FRAME_BUFFER frame_buffer = nullptr;
  // DQ = DeQueue，从 SDK 队列取出一帧
  GX_STATUS status = GXDQBuf(device_, &frame_buffer, timeout_ms);
  if (status != GX_STATUS_SUCCESS) {
    setError("GXDQBuf 超时/失败: " + getErrorString(status));
    return false;
  }

  // 帧状态检查：丢包/传输错误的帧跳过，但必须照常 QB 还回去
  if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
    setError("帧状态异常");
    GXQBuf(device_, frame_buffer);
    return false;
  }

  // Bayer RG8 -> BGR24（邻域插值，不翻转，输出 OpenCV BGR 通道序）
  unsigned char * rgb_out = rgb_buffer_[rgb_buffer_index_];
  VxInt32 dx_status = DxRaw8toRGB24Ex(
    frame_buffer->pImgBuf, rgb_out, frame_buffer->nWidth, frame_buffer->nHeight,
    RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(color_filter_), false, DX_ORDER_BGR);
  if (dx_status != DX_OK) {
    setError("DxRaw8toRGB24Ex 失败");
    GXQBuf(device_, frame_buffer);
    return false;
  }

  // 直接共享转换缓冲（不 clone），下次取帧改用另一块缓冲
  image = cv::Mat(frame_buffer->nHeight, frame_buffer->nWidth, CV_8UC3, rgb_out);
  rgb_buffer_index_ ^= 1;

  // QB = EnQueue，用完还回队列，漏还会把队列掏空导致帧率掉 0
  GXQBuf(device_, frame_buffer);
  return true;
}

void DahengCamera::stopStream()
{
  if (streaming_) {
    GXStreamOff(device_);
    streaming_ = false;
  }
  for (int i = 0; i < 2; i++) {
    if (rgb_buffer_[i]) {
      free(rgb_buffer_[i]);
      rgb_buffer_[i] = nullptr;
    }
  }
}

void DahengCamera::close()
{
  stopStream();
  if (device_opened_) {
    GXCloseDevice(device_);
    device_opened_ = false;
    device_ = nullptr;
  }
  if (lib_initialized_) {
    GXCloseLib();
    lib_initialized_ = false;
  }
}

bool DahengCamera::setExposure(double us)
{
  double value = us < 1.0 ? 1.0 : (us > 10000.0 ? 10000.0 : us);
  GX_STATUS status = GXSetFloatValue(device_, "ExposureTime", value);
  if (status != GX_STATUS_SUCCESS) {
    setError("Set ExposureTime: " + getErrorString(status));
    return false;
  }
  return true;
}

bool DahengCamera::setGain(double db)
{
  double value = db < 0.0 ? 0.0 : (db > 32.0 ? 32.0 : db);
  GX_STATUS status = GXSetFloatValue(device_, "Gain", value);
  if (status != GX_STATUS_SUCCESS) {
    setError("Set Gain: " + getErrorString(status));
    return false;
  }
  return true;
}

}  // namespace armor_track_ros2
