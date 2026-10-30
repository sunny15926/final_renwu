#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "armor_track_ros2/daheng_camera.hpp"

using namespace std::chrono_literals;

namespace armor_track_ros2
{

// 曝光时间（微秒）常量：直接改这个数即可微调画面亮度，改完重新编译生效。
// 曝光只改变画面亮度，不改变图像几何信息，因此不影响内参 / PnP 解算，也不影响送入 detector 的原图。
// yaml 里 exposure_us 填了非 0 值时以 yaml 为准；此处作为 yaml 未填时的默认值。
constexpr double kExposureUs = 6000.0;

// 大恒 Galaxy 相机 ROS2 驱动：
//   发布 image_raw (bgr8) + camera_info（经 camera_info_manager，支持 camera_info_url 加载标定内参）。
//   命名空间建议跑在 /camera，与 armor_track.launch.py 的重映射 (/camera/image_raw、/camera/camera_info) 对齐。
class DahengCameraNode : public rclcpp::Node
{
public:
  explicit DahengCameraNode(const std::string & config_path)
  : Node("daheng_camera_node")
  {
    auto yaml = YAML::LoadFile(config_path);
    serial_ = yaml["serial_number"].as<std::string>("");
    exposure_us_ = yaml["exposure_us"].as<double>(kExposureUs);
    gain_db_ = yaml["gain_db"].as<double>(0.0);
    frame_id_ = yaml["frame_id"].as<std::string>("camera_optical_frame");
    camera_name_ = yaml["camera_name"].as<std::string>("daheng");
    camera_info_url_ = yaml["camera_info_url"].as<std::string>("");
    grab_timeout_ms_ = yaml["grab_timeout_ms"].as<int>(1000);

    // 运行时调参：ros2 param set /camera/daheng_camera_node exposure_us 4000.0
    this->declare_parameter<double>("exposure_us", exposure_us_);
    this->declare_parameter<double>("gain_db", gain_db_);
    pending_exposure_us_.store(exposure_us_);
    pending_gain_db_.store(gain_db_);
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "exposure_us") {
            pending_exposure_us_.store(p.as_double());
            params_dirty_.store(true);
          } else if (p.get_name() == "gain_db") {
            pending_gain_db_.store(p.as_double());
            params_dirty_.store(true);
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // 内参管理：带 set_camera_info 服务，标定工具 COMMIT 后可直接回写；
    // camera_info_url 指向标定 yaml（如 file:///home/sunny/camera_calib/calibration.yaml）。
    cinfo_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name_, camera_info_url_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", sensor_qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", sensor_qos);

    grab_thread_ = std::thread([this]() { grabLoop(); });
    RCLCPP_INFO(get_logger(), "daheng_camera_node 启动，config: %s", config_path.c_str());
  }

  ~DahengCameraNode() override
  {
    stop_ = true;
    if (grab_thread_.joinable()) grab_thread_.join();
    camera_.close();
  }

private:
  // 打开相机并开始取流（由取流线程调用）。返回 false 表示失败，稍后重试。
  bool openCamera()
  {
    if (!lib_initialized_) {
      if (!camera_.initLib()) {
        RCLCPP_ERROR(get_logger(), "%s", camera_.lastError().c_str());
        return false;
      }
      lib_initialized_ = true;
    }

    std::string serial_to_open = serial_;
    if (serial_to_open.empty()) {
      auto devs = camera_.enumerate();
      if (devs.empty()) {
        RCLCPP_ERROR(get_logger(), "未找到任何大恒相机，检查：1) 上电 2) USB3.0 口 3) 设备权限");
        return false;
      }
      serial_to_open = devs.front().serial;
      RCLCPP_INFO(get_logger(), "serial_number 未填，自动使用第一台: %s", serial_to_open.c_str());
    }

    if (!camera_.open(serial_to_open)) {
      RCLCPP_ERROR(get_logger(), "打开相机失败: %s", camera_.lastError().c_str());
      // 帮助定位：列出所有枚举到的设备序列号
      auto devs = camera_.enumerate();
      for (auto & d : devs) {
        RCLCPP_INFO(get_logger(), "可用设备 -> 型号:%s  序列号:%s", d.model.c_str(), d.serial.c_str());
      }
      return false;
    }

    RCLCPP_INFO(get_logger(), "打开相机成功，分辨率 %dx%d", camera_.width(), camera_.height());

    if (!camera_.startStream()) {
      RCLCPP_ERROR(get_logger(), "取流失败: %s", camera_.lastError().c_str());
      camera_.close();
      return false;
    }
    if (exposure_us_ > 0.0 && !camera_.setExposure(exposure_us_)) {
      RCLCPP_WARN(get_logger(), "设置曝光失败: %s", camera_.lastError().c_str());
    }
    if (gain_db_ > 0.0 && !camera_.setGain(gain_db_)) {
      RCLCPP_WARN(get_logger(), "设置增益失败: %s", camera_.lastError().c_str());
    }

    RCLCPP_INFO(get_logger(), "取流开始，发布 image_raw + camera_info");
    return true;
  }

  void grabLoop()
  {
    int fail_count = 0;
    while (!stop_) {
      if (!streaming_) {
        if (openCamera()) {
          streaming_ = true;
          fail_count = 0;
        } else {
          std::this_thread::sleep_for(2s);  // 打不开则 2s 后重试
          continue;
        }
      }

      // 运行时调参：只在取流线程里写相机参数，避免与取流并发
      if (params_dirty_.exchange(false)) {
        double e = pending_exposure_us_.load();
        double g = pending_gain_db_.load();
        if (e > 0.0) camera_.setExposure(e);
        if (g > 0.0) camera_.setGain(g);
        RCLCPP_INFO(get_logger(), "调参生效: exposure=%.1fus gain=%.1fdB", e, g);
      }

      cv::Mat bgr;
      if (camera_.grab(bgr, grab_timeout_ms_)) {
        fail_count = 0;
        publishFrame(bgr);
      } else {
        fail_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "取帧失败: %s", camera_.lastError().c_str());
        // 连续失败视为掉线，重连
        if (fail_count > 5) {
          RCLCPP_WARN(get_logger(), "连续取帧失败，尝试重连相机");
          camera_.stopStream();
          camera_.close();
          streaming_ = false;
          fail_count = 0;
        }
      }
    }
  }

  void publishFrame(const cv::Mat & bgr)
  {
    auto stamp = now();

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = stamp;
    cv_img.header.frame_id = frame_id_;
    cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img.image = bgr;  // 浅共享；toImageMsg() 会深拷贝到消息里
    image_pub_->publish(*cv_img.toImageMsg());

    auto ci = cinfo_->getCameraInfo();
    ci.header.stamp = stamp;
    ci.header.frame_id = frame_id_;
    info_pub_->publish(ci);
  }

  // 参数
  std::string serial_;
  double exposure_us_ = 0.0;
  double gain_db_ = 0.0;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  int grab_timeout_ms_ = 1000;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  DahengCamera camera_;
  std::thread grab_thread_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::atomic<bool> stop_{false};
  std::atomic<double> pending_exposure_us_{0.0};  // 待应用的曝光（由参数回调写入，取流线程读取）
  std::atomic<double> pending_gain_db_{0.0};     // 待应用的增益
  std::atomic<bool> params_dirty_{false};        // 有待应用的参数变更
  bool lib_initialized_ = false;
  bool streaming_ = false;
};

}  // namespace armor_track_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_FATAL(rclcpp::get_logger("daheng_camera_node"),
                 "用法: daheng_camera_node <config.yaml>");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(std::make_shared<armor_track_ros2::DahengCameraNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
