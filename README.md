# armor_track_ros2

小云台跟随手持小装甲板的 ROS2 视觉主链路（Foxy / Ubuntu 20.04）。

## 功能链路

```
camera (image_raw + camera_info)
   │  cv_bridge -> BGR
   ▼
装甲四点检测 detectArmorPoints(bgr)   ← 你的检测器接入点（无序四点）
   │
   ▼
四点排序 [左上, 右上, 右下, 左下]
   │
   ├──► 裁剪 ROI（透视图 96x96）+ 数字识别 → 编号绑定 armor.name
   │
   ▼
PnP 解算(solvePnP IPPE) → 相机系 tvec
   │
   ▼
坐标变换：相机系 → 云台系(R_camera2gimbal) → 世界系(R_gimbal2world)
   │
   ▼
6 维 EKF [x,vx,y,vy,z,vz] → 线速度 + 视线角速度(yaw_rate/pitch_rate)
   │
   ▼
延迟补偿 + yaw/pitch 偏移 → 发布 GimbalCmd（无串口，只到 ROS2 话题）
```

## 目录结构

```
armor_track_ros2/
├── CMakeLists.txt
├── package.xml
├── msg/GimbalCmd.msg                 # 下发消息
├── config/auto_aim.yaml              # 内参外参/尺寸/跟踪参数
├── launch/armor_track.launch.py
├── assets/tiny_resnet.onnx           # 数字识别模型（已从同济仓库复制）

├── include/
│   ├── armor_track_ros2/armor_detector.hpp
│   ├── auto_aim/{armor,classifier,solver,target}.hpp
│   └── tools/{math_tools,extended_kalman_filter}.hpp
└── src/
    ├── auto_aim_node.cpp             # 主节点
    ├── armor_detector.cpp            # 四点检测占位
    ├── auto_aim/{classifier,solver,target}.cpp
    └── tools/{math_tools,extended_kalman_filter}.cpp
```

## 编译

```bash
cd <你的 ros2 工作空间>/src
cp -r /home/sunny/Downloads/armor_track_ros2 .
cd ..
colcon build --packages-select armor_track_ros2
source install/setup.bash
```

## 运行

```bash
# 先启动你的相机节点，保证话题为：
#   /camera/image_raw   (sensor_msgs/Image, bgr8 或可转换编码)
#   /camera/camera_info (sensor_msgs/CameraInfo)
ros2 launch armor_track_ros2 armor_track.launch.py

# 查看输出
ros2 topic echo /gimbal_cmd
```

## 关键适配点（⚠️ 上线前必须核对）

1. **装甲板物理尺寸** —— `config/auto_aim.yaml` 中 `small_armor_width`、`lightbar_length`
   必须等于你手持板的实际宽高，否则 PnP 测距成比例地错。
2. **四点检测** —— `src/armor_detector.cpp` 里 `detectArmorPoints` 目前是占位实现，
   把它替换成你的传统装甲板检测器（返回无序 `std::vector<cv::Point2f>`，恰好 4 点）。
3. **相机 → 云台外参** —— `R_camera2gimbal`（3x3 行主序）、`t_camera2gimbal`（m）
   按你的相机安装位置填，单位阵/零向量只适合相机装在云台旋转中心的情况。
4. **pitch 符号** —— `pitch_offset` 与下发 `pitch` 的正负取决于你的云台协议，
   现场标定一下「向上/向下为正」。
5. **数字识别模型** —— 默认用同济 `tiny_resnet.onnx`（9 类，见 `armor.hpp` 的
   `ArmorName`）。若你的分类模型类别顺序不同，同步改 `ArmorName` 枚举与模型文件。
6. **话题名** —— `launch/armor_track.launch.py` 里的 remap 改为你的实际相机话题。
7. **串口下发** —— 本包只发到 `/gimbal_cmd` 话题，未实现下位机串口；你已有的
   「视觉底层通信驱动」订阅此话题并透传角度即可。

## 输出消息 GimbalCmd

```yaml
bool    control      # true=锁定，下位机执行跟随
float64 yaw          # 云台 yaw   (rad)
float64 pitch        # 云台 pitch (rad)
float64 distance     # 目标距离   (m)
float64 vx vy vz     # 目标线速度 (m/s, 世界系)
float64 yaw_rate     # 视线角速度 yaw   (rad/s)
float64 pitch_rate   # 视线角速度 pitch (rad/s)
```

## 依赖

`rclcpp` `sensor_msgs` `cv_bridge` `OpenCV` `Eigen3` `yaml-cpp`
（均为 Ubuntu 20.04 上 ROS2 Foxy 常见依赖）。
