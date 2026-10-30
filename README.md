# armor_track_ros2

RoboMaster 风格的 **ROS2（Humble / Ubuntu 22.04）视觉自瞄包**：把「相机取流 → 传统视觉识别装甲板 → PnP 测距 → 闭环云台跟随 → 串口下发云台角度」整条链路串起来，并自带大恒（Galaxy）相机驱动与标定流程。

---

## 一、这套代码能完成什么任务

一句话：**让云台自动瞄准并持续跟随敌方装甲板。**

具体能做的几件事：

1. **相机驱动 + 标定**：内置大恒 Galaxy 相机驱动 `daheng_camera_node`，发布 `image_raw` + `camera_info`，配合 ROS 的 `cameracalibrator` 完成内参标定，为 PnP 测距提供内参。
2. **实时装甲板检测**：纯传统视觉（HSV 颜色分割 → 形态学 → 灯条拟合 → 灯条配对 → 装甲四点），不依赖深度学习也能跑，帧率高、对算力要求低。
3. **目标 3D 定位与测距**：对装甲四点做 `solvePnP`（IPPE）解出相机系下的 3D 位置，再经云台当前 yaw/pitch 反馈构造旋转矩阵换到世界系（Eye-in-Hand 闭环，无手眼外参），得到目标方位。
4. **运动估计**：6 维扩展卡尔曼滤波（`[x,vx,y,vy,z,vz]`）估计目标线速度与视线角速度，随 `GimbalCmd` 下发并叠加到调试窗口。
5. **云台控制下发**：把解算出的 yaw/pitch/距离/角速度打包成 `GimbalCmd`，再由 `serial_node` 通过串口（`0x55` 帧头 / `0xFF` 帧尾、无 CRC）发给电控执行跟随。

### 完整链路

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  大恒相机 daheng_camera_node  (namespace /camera)                            │
│    发布 /camera/image_raw (bgr8)  +  /camera/camera_info (内参)              │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  armor_track_node                                                            │
│   传统装甲识别（HSV→灯条→配对→四点）                                     │
│        → PnP(solvePnP IPPE) 相机系 3D 位置                                   │
│        → 云台 yaw/pitch 反馈旋转 → 世界系目标方位                                        │
│        → atan2 目标角 + 一阶低通 + 限幅                                            │
│        → 发布 /gimbal_cmd (GimbalCmd)                                        │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  serial_node                                                                 │
│    订阅 /gimbal_cmd → 打包 AimbotFrame(0x55/0xFF) → 串口下发                 │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │   /dev/ttyACM0 @ 460800 8N1
                                ▼
                            电控（云台 MCU）
```

---

## 二、系统组成（三个整合进来的模块）

| 模块 | 文件 | 说明 |
|---|---|---|
| 传统装甲识别 | `include/armor_track_ros2/traditional_detector.hpp` / `src/traditional_detector.cpp` | 从 `dahengshibie` 移植，去掉了 `spdlog`/`rm_log` 依赖；HSV 分割 → 灯条筛选 → 灯条配对 → 四点 `[左上,右上,右下,左下]` |
| 串口通信 | `include/armor_track_ros2/serial_driver.hpp` / `src/serial_driver.cpp` / `src/serial_node.cpp` | 自包含 POSIX termios 串口驱动（无 `utils` 框架依赖），协议 `0x55` 头 / `0xFF` 尾 / 无 CRC |
| 相机标定 / 大恒驱动 | `include/armor_track_ros2/daheng_camera.hpp` / `src/daheng_camera.cpp` / `src/daheng_camera_node.cpp` / `docs/相机标定流程-humble.md` | Galaxy SDK 取流 → Bayer 转 BGR → 发布 `image_raw` + `camera_info`（经 `camera_info_manager`） |

---

## 三、目录结构

```
armor_track_ros2/
├── CMakeLists.txt / package.xml
├── msg/GimbalCmd.msg                    # 云台下发指令定义
├── msg/GimbalFeedback.msg                # 云台 IMU 反馈（yaw/pitch）
├── config/
│   ├── auto_aim.yaml                    # 识别 / 装甲尺寸 / 闭环跟踪 / 串口 总配置
│   └── daheng_camera.yaml               # 大恒相机驱动参数（序列号/曝光/内参路径…）
├── launch/
│   ├── armor_track.launch.py            # 自瞄 + 串口
│   ├── visualize.launch.py              # 相机 + 识别可视化（不含串口）
│   └── daheng_camera.launch.py          # 大恒相机驱动（跑在 /camera）
├── docs/相机标定流程-humble.md           # 标定完整指南
├── scripts/                             # 调试/诊断脚本（move_gimbal、read_gimbal、preview_camera …）
├── include/
│   ├── armor_track_ros2/
│   │   ├── traditional_detector.hpp     # 传统装甲识别
│   │   ├── traditional_armor.hpp
│   │   ├── serial_driver.hpp            # 串口驱动
│   │   └── daheng_camera.hpp            # 大恒相机封装
│   ├── auto_aim/
│   │   ├── closed_loop_solver.hpp        # 闭环解算（PnP + 坐标变换 + 低通 + 限幅）
│   │   └── ekf.hpp                       # 6 维 EKF
│   └── sdk/                             # 大恒 Galaxy SDK 头文件（已内置，构建需 libgxiapi.so）
└── src/
    ├── auto_aim_node.cpp                # 主节点：识别 → PnP → 闭环解算 → 发布 gimbal_cmd
    ├── traditional_detector.cpp
    ├── serial_node.cpp / serial_driver.cpp
    ├── daheng_camera_node.cpp / daheng_camera.cpp
    ├── closed_loop_solver.cpp / ekf.cpp
```

---

## 四、依赖

- ROS2 **Humble**：`rclcpp`、`sensor_msgs`、`cv_bridge`、`camera_info_manager`（驱动节点用）
- 三方库：`OpenCV`、`Eigen3`、`yaml-cpp`
- 大恒相机：官方 Galaxy 驱动已安装（`libgxiapi.so` 在 `/usr/lib`），SDK 头文件已随包内置

---

## 五、编译

```bash
# 相机相关依赖（daheng_camera_node 需要；没装会被 CMake 自动跳过，不影响其它节点）
sudo apt install -y ros-humble-camera-info-manager ros-humble-camera-calibration

# 构建（包目录本身即可当工作空间根，也可放到你的 ws/src/ 下）
cd ~/Downloads/armor_track_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 确认三个可执行都出来了
ls install/armor_track_ros2/lib/armor_track_ros2/
# armor_track_node   serial_node   daheng_camera_node
```

---

## 六、运行

```bash
source install/setup.bash

# 1) 大恒相机驱动（发布 /camera/image_raw + /camera/camera_info）
ros2 launch armor_track_ros2 daheng_camera.launch.py

# 2) 自瞄 + 串口下发
ros2 launch armor_track_ros2 armor_track.launch.py

# 查看下发指令
ros2 topic echo /gimbal_cmd
```

> 首次跑相机驱动先看日志里列出的序列号，填进 `config/daheng_camera.yaml` 的 `serial_number`（留空则自动用第一台）；标定步骤见 `docs/相机标定流程-humble.md`。

---

## 七、关键配置

### 相机内参（PnP 输入）

PnP（`solvePnP`）用的内参**不写死在代码里**，而是运行时从 `/camera/camera_info` 话题订阅，链路如下：

```
config/daheng_camera.yaml → camera_info_url → 标定 yaml
        │  (camera_info_manager 解析)
        ▼
daheng_camera_node 发布 /camera/camera_info  (k=相机矩阵, d=畸变系数)
        │  (auto_aim_node 订阅)
        ▼
solver_.set_camera(msg->k, msg->d) → cv::solvePnP(..., camera_matrix_, distort_coeffs_, ...)
```

- 内参路径：`config/daheng_camera.yaml` 的 `camera_info_url`，当前指向 `/home/sunny/camera_calib/calibration.yaml`（**占位估算值**）。
- 当前生效内参（1920×1200，`camera_name: daheng`）：
  - 内参矩阵 `K`：fx=1574.77，fy=1574.32，cx=962.50，cy=596.60
  - 畸变系数 `D`（`plumb_bob`，顺序 k1,k2,p1,p2,k3）：`[-0.242, 0.157, -0.000127, 0.000891, 0]`
- 注意：该 yaml 里 `projection_matrix`（fx≈1455 / fy≈1520）与 `camera_matrix` 不一致，但节点只取 `k`/`d`，**PnP 不受 P 影响**；若后续要做去畸变/rectify，再核对 P。
- 另有 `/home/sunny/camera_calib/calibration_chessboard.yaml` 未被 `camera_info_url` 引用，不生效。

### 自瞄参数（`config/auto_aim.yaml`）

| 参数 | 说明 |
|---|---|
| `enemy_color` | 敌方颜色 `red`/`blue`（只分割该颜色） |
| `red_hue_low/high` … `value_high` | HSV 阈值（红色跨色相环，用两段取并集） |
| `max_angle_error` `min/max_lightbar_ratio` … | 灯条筛选与装甲配对参数 |
| `small/big_armor_width` `lightbar_length` | **装甲板物理尺寸（m）**，必须与实际一致，否则 PnP 测距成比例地错（当前大小装甲判定未接入，恒按小装甲） |
| `closed_loop_alpha` / `closed_loop_angle_limit` | 一阶低通滤波系数 / 输出角度限幅（rad） |
| `closed_loop_imu_timeout_ms` / `closed_loop_debug` | IMU 超时判定（ms）/ 闭环调试打印开关 |
| `ekf_sigma_acc` `ekf_sigma_x/y/z` `ekf_max_dt` | 6 维 EKF 过程/量测噪声与预测步长上限 |
| `publish_rate` | 下发频率（Hz），0 表示按相机帧率 |
| `serial_port` / `serial_baud` | 串口设备与波特率（默认 `/dev/ttyACM0` @ `460800` 8N1） |

---

## 八、串口协议

帧头 `0x55`、帧尾 `0xFF`、无 CRC；发送帧 ID=`0x02`，接收帧 ID=`0x01`。

视觉端 → 电控 发送帧 `AimbotFrame`（`__attribute__((packed))`，共 11 字节）：

| 字节 | 字段 | 说明 |
|---|---|---|
| 1 | `_SOF = 0x55` | 帧头 |
| 1 | `ID = 0x02` | 帧类型 |
| 4 | `PitchRelativeAngle` | pitch 相对云台零位的期望角（**rad**，正 pitch 向上） |
| 4 | `YawRelativeAngle` | yaw 相对云台零位的期望角（**rad**，正 yaw 向右） |
| 1 | `_EOF = 0xFF` | 帧尾 |

电控 → 视觉 接收帧 `GimbalImuFrame`（`__attribute__((packed))`，共 19 字节）：

| 字节 | 字段 | 说明 |
|---|---|---|
| 1 | `_SOF = 0x55` | 帧头 |
| 1 | `ID = 0x01` | 帧类型 |
| 16 | `q0 q1 q2 q3` | 云台四元数（w x y z，各 4 字节 float） |
| 1 | `_EOF = 0xFF` | 帧尾 |

> ⚠️ 发送帧 yaw/pitch 为弧度；正 yaw 向右、正 pitch 向上。正负号开关在 `src/serial_node.cpp` 的 `CMD_YAW_SIGN` / `CMD_PITCH_SIGN`（当前 pitch 已取反）。
> **跟随规则**：`control=true` 时持续下发 = 跟随；`control=false` 时停发 = 不跟随（实现见 `src/serial_node.cpp`）。

---

## 九、输出消息 `GimbalCmd`

```yaml
bool    control      # true=锁定，下位机执行跟随
float64 yaw          # 云台 yaw   (rad)
float64 pitch        # 云台 pitch (rad)
float64 distance     # 目标距离   (m)
float64 vx vy vz     # 目标线速度 (m/s，世界系)
float64 yaw_rate     # 视线角速度 yaw   (rad/s)
float64 pitch_rate   # 视线角速度 pitch (rad/s)
```

---

## 十、待办 / 已知限制

1. ~~串口符号/单位~~：已按电控协议定稿（rad、0x55/0xFF 无 CRC、持续发包=跟随），见 §八。
2. **相机标定**：当前 `camera_info_url` 指向 `/home/sunny/camera_calib/calibration.yaml`，内参为**占位估算值**（K≈[1574.77, 962.50; 1574.32, 596.60]，D=[-0.242, 0.157, -0.0001, 0.0009, 0]），上线前跑 `cameracalibrator` 正式标定并覆盖该 yaml（见 §七 相机内参）。
3. **装甲尺寸与标定**：装甲板尺寸务必按实物填（否则 PnP 测距成比例地错）；当前闭环用云台 yaw/pitch 反馈（Eye-in-Hand）做坐标变换，无手眼外参，大小装甲判定未接入（恒按小装甲）。
