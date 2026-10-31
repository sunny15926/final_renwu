# armor_track_ros2

RoboMaster 风格的 **ROS2（Humble / Ubuntu 22.04）视觉自瞄包**：把「相机取流 → 传统视觉识别装甲板 → PnP 测距 → 6 维 EKF 跟踪 → 串口下发云台角度」整条链路串起来，并自带大恒（Galaxy）相机驱动与标定流程。

---

## 一、这套代码能完成什么任务

一句话：**让云台自动瞄准并持续跟随敌方装甲板。**

具体能做的几件事：

1. **相机驱动 + 标定**：内置大恒 Galaxy 相机驱动 `daheng_camera_node`，发布 `image_raw` + `camera_info`，配合 ROS 的 `cameracalibrator` 完成内参标定，为 PnP 测距提供内参。
2. **实时装甲板检测**：纯传统视觉（HSV 颜色分割 → 形态学 → 灯条拟合 → 灯条配对 → 装甲四点），不依赖深度学习也能跑，帧率高、对算力要求低。
3. **目标 3D 定位与测距**：对装甲四点做 `solvePnP`（IPPE）解出相机系下的 3D 位置，再用固定轴排列矩阵 `R0` + 云台 IMU 回传的 yaw/pitch 构造闭环旋转矩阵，换到世界系得到距离。
4. **运动跟踪与预测**：6 维扩展卡尔曼滤波（`[x,vx,y,vy,z,vz]`）估计目标线速度与视线角速度，并做延迟补偿，预测下发时刻的位置。
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
│        → 闭环变换（R0 + 云台 yaw/pitch）相机系→世界系                          │
│        → 6 维 EKF 跟踪 + 延迟补偿                                            │
│        → 发布 /gimbal_cmd (GimbalCmd)                                        │
└───────────────────────────────┬─────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  serial_node                                                                 │
│    订阅 /gimbal_cmd → 打包 SendPacket(0x55/0xFF) → 串口下发                 │
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
├── config/
│   ├── auto_aim.yaml                    # 识别 / 外参 / 装甲尺寸 / 跟踪 / 串口 总配置
│   └── daheng_camera.yaml               # 大恒相机驱动参数（序列号/曝光/内参路径…）
├── launch/
│   ├── armor_track.launch.py            # 自瞄 + 串口
│   └── daheng_camera.launch.py          # 大恒相机驱动（跑在 /camera）
├── docs/相机标定流程-humble.md           # 标定完整指南
├── scripts/                             # 诊断/采集脚本（frame_sweep、telemetry_probe …）
├── include/
│   ├── armor_track_ros2/
│   │   ├── traditional_detector.hpp     # 传统装甲识别
│   │   ├── traditional_armor.hpp
│   │   ├── serial_driver.hpp            # 串口驱动
│   │   └── daheng_camera.hpp            # 大恒相机封装
│   ├── auto_aim/{armor,classifier,solver,target}.hpp
│   ├── tools/{math_tools,extended_kalman_filter}.hpp
│   └── sdk/                             # 大恒 Galaxy SDK 头文件（已内置，构建需 libgxiapi.so）
└── src/
    ├── auto_aim_node.cpp                # 主节点：识别 → PnP → EKF → 发布 gimbal_cmd
    ├── traditional_detector.cpp
    ├── serial_node.cpp / serial_driver.cpp
    ├── daheng_camera_node.cpp / daheng_camera.cpp
    ├── auto_aim/{classifier,solver,target}.cpp
    └── tools/{math_tools,extended_kalman_filter}.cpp
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
| `red_hue_low/high` `blue_hue_low/high` `saturation_*` `value_*` | HSV 阈值（红色跨色相环 0°/180°，用两段取并集） |
| `morph_kernel_size` | 形态学闭运算核大小 |
| `max_angle_error` `min/max_lightbar_ratio` `min_lightbar_length` `min/max_armor_ratio` `max_side_ratio` `max_rectangular_error` | 灯条筛选与装甲配对参数 |
| `small/big_armor_width` `lightbar_length` | **PnP 物理尺寸（m）= 灯条围成矩形的尺寸，非装甲外壳尺寸**：宽为左右灯条矩形宽、高为灯条长；四个世界角点对应四根灯条的四个顶点。当前固定步兵小装甲灯条宽 `0.140` / 高 `0.055`。尺寸不符会导致 PnP 测距成比例地错 |
| `closed_loop_alpha` | 一阶低通滤波系数 `(0,1]`，越小越平滑 |
| `yaw_limit` / `pitch_limit` | **云台角度输出限幅（rad，独立）**：yaw `1.57`=±90°、pitch `0.35`=±20°。限幅只作用于输出，滤波内部存未限幅值避免饱和 |
| `closed_loop_imu_timeout_ms` | IMU 超时判定失效（ms），超时角度置 0 停止跟随 |
| `ekf_sigma_acc` `ekf_sigma_x/y/z` `ekf_max_dt` | 6 维 EKF 过程/量测噪声与预测步长上限 |
| `publish_rate` | 下发频率（Hz），0 表示按相机帧率 |
| `serial_port` / `serial_baud` | 串口设备与波特率（默认 `/dev/ttyACM0` @ `460800` 8N1） |

> ⚠️ **无固定外参**：当前实现不做 `R_camera2gimbal`/`t_camera2gimbal` 手眼标定，而是用固定的相机→世界轴排列矩阵 `R0` 加云台 IMU 回传的 yaw/pitch 构造闭环旋转矩阵（Eye-in-Hand），见 `src/closed_loop_solver.cpp` 的 `R_C2W = R_z(+yaw)*R_y(-pitch)*R0`。

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

> ⚠️ 发送帧 yaw/pitch 为弧度、**不取负**（直接发期望角）；正 yaw 向右、正 pitch 向上。
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

## 十、调试窗口叠加数据说明

当 `config/auto_aim.yaml` 里 `debug: true` 时，会弹出名为 `armor` 的可视化窗口（图像倒置、放大 2 倍），左上角叠加以下实时数据（标签用英文，因 OpenCV Hershey 字体不支持中文）：

```
PnP dist   : 0.486 m                       # ① PnP 测距
EKF [x,vx,y,vy,z,vz]                       # ② 标签行：下方数据来自 6 维 EKF
 height z   : -0.085 m                     # ③ 目标高度（世界系 Z）
 linear vel : 0.062 m/s                    # ④ 目标线速度模长
 LOS rate   : yaw 0.066  pitch 0.050 rad/s # ⑤ 视线角速度
```

| 显示项 | 对应变量 | 含义 |
|---|---|---|
| `PnP dist` | `cv::norm(res.tvec)` | solvePnP 解出的相机系平移向量 `tvec` 的欧氏长度，即**相机光心到装甲板中心的直线距离**（传统 PnP 测距的直接结果），单位 m |
| `EKF [x,vx,y,vy,z,vz]` | — | 标签行：说明下面几项来自 6 维扩展卡尔曼滤波，状态向量为世界系（X 前 / Y 右 / Z 上）下的位置与速度分量 `[x,vx,y,vy,z,vz]` |
| `height z` | `res.ekf_state[4]` | EKF 估计的目标在世界系 **Z 坐标（高度）**，带符号：`+` 在云台零位上方，`-` 在下方，单位 m |
| `linear vel` | `res.linear_speed` | 目标**线速度模长** = √(vx²+vy²+vz²)，单位 m/s |
| `LOS rate yaw / pitch` | `res.yaw_rate` / `res.pitch_rate` | **视线角速度**（Line-of-Sight rate）：目标相对云台的方位角/俯仰角随时间的变化率，yaw 向右为正、pitch 向上为正，单位 rad/s |

> 坐标约定：世界系 **X 前 / Y 右 / Z 上**；相机系 **Z 前 / X 右 / Y 下**（OpenCV）。注意 `PnP dist` 是相机系直线距离，与下发消息 `GimbalCmd.distance`（世界系 `xyz_in_world` 的模长）略有差异。
>
> 目标短暂丢失时，叠加数据会保留上一帧有效值（持续显示、不闪烁），恢复锁定后自动刷新。

---

## 十一、待办 / 已知限制

1. ~~串口符号/单位~~：已按电控协议定稿（rad、不取负、正 yaw 向右 / 正 pitch 向上、持续发包=跟随），见 §八。
2. **相机标定**：当前 `camera_info_url` 指向 `/home/sunny/camera_calib/calibration.yaml`，内参为**占位估算值**（K≈[1574.77, 962.50; 1574.32, 596.60]，D=[-0.242, 0.157, -0.0001, 0.0009, 0]），上线前跑 `cameracalibrator` 正式标定并覆盖该 yaml（见 §七 相机内参）。
3. **装甲尺寸**：`small/big_armor_width`、`lightbar_length` 务必按实物（灯条矩形）标定，否则测距与角度偏；闭环外参用固定 `R0` + 云台 IMU 回传实现，无需手眼标定。
