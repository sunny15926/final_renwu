# 相机标定流程（ROS2 Humble）

目标：让相机驱动发布 `/camera/camera_info`（内参 + 畸变），`armor_track_node` 订阅后即可 PnP 测距。
`armor_track` 的 launch 已把话题重映射为 `/camera/image_raw` 和 `/camera/camera_info`，所以相机驱动也要跑在 `/camera` 命名空间（或反过来改 launch 的重映射）。

---

## 0. 前提：确认相机类型

```bash
ls /dev/video*                # 是否有 UVC 设备节点
v4l2-ctl --list-devices       # 列出 V4L2 相机
```

- **USB 相机 / 支持 UVC 协议的相机** → 走第 1 节的 `v4l2_camera` 或 `usb_cam`（最简单，推荐）。
- **大恒 Galaxy / 海康 HIK 工业相机（非 UVC）** → 见第 6 节，需用自带 SDK 驱动并小改。

---

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-camera-calibration \
  ros-humble-camera-calibration-parsers \
  ros-humble-camera-info-manager \
  ros-humble-image-pipeline

# USB(UVC) 相机驱动，二选一（都行）：
sudo apt install -y ros-humble-v4l2-camera
# 或
sudo apt install -y ros-humble-usb-cam
```

---

## 2. 跑相机驱动（以 v4l2_camera 为例）

先跑一个能发布 `/image_raw` + `/camera_info` 的驱动：

```bash
source /opt/ros/humble/setup.bash
# 命名空间 /camera，话题即 /camera/image_raw、/camera/camera_info
ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera
```

验证话题：

```bash
ros2 topic list | grep camera
# 应看到 /camera/image_raw 和 /camera/camera_info
```

用 `image_view` 确认画面：

```bash
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

> 注：`v4l2_camera` 支持用参数改分辨率，例如
> `--ros-args -p image_size:="[640,480]"`（默认可能直接用相机原生分辨率，标定前最好固定一个分辨率）。

---

## 3. 标定

棋盘格标定板：打印一张（格子数 12×9，即内角点 11×8），量好每个格子边长（单位 m）。

```bash
ros2 run camera_calibration cameracalibrator \
  --size 11x8 --square 0.03 \
  --ros-args -r image:=/camera/image_raw
```

参数：
- `--size 11x8`：**内角点**数量（格子数 - 1）。
- `--square 0.03`：单个格子边长（米）。

GUI 操作：
1. 手持标定板在画面里 **左右平移 / 上下平移 / 前后移动 / 倾斜旋转**，让右侧 `X`、`Y`、`Size`、`Skew` 四条进度条都变绿（越满越好）。
2. `CALIBRATE` 按钮亮起后点击，等待计算完成。
3. 点 **`COMMIT`**，会把结果存到 `/tmp/calibrationdata.tar.gz`。
4. 解压取出内参 yaml：

```bash
mkdir -p ~/camera_calib && cd ~/camera_calib
tar -xzf /tmp/calibrationdata.tar.gz
ls *.yaml          # calibrationdata 里的 calibration.yaml 就是内参文件
```

得到的 yaml 含 `camera_matrix`、`distortion_coefficients`、`image_width/height` 等，即 `camera_info` 的内容。

---

## 4. 把内参接回驱动

把标定出来的 yaml 路径通过 `camera_info_url` 传给驱动，重启驱动：

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/camera \
  -p camera_info_url:=file:///home/sunny/camera_calib/calibration.yaml
```

验证内参已发布：

```bash
ros2 topic echo /camera/camera_info --once
# 看 camera_matrix / distortion_coefficients 是否与标定结果一致
```

> 不同驱动的参数名略有差异，但都是走 `camera_info_manager` 的 `camera_info_url`：
> - `v4l2_camera` → `camera_info_url`
> - `usb_cam` → `camera_info_url`
> - `ros2_hik_camera` → `camera_info_url`（见第 6 节）

---

## 5. 跑 armor_track 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch armor_track_ros2 armor_track.launch.py
```

看到 `armor_track_node` 不再刷 `waiting for camera_info ...`，即内参已接入。若话题名和你驱动不一致，改 `launch/armor_track.launch.py` 里的 remappings 即可。

---

## 6. 工业相机（大恒 / 海康）说明

你 `Downloads/calibration` 里有两个自带的驱动，但都有坑，需要小改才能用：

### 6.1 海康 HIK —— `ros2_hik_camera`
- 依赖海康 MVS SDK（`MvCameraControl.h` + `libMvCameraControl.so`），先装官方 SDK。
- **坑**：`src/hik_camera_node.cpp` 里加载了 `camera_info` 但 `publish` 被注释掉了，**只发 `image_raw` 不发 `camera_info`**。
- 改法：在发布 `image_msg_` 的地方，同时用 `camera_info_manager_` 的 `getCameraInfo()` 发一帧 `camera_info`（或加一个 `Publisher<CameraInfo>`）。内参直接填 `config/camera_info.yaml`。

### 6.2 大恒 Galaxy —— 自带 `daheng_camera_node`（推荐，已内置进 armor_track_ros2）

本包已经内置了一个自包含的大恒驱动节点 `daheng_camera_node`（用 Galaxy SDK 取流 → Bayer 转 BGR → 发布 `image_raw` + `camera_info`），不用再折腾 `rmos_cam` 那套（它的 launch 引用了缺失的 `rmos_bringup`，且只发 `image_raw` 不发内参）。

依赖与安装：
- 大恒官方 Galaxy 驱动已装（`libgxiapi.so` 在 `/usr/lib`，已确认）。
- `ros-humble-camera-info-manager`（构建时若没装，`daheng_camera_node` 会被跳过，其余节点不受影响）。

```bash
sudo apt install -y ros-humble-camera-info-manager ros-humble-camera-calibration
```

装完重新构建，确认 `daheng_camera_node` 编出来了：

```bash
source /opt/ros/humble/setup.bash
cd ~/Downloads/armor_track_ros2 && colcon build
ls install/armor_track_ros2/lib/armor_track_ros2/daheng_camera_node   # 存在即成功
```

跑驱动（跑在 `/camera` 命名空间，和 armor_track 的 launch 重映射一致）：

```bash
source install/setup.bash
ros2 launch armor_track_ros2 daheng_camera.launch.py
```

验证话题与画面：

```bash
ros2 topic list | grep camera     # 应看到 /camera/image_raw 和 /camera/camera_info
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

首次运行看日志里列出的序列号，填进 `config/daheng_camera.yaml` 的 `serial_number`（留空则自动用第一台）：

```text
日志会打印：可用设备 -> 型号:xxx  序列号:xxx
```

然后按第 3 节标定，标定完成把内参 yaml 路径填进 `config/daheng_camera.yaml` 的 `camera_info_url`：

```yaml
camera_info_url: "file:///home/sunny/camera_calib/calibration.yaml"
```

重启 `daheng_camera.launch.py`，`ros2 topic echo /camera/camera_info --once` 确认 `camera_matrix`/`distortion_coefficients` 已填上，即可跑 armor_track（第 5 节）。

> 小坑：标定 yaml 里的 `camera_name` 和驱动配置的 `camera_name`（默认 `daheng`）不一致时，`camera_info_manager` 会打一条警告但仍会正常加载，不影响使用；想消除警告就把两边改成一致。

> 若相机支持 UVC（插上能被 `v4l2-ctl --list-devices` 识别），第 1 节的 `v4l2_camera` 最省事；否则用上面的 `daheng_camera_node`。
