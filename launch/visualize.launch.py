import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """相机 + 装甲识别（含数字识别可视化），不含串口节点。

    可视化窗口需要图形显示环境：运行时 DISPLAY 需指向用户桌面（如 :0）。
    """
    pkg_dir = get_package_share_directory("armor_track_ros2")
    aim_config = os.path.join(pkg_dir, "config", "auto_aim.yaml")
    cam_config = os.path.join(pkg_dir, "config", "daheng_camera.yaml")

    camera_node = Node(
        package="armor_track_ros2",
        executable="daheng_camera_node",
        name="daheng_camera_node",
        namespace="camera",  # 话题即 /camera/image_raw、/camera/camera_info
        output="screen",
        arguments=[cam_config],
    )

    aim_node = Node(
        package="armor_track_ros2",
        executable="armor_track_node",
        name="armor_track_node",
        output="screen",
        arguments=[aim_config],
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    return LaunchDescription([camera_node, aim_node])
