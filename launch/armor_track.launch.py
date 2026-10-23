import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("armor_track_ros2")
    config_path = os.path.join(pkg_dir, "config", "auto_aim.yaml")

    aim_node = Node(
        package="armor_track_ros2",
        executable="armor_track_node",
        name="armor_track_node",
        output="screen",
        arguments=[config_path],
        remappings=[
            # 按你的相机话题调整
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    serial_node = Node(
        package="armor_track_ros2",
        executable="serial_node",
        name="serial_node",
        output="screen",
        arguments=[config_path],
    )

    return LaunchDescription([aim_node, serial_node])
