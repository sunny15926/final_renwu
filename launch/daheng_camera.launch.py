import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("armor_track_ros2")
    config_path = os.path.join(pkg_dir, "config", "daheng_camera.yaml")

    camera_node = Node(
        package="armor_track_ros2",
        executable="daheng_camera_node",
        name="daheng_camera_node",
        namespace="camera",  # 话题即 /camera/image_raw、/camera/camera_info
        output="screen",
        arguments=[config_path],
    )

    return LaunchDescription([camera_node])
