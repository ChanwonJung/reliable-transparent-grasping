"""Launch sam_mask_node alone.

    ros2 launch sam_server sam_server.launch.py

Needs the port-20000 tunnel and an upstream publishing /qwen/source_image plus
/qwen/labeled_detections — normally qwen_server. For the two together, use
qwen_server's slow_brain.launch.py with enable_sam:=true.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("sam_server"), "config", "sam_server_params.yaml")

    args = [
        DeclareLaunchArgument("zmq_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("zmq_port", default_value="20000"),
        DeclareLaunchArgument("ee_camera_info_topic",
                              default_value="/camera/camera/aligned_depth_to_color/camera_info",
                              description="depth CameraInfo — sets the mask size"),
        DeclareLaunchArgument("multimask", default_value="false"),
        DeclareLaunchArgument("target_priority", default_value="true"),
        DeclareLaunchArgument("publish_annotated", default_value="true"),
    ]

    node = Node(
        package="sam_server",
        executable="sam_mask_node",
        name="sam_mask_node",
        output="screen",
        emulate_tty=True,
        parameters=[params, {
            "zmq_host": LaunchConfiguration("zmq_host"),
            "zmq_port": LaunchConfiguration("zmq_port"),
            "ee_camera_info_topic": LaunchConfiguration("ee_camera_info_topic"),
            "multimask": LaunchConfiguration("multimask"),
            "target_priority": LaunchConfiguration("target_priority"),
            "publish_annotated": LaunchConfiguration("publish_annotated"),
        }],
    )
    return LaunchDescription([*args, node])
