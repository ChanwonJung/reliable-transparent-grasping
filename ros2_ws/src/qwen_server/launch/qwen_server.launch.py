"""Launch qwen_bridge_node plus the stdin instruction prompt.

    ros2 launch qwen_server qwen_server.launch.py

The prompt node runs inside an xterm because `ros2 launch` does not forward
stdin to child processes. Set use_xterm:=false when driving /user_instruction
from elsewhere (a replay bag, another node, or `ros2 topic pub`).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("qwen_server")
    params = os.path.join(pkg_share, "config", "qwen_server_params.yaml")

    args = [
        DeclareLaunchArgument("vllm_endpoint_url", default_value="http://localhost:10000/v1"),
        DeclareLaunchArgument("model_name", default_value="qwen35-local",
                              description="served model id (Qwen3.5-27B)"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("image_path", default_value="",
                              description="use this file as the frame instead of "
                                          "image_topic (offline test)"),
        DeclareLaunchArgument("bbox_convention", default_value="normalized_1000"),
        DeclareLaunchArgument("instruction", default_value=""),
        DeclareLaunchArgument("use_xterm", default_value="true"),
    ]

    overrides = {
        "vllm_endpoint_url": LaunchConfiguration("vllm_endpoint_url"),
        "model_name": LaunchConfiguration("model_name"),
        "image_topic": LaunchConfiguration("image_topic"),
        "image_path": LaunchConfiguration("image_path"),
        "bbox_convention": LaunchConfiguration("bbox_convention"),
        "instruction": LaunchConfiguration("instruction"),
    }

    bridge = Node(
        package="qwen_server",
        executable="qwen_bridge_node",
        name="qwen_bridge_node",
        output="screen",
        parameters=[params, overrides],
    )

    prompt_xterm = Node(
        package="qwen_server",
        executable="instruction_prompt_node",
        name="instruction_prompt_node",
        output="screen",
        parameters=[params],
        prefix="xterm -geometry 100x16 -T 'Slow Brain instruction' -e",
        condition=IfCondition(LaunchConfiguration("use_xterm")),
    )

    prompt_plain = Node(
        package="qwen_server",
        executable="instruction_prompt_node",
        name="instruction_prompt_node",
        output="screen",
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration("use_xterm")),
    )

    return LaunchDescription([*args, bridge, prompt_xterm, prompt_plain])
