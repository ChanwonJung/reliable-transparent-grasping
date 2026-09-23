"""bt_system.launch.py

Launches bt_executor_node for the Kinova Gen3 Lite.

Prerequisites (must be running before this launch):
  • gen3_bringup gen3_lite_moveit.launch.py
      → /move_action, /gen3_lite_2f_gripper_controller/gripper_cmd, /joint_states
  • perception pipeline → /world_map_result, /grasp_candidates, /qwen/grounding_result

Usage:
  ros2 launch bt_pkg bt_system.launch.py
  ros2 launch bt_pkg bt_system.launch.py use_fake_hardware:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("bt_pkg")
    params_file = os.path.join(pkg_share, "config", "bt_params.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────
    tree_arg = DeclareLaunchArgument(
        "tree_file",
        default_value=os.path.join(pkg_share, "behavior_trees", "pick_and_place.xml"),
        description="Absolute path to BT XML file (override for custom trees)",
    )

    # bt_params.yaml 의 0.12 를 덮어쓴다. SelectGraspCandidate 가 이 거리만큼
    # grasp pose 에서 접근축 반대로 물러난 지점을 pre_grasp 으로 쓰는데, 길수록
    # 하강 구간에서 OMPL 이 옆으로 새어 물체를 칠 여지가 커진다. 0.03 은 Isaac
    # (Panda) 검증값 — Gen3 Lite 실기에서는 아직 검증되지 않았다.
    pre_grasp_arg = DeclareLaunchArgument(
        "pre_grasp_z_offset",
        default_value="0.03",
        description=("grasp pose 에서 접근축 반대로 물러나는 거리(m). "
                     "bt_params.yaml 값을 덮어쓴다."),
    )

    # Fake hardware never stalls the gripper on an object, so a close always
    # "misses". This flips gripper_require_contact off to let the tree run.
    fake_hw_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Match gen3_lite_moveit.launch.py; disables gripper contact check",
    )

    # BT executor — delayed to give move_group and the controllers time to
    # finish their own startup.
    bt_executor = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="bt_pkg",
                executable="bt_executor_node",
                name="bt_executor_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "tree_file": LaunchConfiguration("tree_file"),
                        # params_file 뒤에 와야 덮어쓴다.
                        "pre_grasp_z_offset": ParameterValue(
                            LaunchConfiguration("pre_grasp_z_offset"),
                            value_type=float),
                        "gripper_require_contact": ParameterValue(
                            NotSubstitution(LaunchConfiguration("use_fake_hardware")),
                            value_type=bool),
                    },
                ],
            )
        ],
    )

    return LaunchDescription([
        tree_arg,
        pre_grasp_arg,
        fake_hw_arg,
        bt_executor,
    ])
