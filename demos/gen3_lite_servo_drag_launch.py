# Manual demo: drag a marker in RViz and the Gen3 Lite follows it live, via
# MoveIt Servo. Not a project ROS package — like the other files in demos/.
#
# Starts:
#   gen3_bringup/gen3_lite_moveit.launch.py  (ros2_control, controllers,
#                                              move_group — its RViz is off)
#   moveit_servo servo_node                  (pose tracking -> joint_trajectory_controller)
#   demos/gen3_lite_drag_marker.py           (interactive marker -> Servo pose targets)
#   RViz with demos/gen3_lite_servo_drag.rviz (robot + marker only)
#
# Usage:
#   ros2 launch demos/gen3_lite_servo_drag_launch.py                        # fake hardware
#   ros2 launch demos/gen3_lite_servo_drag_launch.py use_fake_hardware:=false robot_ip:=<ip>
#   ros2 launch demos/gen3_lite_servo_drag_launch.py rviz:=false            # headless
#   ros2 launch demos/gen3_lite_servo_drag_launch.py max_speed:=0.05        # slower (m/s)
#
# In RViz: drag the arrows/rings (or the blue box) — the arm follows at up to
# max_speed. Right-click the marker: "Snap to gripper", "Move to ready pose".
#
# Needs: sudo apt install ros-jazzy-moveit-servo

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

HERE = os.path.dirname(os.path.abspath(__file__))


def launch_setup(context):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)

    bringup_share = get_package_share_directory("gen3_bringup")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "gen3_lite_moveit.launch.py")),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "false",
        }.items(),
    )

    # Same robot model as the bringup (D435i-equipped URDF/SRDF), rebuilt here
    # because Servo needs it as node parameters, not just on /robot_description.
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(
            file_path=os.path.join(bringup_share, "urdf", "gen3_lite_d435i.xacro"),
            mappings={
                "robot_ip": robot_ip,
                "use_fake_hardware": use_fake_hardware,
                "gripper": "gen3_lite_2f",
                "gripper_joint_name": "right_finger_bottom_joint",
                "dof": "6",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(bringup_share, "srdf", "gen3_lite_d435i.srdf.xacro"))
        .to_moveit_configs()
    )

    with open(os.path.join(HERE, "gen3_lite_servo.yaml")) as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Fake hardware starts in the straight-up singularity, where Servo will not
    # move: go to the ready pose first. Never automatic on the real arm — use
    # the marker's "Move to ready pose" menu entry there.
    marker_cmd = [
        "python3", os.path.join(HERE, "gen3_lite_drag_marker.py"),
        "--max-speed", LaunchConfiguration("max_speed").perform(context),
        "--max-ang-speed", LaunchConfiguration("max_ang_speed").perform(context),
    ]
    if use_fake_hardware == "true":
        marker_cmd.append("--go-ready")
    drag_marker = ExecuteProcess(cmd=marker_cmd, output="screen")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_servo_drag",
        output="log",
        arguments=["-d", os.path.join(HERE, "gen3_lite_servo_drag.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    if use_fake_hardware == "true":
        banner = ("*** FAKE HARDWARE — simulated arm, the real robot is NOT connected. "
                  "Real arm: use_fake_hardware:=false robot_ip:=<ip> ***")
    else:
        banner = f"*** REAL ARM at {robot_ip} — keep the E-stop within reach ***"

    return [
        LogInfo(msg=banner),
        bringup,
        rviz,
        # Controllers and move_group come up first; Servo needs /joint_states
        # and the planning scene, the marker needs Servo.
        TimerAction(period=4.0, actions=[servo_node, drag_marker]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0",
                              description="Gen3 Lite IP (ignored with fake hardware)"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="false = drive the real arm"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("max_speed", default_value="0.10",
                              description="max tool speed toward the marker (m/s)"),
        DeclareLaunchArgument("max_ang_speed", default_value="0.6",
                              description="max tool rotation speed toward the marker (rad/s)"),
        OpaqueFunction(function=launch_setup),
    ])
