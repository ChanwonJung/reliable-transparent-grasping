# Copyright (c) 2024 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Adapted from kinova_gen3_lite_moveit_config/launch/robot.launch.py.
# The only change from upstream: robot_description is loaded from this
# repo's own gen3_lite_d435i.xacro (arm + gripper + wrist-mounted D435i)
# instead of kinova_gen3_lite_moveit_config's bare arm+gripper xacro.
# Everything else -- SRDF, kinematics, joint limits, ros2_control config,
# controller spawners, move_group -- is the vendored Gen3 Lite config,
# unmodified. No hybrid planning, no hazard/BT nodes: MoveGroup only.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "gen3_lite_2f",
        "gripper_joint_name": "right_finger_bottom_joint",
        "dof": "6",
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
    }

    gen3_bringup_share = get_package_share_directory("gen3_bringup")
    gen3_bringup_urdf = os.path.join(gen3_bringup_share, "urdf", "gen3_lite_d435i.xacro")
    gen3_bringup_srdf = os.path.join(gen3_bringup_share, "srdf", "gen3_lite_d435i.srdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(
            "gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config"
        )
        .robot_description(file_path=gen3_bringup_urdf, mappings=launch_arguments)
        .robot_description_semantic(file_path=gen3_bringup_srdf)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_lite_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gen3_lite_2f_gripper_controller", "-c", "/controller_manager"],
    )

    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_fake_hardware),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("kinova_gen3_lite_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager",
            # 100 Hz instead of the controller manager's 1 kHz — see the file.
            "--param-file", os.path.join(gen3_bringup_share, "config", "controller_overrides.yaml"),
        ],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        fault_controller_spawner,
        move_group_node,
        static_tf,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "gripper_max_velocity", default_value="100.0",
            description="Max velocity for gripper commands",
        ),
        DeclareLaunchArgument(
            "gripper_max_force", default_value="100.0",
            description="Max force for gripper commands",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulated clock",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
