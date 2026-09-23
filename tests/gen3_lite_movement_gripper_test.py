#!/usr/bin/env python3
"""Real-hardware smoke test: nudge the Gen3 Lite outward via MoveIt2, then
close and open the gripper. Not a project ROS package -- a standalone rclpy
script, like the other files in tests/.

"Outward" is computed as a real Cartesian step away from base_link in the
current horizontal direction of the end effector (tool_frame), not a guessed
joint-space direction -- this way it's correct regardless of the arm's
current pose, and stays small and predictable (5cm default).

Requires gen3_bringup's MoveGroup bringup already running:
  ros2 launch gen3_bringup gen3_lite_moveit.launch.py robot_ip:=<ip> [use_fake_hardware:=true]

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/workspace/ros2_kortex_ws/install/setup.bash
  source <this repo>/ros2_ws/install/setup.bash
  python3 tests/gen3_lite_movement_gripper_test.py [--step-m 0.05] [--velocity-scale 0.1] [--yes]

Prompts for confirmation before moving the real arm unless --yes is passed.
"""
import argparse
import math
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MotionPlanRequest, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

BASE_FRAME = "base_link"
EE_LINK = "tool_frame"          # "arm" group's actual IK tip -- see gen3_lite.srdf / tool_frame_joint
PLANNING_GROUP = "arm"
MOVE_ACTION = "/move_action"
GRIPPER_ACTION = "/gen3_lite_2f_gripper_controller/gripper_cmd"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.85             # rad -- real hardware joint limit (gen3_lite_2f_macro.xacro)
GRIPPER_MAX_EFFORT = 50.0
MOVE_GOAL_SUCCESS = 1            # moveit_msgs/MoveItErrorCodes.SUCCESS


class MovementGripperTest(Node):
    def __init__(self, step_m: float, velocity_scale: float) -> None:
        super().__init__("gen3_lite_movement_gripper_test")
        self._step_m = step_m
        self._velocity_scale = velocity_scale

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._move_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self._gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

    def wait_for_tf(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._tf_buffer.can_transform(BASE_FRAME, EE_LINK, rclpy.time.Time()):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def current_pose(self):
        tf = self._tf_buffer.lookup_transform(BASE_FRAME, EE_LINK, rclpy.time.Time())
        return tf.transform.translation, tf.transform.rotation

    def build_outward_goal(self):
        translation, rotation = self.current_pose()
        x, y, z = translation.x, translation.y, translation.z
        horiz = math.hypot(x, y)
        if horiz < 1e-6:
            # Directly above the base -- no horizontal direction to extend along; pick +X.
            dir_x, dir_y = 1.0, 0.0
        else:
            dir_x, dir_y = x / horiz, y / horiz
        target = (x + self._step_m * dir_x, y + self._step_m * dir_y, z)
        return target, rotation, (x, y, z)

    def send_move_goal(self, target_xyz, orientation) -> bool:
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = BASE_FRAME
        pos_constraint.link_name = EE_LINK
        pos_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm goal tolerance
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target_xyz
        pose.orientation = orientation
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(pose)
        pos_constraint.weight = 1.0

        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = BASE_FRAME
        orient_constraint.link_name = EE_LINK
        orient_constraint.orientation = orientation
        orient_constraint.absolute_x_axis_tolerance = 0.15
        orient_constraint.absolute_y_axis_tolerance = 0.15
        orient_constraint.absolute_z_axis_tolerance = 0.15
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.pipeline_id = "ompl"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self._velocity_scale
        req.max_acceleration_scaling_factor = self._velocity_scale
        req.start_state.is_diff = True
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return False

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        self.get_logger().info(f"Move finished with MoveItErrorCode {code}")
        return code == MOVE_GOAL_SUCCESS

    def send_gripper_goal(self, position: float, max_effort: float = GRIPPER_MAX_EFFORT):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available")
            return None

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-m", type=float, default=0.05, help="Outward Cartesian step, meters (default 5cm)")
    parser.add_argument(
        "--velocity-scale", type=float, default=0.1,
        help="Max velocity/acceleration scaling factor, 0-1 (default 0.1 = slow)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    args, _ = parser.parse_known_args()

    velocity_scale = min(max(args.velocity_scale, 0.01), 1.0)

    rclpy.init()
    node = MovementGripperTest(step_m=args.step_m, velocity_scale=velocity_scale)

    try:
        node.get_logger().info(f"Waiting for TF ({BASE_FRAME} -> {EE_LINK})...")
        if not node.wait_for_tf():
            node.get_logger().error(
                "TF not available -- is gen3_bringup's gen3_lite_moveit.launch.py running?"
            )
            sys.exit(1)

        target_xyz, orientation, current_xyz = node.build_outward_goal()
        print(f"\nCurrent EE position ({BASE_FRAME}): {tuple(round(v, 4) for v in current_xyz)}")
        print(f"Target  EE position ({BASE_FRAME}): {tuple(round(v, 4) for v in target_xyz)}")
        print(f"Step: {args.step_m * 100:.1f} cm outward, velocity scale {velocity_scale}\n")

        if not args.yes:
            resp = input("Type 'go' to execute this motion on the real robot: ").strip()
            if resp != "go":
                print("Aborted.")
                sys.exit(0)

        if not node.send_move_goal(target_xyz, orientation):
            node.get_logger().error("Arm motion failed -- stopping before gripper test")
            sys.exit(1)
        node.get_logger().info("Arm motion succeeded.")

        node.get_logger().info("Closing gripper...")
        close_result = node.send_gripper_goal(GRIPPER_CLOSE)
        if close_result is not None:
            node.get_logger().info(
                f"Close result: position={close_result.position:.3f} "
                f"stalled={close_result.stalled} reached_goal={close_result.reached_goal}"
            )
        time.sleep(1.0)

        node.get_logger().info("Opening gripper...")
        open_result = node.send_gripper_goal(GRIPPER_OPEN)
        if open_result is not None:
            node.get_logger().info(
                f"Open result: position={open_result.position:.3f} "
                f"stalled={open_result.stalled} reached_goal={open_result.reached_goal}"
            )

        node.get_logger().info("Movement + gripper test complete.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
