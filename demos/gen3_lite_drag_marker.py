#!/usr/bin/env python3
"""Drag-to-follow teleop for the Gen3 Lite: an RViz interactive marker whose
pose is streamed to MoveIt Servo as a pose target. Not a project ROS package --
a standalone rclpy script, like the other files in demos/.

The marker starts on tool_frame (the "arm" group's IK tip). Drag it and Servo
drives the arm toward it continuously, with its own velocity limits,
singularity slowdown and collision checking. Right-click the marker for:
  Snap to gripper    — marker back onto the gripper (after dragging somewhere
                       unreachable)
  Move to ready pose — planned move_group motion to READY_JOINTS, Servo paused

Servo refuses to move from a singularity, and the all-zero "candlestick" pose
fake hardware starts in is one (arm fully straight up). --go-ready moves to
the ready pose first; the launch file passes it only with fake hardware, so
the real arm never moves on its own at startup.

Normally started by demos/gen3_lite_servo_drag_launch.py. Standalone:
  python3 demos/gen3_lite_drag_marker.py [--servo-ns /servo_node] [--rate 50] [--go-ready]
      [--max-speed 0.10] [--max-ang-speed 0.6]

Why the target is rate-limited here: Servo's pose tracking (Jazzy, 2.12)
tries to close the WHOLE pose error every control cycle, limited only by the
joint velocity limits — so any jump of the marker is chased at full joint
speed and overshoots. The published target instead walks toward the marker at
--max-speed / --max-ang-speed, so Servo only ever sees small, reachable steps.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

BASE_FRAME = "base_link"
EE_LINK = "tool_frame"
PLANNING_GROUP = "arm"
MARKER_NAME = "gen3_lite_target"
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
# tool_frame at (0.30, 0, 0.30) in base_link pointing straight down — a
# /compute_ik solution, well clear of the straight-up singularity.
READY_JOINTS = [-0.157, -0.186, 1.317, 1.571, 1.639, -0.157]


def _step_toward(cur: Pose, goal: Pose, max_lin: float, max_ang: float) -> Pose:
    """Move `cur` toward `goal` by at most max_lin (m) and max_ang (rad)."""
    out = Pose()
    d = [goal.position.x - cur.position.x,
         goal.position.y - cur.position.y,
         goal.position.z - cur.position.z]
    dist = math.sqrt(sum(v * v for v in d))
    k = 1.0 if dist <= max_lin else max_lin / dist
    out.position.x = cur.position.x + k * d[0]
    out.position.y = cur.position.y + k * d[1]
    out.position.z = cur.position.z + k * d[2]

    # Quaternion slerp, fraction chosen so the rotation step is <= max_ang.
    a = (cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w)
    b = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:                      # shortest path
        b, dot = [-v for v in b], -dot
    dot = min(1.0, dot)
    angle = 2.0 * math.acos(dot)       # rotation between the two
    if angle < 1e-6:
        q = b
    else:
        t = min(1.0, max_ang / angle)
        half = angle / 2.0
        s0 = math.sin((1.0 - t) * half) / math.sin(half)
        s1 = math.sin(t * half) / math.sin(half)
        q = [s0 * x + s1 * y for x, y in zip(a, b)]
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
    return out


def _axis_controls(marker: InteractiveMarker) -> None:
    """Move + rotate handles on all three axes (the standard 6-DOF set)."""
    # Control axis = the control frame's +X; these rotate +X onto each axis.
    h = 0.5 ** 0.5
    for name, (w, x, y, z) in {
        "x": (h, h, 0.0, 0.0),
        "y": (h, 0.0, 0.0, h),
        "z": (h, 0.0, h, 0.0),
    }.items():
        for mode, prefix in (
            (InteractiveMarkerControl.MOVE_AXIS, "move_"),
            (InteractiveMarkerControl.ROTATE_AXIS, "rotate_"),
        ):
            c = InteractiveMarkerControl()
            c.name = prefix + name
            c.orientation.w, c.orientation.x = w, x
            c.orientation.y, c.orientation.z = y, z
            c.interaction_mode = mode
            marker.controls.append(c)


class DragMarker(Node):
    def __init__(self, servo_ns: str, rate_hz: float, go_ready: bool,
                 max_speed: float, max_ang_speed: float) -> None:
        super().__init__("gen3_lite_drag_marker")
        self._servo_ns = servo_ns.rstrip("/")
        self._go_ready = go_ready
        self._max_step = max_speed / rate_hz
        self._max_ang_step = max_ang_speed / rate_hz

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._target_pub = self.create_publisher(
            PoseStamped, f"{self._servo_ns}/pose_target_cmds", 10)
        self._switch_cli = self.create_client(
            ServoCommandType, f"{self._servo_ns}/switch_command_type")
        self._pause_cli = self.create_client(SetBool, f"{self._servo_ns}/pause_servo")
        self._move_cli = ActionClient(self, MoveGroup, "/move_action")

        self._server = InteractiveMarkerServer(self, "gen3_lite_drag")
        self._menu = MenuHandler()
        self._menu.insert("Snap to gripper", callback=self._on_snap)
        self._menu.insert("Move to ready pose", callback=lambda _fb: self._move_to_ready())

        self._target: Pose | None = None   # where the marker is (None until it exists)
        self._command: Pose | None = None  # what Servo is told: walks toward _target
        self._streaming = False            # off while move_group owns the arm
        self._started = False
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    # ── setup ────────────────────────────────────────────────────────────────
    def _ee_pose(self) -> Pose | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_FRAME, EE_LINK, Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        p = Pose()
        p.position.x = tf.transform.translation.x
        p.position.y = tf.transform.translation.y
        p.position.z = tf.transform.translation.z
        p.orientation = tf.transform.rotation
        return p

    def _create_marker(self, pose: Pose) -> None:
        im = InteractiveMarker()
        im.header.frame_id = BASE_FRAME
        im.name = MARKER_NAME
        im.description = "drag: Servo follows\nright-click: snap back"
        im.scale = 0.2
        im.pose = pose

        # A small box at the marker centre, so there is something to grab
        # besides the arrows/rings.
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = box.scale.z = 0.04
        box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.6, 1.0, 0.8
        centre = InteractiveMarkerControl()
        centre.always_visible = True
        centre.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        centre.markers.append(box)
        im.controls.append(centre)
        _axis_controls(im)

        self._server.insert(im, feedback_callback=self._on_feedback)
        self._menu.apply(self._server, MARKER_NAME)
        self._server.applyChanges()
        self._target = pose
        self._command = pose

    def _switch_to_pose_mode(self) -> None:
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.POSE
        self._switch_cli.call_async(req).add_done_callback(self._on_switched)

    def _on_switched(self, future) -> None:
        res = future.result()
        if res is not None and res.success:
            self._streaming = True
            self.get_logger().info("Servo in POSE mode — drag the marker")
        else:
            self.get_logger().error("Servo refused POSE mode — is servo_node running?")

    # ── ready pose (move_group, Servo paused) ────────────────────────────────
    def _move_to_ready(self) -> None:
        # Two writers on joint_trajectory_controller would fight: stop our
        # stream and pause Servo before move_group executes.
        self._streaming = False
        self._pause_cli.call_async(SetBool.Request(data=True)).add_done_callback(
            lambda _f: self._send_ready_goal())

    def _send_ready_goal(self) -> None:
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.start_state.is_diff = True
        c = Constraints()
        for name, val in zip(ARM_JOINTS, READY_JOINTS):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=val,
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        self.get_logger().info("Moving to ready pose (move_group)...")
        self._move_cli.send_goal_async(goal).add_done_callback(self._on_ready_accepted)

    def _on_ready_accepted(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Ready-pose goal rejected")
            self._resume_servo()
            return
        handle.get_result_async().add_done_callback(self._on_ready_done)

    def _on_ready_done(self, future) -> None:
        code = future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("At ready pose")
        else:
            self.get_logger().error(f"Ready-pose move failed (MoveItErrorCode {code})")
        self._resume_servo()

    def _resume_servo(self) -> None:
        def after_unpause(_f):
            # The arm moved without the marker: put the marker back on it, or
            # Servo would immediately drive to the stale target.
            pose = self._ee_pose()
            if pose is not None:
                if self._target is None:
                    self._create_marker(pose)
                else:
                    self._set_marker(pose)
            self._switch_to_pose_mode()
        self._pause_cli.call_async(SetBool.Request(data=False)).add_done_callback(
            after_unpause)

    # ── marker callbacks ─────────────────────────────────────────────────────
    def _on_feedback(self, fb: InteractiveMarkerFeedback) -> None:
        if fb.event_type in (InteractiveMarkerFeedback.POSE_UPDATE,
                             InteractiveMarkerFeedback.MOUSE_UP):
            self._target = fb.pose

    def _on_snap(self, _fb) -> None:
        pose = self._ee_pose()
        if pose is not None:
            self._set_marker(pose)

    def _set_marker(self, pose: Pose) -> None:
        self._target = pose
        self._command = pose
        self._server.setPose(MARKER_NAME, pose)
        self._server.applyChanges()

    # ── loop ─────────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        if not self._started:
            # Wait for TF (robot_state_publisher), Servo and move_group.
            pose = self._ee_pose()
            if (pose is None or not self._switch_cli.service_is_ready()
                    or not self._pause_cli.service_is_ready()
                    or not self._move_cli.server_is_ready()):
                return
            self._started = True
            if self._go_ready:
                self._move_to_ready()   # creates the marker when done
            else:
                self._create_marker(pose)
                self._switch_to_pose_mode()
            return
        if not self._streaming or self._target is None:
            return

        # Stream continuously: Servo halts after incoming_command_timeout
        # without a fresh command, and the target is where the marker IS, not
        # a one-shot goal.
        msg = PoseStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self._command = _step_toward(
            self._command, self._target, self._max_step, self._max_ang_step)
        msg.pose = self._command
        self._target_pub.publish(msg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--servo-ns", default="/servo_node")
    ap.add_argument("--rate", type=float, default=50.0, help="target publish rate (Hz)")
    ap.add_argument("--go-ready", action="store_true",
                    help="move to the ready pose via move_group before servoing")
    ap.add_argument("--max-speed", type=float, default=0.10,
                    help="max target speed toward the marker (m/s)")
    ap.add_argument("--max-ang-speed", type=float, default=0.6,
                    help="max target rotation speed toward the marker (rad/s)")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = DragMarker(args.servo_ns, args.rate, args.go_ready,
                      args.max_speed, args.max_ang_speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
