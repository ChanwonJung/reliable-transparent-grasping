"""Full Slow Brain pipeline: instruction -> Qwen -> SAM 2.1 -> projection.

    ros2 launch qwen_server slow_brain.launch.py

Stage order is CAUSAL, not launch-order: each stage is triggered by the
previous stage's output topic, so nothing here enforces sequencing. The
TimerActions below only stagger console output and give the remote clients a
moment to open their sockets. Starting these nodes in any order still works.

Default (Gen3 Lite, wrist D435i only — single view):

    /user_instruction ───────────────┐
    /camera/camera/color/image_raw ──┴─→ qwen_bridge_node
                         ├─→ /qwen/labeled_detections ──┐
                         ├─→ /qwen/grounding_result ────│──→ bt_pkg
                         └─→ /qwen/source_image ────────┤
                                                        ↓
                                               sam_mask_node ─→ /sam/mask_image
                                                        ↓
    /camera/camera/aligned_depth_to_color/image_raw ─→ multi_view_projector_node
                                      (camera pose from TF) └─→ /world_map_result

Single view keeps BOTH categories from the wrist frame: a destination gets a
centroid whenever it is inside that frame. (In the Isaac scene it never was —
the basket sat below the wrist camera's near limit — which is why dual view
existed and was the default there.)

enable_dual_view:=true re-enables the overhead pass for a second, fixed camera:
set top_image_topic, top_depth_topic and top_camera_info_topic. The wrist then
owns TARGET and the overhead view owns DESTINATION.

The RealSense driver must run with align_depth.enable:=true.

Requires the SSH tunnels environments/ros2/seraph_env.sh opens: 10000 (Qwen) and 20000 (SAM 2.1).

Set enable_sam:=false to run grounding alone, or enable_projector:=false to
stop before the 3D fusion stage (useful when there is no depth stream).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _when_dual(value: str):
    """Substitute `value` when dual view is on, otherwise the empty string.

    The projector and qwen_bridge both read an empty topic name as "this view is
    disabled". Passing the real names unconditionally would make the projector
    wait out its top-mask timeout on every single-view scan for a mask that is
    never coming.
    """
    return PythonExpression([
        "'", value, "' if '", LaunchConfiguration("enable_dual_view"),
        "'.lower() in ('true', '1') else ''",
    ])


def _when_dual_cfg(arg_name: str):
    """Substitute another launch argument's value, gated the same way."""
    return PythonExpression([
        "'", LaunchConfiguration(arg_name), "' if '",
        LaunchConfiguration("enable_dual_view"),
        "'.lower() in ('true', '1') else ''",
    ])


def generate_launch_description() -> LaunchDescription:
    qwen_share = get_package_share_directory("qwen_server")
    qwen_params = os.path.join(qwen_share, "config", "qwen_server_params.yaml")

    args = [
        # ── Qwen ─────────────────────────────────────────────────────────────
        DeclareLaunchArgument("vllm_endpoint_url", default_value="http://localhost:10000/v1"),
        DeclareLaunchArgument("model_name", default_value="qwen35-local",
                              description="served model id (Qwen3.5-27B)"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("top_image_topic", default_value="",
                              description="overhead RGB — grounds the DESTINATION"),
        DeclareLaunchArgument("enable_dual_view", default_value="false",
                              description="false = one VLM call on the wrist view "
                                          "only; no destination centroid"),
        DeclareLaunchArgument("image_path", default_value="",
                              description="use this file as the frame instead of "
                                          "image_topic (offline test)"),
        DeclareLaunchArgument("top_image_path", default_value="",
                              description="same, for the overhead view"),
        DeclareLaunchArgument(
            "bbox_convention", default_value="normalized_1000",
            description="absolute | normalized_1000 | normalized_1 — "
                        "normalized_1000 verified against qwen35-local; "
                        "re-check with qwen_cli --annotate if the model changes"),
        DeclareLaunchArgument("instruction", default_value="",
                              description="seed instruction; empty waits for /user_instruction"),
        DeclareLaunchArgument("use_xterm", default_value="true",
                              description="false when driving /user_instruction externally"),
        # ── Stages ───────────────────────────────────────────────────────────
        DeclareLaunchArgument("enable_sam", default_value="true"),
        DeclareLaunchArgument("enable_projector", default_value="true"),
        # ── SAM 2.1 ──────────────────────────────────────────────────────────
        DeclareLaunchArgument("sam_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("sam_port", default_value="20000"),
        DeclareLaunchArgument(
            "ee_camera_info_topic",
            default_value="/camera/camera/aligned_depth_to_color/camera_info",
            description="EE DEPTH CameraInfo — sizes the label map AND gates the projector"),
        DeclareLaunchArgument("multimask", default_value="false",
                              description="3 SAM candidates per box, keep the best"),
        DeclareLaunchArgument("target_priority", default_value="true",
                              description="TARGET wins where masks overlap"),
        DeclareLaunchArgument(
            "annotated_save_history", default_value="false",
            description="keep every scan's overlay as output/annotated/"
                        "<view>_<stamp>.png, not just <view>_latest.png"),
        # ── Projection ───────────────────────────────────────────────────────
        DeclareLaunchArgument("ee_depth_topic",
                              default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("top_depth_topic", default_value=""),
        DeclareLaunchArgument("top_camera_info_topic", default_value=""),
        DeclareLaunchArgument("extrinsics_config", default_value=""),
    ]

    qwen_bridge = Node(
        package="qwen_server",
        executable="qwen_bridge_node",
        name="qwen_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[qwen_params, {
            "vllm_endpoint_url": LaunchConfiguration("vllm_endpoint_url"),
            "model_name": LaunchConfiguration("model_name"),
            "image_topic": LaunchConfiguration("image_topic"),
            "image_path": LaunchConfiguration("image_path"),
            "bbox_convention": LaunchConfiguration("bbox_convention"),
            "instruction": LaunchConfiguration("instruction"),
            # Empty disables the second VLM call inside the node.
            "top_image_topic": _when_dual_cfg("top_image_topic"),
            "top_image_path": _when_dual_cfg("top_image_path"),
        }],
    )

    sam_params = os.path.join(
        get_package_share_directory("sam_server"), "config", "sam_server_params.yaml")

    # Where each view's annotated overlay is mirrored to disk:
    # <repo>/experiments/logs/slow_brain/annotated (gitignored). The repo root is
    # found from the install share path; empty (= saving off) if it is not there.
    annotated_dir = ""
    for parent in Path(qwen_share).resolve().parents:
        if (parent / "ros2_ws").is_dir() and (parent / "experiments").is_dir():
            annotated_dir = str(parent / "experiments" / "logs" / "slow_brain" / "annotated")
            break

    sam_node = Node(
        package="sam_server",
        executable="sam_mask_node",
        name="sam_mask_node",
        output="screen",
        emulate_tty=True,
        parameters=[sam_params, {
            "zmq_host": LaunchConfiguration("sam_host"),
            "zmq_port": LaunchConfiguration("sam_port"),
            "ee_camera_info_topic": LaunchConfiguration("ee_camera_info_topic"),
            "multimask": LaunchConfiguration("multimask"),
            "target_priority": LaunchConfiguration("target_priority"),
            # Pinned rather than left to defaults so a change on either side
            # cannot silently unwire the Qwen -> SAM handoff.
            "source_image_topic": "/qwen/source_image",
            "detections_topic": "/qwen/labeled_detections",
            "mask_topic": "/sam/mask_image",
            "annotated_save_dir": annotated_dir,
            "annotated_save_name": "ee",
            "annotated_save_history": LaunchConfiguration("annotated_save_history"),
        }],
        condition=IfCondition(LaunchConfiguration("enable_sam")),
    )

    # Second instance, overhead view. Same executable, different topics — the
    # node is deliberately view-agnostic. The only subtle parameter is
    # camera_info_topic: the label map has to match the TOP depth resolution,
    # because that is the depth image the projector indexes it against.
    sam_top_node = Node(
        package="sam_server",
        executable="sam_mask_node",
        name="sam_mask_top_node",
        output="screen",
        emulate_tty=True,
        parameters=[sam_params, {
            "zmq_host": LaunchConfiguration("sam_host"),
            "zmq_port": LaunchConfiguration("sam_port"),
            "camera_info_topic": LaunchConfiguration("top_camera_info_topic"),
            "multimask": LaunchConfiguration("multimask"),
            "target_priority": LaunchConfiguration("target_priority"),
            "source_image_topic": "/qwen/top/source_image",
            "detections_topic": "/qwen/top/labeled_detections",
            "mask_topic": "/sam/top/mask_image",
            "annotated_topic": "/sam/top/annotated_image",
            "annotated_save_dir": annotated_dir,
            "annotated_save_name": "top",
            "annotated_save_history": LaunchConfiguration("annotated_save_history"),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("enable_sam"), "'.lower() in ('true', '1')",
            " and '", LaunchConfiguration("enable_dual_view"),
            "'.lower() in ('true', '1')",
        ])),
    )

    projector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("mask_projection_pkg"),
            "launch", "multi_view_projector.launch.py")),
        launch_arguments={
            "ee_depth_topic": LaunchConfiguration("ee_depth_topic"),
            "top_depth_topic": LaunchConfiguration("top_depth_topic"),
            # REQUIRED: the projector gates on EE depth + EE camera_info before
            # it will act on a mask. Isaac publishes these under /*_rgbd_camera/,
            # not /ee_camera/ — omitting them stalls the pipeline silently.
            "ee_camera_info_topic": LaunchConfiguration("ee_camera_info_topic"),
            "top_camera_info_topic": LaunchConfiguration("top_camera_info_topic"),
            "extrinsics_config": LaunchConfiguration("extrinsics_config"),
            # Defaults already point here, but pin them so a change to the
            # projector's defaults cannot silently unwire this pipeline.
            "mask_topic": "/sam/mask_image",
            "detections_topic": "/qwen/labeled_detections",
            # Empty when dual view is off — the projector reads that as
            # "top labeled pass disabled" and never waits for a second mask.
            "top_mask_topic": _when_dual("/sam/top/mask_image"),
            "top_detections_topic": _when_dual("/qwen/top/labeled_detections"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_projector")),
    )

    prompt_common = {
        "package": "qwen_server",
        "executable": "instruction_prompt_node",
        "name": "instruction_prompt_node",
        "output": "screen",
        "parameters": [qwen_params],
    }
    prompt_xterm = Node(
        **prompt_common,
        prefix="xterm -geometry 100x16 -T 'Slow Brain instruction' -e",
        condition=IfCondition(LaunchConfiguration("use_xterm")),
    )
    prompt_plain = Node(
        **prompt_common,
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("use_xterm")),
    )

    return LaunchDescription([
        *args,
        # Consumers first so they are subscribed before anything can publish.
        # Cosmetic only — every Slow Brain topic is latched, so a late
        # subscriber still receives the last message.
        projector,
        TimerAction(period=1.0, actions=[GroupAction([sam_node, sam_top_node])]),
        TimerAction(period=2.0, actions=[GroupAction([qwen_bridge])]),
        # Prompt last: nothing to type until the pipeline is listening.
        TimerAction(period=3.0, actions=[GroupAction([prompt_xterm, prompt_plain])]),
    ])
