"""
multi_view_projector.launch.py — SAM mask + aligned depth → /world_map_result.

Default wiring is the Gen3 Lite wrist D435i (single view):

  realsense2_camera (align_depth.enable:=true)
    /camera/camera/aligned_depth_to_color/image_raw   ─┐
    /camera/camera/aligned_depth_to_color/camera_info ─┤
  sam_server  /sam/mask_image                          ─┼─→ multi_view_projector_node
  qwen_server /qwen/labeled_detections                 ─┘        └─→ /world_map_result

Camera pose comes from TF (base_link <- camera_color_optical_frame) at each
depth frame's stamp. extrinsics_source:=yaml restores the Isaac static R/t.

The top_* arguments add an optional second, fixed camera; empty = off.

Usage:
  ros2 launch mask_projection_pkg multi_view_projector.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_extrinsics() -> str:
    """이 패키지 안의 camera_extrinsics.yaml — Isaac 씬에서 잰 R/t (yaml 모드 전용)."""
    return os.path.join(
        get_package_share_directory("mask_projection_pkg"),
        "config", "camera_extrinsics.yaml")


def generate_launch_description() -> LaunchDescription:
    # ── declare all arguments ─────────────────────────────────────────────────
    args = [
        # ── Top camera topics ─────────────────────────────────────────────────
        # Empty = no overhead camera (the Gen3 Lite setup has only the wrist
        # D435i). Set these only if a second, fixed camera is added.
        DeclareLaunchArgument('top_depth_topic', default_value=''),
        DeclareLaunchArgument('top_camera_info_topic', default_value=''),

        # ── EE camera topics ──────────────────────────────────────────────────
        # Wrist D435i via realsense2_camera with align_depth.enable:=true —
        # depth must be aligned to COLOR, because the SAM mask is drawn on the
        # color image and indexed pixel-for-pixel against this depth.
        DeclareLaunchArgument('ee_depth_topic',
                              default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('ee_camera_info_topic',
                              default_value='/camera/camera/aligned_depth_to_color/camera_info'),

        # ── Qwen / stub output topics ─────────────────────────────────────────
        # Default: qwen_stub_node (or real Qwen) output.
        # To bypass Qwen and connect directly to GSAM, override:
        #   mask_topic:=/grounded_sam/mask_image
        #   detections_topic:=/grounded_sam/detections_json
        DeclareLaunchArgument('mask_topic',
                              default_value='/qwen/mask_image'),
        DeclareLaunchArgument('detections_topic',
                              default_value='/qwen/labeled_detections'),

        # ── Top-view labeled pass (dual-view Slow Brain) ──────────────────────
        # Both empty = off; the top camera then contributes UNKNOWN geometry
        # only, as before. Set both to project the overhead SAM mask with the
        # top depth image — the only source of a DESTINATION centroid, since the
        # wrist camera cannot see the basket (near limit x ~ 0.58 m in
        # panda_link0 vs the basket at x = 0.48).
        DeclareLaunchArgument('top_mask_topic', default_value=''),
        DeclareLaunchArgument('top_detections_topic', default_value=''),
        DeclareLaunchArgument('top_mask_timeout_sec', default_value='8.0',
                              description='publish EE-only if the top mask has '
                                          'not arrived this long after the EE one'),

        # ── Output topics ─────────────────────────────────────────────────────
        DeclareLaunchArgument('output_cloud_topic',
                              default_value='/world_map'),
        DeclareLaunchArgument('output_result_topic',
                              default_value='/world_map_result'),
        DeclareLaunchArgument('output_raw_cloud_topic',
                              default_value='/world_cloud_raw'),

        # ── Depth filter ──────────────────────────────────────────────────────
        DeclareLaunchArgument('min_depth', default_value='0.05'),
        DeclareLaunchArgument('max_depth', default_value='15.0'),

        # ── Optional filename prefix for saved PLY files ──────────────────────
        DeclareLaunchArgument('initials', default_value=''),

        # ── Camera extrinsics ─────────────────────────────────────────────────
        # tf: camera pose from TF at each depth stamp (real robot, default).
        # yaml: static R/t from extrinsics_config (Isaac scene values).
        DeclareLaunchArgument('extrinsics_source', default_value='tf'),
        DeclareLaunchArgument('world_frame', default_value='base_link'),
        DeclareLaunchArgument('extrinsics_config',
                              default_value=_default_extrinsics()),

        # ── Freeze after first publish ────────────────────────────────────────
        # true 면 최초 성공 publish 이후 신규 mask 를 무시한다. extrinsics_source
        # :=yaml 일 때만 의미가 있다 — 정적 R/t 는 팔이 움직인 뒤 틀린다. tf 모드
        # 에서는 매 스캔 시점의 카메라 포즈를 쓰므로 필요 없다.
        DeclareLaunchArgument('freeze_after_first_publish',
                              default_value='false'),
    ]

    node = Node(
        package    = 'mask_projection_pkg',
        executable = 'multi_view_projector_node',
        name       = 'multi_view_projector_node',
        output     = 'screen',
        parameters = [
            {
                'top_depth_topic':        LaunchConfiguration('top_depth_topic'),
                'top_camera_info_topic':  LaunchConfiguration('top_camera_info_topic'),
                'ee_depth_topic':         LaunchConfiguration('ee_depth_topic'),
                'ee_camera_info_topic':   LaunchConfiguration('ee_camera_info_topic'),
                'mask_topic':             LaunchConfiguration('mask_topic'),
                'detections_topic':       LaunchConfiguration('detections_topic'),
                'top_mask_topic':         LaunchConfiguration('top_mask_topic'),
                'top_detections_topic':   LaunchConfiguration('top_detections_topic'),
                'top_mask_timeout_sec':   LaunchConfiguration('top_mask_timeout_sec'),
                'output_cloud_topic':     LaunchConfiguration('output_cloud_topic'),
                'output_result_topic':    LaunchConfiguration('output_result_topic'),
                'output_raw_cloud_topic': LaunchConfiguration('output_raw_cloud_topic'),
                'min_depth':              LaunchConfiguration('min_depth'),
                'max_depth':              LaunchConfiguration('max_depth'),
                'initials':               LaunchConfiguration('initials'),
                'extrinsics_config':      LaunchConfiguration('extrinsics_config'),
                'extrinsics_source':      LaunchConfiguration('extrinsics_source'),
                'world_frame':            LaunchConfiguration('world_frame'),
                'freeze_after_first_publish':
                    LaunchConfiguration('freeze_after_first_publish'),
            },
        ],
    )

    return LaunchDescription(args + [node])
