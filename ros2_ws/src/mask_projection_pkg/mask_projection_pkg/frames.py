"""Camera pose from TF, and depth images decoded to metres.

Both exist for the real Gen3 Lite + wrist D435i, where the Isaac assumptions
behind camera_extrinsics.yaml stop holding:

* The camera rides on the wrist, so its pose in base_link changes every time
  the arm moves. A static R/t is only right at the one pose it was measured at;
  TF (robot_state_publisher + the gen3_bringup URDF mount) is right at every
  pose, looked up at the depth frame's own timestamp.
* The RealSense driver publishes depth as 16UC1 millimetres. cv_bridge's
  desired_encoding='32FC1' only casts the dtype — it does not rescale — so a
  1.2 m reading arrives as 1200.0 "metres" and falls outside max_depth.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image


def depth_to_metres(msg: Image) -> np.ndarray:
    """32FC1 (metres) or 16UC1 (millimetres) → (H, W) float32 metres."""
    if msg.encoding == '32FC1':
        buf = np.frombuffer(msg.data, dtype=np.float32)
        scale = 1.0
    elif msg.encoding in ('16UC1', 'mono16'):
        buf = np.frombuffer(msg.data, dtype=np.uint16)
        scale = 0.001
    else:
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
    if msg.is_bigendian:
        buf = buf.byteswap()
    # step can include row padding, so reshape by step rather than width.
    per_row = msg.step // buf.itemsize
    depth = buf.reshape(msg.height, per_row)[:, :msg.width].astype(np.float32)
    return depth * scale if scale != 1.0 else depth


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w),       s * (x * z + y * w)],
        [s * (x * y + z * w),       1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),       s * (y * z + x * w),       1.0 - s * (x * x + y * y)],
    ])


def lookup_camera_pose(
    tf_buffer,
    world_frame: str,
    msg: Image,
    timeout_sec: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """(R, t) with p_world = R @ p_cam + t, for the frame `msg` was taken in.

    Same convention as camera_extrinsics.yaml, so callers can switch between
    the two without touching the projection maths. Raises the tf2 exception
    when the transform is unavailable — the caller decides whether to skip.
    """
    tf = tf_buffer.lookup_transform(
        world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
        timeout=Duration(seconds=timeout_sec))
    q = tf.transform.rotation
    p = tf.transform.translation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])
