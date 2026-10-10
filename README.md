# FRAME: A Confidence-Guided Fusion Model of RGB-D and Monocular Priors for Transparent and Specular Object Depth Restoration

FRAME is the third depth-restoration model, served on port **32000**. Its code/config
identifier is `frame`; model selection also accepts `FRAME`.

This branch adds the local tools needed to view, score, and consume FRAME depth:

- A live RGB-D restoration viewer with SAM2 click prompts and confidence maps.
- Ground-truth capture, restoration metrics, and saved HTML/JSON evaluation reports.
- A ROS 2 adapter that matches captured RGB-D scans to their target masks and publishes restored depth.
- Documentation and SSH forwarding for the restoration services.

The model architecture, training implementation, checkpoints, and inference server
live separately on Seraph and are not present in this checkout. SwinDRNet (30000)
and ReMake (31000) remain selectable in the shared comparison tools. This branch
does not include the pending robot motion, grasp planning, calibration, or combined
robot-pipeline changes from the local working tree.

## View and score FRAME

Use a ROS 2 environment with `realsense2_camera`, `cv_bridge`, `message_filters`,
NumPy, and OpenCV. Open the SSH tunnels on the host, overriding the login names
for your cluster account:

```bash
export SERAPH_JUMP=user@aurora.khu.ac.kr:30080 SERAPH_HOST=user@aurora-g6
source environments/ros2/seraph_env.sh
```

In a terminal with ROS sourced, start one aligned RGB-D camera:

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera \
  align_depth.enable:=true enable_sync:=true
```

In another terminal with the same ROS environment:

```bash
python3 demos/restore_depth_viewer.py --model FRAME
```

`--model 32000` selects the same service. Click the transparent object to supply
SAM2 prompts; right-click marks background, `c` clears prompts, and `u` undoes.
The confidence row shows `C = 1 - pred_weight`, the confidence in raw depth.

With a fixed camera, press `g` while an opaque stand-in occupies the object's
position to capture reference depth. Replace it with the transparent object,
click it, then press Space to score the next restored frames. Captures go to
`data/captures/` and include `metrics.json` and a self-contained `report.html`.
See [the viewer guide](demos/README.md#3-live-depth-restoration-viewer) for metric
definitions, options, and troubleshooting.

## Consume FRAME depth in ROS

```bash
cd ros2_ws
colcon build --packages-select depth_restoration_ros
source install/setup.bash
ros2 run depth_restoration_ros depth_restoration_node --ros-args -p model:=FRAME
```

Use `-p frame_url:=http://HOST:32000/restore_depth` to override the endpoint.
Inputs are a captured RGB-D scan, camera intrinsics, stamped detections, and a
matching SAM label map. Output depth uses float32 metres and retains raw depth
outside the target by default. The [adapter guide](ros2_ws/src/depth_restoration_ros/README.md)
defines topics, timestamp matching, units, and failure handling.

## Validate without model services or robot hardware

```bash
PYTHONPATH=ros2_ws/src/depth_restoration_ros python3 -m pytest -q \
  ros2_ws/src/depth_restoration_ros/test demos/test_frame_restoration.py
```

Tests exercise real multipart HTTP requests against a loopback service double,
FRAME selection, response validation, depth units, scan matching, and capture naming.
