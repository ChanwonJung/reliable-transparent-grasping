# depth_restoration_ros

Single-EE-camera adapter to the SwinDRNet, ReMake, and FRAME HTTP services.
No training or upstream model code is imported. FRAME uses port 32000; model
names are case-insensitive. Launch the standalone adapter after building and
sourcing the ROS workspace:

```bash
ros2 run depth_restoration_ros depth_restoration_node --ros-args -p model:=FRAME
```

The node's default model is `swindrnet`; select `remake`, `frame`, or explicit
`raw` mode with the `model` parameter. Default URLs use `server_host` (127.0.0.1)
with ports 30000, 31000, and 32000 respectively. Set `frame_url` to override the
full FRAME endpoint. Configuration is read at startup; restart to change models.

The HTTP request is multipart `POST /restore_depth` with full-frame `rgb`,
`depth`, and `mask` PNG files: RGB colour, uint16 millimetres, and binary TARGET
mask. Response `depth_png_base64` must decode to a uint16 single-channel image
with the original dimensions. By default, only TARGET pixels are replaced;
non-target pixels retain raw aligned depth. `preserve_background:=false` uses
the full restored image. Output is always `32FC1` metres. Invalid responses,
empty target masks/depth, missing calibration, timeouts, and HTTP errors publish
status `error` and no output scan. There is no silent raw-depth fallback.

Inputs are retained captured messages, not live “latest” camera streams:

| Parameter | Default topic | Pairing |
| --- | --- | --- |
| `source_image_topic` | `/qwen/source_image` | RGB source stamp |
| `source_depth_topic` | `/qwen/source_depth` | Within `pair_slop_s` (0.05 s) of RGB |
| `source_camera_info_topic` | `/qwen/source_camera_info` | Within slop of depth |
| `source_mask_topic` | `/sam/mask_image` | Exact RGB stamp |
| `source_detections_topic` | `/qwen/labeled_detections` | Exact RGB stamp in metadata |

Detections may be a legacy JSON list whose entries all contain the same
`source_stamp_ns`, or an envelope `{source_stamp_ns, detections}`. Unstamped
lists are rejected so a previous scan cannot be mistaken for the current scan.
SAM's label value `i+1` maps to the original detection list's entry `i`; only
entries categorized TARGET become the restoration mask. All dimensions must
match the aligned depth image; no fixed crops or object-shape presets are used.

The output bundle is `/depth_restoration/depth_image`, `source_image`,
`camera_info`, `mask_image`, and `labeled_detections`. All image/CameraInfo
headers preserve the original depth stamp and optical frame. Detections are a
JSON envelope `{source_stamp_ns, frame_id, detections}` with that same stamp.
Consumers must match stamps, regardless of cross-topic DDS arrival order.
The node publishes the mask last as a convenient completion signal, with
reliable transient-local QoS on every bundle topic. `/depth_restoration/status`
publishes JSON states `idle`, `running`, `ready`, and `error` with scan metadata.

`/slow_brain/status` (shared with Qwen, told apart by `stage`) gets exactly two lines per model run: one when
the request to the service starts and one when it ends. Raw mode runs no model and publishes none. Pre-request
failures (bad frames, empty mask) appear only on `/depth_restoration/status`.

```json
{"stage": "depth_restoration", "state": "running", "model": "frame", "port": 32000, "inference_time_s": null}
{"stage": "depth_restoration", "state": "ok",      "model": "frame", "port": 32000, "inference_time_s": 0.31}
```

`state` is `running`, then `ok` or `failed` (a `reason` field carries the error). `port` is the port of the
endpoint actually used, so it follows an overridden `*_url`. `inference_time_s` is the whole model call as the node
sees it: PNG encoding, the HTTP round trip, the server's inference and decoding the reply. It is `null` on the
start line. The topic is `slow_brain_status_topic`; latched with depth 2, so a late subscriber sees the latest
start and end lines.

Cache size is bounded (`cache_size`, default 8); pending input sets expire
after `input_timeout_s` (120 s). HTTP work happens off the ROS callback thread,
with one request in flight and `http_timeout_s` (30 s). A scan is processed once;
request a new scan/replan to retry a failed restoration.

Build with `colcon build --packages-select depth_restoration_ros`; the client
and synchronization tests run with `colcon test --packages-select
depth_restoration_ros`. Dependencies are ROS Jazzy `cv_bridge`, NumPy, OpenCV,
and the standard-library HTTP client. Services/tunnels must be reachable from
inside the container; host networking keeps localhost tunnel endpoints visible.
