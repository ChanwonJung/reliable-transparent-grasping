# Demos

Manual, hands-on demos for the robot hardware. Not ROS packages and not automated tests — each one is launched straight from this folder.

| Demo | What it shows | Hardware |
|---|---|---|
| [RealSense RGB-D viewer](#1-realsense-rgb-d-viewer) | D435i color, depth and point cloud in RViz | D435i |
| [Drag-to-follow arm](#2-drag-to-follow-arm-moveit-servo) | Drag a marker in RViz, the Gen3 Lite follows live | Gen3 Lite (or simulated) |
| [Live depth restoration](#3-live-depth-restoration-viewer) | Raw depth next to a Seraph model's restored depth, live | D435i + Seraph tunnels |

## Before any demo

One-time setup (see [`environments/ros2/README.md`](../environments/ros2/README.md)):

```bash
sudo apt install -y python3.12-venv ros-jazzy-behaviortree-cpp nlohmann-json3-dev ros-jazzy-moveit-servo
python3 -m venv --system-site-packages .venv && .venv/bin/pip install -r requirements/ros2_runtime.txt
cd ros2_ws && colcon build && cd ..
```

Every new terminal, from the repo root:

```bash
source environments/ros2/seraph_env.sh --no-tunnel
```

`--no-tunnel` skips the Seraph SSH tunnels — demos 1 and 2 don't use the remote models. Demo 3 does: source without the flag.

---

## 1. RealSense RGB-D viewer

Starts the D435i driver and opens RViz showing the color image, depth image and point cloud.

### Connect the camera (WSL2)

WSL2 has no native USB, so the camera is passed through with [`usbipd-win`](https://github.com/dorssel/usbipd-win). In an **admin PowerShell on Windows**, after every replug or Windows reboot:

```powershell
usbipd list                        # find the RealSense BUSID, e.g. 2-3
usbipd bind --busid <BUSID>        # first time only
usbipd attach --wsl --busid <BUSID>
```

Check from WSL: `lsusb | grep -i intel`.

### Run

```bash
ros2 launch demos/view_realsense_rgbd_launch.py                 # color + depth + point cloud
ros2 launch demos/view_realsense_rgbd_launch.py stream:=depth   # depth only
ros2 launch demos/view_realsense_rgbd_launch.py stream:=color   # color only
```

`stream:=depth` / `stream:=color` cut USB bandwidth — the usbip passthrough can't reliably carry all three streams at once.

### What to expect

- Topics under `/camera/camera/...` (color, depth, point cloud).
- Depth runs at ~5 Hz over WSL2 passthrough even though 15 fps is configured — known and accepted for now.
- Garbled or torn frames were traced to a bad USB cable once; try another cable first.

---

## 2. Drag-to-follow arm (MoveIt Servo)

RViz shows the Gen3 Lite with a drag handle on the gripper (`tool_frame`). Drag it and the arm follows live through MoveIt Servo, with Servo's singularity slowdown and collision checking.

### Run

**Simulated arm (default)** — nothing physical moves:

```bash
ros2 launch demos/gen3_lite_servo_drag_launch.py
```

**Real arm** — keep the E-stop within reach:

```bash
ros2 launch demos/gen3_lite_servo_drag_launch.py use_fake_hardware:=false robot_ip:=192.168.1.10 max_speed:=0.05
```

The first line of the launch output says which one is running: `*** FAKE HARDWARE ...` or `*** REAL ARM at ... ***`. If RViz shows the arm pointing straight up while the real arm is elsewhere, you launched the simulated one.

| Argument | Default | Meaning |
|---|---|---|
| `use_fake_hardware` | `true` | `false` = drive the real arm through the Kortex driver |
| `robot_ip` | `0.0.0.0` | Gen3 Lite IP (Kinova default `192.168.1.10`); ignored when simulated |
| `max_speed` | `0.10` | max tool speed toward the marker, m/s |
| `max_ang_speed` | `0.6` | max tool rotation speed toward the marker, rad/s |
| `rviz` | `true` | `false` = headless |

### Controls (in RViz)

- **Drag** the arrows (move along an axis), the rings (rotate about an axis), or the blue box (free move).
- The arm moves toward the marker at up to `max_speed` and stops when it gets there.
- **Right-click the marker:**
  - **Snap to gripper** — puts the marker back on the gripper, e.g. after dragging somewhere unreachable.
  - **Move to ready pose** — a planned `move_group` motion to a bent pose (tool pointing down, 30 cm in front of the base, 30 cm up), with Servo paused.

### Startup behaviour

- **Simulated:** the arm starts straight up — a singularity where Servo refuses to move — so the demo first moves it to the ready pose automatically.
- **Real:** the arm never moves on its own at startup. If it is near straight-up, use **Move to ready pose** before dragging.

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Arm doesn't move, RViz arm straight up | Simulated arm — relaunch with `use_fake_hardware:=false robot_ip:=...` |
| Arm stops, log says `Very close to a singularity, emergency stop` | Arm too stretched or folded. **Snap to gripper**, then **Move to ready pose** |
| Arm stops short of the marker | Target unreachable or would collide — **Snap to gripper** |
| Marker never appears | Servo or `move_group` didn't start — check the launch log for errors |

### Files

| File | Role |
|---|---|
| `gen3_lite_servo_drag_launch.py` | Starts `gen3_bringup` (MoveIt, controllers), Servo, the marker node, RViz |
| `gen3_lite_drag_marker.py` | Interactive marker → rate-limited Servo pose targets; right-click menu |
| `gen3_lite_servo.yaml` | Servo parameters (why positions-only output and Butterworth smoothing: see comments) |
| `gen3_lite_servo_drag.rviz` | RViz view: robot model + drag marker |

---

## 3. Live depth restoration viewer

Raw D435i depth on the left, a Seraph model's restored depth on the right. `--model` accepts a model name or its port:

| Port | Model | Needs |
|---|---|---|
| 30000 | `swindrnet` | colour + depth |
| 31000 | `remake` | colour + depth + **where the object is** |
| 32000 | FRAME (`frame`) | colour + depth + **where the object is** |

The services are plain HTTP (FastAPI): `GET /health`, `POST /restore_depth` with an `rgb` PNG and a 16-bit millimetre `depth` PNG. 31000 and 32000 also need a transparent-object mask, or SAM2.1 prompts so the server makes one — this viewer gives it prompts from your clicks.

### Run

Needs the Seraph tunnels (`source environments/ros2/seraph_env.sh`, **without** `--no-tunnel`) and the camera publishing colour + aligned depth. Source the ROS environment in the camera/viewer terminals:

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera align_depth.enable:=true enable_sync:=true
python3 demos/restore_depth_viewer.py --model FRAME        # also accepts 32000
```

No extra Python packages: it uses the apt ROS/OpenCV/numpy and the standard library, so the `.venv` is not needed.

**Prompting (31000 / 32000)** — click in either depth panel. Until the first click nothing is sent.

| Input | Does |
|---|---|
| left click | this pixel is the object (green dot) |
| right click | this pixel is background (red dot) |
| `c` / `u` | clear all points / undo the last one |
| `space` | capture and score the next restored frames (below) |
| `g` | capture the ground-truth depth (below) |
| `q` / Esc / Ctrl-C | quit |

The points go out with **every** frame and the server re-runs SAM2.1 each time. The services reply with the SAM2 score but not the mask, so the mask itself isn't drawn.

Check what the three services are, and what SAM2 prompting costs, without a camera:

```bash
python3 demos/restore_depth_viewer.py --probe
```

| Argument | Default | Meaning |
|---|---|---|
| `--model` | `30000` | model name (SwinDRNet, ReMake, FRAME) or its localhost port |
| `--crop` | `640x360` | window cut from each frame and sent to the model; centred unless `--crop-origin X Y` |
| `--range` | `0.25 1.0` | depth range of the colour scale, metres (same on both panels) |
| `--live-left` | off | left panel = live camera instead of the frame the right panel came from |
| `--confidence` / `--no-confidence` | on | the C / 1 - C row under the depth panels; C is the confidence in the raw depth. 30000 replies with `confidence`, 32000 with `pred_weight` (the weight on the predicted depth, shown as C = 1 - `pred_weight`), 31000 with neither, so its two panels stay empty |
| `--conf-key` | auto | reply image to show as C; default is the first named `confidence` / `conf` / `confidence_map` / `conf_map` / `pred_weight` (the last is inverted, also when you pass it here) |
| `--gt` | none | ground-truth depth for scoring: 16-bit PNG in mm, exactly the `--crop` size (alternative to pressing `g`) |
| `--capture-frames` | `5` | how many restored frames `space` captures and scores |
| `--fail-mm` | `20` | the *object* region is where the raw depth is missing or differs from the ground truth by more than this |
| `--out-dir` | `data/captures` | where captures go (git ignores `data/`) |
| `--timeout-ms` | `10000` | per-request timeout (the first request after idle takes ~2.5 s) |

### Scoring a capture

Restoration metrics need a **ground-truth depth**, which a live camera cannot give you, so you supply one — with the **camera fixed** between the two steps:

1. Put an **opaque stand-in** (same shape and place as the transparent object) in view and press `g`. The ground truth is the per-pixel median of 15 raw frames. Or pass `--gt depth.png` (16-bit mm, exactly the crop size) to use one you made elsewhere.
2. Swap in the **real transparent object**, click it (31000/32000) and press `space`. The next `--capture-frames` restored frames are scored and saved.

The footer shows progress and then where the capture went. The folder `data/captures/restore_<port>_<time>/` holds `report.html` (self-contained, open it in a browser), `metrics.json`, `gt_depth.png`, and per frame the rgb, the raw and restored depth as 16-bit mm PNGs, a side-by-side view with an error map, and the object-region mask.

Both the **restored** and the **raw sensor** depth are scored against the ground truth, in two regions: `object` (where the sensor failed — raw depth missing or off by more than `--fail-mm`) and `all` (every pixel with ground truth, which dilutes the object with the easy background). Metrics: RMSE and MAE in metres, AbsRel, and δ<1.05 / 1.10 / 1.25 (fraction of pixels whose prediction is within that ratio of the truth). Per-frame values are averaged, with the sample std. A pixel where the model returned 0 is left out of the errors and shows up in `coverage` instead. Exact definitions are in the report. With no ground truth the frames are still saved, just not scored.

If the camera moved after the ground truth was captured the numbers are meaningless; the report flags it when the median |raw − ground truth| is above 10 mm (`align mm`).

### What to expect

Measured 2026-10-02 with a simulated 30 Hz camera feeding the real services (640×360 crop):

| Model | Restored rate | Camera frame → restored image | Of which SAM2 prompting |
|---|---|---|---|
| 30000 | ~10 Hz | ~100 ms | — |
| 31000 | ~4.6 Hz | ~240 ms | ~85 ms |
| 32000 | ~5.5 Hz | ~200 ms | ~95 ms |

- Left and right are the **same frame** by default, so they stay comparable while the scene moves. The header on the right shows the restored rate, the server's time (`server`) and the time from the camera frame arriving to the restored image being ready (`frame to restored`).
- Only the newest frame is ever sent, one request at a time, so the restored rate is set by the server, never by a backlog. Camera frames in between are dropped.
- SAM2 re-prompting on every frame roughly doubles the latency of 31000/32000. If the camera is fixed and the object stays put, the mask could be made once per click and reused — that needs the service to return the mask it made.
- Depth is sent as measured: the viewer does not zero a cup region first or track the cup, as `graspgen_node` does with a SAM mask before calling SwinDRNet.
- The crop is a literal cut at native pixel scale: at 1280×720 it is the central quarter of the view, at 640×480 it drops 60 rows top and bottom.

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `nothing is listening` (probe) / `is the tunnel to :PORT open?` | Tunnel closed — `source environments/ros2/seraph_env.sh` and check the `✓` lines |
| `connection closed from the Seraph side` | The tunnel is up but the service on that port isn't running |
| Right panel: `click the transparent object ...` | 31000/32000 are waiting for a prompt — click the object in the left image |
| Right panel `server error` with `HTTP 400/422` | The service rejected the request; the message is the service's own |
| Footer `nothing restored yet` | `space` before the first reply — on 31000/32000 click the object first |
| Footer `saved ... WITHOUT metrics` | no ground truth yet — press `g` (or pass `--gt`) and capture again |
| Report warns about alignment | the camera moved since the ground truth; capture it again |
| `--gt: ... but --crop is ...` | the ground-truth PNG must be exactly the crop's size |
| Left panel `no colour + depth pair` | Driver not running, or started without `align_depth.enable:=true` |
| `camera frames rejected: ... smaller than the crop` | Camera stream is smaller than `--crop`; lower `--crop` or raise the stream size |
| C and 1 - C panels are empty | The reply has no confidence-named image — expected on 31000, which sends only `depth`. The terminal prints `no confidence map: ...` once, listing the images the reply does have; use `--conf-key NAME` if it is under another name |
