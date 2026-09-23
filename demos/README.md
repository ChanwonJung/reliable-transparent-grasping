# Demos

Manual, hands-on demos for the robot hardware. Not ROS packages and not automated tests — each one is launched straight from this folder.

| Demo | What it shows | Hardware |
|---|---|---|
| [RealSense RGB-D viewer](#1-realsense-rgb-d-viewer) | D435i color, depth and point cloud in RViz | D435i |
| [Drag-to-follow arm](#2-drag-to-follow-arm-moveit-servo) | Drag a marker in RViz, the Gen3 Lite follows live | Gen3 Lite (or simulated) |

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

`--no-tunnel` skips the Seraph SSH tunnels — neither demo uses the remote models.

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
