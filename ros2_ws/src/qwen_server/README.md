# qwen_server / sam_server — Slow Brain on the Seraph GPUs

Instruction + wrist RGB-D → `/world_map_result` + `/grasp_candidates` for `bt_pkg`. Ported from `robot_capstone` (`slow_brain/qwen_a100`, `slow_brain/sam_a100`, `mask_projection_pkg`, `graspgen_pkg`).

```
/user_instruction ─────────────┐
/camera/camera/color/image_raw ┴→ qwen_bridge_node (qwen_server) ──→ /qwen/grounding_result ──→ bt_pkg
                                    ├→ /qwen/labeled_detections ─┐
                                    └→ /qwen/source_image ───────┴→ sam_mask_node (sam_server) → /sam/mask_image
/camera/camera/aligned_depth_to_color/image_raw ─→ multi_view_projector_node → /world_map_result ─→ bt_pkg
                                                   graspgen_node (+ SwinDRNet) → /grasp_candidates ─→ bt_pkg
```

## Remote services

Tunnels are opened by [`environments/ros2/seraph_env.sh`](../../../environments/ros2/seraph_env.sh) (aurora-g6 via the `aurora.khu.ac.kr:30080` jump host; override with `SERAPH_JUMP` / `SERAPH_HOST`). The script only opens tunnels — **the ports below are also the code defaults**, change both together:

| Service | localhost port | Protocol | Client |
|---|---|---|---|
| Qwen3.5-27B (SGLang) | 10000 | HTTP, OpenAI-compatible `/v1` | `qwen_server/qwen_call.py` |
| SAM 2.1 | 20000 | ZMQ REQ/REP + msgpack | `sam_server/sam_client.py` |
| SwinDRNet | 30000 | ZMQ REQ/REP + msgpack | `graspgen_pkg/swindrnet_client.py` |
| GraspGen | 40000 | ZMQ REQ/REP + msgpack | `graspgen_pkg/zmq_client.py` |

## Bring-up

```bash
# once, from the repo root (needs: sudo apt install python3.12-venv)
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements/ros2_runtime.txt
```

```bash
# every shell — ROS + Kinova underlay + ros2_ws overlay + .venv + tunnels
source environments/ros2/seraph_env.sh            # --no-tunnel when off-network
```

```bash
# T1 — arm + MoveIt (TF base_link → camera_color_optical_frame comes from here)
ros2 launch gen3_bringup gen3_lite_moveit.launch.py robot_ip:=<ip>

# T2 — wrist camera, depth aligned to color
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# T3 — Qwen → SAM → projection
ros2 launch qwen_server slow_brain.launch.py

# T4 — grasps
ros2 launch graspgen_pkg graspgen.launch.py

# T5 — behavior tree
ros2 launch bt_pkg bt_system.launch.py
```

## Smoke tests (no ROS, tunnels up)

```bash
curl -s localhost:10000/v1/models              # model id must equal model_name (default qwen35-local)
python3 -m qwen_server.qwen_cli --image <rgb.png> --instruction "pick up the cup"
python3 -m sam_server.sam_cli --image <rgb.png> ...
python3 -m graspgen_pkg.graspgen_cli --host 127.0.0.1 --port 40000 --shape cube --num-grasps 100 --topk 10
```

## Port notes (from robot_capstone)

| Area | robot_capstone | here |
|---|---|---|
| Packages | `qwen_a100`, `sam_a100` | `qwen_server`, `sam_server` |
| Ports | 8000 / 5558 / 5557 / 5556 (A100) | 10000 / 20000 / 30000 / 40000 (Seraph) |
| Camera | Isaac EE + overhead (`/ee_rgbd_camera/*`, `/rgbd_camera/*`) | wrist D435i only (`/camera/camera/...`, aligned depth) |
| Views | dual by default | single by default — TARGET and DESTINATION both from the wrist frame |
| Camera pose | static `camera_extrinsics.yaml` (Isaac, one arm pose) | TF at each depth stamp (`extrinsics_source: tf`); `yaml` still available |
| Depth units | 32FC1 metres (Isaac) | 16UC1 mm decoded to metres (projector previously cast without scaling) |
| Frames | `panda_link0` / `world` | `base_link` |
| GraspGen IK filter | `panda_arm` / `panda_link8` | `arm` / `tool_frame` |
| Global defaults | `robot_defaults.yaml` via `ROBOT_CAPSTONE_ROOT` | not loaded — it would inject Panda/Isaac values |
| Output files | `robot_capstone/output/` | `experiments/logs/slow_brain/` (gitignored) |
| Environment | `launch_env_seraph.sh` + `gsam_venv` | `environments/ros2/seraph_env.sh` + repo `.venv` (`requirements/ros2_runtime.txt`) |

## Provisional / unverified

- `model_name: qwen35-local` was the A100 vLLM id; SGLang on Seraph may serve a different id — check `curl localhost:10000/v1/models`.
- GraspGen runs `graspgen_franka_panda.yml` — Franka-hand grasps used as a proxy for the Gen3 Lite 2F (fingertips ~0.11 m apart open vs Franka 0.08 m).
- `tcp_offset` (GraspGen origin → `tool_frame`): 0.0 assumes GraspGen returns the grasp centre; if it returns the Franka hand base it should be ~-0.10. Check with `graspgen_cli --shape cube` (positions inside the cube vs ~0.1 m outside it).
- `ik_tip_to_fingertip_m` 0.0 / `jaw_axis_yaw_deg` 0.0 — from URDF geometry (jaws close along `tool_frame` X), not measured.
- Camera pose accuracy = the placeholder wrist mount in `gen3_bringup` until it is measured/calibrated.
- `marker_publisher.py` still draws a Franka gripper.
