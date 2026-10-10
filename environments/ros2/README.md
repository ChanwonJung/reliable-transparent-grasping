# Robot environment

Host target: Ubuntu 24.04, ROS 2 Jazzy. The Kinova driver lives in the separately installed `~/workspace/ros2_kortex_ws`; this repo's packages are an overlay on top of it.

## One-time setup

```bash
sudo apt install -y python3.12-venv ros-jazzy-behaviortree-cpp nlohmann-json3-dev
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements/ros2_runtime.txt
```

`--system-site-packages` keeps rclpy, cv_bridge, numpy and cv2 on the system builds; the venv only adds the remote-model clients (ZMQ, msgpack, OpenAI, pydantic, scipy).

## Every shell

```bash
source environments/ros2/seraph_env.sh               # ROS + Kinova underlay + ros2_ws + .venv + Seraph tunnels
source environments/ros2/seraph_env.sh --no-tunnel   # same, without SSH (off-network)
```

Tunnels (aurora-g6 via `aurora.khu.ac.kr:30080`; override with `SERAPH_JUMP` / `SERAPH_HOST`):

| Port | Service |
|---|---|
| 10000 | Qwen3.5-27B (SGLang, OpenAI-compatible HTTP) |
| 20000 | SAM 2.1 (ZMQ) |
| 30000 | SwinDRNet (HTTP; legacy graspgen client uses ZMQ) |
| 31000 | ReMake depth completion (HTTP) |
| 32000 | FRAME depth restoration (HTTP) |
| 40000 | GraspGen (ZMQ) |

The same ports are the client defaults in `qwen_server`, `sam_server` and `graspgen_pkg` — change both together.
