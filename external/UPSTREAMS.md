# Upstream dependencies

Clone or register upstream code without modifying it in place.

| Dependency | Intended location | Pin required before experiments |
| --- | --- | --- |
| SwinDRNet / DREDS | `external/SwinDRNet/` | repository URL, commit, license, setup instructions |
| GraspGen | `external/GraspGen/` | repository URL, commit, license, runtime interface |
| BehaviorTree.ROS2 | `ros2_ws/src/external/BehaviorTree.ROS2/` | https://github.com/BehaviorTree/BehaviorTree.ROS2, commit `6c6aa078ee7bc52fec98984bed4964556abf5beb` (humble branch, 2025-11-25), Apache-2.0. Not on apt for Jazzy; needed by `bt_pkg`. `behaviortree_cpp` itself comes from apt (`ros-jazzy-behaviortree-cpp`). |

Preferred options:

1. Git submodule pinned to a commit, if redistribution is acceptable.
2. A separately cloned repository with commit recorded in experiment metadata.

Adapters live outside these directories. Do not place modified upstream code in a commit without making the fork/patch provenance explicit.

### BehaviorTree.ROS2 checkout

```bash
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git ros2_ws/src/external/BehaviorTree.ROS2
git -C ros2_ws/src/external/BehaviorTree.ROS2 checkout 6c6aa078ee7bc52fec98984bed4964556abf5beb
```
