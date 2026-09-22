# Project ROS 2 workspace

This is an overlay workspace for project-specific adapters. It is intentionally separate from `~/workspace/ros2_kortex_ws`, which provides the official Kinova driver.

Future build order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
cd ~/reliable-transparent-grasping/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Do not add build, install, or log directories to Git. Package skeletons below are documentation-only until their topic, frame, and safety contracts are agreed.
