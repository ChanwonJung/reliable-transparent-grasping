# ROS 2 dependencies

ROS 2 Jazzy and `ros2_kortex` are installed in the separate system workspace. ROS package dependencies are declared in each package's `package.xml`.

Python packages the perception nodes need but rosdep cannot provide go in [`ros2_runtime.txt`](ros2_runtime.txt), installed into the repo `.venv` — see [`environments/ros2/README.md`](../environments/ros2/README.md).
