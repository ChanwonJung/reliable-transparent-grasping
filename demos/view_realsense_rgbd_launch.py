# Manual smoke test: launches the RealSense D435i driver (color + depth +
# pointcloud) and opens RViz pre-configured to show all three. Not a project
# ROS package — this targets realsense2_camera's raw default topics, before
# gen3_perception's topic/frame contracts (see ROADMAP Phase 7) exist.
#
# Usage:
#   ros2 launch demos/view_realsense_rgbd_launch.py
#   ros2 launch demos/view_realsense_rgbd_launch.py stream:=depth   # depth only
#   ros2 launch demos/view_realsense_rgbd_launch.py stream:=color   # color only
#
# stream:=rgbd (default) enables color + depth + pointcloud. depth/color
# disable pointcloud and the other image stream — useful for cutting
# bandwidth when running the camera over a WSL2 usbip passthrough, which
# can't reliably sustain all three streams at once.
#
# Adapted from realsense2_camera's bundled rs_pointcloud_launch.py example,
# pointed at this repo's own RViz config (demos/realsense_rgbd.rviz).

import os
import sys
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution, ThisLaunchFileDir
import launch_ros.actions

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch  # noqa: E402

STREAM_CHOICES = ('rgbd', 'depth', 'color')

local_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': 'camera', 'description': 'camera namespace'},
    {
        'name': 'stream',
        'default': 'rgbd',
        'description': "which data to pass through: 'rgbd' (color+depth+pointcloud), "
                        "'depth' (depth only), 'color' (color only). Overrides "
                        "enable_color/enable_depth/pointcloud.enable.",
    },
]


def set_configurable_parameters(local_params):
    return {param['name']: LaunchConfiguration(param['name']) for param in local_params}


def launch_setup(context, params):
    stream = LaunchConfiguration('stream').perform(context)
    if stream not in STREAM_CHOICES:
        raise RuntimeError(f"stream:='{stream}' is not one of {STREAM_CHOICES}")

    params = dict(params)
    params['enable_color'] = TextSubstitution(text='true' if stream in ('rgbd', 'color') else 'false')
    params['enable_depth'] = TextSubstitution(text='true' if stream in ('rgbd', 'depth') else 'false')
    params['pointcloud.enable'] = TextSubstitution(text='true' if stream == 'rgbd' else 'false')

    return rs_launch.launch_setup(context, params=params) + [
        launch_ros.actions.Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [ThisLaunchFileDir(), '/realsense_rgbd.rviz']],
        ),
    ]


def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters)
        + rs_launch.declare_configurable_parameters(params)
        + [OpaqueFunction(function=launch_setup, kwargs={'params': set_configurable_parameters(params)})]
    )
