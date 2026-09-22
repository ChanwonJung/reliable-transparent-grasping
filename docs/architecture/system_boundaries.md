# Architecture boundaries

```text
Offline research
RGB + raw depth + optional priors
  -> depth_restoration (model / evaluation)
  -> checkpoint + restored depth artifact

Online robot system
RealSense ROS 2 topics
  -> depth_restoration_ros (inference adapter)
  -> point cloud / graspgen_ros
  -> grasp pose in camera frame
  -> TF to base frame
  -> gen3_grasp_execution
```

## Dependency rule

- `src/` must run without ROS 2 imports.
- `ros2_ws/src/` must not import training datasets or model internals. It calls a versioned inference boundary (process, service, or exported runtime artifact).
- `external/` holds unmodified upstream repositories. Local changes are adapters in `src/` or `ros2_ws/src/`, never edits mixed into upstream source.
- `~/workspace/ros2_kortex_ws` is an external ROS 2 underlay, not tracked here.

## ROS 2 package responsibilities

| Package | Responsibility |
| --- | --- |
| `gen3_bringup` | Project launch composition, Kinova driver configuration references; no driver fork. |
| `gen3_perception` | RealSense topic contracts, synchronization, camera metadata, TF validation. |
| `depth_restoration_ros` | RGB-D input/output messages and inference-runtime adapter. |
| `graspgen_ros` | Restored depth / point cloud to GraspGen request and grasp candidates. |
| `gen3_grasp_execution` | Transform checks, MoveIt/Kinova execution boundary, safety checks and logging. |

No ROS 2 package is created as buildable code yet: message types, frame conventions, and runtime deployment decisions remain open.
