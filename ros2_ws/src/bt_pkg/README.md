# bt_pkg — BehaviorTree.ROS2 Pick-and-Place Executor

Behavior tree that turns perception output (target/destination geometry + grasp candidates) into MoveIt motions and gripper commands on the **Kinova Gen3 Lite**.

Ported from `robot_capstone/ros_pkgs/src/bt_pkg` (Isaac Sim + Franka Panda + MoveIt hybrid planning). See [Port notes](#port-notes-from-robot_capstone) for what changed.

---

## Dependencies

| Library | Source |
|---|---|
| `behaviortree_cpp` (BT.CPP v4) | apt: `ros-jazzy-behaviortree-cpp` |
| `behaviortree_ros2` | not on apt for Jazzy — pinned clone in `ros2_ws/src/external/BehaviorTree.ROS2` (see `external/UPSTREAMS.md`) |
| `nlohmann_json` | apt: `nlohmann-json3-dev` |
| MoveIt / Kinova driver | `~/workspace/ros2_kortex_ws` underlay |

```bash
sudo apt install -y ros-jazzy-behaviortree-cpp nlohmann-json3-dev
```

## Build

```bash
bash -lc '
  source /opt/ros/jazzy/setup.bash
  source ~/workspace/ros2_kortex_ws/install/setup.bash
  cd ~/NOVA_system/reliable-transparent-grasping/ros2_ws
  colcon build --packages-up-to bt_pkg
'
```

`--packages-up-to` skips `btcpp_ros2_samples` from the BehaviorTree.ROS2 checkout.

## Launch

```bash
# terminal 1 — MoveGroup + controllers
ros2 launch gen3_bringup gen3_lite_moveit.launch.py robot_ip:=0.0.0.0 use_fake_hardware:=true

# terminal 2 — BT (starts ticking after a 5 s delay)
ros2 launch bt_pkg bt_system.launch.py use_fake_hardware:=true
```

`use_fake_hardware:=true` sets `gripper_require_contact:=false`: fake fingers always close fully, which would otherwise read as a missed grasp every time.

Launch args: `tree_file`, `pre_grasp_z_offset` (default 0.03), `use_fake_hardware`.

---

## Interfaces

| Direction | Topic / action | Type |
|---|---|---|
| sub | `/world_map_result` (transient-local) | `std_msgs/String` (JSON) |
| sub | `/grasp_candidates` (transient-local) | `std_msgs/String` (JSON) |
| sub | `/qwen/grounding_result` (transient-local) | `std_msgs/String` (JSON) |
| sub | `/joint_states` | `sensor_msgs/JointState` |
| sub | `/yolo/world_map`, `/yolo/target_centroid` | optional — no publisher in this repo |
| pub | `/bt/replan_request` | `std_msgs/Empty` |
| action client | `/move_action` | `moveit_msgs/action/MoveGroup` |
| action client | `/gen3_lite_2f_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |

Grasp candidate poses are for **`tool_frame`** in **`base_link`** (candidate `frame` field defaults to `base_link`). `tool_frame` is the `arm` group's IK tip, 13 cm past `end_effector_link`, with +Z along the approach axis — the same convention `retract_along_approach` assumed for `panda_link8`.

---

## How it works

### Shared state: `SceneData`

`bt_executor_node` owns all subscriptions and writes into a mutex-guarded `SceneData`, injected into every BT node at construction. Nodes lock `scene->mtx` in `tick()` to read it. Computed poses go on the BT blackboard.

### Tree (`behavior_trees/pick_and_place.xml`)

```
KeepRunningUntilFailure
  Inverter
    Sequence "Main Pipeline"
      WaitForScene              ← RUNNING until fresh /world_map_result + /grasp_candidates
      ParseScene                ← blackboard keys, resets grasp_index, latches observation pose
      Fallback "Pick or Recover"
        ReactiveSequence
          AlwaysSuccess         ← TargetVisible stub (needs YOLO, not ported)
          RetryUntilSuccessful {bt_pick_retries}
            Sequence "Pick Attempt"
              SelectGraspCandidate
              GripperAction open
              MoveAction pre_grasp_pose
              MoveAction grasp_pose
              GripperAction close   ← SUCCESS only on stall (contact)
              MoveAction retreat_pose
        Sequence "Pick Recovery"
          MoveToHome
          RequestReplan         ← FAILURE on purpose → restart from WaitForScene
      Fallback "Place If Grounded"
        Sequence: HasDestinationCentroid → UpdateTargetPose → Retry(3){pre_place, place, open, post_place}
        AlwaysSuccess
      Sequence "Return To Observation"
        MoveToObservation (failure tolerated)
        GripperAction close (failure tolerated)
```

A completed pipeline returns SUCCESS → Inverter → FAILURE → `KeepRunningUntilFailure` exits; the next tick starts a new cycle. Root FAILURE is logged as "BT cycle complete", not as an error.

### `MoveAction` / `MoveToJointGoal`

Both send a single `MoveGroup` goal (`plan_only = false`, plan + execute) to `/move_action`:

- `start_state.is_diff = true` — move_group plans from its own monitored state.
- `MoveAction`: position (sphere) + orientation goal constraint on `tool_frame`, optional path constraints via ports: `lock_orientation`, `lock_base_joints` (`base_joint_names`), `min_path_z`.
- `MoveToJointGoal` is registered as `MoveToHome` (`home_joint_values`) and `MoveToObservation` (latched from the first scan unless `observation_joint_values` is set).

### `GripperAction`

Served natively by the kortex `GripperActionController` (`allow_stalling: true`). Positions are `right_finger_bottom_joint` radians: open `0.0`, closed `0.85`.

- close → `SUCCESS` if `result.stalled` (object in the jaws), `FAILURE` if it closed fully (missed) — unless `gripper_require_contact: false`.
- open → always `SUCCESS`.

---

## Port notes (from robot_capstone)

| Area | robot_capstone | here |
|---|---|---|
| Motion action | `/run_hybrid_planning` (`HybridPlanner`, `MotionSequenceRequest`) | `/move_action` (`MoveGroup`, single `MotionPlanRequest`) |
| Start state | copied from cached `/joint_states` | `is_diff = true` |
| Group / tip / frame | `panda_arm` / `panda_link8` / `panda_link0` | `arm` / `tool_frame` / `base_link` |
| Gripper | `/gripper_command`, width in m (open 0.08, closed 0.0) | `/gen3_lite_2f_gripper_controller/gripper_cmd`, rad (open 0.0, closed 0.85) |
| Hazard / E-stop | `EmergencyStopClear` + `hazard_level_translator_node` | removed (physical Kinova E-stop only) |
| YOLO world map | `yolo_world_map_node.py` | not ported; `TargetVisible` stays stubbed |
| Config | `robot_defaults.yaml` + `bt_params.yaml` | `bt_params.yaml` only |

### Provisional values — to measure on the real arm

- `home_joint_values` — placeholder all-zeros (arm straight up).
- `max_place_reach_m` 0.65 / `region_max_offset_m` 0.35 — scaled down from Panda values for the 0.76 m Gen3 Lite reach, not measured.
- `min_path_z="0.15"` on `pre_grasp_pose` — Isaac table geometry; depends on the real table height in `base_link`.
- `lock_base_joints="false"` on `pre_grasp_pose` (was `true` on the 7-DOF Panda) — a 6-DOF arm has little slack for a ±0.3 rad lock over a large move. Re-enable if twisted IK branches show up.
- `pre_grasp_z_offset` 0.03 — Isaac-validated, not yet on the Gen3 Lite.
- The observation-pose latch assumes static world extrinsics (the Isaac design). With a wrist camera calibrated as a TF (`gen3_perception`, planned), scans are valid from any pose; revisit whether the latch is still needed then.
