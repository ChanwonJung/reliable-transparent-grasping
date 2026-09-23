# Temp handoff notes (WSL restart) — 2026-09-22

Delete this file once you're back up and don't need it anymore — it's a
session handoff, not project documentation.

## State as of shutdown

**Uncommitted changes** (nothing committed today — `git status --short`):
```
 M .gitignore
 M src/depth_restoration/evaluation/oracle_fusion.py   (ruff --fix: typing.Mapping -> collections.abc.Mapping)
?? requirements/dev.txt        (ruff pinned)
?? ruff.toml
?? tests/                      (see below)
?? ros2_ws/src/gen3_bringup/   (see below)
```
`CLAUDE.local.md` also exists (gitignored, not shown above — personal notes, untouched).

**External workspaces built and working, not part of this repo's git:**
- `~/workspace/ros2_kortex_ws` — Kinova's `ros2_kortex` (jazzy branch) + deps.
  Built clean: 69/69 packages, zero failures (steering-controller packages
  removed from `ros2_kortex_ws` are gone from the workspace on purpose — see
  "Gotchas" below, don't reinstall them).
- RealSense driver: `ros-jazzy-realsense2-camera` etc. installed via apt (not
  a workspace, just system packages).

## What's done

1. **Kinova driver built** — `~/workspace/ros2_kortex_ws`, real Gen3 Lite +
   Robotiq/Kinova gripper support, MoveIt configs included.
2. **RealSense D435i working over WSL2 USB passthrough** — `usbipd-win`
   attach/bind confirmed working. Frame corruption issue traced to a bad
   cable, fixed. Depth topic still runs under target fps under WSL2 passthrough
   (~5Hz vs 15fps configured) — accepted as good-enough for now, revisit if it
   becomes a real blocker.
3. **`tests/realsense_rgbd.rviz` + `tests/view_realsense_rgbd_launch.py`** —
   manual RGB-D viewer (not a project package), `stream:=rgbd|depth|color` arg
   added later to manage bandwidth.
4. **`ros2_ws/src/gen3_bringup/`** — new package, first real content in this
   repo's `ros2_ws`:
   - `urdf/gen3_lite_d435i.xacro` — vendor Gen3 Lite + gripper xacro, unmodified,
     plus a wrist-mounted D435i (bland box geometry, `use_mesh:=false`) added
     as a sibling off `end_effector_link`. **Mount offset is a placeholder**
     (`xyz="0 0.05 0.04" rpy="0 0 0"`), not measured against the real arm.
   - `srdf/gen3_lite_d435i.srdf.xacro` — vendor SRDF + `disable_collisions`
     entries for `camera_link` vs `gripper_base_link` / `left_finger_prox_link`
     / `right_finger_prox_link` / `upper_wrist_link` / `end_effector_link`.
     Needed because the placeholder mount overlaps the gripper by <1mm at the
     home pose, which blocked all planning (`START_STATE_IN_COLLISION`) until
     this was added.
   - `launch/gen3_lite_moveit.launch.py` — real hardware MoveGroup bringup,
     adapted from `kinova_gen3_lite_moveit_config/launch/robot.launch.py`
     (Apache-2.0, PickNik). Only change from upstream: robot_description +
     robot_description_semantic point at this package's D435i-equipped
     xacro/srdf instead of the vendor's bare ones. Plain MoveGroup only — no
     hybrid planning, no hazard/BT code (scope was explicitly cut per your
     call: "fast brain" obstacle avoidance isn't needed for this project).
5. **Gripper action confirmed working with zero new code** —
   `gen3_lite_2f_gripper_controller` (`position_controllers/GripperActionController`,
   `allow_stalling: true`) already serves `control_msgs/action/GripperCommand`
   natively at `/gen3_lite_2f_gripper_controller/gripper_cmd`. Verified
   open/close against fake hardware, correct `stalled`/`reached_goal` results.
6. **`tests/gen3_lite_movement_gripper_test.py`** — end-to-end smoke test:
   TF-based outward Cartesian step (5cm default, robust to current pose) via
   `/move_action`, then close/open via the real gripper action. Confirmed
   working end-to-end against fake hardware (`use_fake_hardware:=true`).
   Prompts for typed `go` confirmation before moving the real arm (`--yes`
   to skip).

## Robot_capstone migration context (for later steps)

Source project: `~/NOVA_system/robot_capstone` (Isaac Sim + Franka Panda,
being ported to real Gen3 Lite). Full analysis is earlier in this
conversation's history — key facts if that context isn't available next
session:
- Real Gen3 Lite names: planning group `arm` (not `panda_arm`), gripper group
  `gripper`, base `base_link`, IK tip `tool_frame` (not `end_effector_link` —
  `tool_frame` is 13cm further out, see `gen3_lite_2f_macro.xacro`
  `tool_frame_joint`).
- Gripper: 1 actuated joint (`right_finger_bottom_joint`, revolute, radians),
  not 2 independent prismatic joints like Franka. Open=0.0 rad, Closed≈0.85
  rad (inverted vs Franka's open/closed sense).
- Dropped from migration scope (user's call): hybrid planning, hazard
  avoidance / "fast brain" (`hazard_collision_injector_node`,
  `hazard_level_translator_node`, `yolo_world_map_node`, `yolo_hazard_pkg`,
  `hybrid_pose_client_node`, all Isaac-only bridge nodes).
- Still portable, not yet migrated: `grounded_sam_pkg`, `qwen_pkg` /
  `slow_brain/qwen_a100`, `mask_projection_pkg`, `graspgen_pkg`, `bt_pkg`
  (needs its `MoveAction` BT leaf re-pointed from `/run_hybrid_planning` to
  plain `/move_action`, modeled on its existing `MoveToHome` leaf — same
  action type, not new code).
- Single-camera decision made: EE-only (robot_capstone's dual EE+top design
  assumed two cameras; we have one D435i).
- `bt_pkg`'s `GripperAction` leaf will need a remap from `/gripper_command`
  to `/gen3_lite_2f_gripper_controller/gripper_cmd` when ported (same action
  type, just the name).

## Gotchas / things not to redo

- **Don't reinstall `ros-jazzy-steering-controllers-library`** (or
  `ros-jazzy-ros2-controllers`, `ros-jazzy-ackermann/bicycle/tricycle-steering-controller`,
  `ros-jazzy-gz-ros2-control-demos`). These were deliberately removed —
  the system binary conflicted with the source-built copy in
  `ros2_kortex_ws`, causing `START_STATE_IN_COLLISION`-unrelated build
  failures (`parameter_traits/parameter_traits.hpp: No such file`) for
  `ackermann_steering_controller` and friends. None of them are needed for
  the Gen3 Lite arm.
- **XML comments**: xacro/URDF/SRDF comments can't contain `--` anywhere
  except the closing `-->`. Hit this twice already (own xacro + own srdf).
- **This is WSL2**: no native USB passthrough. D435i needs `usbipd-win` on
  the Windows side (`usbipd list` / `bind` / `attach --wsl`) re-run after
  every replug or Windows reboot. `rosdep`/`apt` commands needing sudo can't
  be run by the assistant directly in this sandbox — always handed to you as
  copy-paste commands.
- `bash -lc '...'` is required (not plain `source ... && ...`) when sourcing
  ROS setup files in this shell — the default shell here doesn't set
  `BASH_SOURCE`, which ROS's setup scripts need for path resolution.

## Suggested next steps, in order

1. Get a real physical D435i mount offset measurement and update
   `gen3_bringup/urdf/gen3_lite_d435i.xacro`'s placeholder `<origin>`.
2. Try the RViz `MotionPlanning` panel interactively (drag the marker →
   Plan → Execute) against fake hardware first, then real hardware — this
   was the in-progress ask when the session paused. `gen3_lite_moveit.launch.py`
   already launches RViz with everything wired up for this; likely no new
   code needed, just verifying it live. If you actually want continuous
   real-time jogging instead of discrete plan/execute, that's MoveIt Servo —
   a separate integration, not yet started.
3. Port `bt_pkg` (behavior tree executor) — re-point its `MoveAction` leaf
   to plain `/move_action`, drop the `/run_hybrid_planning` action client.
4. Port `qwen_a100` / `mask_projection_pkg` / `graspgen_pkg` with camera
   topics remapped to the real driver's `/camera/camera/...` names
   (single EE view only).
5. Real calibration: `gen3_perception` package (still just a README) needs
   to own the actual measured camera-to-robot TF once the mount is final.

## How to relaunch after WSL comes back

```bash
# terminal 1 — MoveGroup bringup (fake hardware to sanity check first)
bash -lc '
  source /opt/ros/jazzy/setup.bash
  source ~/workspace/ros2_kortex_ws/install/setup.bash
  source ~/NOVA_system/reliable-transparent-grasping/ros2_ws/install/setup.bash
  ros2 launch gen3_bringup gen3_lite_moveit.launch.py robot_ip:=0.0.0.0 use_fake_hardware:=true
'

# terminal 2 — movement + gripper smoke test
bash -lc '
  source /opt/ros/jazzy/setup.bash
  source ~/workspace/ros2_kortex_ws/install/setup.bash
  source ~/NOVA_system/reliable-transparent-grasping/ros2_ws/install/setup.bash
  python3 tests/gen3_lite_movement_gripper_test.py
'
```

For the RealSense camera, remember `usbipd attach --wsl --busid <id>` on the
Windows side first (see Gotchas above).
