# Roadmap

Each phase has an explicit exit criterion. Open research choices stay as TODOs rather than becoming hidden defaults.

## Phase 1 — Environment & Baseline

- Pin the upstream SwinDRNet/DREDS revision in `external/UPSTREAMS.md`.
- Define dataset manifests and reproduce original preprocessing, loss, and fusion behavior.
- Save baseline config, checkpoint provenance, and depth metrics.
- Exit: an unmodified baseline evaluation is reproducible from one config.

## Phase 2 — Oracle Upper-Bound Analysis

- Implement the oracle evaluator contract.
- Compare predicted, original confidence-fused, raw, and oracle depth using MAE, RMSE, REL, and delta metrics.
- Report all-pixel, valid-raw, invalid-raw, transparent-mask, and non-mask regions.
- Exit: decide whether the oracle gap justifies reliability modeling.

## Phase 3 — Reliability Modeling

- Define and version candidate raw-depth reliability targets.
- Add an optional decoder without altering baseline behavior.
- Compare reliability-supervised fusion against the baseline.

## Phase 4 — Mask / Monocular Prior Integration

- Add instance-mask and monocular-relative-depth adapters as optional priors.
- Test input-concatenation and feature-level fusion independently.

## Phase 5 — Ablation & Depth Evaluation

- Run the baseline, reliability, mask, monocular, and combined configuration matrix.
- Produce quantitative tables and qualitative error/reliability visualizations.

## Phase 6 — Kinova Gen3 ROS2 Integration

- Validate connection, robot state, gripper, low-risk joint control, and MoveIt 2 planning.
- Keep project packages in `ros2_ws/src`; use the separately installed `ros2_kortex_ws` as an underlay.

## Phase 7 — RealSense D435i Integration

- Bring up RGB-D topics, camera intrinsics, synchronized frames, and camera-to-base TF calibration.

## Phase 8 — GraspGen Integration

- Wrap GraspGen inference, point-cloud conversion, grasp-frame conventions, and transform validation.

## Phase 9 — Real Robot Grasp Experiments

- Add collision checks, grasp execution guardrails, experiment logging, and recovery protocol.
- Compare raw-depth and restored-depth grasping under the same object and scene protocol.

## Phase 10 — KSC Paper Evaluation

- Freeze the method/configuration, aggregate ablations, select qualitative cases, and archive artifacts needed for the paper.
