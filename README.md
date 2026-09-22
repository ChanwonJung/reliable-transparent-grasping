# Reliability-Aware Transparent Object Depth Restoration

Research workspace for KSC 2026: restoring transparent-object depth and evaluating the effect on Kinova Gen3 grasping.

## Scope

This repository intentionally separates two systems:

1. **Offline depth-restoration research** — SwinDRNet-compatible baselines, reliability modeling, datasets, metrics, and reproducible experiments.
2. **Online robot integration** — ROS 2 Jazzy wrappers that consume a restored-depth result and pass a grasp pose to Kinova Gen3. It does not contain the Kinova driver itself.

The existing Kinova installation stays at `~/workspace/ros2_kortex_ws`. This repository owns only project-specific ROS 2 packages and adapters.

## Research question

Can an explicit estimate of *raw-depth reliability*, guided by transparent-object masks and monocular relative-depth priors, improve over SwinDRNet's implicitly learned confidence fusion?

`confidence` and `reliability` are deliberately distinct:

- **Confidence**: the original SwinDRNet fusion weight, trained through final-depth reconstruction loss.
- **Reliability**: a proposed explicit estimate that the measured raw sensor depth is trustworthy at a pixel.

The proposed formulation is not fixed. All choices about target generation, masks, priors, fusion, and loss weights remain configuration options until validated.

## Layout

```text
configs/                 Experiment overlays; no architecture edits per ablation
docs/                    Research assumptions, architecture contracts, experiment protocol
external/                Pointers to immutable upstream SwinDRNet and GraspGen clones/submodules
src/                     Python research code; independent of ROS 2
ros2_ws/src/             Project ROS 2 adapters; independent of training code
scripts/                 Thin train / evaluate / inference entrypoints
experiments/             Ignored outputs, tracked documentation only
requirements/            Dependency manifests split by purpose
environments/            Environment setup notes
data/                    Ignored datasets and dataset manifests
```

## First planned analyses

1. Reproduce the unmodified SwinDRNet baseline.
2. Run the **oracle upper-bound** evaluation: choose raw or predicted depth per pixel using ground truth, then compare it with predicted and original confidence-fused depth.
3. Only pursue an explicit reliability branch if that oracle gap is meaningful under the chosen metrics and regions.

The oracle interface is defined in [`src/depth_restoration/evaluation/oracle_fusion.py`](src/depth_restoration/evaluation/oracle_fusion.py). It is a contract, not an implementation yet.

## Configuration convention

Each file in `configs/` is an experiment overlay with `extends: base`. A future lightweight loader will resolve the overlay explicitly; no configuration framework is required at this stage.

- `swindrnet_baseline.yaml`: original behavior only.
- `reliability*.yaml`: proposed branches and priors, kept separate from the baseline.

Record the resolved config, upstream commit, dataset split, checkpoint path, and metrics with every result; see [`docs/experiments/reproducibility.md`](docs/experiments/reproducibility.md).

## Status

Scaffold only. No SwinDRNet architecture, loss, reliability target, calibration method, or robot behavior has been committed as a research decision.

See [`ROADMAP.md`](ROADMAP.md) for the ordered plan.
