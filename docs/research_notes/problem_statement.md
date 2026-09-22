# Problem statement and non-decisions

## Goal

Transparent objects can contain a mix of valid depth, refracted/background depth, and missing measurements. The goal is to restore depth in a way that improves downstream grasping, while preserving valid raw measurements when appropriate.

## Baseline contract

The baseline is original SwinDRNet/DREDS behavior:

- RGB and raw depth inputs.
- Separate RGB/depth Swin encoders and cross-modal fusion.
- Depth prediction and original confidence-fusion behavior.
- Transparent masks are not network inputs in the baseline.

Any reproduction must keep upstream preprocessing, architecture, loss, and fusion intact. Changes belong in a separately named module/configuration.

## Proposed direction

The candidate reliability branch receives RGB-D features and optional instance-mask / monocular-relative-depth priors, and predicts raw-depth reliability. A candidate fusion is `R * raw_depth + (1 - R) * predicted_depth`, but neither this equation nor the definition of `R` is fixed.

## Open decisions (do not silently choose)

- Reliability target: relative-error threshold, delta threshold, continuous mapping, or another formulation.
- Validity convention for zero, NaN, inf, saturated, and out-of-range depth.
- Whether reliability is supervised in all regions or only selected regions.
- Fusion behavior outside transparent masks.
- Mask source at train/evaluation/robot runtime: GT, predicted instance mask, or both.
- Monocular model, scale alignment, and uncertainty treatment.
- Depth unit, camera intrinsics, crop/resolution, and benchmark split.
