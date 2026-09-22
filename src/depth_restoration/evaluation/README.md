# Oracle evaluation interface

Implement shared metrics in this package, then implement `evaluate_oracle_fusion` without importing model code. The evaluator accepts saved arrays/tensors from SwinDRNet inference and should emit metrics for:

- `raw_depth`
- `predicted_depth`
- `baseline_fused_depth` (when available)
- `oracle_depth`

Required region keys: `all`, `transparent_mask`, `valid_raw`, `invalid_raw`; add intersections only when their masking convention is recorded in configuration.
