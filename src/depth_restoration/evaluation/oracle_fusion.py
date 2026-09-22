"""Contract for the first oracle upper-bound experiment.

This module intentionally does not define a tensor framework or metric implementation.
Its caller will provide arrays/tensors with identical spatial alignment and a metrics
adapter. The validity and tie-breaking policies must be recorded in the resolved config.
"""

from dataclasses import dataclass
from typing import Any, Mapping, Protocol


@dataclass(frozen=True)
class OracleFusionInputs:
    """Aligned depth inputs for one evaluation sample or batch.

    Required values must have compatible shape ``[..., H, W]`` and identical units.
    ``raw_valid_mask`` is explicit because zero/NaN/out-of-range policy is an open
    research decision. ``transparent_mask`` and ``baseline_fused_depth`` are optional
    but enable region-wise and baseline-fusion comparisons.
    """

    raw_depth: Any
    predicted_depth: Any
    ground_truth_depth: Any
    raw_valid_mask: Any
    baseline_fused_depth: Any | None = None
    transparent_mask: Any | None = None
    evaluation_mask: Any | None = None


class DepthMetrics(Protocol):
    """Adapter implemented later by the common metric module."""

    def __call__(self, depth: Any, ground_truth: Any, mask: Any) -> Mapping[str, float]:
        """Return at least MAE, RMSE, REL, and delta metrics for a masked region."""


@dataclass(frozen=True)
class OracleFusionReport:
    """Expected return object for reproducible oracle evaluation."""

    oracle_depth: Any
    selected_raw_mask: Any
    metrics_by_variant_and_region: Mapping[str, Mapping[str, Mapping[str, float]]]


def evaluate_oracle_fusion(
    inputs: OracleFusionInputs,
    metrics: DepthMetrics,
    *,
    tie_break: str = "predicted",
) -> OracleFusionReport:
    """Evaluate raw, predicted, original-fused, and per-pixel oracle depth.

    Oracle selection is defined only over valid evaluation pixels:
    choose raw depth when its absolute error to ground truth is lower than the predicted
    depth error; otherwise choose predicted depth. Invalid raw pixels must select the
    prediction. The exact treatment of invalid ground truth and equal errors is governed
    by the future implementation/configuration.
    """

    raise NotImplementedError("Implement after depth units and validity policy are frozen.")
