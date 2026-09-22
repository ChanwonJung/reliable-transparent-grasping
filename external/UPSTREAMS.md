# Upstream dependencies

Clone or register upstream code without modifying it in place.

| Dependency | Intended location | Pin required before experiments |
| --- | --- | --- |
| SwinDRNet / DREDS | `external/SwinDRNet/` | repository URL, commit, license, setup instructions |
| GraspGen | `external/GraspGen/` | repository URL, commit, license, runtime interface |

Preferred options:

1. Git submodule pinned to a commit, if redistribution is acceptable.
2. A separately cloned repository with commit recorded in experiment metadata.

Adapters live outside these directories. Do not place modified upstream code in a commit without making the fork/patch provenance explicit.
