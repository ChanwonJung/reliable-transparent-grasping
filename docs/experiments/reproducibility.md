# Experiment record contract

Every result directory must contain:

- resolved configuration (including defaults and overrides);
- Git commit of this repository and pinned upstream revisions;
- dataset name, split manifest, preprocessing version, and depth unit;
- checkpoint input/output paths and checksum when applicable;
- seed, hardware/runtime versions, and command line;
- metric table for required regions and paths to qualitative visualizations.

Results are ignored by Git by default. Commit small CSV/Markdown summary tables only after they are curated for a report or paper.
