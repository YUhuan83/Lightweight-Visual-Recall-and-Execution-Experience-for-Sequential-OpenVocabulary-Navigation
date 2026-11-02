# Cleanroom ROS 2 workspace

This directory is the independent single-colcon workspace used for the WildOS + visual recall + BeamNG tests on 2026-11-02. It is separate from the historical `project/` snapshot and `candidates/v74_document_handoff/` above.

It contains the workspace source (`src/`), tests, launch/config files, vendor source dependencies and architecture notes. Build/install/log/run outputs, captured videos, model weights, and local credentials are intentionally excluded. RADIO/SigLIP checkpoint files must be provided locally as configured in `config/beamng_wildos.yaml`; this repository is source, not a model or BeamNG asset bundle.

The exact test scope and the incomplete field result are recorded in [`VALIDATION_20261102.md`](VALIDATION_20261102.md). Do not treat the first target's pass as a complete three-target mission pass.
