# Cleanroom ROS 2 workspace

This directory is the independent single-colcon workspace for WildOS visual recall and BeamNG navigation. Architecture and data-flow contracts are described in `docs/`.

It contains the workspace source (`src/`), tests, launch/config files, vendor source dependencies and architecture notes. Build/install/log/run outputs, captured videos, model weights, and local credentials are intentionally excluded. RADIO/SigLIP checkpoint files must be provided locally as configured in `config/beamng_wildos.yaml`; this repository is source, not a model or BeamNG asset bundle.

The current validation scope and unresolved issues are recorded in [`VALIDATION.md`](VALIDATION.md). [`VALIDATION_20261102.md`](VALIDATION_20261102.md) preserves an earlier mission result. A source snapshot or passing component test does not constitute full mission acceptance.
