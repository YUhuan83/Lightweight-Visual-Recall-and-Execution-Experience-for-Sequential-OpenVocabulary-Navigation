#!/usr/bin/env bash
# Read-only material diagnosis of the American Road map archive.
set -euo pipefail
"$HOME/venvs/beamng-039/bin/python" -B "$HOME/diagnose_road1_materials.py" \
  --archive "$HOME/beamng-maps/american_road_0.39.zip"
