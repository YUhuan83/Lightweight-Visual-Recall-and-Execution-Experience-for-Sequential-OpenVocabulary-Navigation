#!/usr/bin/env bash
# Diagnose one captured frame's region projection. Usage: bash diag.sh [RUN_ID] [FRAME]
set -euo pipefail
RUN="${1:-a221141e}"
FRAME="${2:-frame_000}"
cd ~/vision_query
~/vision-cu128/bin/python -m integration.diagnose \
  --frames-dir "$HOME/capture_out/$RUN/frames" \
  --frame "$FRAME" \
  --query-result "$HOME/vision_query/vocabulary_query/integration_${RUN}_${FRAME}/result.json" \
  --calibration "$HOME/e2e_out/calibration.json"
