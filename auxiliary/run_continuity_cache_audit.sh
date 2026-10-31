#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v30_confirmation_history/run_full.sh")
for version in v30_confirmation_history v28_intent_handoff; do
  root="$base/$version"
  trial=$(find "$root/full" -maxdepth 1 -type d -name 'trial_*' | sort | tail -1)
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_continuity_cache.py" "$trial"
done
