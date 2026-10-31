#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v43_abort_recovery v45_approach_contract; do
  (
    source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
    export PYTHONPATH="$base/$version/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
    cd "$base"
    for run in 1 2; do
      /home/ly/beamng_ros2_0394_ws/.venv/bin/python audit_mask_surface_alignment.py "$version" >"$version/mask_surface_alignment_$run.log" 2>&1
      cp "$version/mask_surface_alignment.json" "$version/mask_surface_alignment_$run.json"
    done
    cmp "$version/mask_surface_alignment_1.json" "$version/mask_surface_alignment_2.json"
    cat "$version/mask_surface_alignment_2.log"
  )
done
