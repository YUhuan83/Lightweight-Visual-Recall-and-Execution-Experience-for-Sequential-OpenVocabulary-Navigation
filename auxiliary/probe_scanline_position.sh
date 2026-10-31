#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v46_approach_feedback/run_full.sh")
export PYTHONPATH="$base/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_scanline_position.py >v46_approach_feedback/scanline_position_replay.log 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_scanline_sampling.py dense >v46_approach_feedback/scanline_sampling_dense.log 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import json
from pathlib import Path
root=Path('v46_approach_feedback')
for name in ('scanline_position_replay.json','scanline_sampling_dense.json'):
 d=json.loads((root/name).read_text());r=d['rounds'][0]
 print(name,r['results'])
 for row in r['rows']:
  if row['result'] in ('lidar','multiview'): print(row['stamp_ns'],row['result'],row.get('position'))
PY
