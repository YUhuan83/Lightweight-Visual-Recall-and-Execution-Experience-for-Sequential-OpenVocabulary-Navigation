#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v49_localization_range v50_confirmed_continuation; do
 (
  source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
  export ROS_DOMAIN_ID=214 ROS_LOCALHOST_ONLY=1
  for run in 1 2; do
   /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_confirmed_window.py" "$base/v50_confirmed_continuation/replay_${version}_$run.json" >"$base/v50_confirmed_continuation/replay_${version}_$run.log" 2>&1
  done
  cmp "$base/v50_confirmed_continuation/replay_${version}_1.json" "$base/v50_confirmed_continuation/replay_${version}_2.json"
 )
done
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v50_confirmed_continuation')
for name in ('v49_localization_range','v50_confirmed_continuation'):
 data=json.loads((root/f'replay_{name}_2.json').read_text())
 print(name,json.dumps(data['reports']))
 assert all(not r.get('object_found',False) for r in data['reports'])
 rows=[r for r in data['reports'] if r.get('route_updated')]
 assert len(rows)==(0 if name.startswith('v49') else 2),(name,rows)
 assert all(r['epoch_unchanged'] and r['accepted_action_retained'] for r in rows)
PY
