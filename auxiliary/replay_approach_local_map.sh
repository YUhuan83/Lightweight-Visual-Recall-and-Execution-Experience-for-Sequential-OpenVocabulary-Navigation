#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v51_confirmed_action_handoff v52_approach_local_map; do
 (
 source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
 export ROS_DOMAIN_ID=214 ROS_LOCALHOST_ONLY=1
 for run in 1 2; do
  python3 "$base/audit_local_collision_prefix.py" "$base/v52_approach_local_map/replay_${version}_$run.json" >"$base/v52_approach_local_map/replay_${version}_$run.log" 2>&1
 done
 cmp "$base/v52_approach_local_map/replay_${version}_1.json" "$base/v52_approach_local_map/replay_${version}_2.json"
 )
done
python3 - <<'PY'
import json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v52_approach_local_map')
for v in ('v51_confirmed_action_handoff','v52_approach_local_map'):
 rows=json.loads((root/f'replay_{v}_2.json').read_text())['cases']
 assert len(rows)==2 and all(x['producer_accepts_proposal']==v.startswith('v51') for x in rows),(v,rows)
 print(v,json.dumps([dict(query_s=x['query_s'],producer_accepts=x['producer_accepts_proposal'],original_four_m_reason=x['original_four_m_reason'],local_blockers=x['proposal']['local_blockers']) for x in rows]))
PY
