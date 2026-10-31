#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
python3 "$base/audit_v54_forward_connectivity.py" >"$base/frontier_endpoint_contract_review/forward_connectivity.log" 2>&1
python3 - <<'PY'
import json
from pathlib import Path
j=json.loads(Path('/home/ly/execution_continuity_review_20261022/frontier_endpoint_contract_review/forward_connectivity.json').read_text())
print('current',j['current'],j['current_position'],'nodes/edges',j['nodes'],j['edges'])
for r in j['cases']:
 print(r['target'],r['position'],[(k,r[k]['reachable'],r[k]['reachable_count'],len(r[k]['path'] or [])) for k in ('raw_directed','finite_directed','finite_undirected')])
PY
