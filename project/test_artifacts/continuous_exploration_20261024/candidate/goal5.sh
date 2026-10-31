#!/usr/bin/env bash
# Re-run pointgoal dry-run with the goal beyond the LiDAR near blind zone.
set -euo pipefail
cd "$HOME/pointgoal-test/pointgoal_kit"
"$HOME/venvs/beamng-039/bin/python" -B beamng_pointgoal.py \
  --goal-forward 5 --goal-left 0 --output-root "$HOME/wildos-runs/pointgoal"

RUN="$(ls -1t "$HOME/wildos-runs/pointgoal" | head -1)"
echo "=== newest run: $RUN ==="
python3 - "$RUN" <<'PY'
import json, os, sys
run = sys.argv[1]
base = os.path.join(os.path.expanduser("~"), "wildos-runs", "pointgoal", run)
d = json.load(open(os.path.join(base, "report.json")))
print("status:", d.get("status"))
print("error:", d.get("error"))
print("goal_world:", d.get("goal_world"))
print("files:", sorted(os.listdir(base)))
PY
