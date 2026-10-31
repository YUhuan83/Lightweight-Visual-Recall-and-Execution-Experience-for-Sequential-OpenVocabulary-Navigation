#!/usr/bin/env bash
# Re-run pointgoal dry-run (2m) after the LiDAR near-field mount change.
set -euo pipefail
cd "$HOME/pointgoal-test/pointgoal_kit"
"$HOME/venvs/beamng-039/bin/python" -B beamng_pointgoal.py \
  --goal-forward 2 --goal-left 0 --output-root "$HOME/wildos-runs/pointgoal"

RUN="$(ls -1t "$HOME/wildos-runs/pointgoal" | head -1)"
echo "=== newest run: $RUN ==="
python3 - "$RUN" <<'PY'
import json, os, sys
run = sys.argv[1]
base = os.path.join(os.path.expanduser("~"), "wildos-runs", "pointgoal", run)
d = json.load(open(os.path.join(base, "report.json")))
print("status:", d.get("status"))
print("error:", d.get("error"))
for s in sorted(os.listdir(base)):
    sub = os.path.join(base, s)
    if os.path.isdir(sub) and os.path.exists(os.path.join(sub, "map.json")):
        print(s, json.load(open(os.path.join(sub, "map.json"))))
PY
