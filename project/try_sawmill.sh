#!/usr/bin/env bash
# Reload the scene at spawns_sawmill and re-run the pointgoal dry-run.
set -euo pipefail
PY="$HOME/venvs/beamng-039/bin/python"
cd "$HOME"

echo "=== load American Road at spawns_sawmill ==="
"$PY" -B american_road_smoke.py --replace-current --spawn spawns_sawmill --warmup-seconds 10 \
  --output-root "$HOME/wildos-runs/american-smoke"

echo "=== dry-run pointgoal (no motion) ==="
cd "$HOME/pointgoal-test/pointgoal_kit"
"$PY" -B beamng_pointgoal.py --goal-forward 2 --goal-left 0 --output-root "$HOME/wildos-runs/pointgoal"

RUN="$(ls -1t "$HOME/wildos-runs/pointgoal" | head -1)"
python3 - "$RUN" <<'PY'
import json, os, sys
run = sys.argv[1]
base = os.path.join(os.path.expanduser("~"), "wildos-runs", "pointgoal", run)
d = json.load(open(os.path.join(base, "report.json")))
print("status:", d.get("status"))
print("up:", d.get("initial_state", {}).get("up"))
print("pos:", d.get("initial_state", {}).get("pos"))
print("error:", d.get("error"))
print("files:", sorted(os.listdir(base)))
PY
