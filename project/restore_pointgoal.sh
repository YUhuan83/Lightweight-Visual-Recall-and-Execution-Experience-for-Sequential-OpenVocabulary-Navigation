#!/usr/bin/env bash
# Restore the American Road pickup scene (replacing the leftover calibration fixture), then dry-run pointgoal.
set -euo pipefail
PY="$HOME/venvs/beamng-039/bin/python"
cd "$HOME"

echo "=== restore American Road scene (spawns_road1) ==="
"$PY" -B american_road_smoke.py --replace-current --spawn spawns_road1 --warmup-seconds 10 \
  --output-root "$HOME/wildos-runs/american-smoke"

echo "=== dry-run pointgoal (no motion) ==="
cd "$HOME/pointgoal-test/pointgoal_kit"
"$PY" -B beamng_pointgoal.py --goal-forward 2 --goal-left 0 --output-root "$HOME/wildos-runs/pointgoal"

RUN="$(ls -1t "$HOME/wildos-runs/pointgoal" | head -1)"
echo "=== newest pointgoal run: $RUN ==="
ls -la "$HOME/wildos-runs/pointgoal/$RUN" || true
python3 - "$RUN" <<'PY'
import json, os, sys
run = sys.argv[1]
base = os.path.join(os.path.expanduser("~"), "wildos-runs", "pointgoal", run)
path = os.path.join(base, "report.json")
if os.path.exists(path):
    d = json.load(open(path))
    print("status:", d.get("status"))
    print("keys:", list(d))
else:
    print("no report.json; files:", os.listdir(base) if os.path.isdir(base) else "missing")
PY
