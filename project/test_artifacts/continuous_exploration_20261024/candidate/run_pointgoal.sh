#!/usr/bin/env bash
# Unzip pointgoal kit, run offline tests, then a no-motion dry-run plan.
set -euo pipefail
PY="$HOME/venvs/beamng-039/bin/python"
rm -rf "$HOME/pointgoal-test"
mkdir -p "$HOME/pointgoal-test"
unzip -q "$HOME/pointgoal_kit.zip" -d "$HOME/pointgoal-test"
cd "$HOME/pointgoal-test/pointgoal_kit"

echo "=== offline unit tests ==="
"$PY" -B test_beamng_pointgoal.py

echo "=== dry-run planning (no motion) ==="
"$PY" -B beamng_pointgoal.py --goal-forward 2 --goal-left 0 \
  --output-root "$HOME/wildos-runs/pointgoal"

RUN="$(ls -1t "$HOME/wildos-runs/pointgoal" | head -1)"
echo "=== newest run: $RUN ==="
ls -la "$HOME/wildos-runs/pointgoal/$RUN"
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
    print("no report.json; files:", os.listdir(base))
PY
