#!/usr/bin/env bash
# Re-run pointgoal after the start-assumption fix; report plan details without aborting.
set -uo pipefail
cd "$HOME/pointgoal-test/pointgoal_kit"
"$HOME/venvs/beamng-039/bin/python" -B beamng_pointgoal.py \
  --goal-forward 2 --goal-left 0 --output-root "$HOME/wildos-runs/pointgoal" || true

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
    if os.path.isdir(sub):
        m = os.path.join(sub, "map.json")
        p = os.path.join(sub, "plan.json")
        if os.path.exists(m):
            print(s, "map:", {k: v for k, v in json.load(open(m)).items() if k.endswith("cells")})
        if os.path.exists(p):
            det = json.load(open(p))
            print(s, "plan keys:", list(det), "| astar_cells:", len(det.get("astar_cells", [])), "| path:", det.get("path_local_xy"))
PY
