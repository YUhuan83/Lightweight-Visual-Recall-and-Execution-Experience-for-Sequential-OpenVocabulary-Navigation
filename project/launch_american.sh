#!/usr/bin/env bash
# Restart BeamNG with the American Road userpath, load the pickup scene, then dry-run pointgoal.
set -euo pipefail
PY="$HOME/venvs/beamng-039/bin/python"
HOME_DIR="/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0"
USERPATH="/home/ly/beamng-american-user"

export DISPLAY=:99
pgrep -a Xvfb >/dev/null || (Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render & sleep 2)

echo "=== stop any running BeamNG ==="
pkill -f "BeamNG.tech.x64" || true
sleep 3

echo "=== launch BeamNG with American Road userpath ==="
"$PY" - <<PY
from beamngpy import BeamNGpy
bng = BeamNGpy('127.0.0.1', 64256, home='$HOME_DIR', user='$USERPATH',
               binary='BinLinux/BeamNG.tech.x64', quit_on_close=False, gfx='vk')
bng.open(launch=True, listen_ip='127.0.0.1')
print('launched; leaving BeamNG running')
PY
sleep 5

echo "=== load American Road scene (spawns_road1) ==="
cd "$HOME"
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
