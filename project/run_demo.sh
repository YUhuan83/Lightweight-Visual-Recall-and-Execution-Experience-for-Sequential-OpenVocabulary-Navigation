#!/usr/bin/env bash
# WildOS-lite complete demo (headless): text goal -> nodes -> plan -> drive, on American Road.
set -uo pipefail
export DISPLAY=:99
pgrep -f "Xvfb :99" >/dev/null 2>&1 || (nohup Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render >/tmp/xvfb99.log 2>&1 &)
sleep 1

D=/home/ly
VQ=$D/vision_query
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
QUERY="${QUERY:-stop sign}"
SPAWN="${SPAWN:-spawns_road1}"
CYCLES="${CYCLES:-30}"

echo "== stopping any running BeamNG =="
pkill -f 'BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64' 2>/dev/null
sleep 4

cd "$VQ"
echo "== WildOS-lite demo: query='$QUERY' spawn=$SPAWN cycles=$CYCLES =="
"$D/venvs/beamng-039/bin/python" -B run_demo.py \
  --home "$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0" \
  --user "$D/beamng-american-user" \
  --map-zip "$D/beamng-maps/american_road_0.39.zip" \
  --spawn "$SPAWN" --query "$QUERY" --cycles "$CYCLES" \
  --source-root "$SRC" --checkpoint-root "$D/ckpts" \
  --query-python "$D/vision-cu128/bin/python" \
  --output "$D/wildos-lite-out" --replace-current --gfx vk

echo "== newest demo report =="
NEW=$(ls -dt "$D"/wildos-lite-out/*/ 2>/dev/null | head -n1)
echo "$NEW"
cat "$NEW/demo_report.json" 2>/dev/null | python3 -c "
import json,sys
r=json.load(sys.stdin)
print('query     =', r['query'])
print('cycles    =', r['cycles'])
print('final     =', r['final_phase'], r['final_target_xy'])
print('trajectory=', [(round(l['car_xy'][0],1), round(l['car_xy'][1],1), l['phase']) for l in r['log']])
"
echo DONE
