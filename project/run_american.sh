#!/usr/bin/env bash
# American Road basic WildOS flow (headless): capture -> text query -> 3D target. No heads.
set -uo pipefail
export DISPLAY=:99
pgrep -f "Xvfb :99" >/dev/null 2>&1 || (nohup Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render >/tmp/xvfb99.log 2>&1 &)
sleep 1

D=/home/ly
VQ=$D/vision_query
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
QUERY="${QUERY:-stop sign}"
SPAWN="${SPAWN:-spawns_road1}"
POSES="${POSES:--4,0,4}"
MAP_ZIP="$D/beamng-maps/american_road_0.39.zip"
GAME_HOME="$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0"
GAME_USER="$D/beamng-american-user"
PY="$D/vision-cu128/bin/python"

echo "== stopping any running BeamNG =="
pkill -f 'BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64' 2>/dev/null
sleep 4

cd "$VQ"
MARK=$(mktemp /tmp/am_mark.XXXXXX)
echo "== American Road capture (spawn=$SPAWN, query='$QUERY') =="
"$D/venvs/beamng-039/bin/python" -B integration/capture_american.py \
  --home "$GAME_HOME" --user "$GAME_USER" --map-zip "$MAP_ZIP" \
  --spawn "$SPAWN" --poses "$POSES" --replace-current --gfx vk \
  --output "$D/capture_out"
CAPRC=$?
if [ "$CAPRC" -ne 0 ]; then echo "capture failed rc=$CAPRC"; exit 1; fi

RUN_DIR=$(find "$D/capture_out" -maxdepth 1 -type d -newer "$MARK" 2>/dev/null | sort | tail -n1)
RUN_ID=$(basename "$RUN_DIR")
echo "RUN_ID=$RUN_ID"

echo "== calibration (nominal K, 960x640 FOV60) =="
"$PY" -B -c "
import sys; sys.path.insert(0, '$VQ')
from integration import make_calibration
c = make_calibration.build('American Road nominal K (FOV 60, 960x640)', None, time_source='beamng_host_monotonic_paused')
make_calibration.write('$D/american_calibration.json', c)
print('calibration', c.get('status'))
"

echo "== text query per frame ('$QUERY') =="
for d in "$RUN_DIR"/frames/frame_*; do
  f=$(basename "$d")
  out="$VQ/vocabulary_query/integration_${RUN_ID}_${f}"
  rm -rf "$out"
  "$PY" -B "$VQ/vocabulary_query/query.py" \
    --source-root "$SRC" --checkpoint-root "$D/ckpts" \
    --image "$d/image.png" --output "$out" --query "$QUERY" > "$d/query.log" 2>&1
  if [ -f "$out/result.json" ]; then
    python3 -c "import json;r=json.load(open('$out/result.json'));print('  $f regions=',[(x['box_xyxy'],round(x['mean_cosine'],3)) for x in r.get('regions',[])])"
  else
    echo "  $f query FAILED (see $d/query.log)"
  fi
done

echo "== 3D target (multi-view triangulation) =="
"$PY" -m object_search.run_object_search \
  --frames-dir "$RUN_DIR/frames" \
  --vocab-dir "$VQ/vocabulary_query" \
  --epoch "$RUN_ID" --prefix query_000 \
  --calibration "$D/american_calibration.json" \
  --output "$D/object_search_out"

echo "== result =="
python3 -c "
import json
t=json.load(open('/home/ly/object_search_out/object_search.json'))
print('query =', '$QUERY')
print('target_world_xyz =', t.get('triangulated_position'))
print('per_frame_lidar_lock =', [p.get('lidar_lock') for p in t.get('per_frame',[])])
"
echo "DONE RUN_ID=$RUN_ID"
