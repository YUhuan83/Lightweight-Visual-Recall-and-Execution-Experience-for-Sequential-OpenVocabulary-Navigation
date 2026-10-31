#!/usr/bin/env bash
# Headless live pipeline: mounted capture -> per-frame heads -> query/localize -> triangulate.
set -uo pipefail
export DISPLAY=:99
pgrep -f "Xvfb :99" >/dev/null 2>&1 || (nohup Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render >/tmp/xvfb99.log 2>&1 &)
sleep 1
D=/home/ly
VQ=$D/vision_query
BASE=$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0
USERP=$D/.local/share/BeamNG/BeamNG.tech
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
POSES="${POSES:--4,1.5;0,1.0;4,1.5}"

echo "== stopping any running BeamNG =="
pkill -f 'BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64' 2>/dev/null
sleep 4

cd "$VQ"
MARK=$(mktemp /tmp/live_mark.XXXXXX)
echo "== headless mounted capture, poses=$POSES =="
"$D/venvs/beamng-039/bin/python" -B integration/live_capture.py \
  --home "$BASE" --user "$USERP" --replace-current \
  --poses="$POSES" --gfx vk --output "$D/capture_out"
CAPRC=$?
echo "capture rc=$CAPRC"
if [ "$CAPRC" -ne 0 ]; then
  echo "capture failed"; exit 1
fi

RUN_DIR=$(find "$D/capture_out" -maxdepth 1 -type d -newer "$MARK" 2>/dev/null | sort | tail -n1)
RUN_ID=$(basename "$RUN_DIR")
echo "RUN_ID=$RUN_ID"

echo "== ExploRFM traversability/frontier per frame =="
for d in "$RUN_DIR"/frames/frame_*; do
  f=$(basename "$d")
  out="$VQ/vocabulary_query/live_heads/${RUN_ID}_${f}"
  rm -rf "$out"
  "$D/vision-cu128/bin/python" -B "$VQ/vocabulary_query/explorfm_heads.py" \
    --image "$d/image.png" --checkpoint-root "$D/ckpts" --source-root "$SRC" \
    --output "$out" --heatmap relative --smooth > "$d/heads.log" 2>&1
  python3 -c "import json;r=json.load(open('$out/result.json'));print('  $f trav',r['traversability_range'],'frontier',r['frontier_range'])" 2>/dev/null || echo "  $f heads FAILED"
done

echo "== e2e query + strict localization =="
rm -rf "$VQ/vocabulary_query/integration_${RUN_ID}_frame_"*
"$D/vision-cu128/bin/python" -m integration.run_e2e \
  --frames-dir "$RUN_DIR/frames" \
  --query "black white checkerboard" \
  --source-root "$SRC" \
  --checkpoint-root "$D/ckpts" \
  --query-script "$VQ/vocabulary_query/query.py" \
  --query-python "$D/vision-cu128/bin/python" \
  --accept-calibration "fixture nominal K (FOV 60deg); api_fit |f| within 0.5% of nominal 554.256; static visual reprojection < 0.4px" \
  --max-extent-m 8 --max-depth-span-m 3.5 \
  --max-ring-same-depth-fraction 0.99 --alpha-silhouette 0.01 \
  --ttl-ns 30000000000 \
  --output "$D/e2e_out"

echo "== multi-view triangulation =="
"$D/vision-cu128/bin/python" -m object_search.run_object_search \
  --frames-dir "$RUN_DIR/frames" \
  --vocab-dir "$VQ/vocabulary_query" \
  --epoch "$RUN_ID" --prefix query_000 \
  --calibration "$D/e2e_out/calibration.json" \
  --hypothesis-xyz 0 -10 2.35 --distribution gaussian_hyp --hypothesis-std 1.0 \
  --output "$D/object_search_out"

echo "== summary =="
python3 -c "
import json
r=json.load(open('/home/ly/e2e_out/integration_report.json'))
print('loc:', json.dumps(r['stages']['localization']))
for f in r['frames']:
    h=f['handoff']; o=h.get('observation')
    print(f['frame'], h.get('state'), h.get('confirmation_count'))
    if o and o.get('candidates'): print('   pos=', o['candidates'][0]['position_m'])
t=json.load(open('/home/ly/object_search_out/object_search.json'))
print('triangulated=', t.get('triangulated_position'))
print('truth checkerboard = (0, -9.999, 2.35)')
"
echo "DONE RUN_ID=$RUN_ID"
