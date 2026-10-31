#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
echo "== processes =="
for p in 'BeamNG.tech.x64 -nosteam' 'visual_navigation.wildos.nav' 'beamng_bridge' 'rviz2 -d' 'watchdog_nizhen.sh' 'viz_recorder.py'; do
  if pgrep -f "$p" >/dev/null 2>&1; then echo "ALIVE  $p"; else echo "DEAD   $p"; fi
done
echo "== rates =="
for t in /spot1/realsense/front/color/image_raw /spot1/model_visualization /spot1/nav_graph_viz /spot1/ouster/front/points_filtered; do
  echo -n "$t  -> "
  timeout 6 ros2 topic hz "$t" 2>&1 | grep -m1 'average rate' || echo "no data"
done
echo "== recorder =="
tail -n 2 /tmp/official_recorder.log 2>/dev/null
ls /home/ly/nizhen_viz_capture 2>/dev/null | wc -l
echo "== scored graph =="
python3 - <<'PY'
import json
try:
    d=json.load(open('/tmp/official_scored_health.json'))
    sg=d.get('details',{}).get('scored_graph',{})
    print('ok=%s scored_nodes=%s score_count=%s max=%s min=%s' % (
        d.get('ok'), sg.get('scored_nodes'), sg.get('score_count'), sg.get('score_max'), sg.get('score_min')))
except Exception as e:
    print('no scored json', e)
PY
