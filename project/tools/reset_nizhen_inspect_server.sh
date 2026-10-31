#!/usr/bin/env bash
set +u
timeout 12 bash /home/ly/wildos_server_stop.sh >/tmp/wildos_reset_stop.log 2>&1 || true
for p in \
  'official_wildos_window_demo.sh' 'launch_nizhen_inspect.sh' 'beamng_bridge' \
  'BeamNG.tech.x64' 'visual_navigation.wildos.nav' \
  'graphnav_mapper' 'graphnav_planner' 'beamng_wildos_adapter' \
  'obj_mask_triangulation' 'rviz2' 'viz_recorder.py' 'mjpeg_stream.py'; do
  pkill -9 -f "$p" 2>/dev/null || true
done
sleep 4
export MAP_MODE=nizhen_countryside
export BNG_USER=/home/ly/nizhen-user2
export BEAMNG_HEADLESS=1
export INSPECT_ONLY=1
export ENABLE_MOTION=0
export ENABLE_RVIZ=0
export ENABLE_RECORDER=0
export OBJECT_SEARCH=false
setsid bash /home/ly/official_wildos_window_demo.sh >/tmp/nizhen_round_runtime.log 2>&1 < /dev/null &
