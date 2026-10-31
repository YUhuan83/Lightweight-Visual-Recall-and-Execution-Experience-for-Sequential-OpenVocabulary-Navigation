#!/usr/bin/env bash
set +u
bash /home/ly/wildos_server_stop.sh >/dev/null 2>&1 || true
pkill -9 -f 'BeamNG.tech.x64' 2>/dev/null || true
pkill -9 -f 'crashReporter' 2>/dev/null || true
pkill -9 -f 'rviz2' 2>/dev/null || true
sleep 4
cd /home/ly || exit 1
export MAP_MODE=nizhen_countryside
export BNG_USER=/home/ly/nizhen-user2
export BEAMNG_HEADLESS=1
export INSPECT_ONLY=1
export ENABLE_MOTION=0
export ENABLE_RVIZ=1
export ENABLE_RECORDER=1
export OBJECT_SEARCH=false
exec bash /home/ly/official_wildos_window_demo.sh
