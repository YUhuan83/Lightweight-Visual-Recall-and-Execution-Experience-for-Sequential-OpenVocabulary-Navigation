#!/usr/bin/env bash
set +u
DUR=${DUR:-240}
TS=$(date +%Y%m%d_%H%M%S)
OUT=/home/ly/nizhen_runs/nizhen_run_$TS.mp4
mkdir -p /home/ly/nizhen_runs

pkill -9 -f 'watchdog_nizhen.sh' 2>/dev/null || true
pkill -9 -f 'video_recorder.py' 2>/dev/null || true

export MAP_MODE=nizhen_countryside
export BNG_USER=/home/ly/nizhen-user2
export BEAMNG_HEADLESS=1
export ENABLE_MOTION=1
export ENABLE_RVIZ=1
export ENABLE_RECORDER=0
export OBJECT_SEARCH=false
export READ_ONLY=0
export INSPECT_ONLY=0
export SEED_DISTANCE=${SEED_DISTANCE:-60.0}
export MOTION_SPEED=${MOTION_SPEED:-1.2}

echo "RUN_OUT=$OUT"
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
pkill -9 -f 'cruise_goal.py' 2>/dev/null || true
sleep 1
setsid nohup /home/ly/vision-cu128/bin/python /home/ly/cruise_goal.py --ros-args -p distance:=12.0 -p period:=3.0 > /tmp/cruise_goal.log 2>&1 < /dev/null &
echo "CRUISE_STARTED"
bash /home/ly/official_wildos_window_demo.sh
rc=$?
pkill -9 -f 'cruise_goal.py' 2>/dev/null || true
echo "DEMO_RC=$rc"
if test "$rc" != 0; then
  echo DEMO_FAILED
  exit 1
fi

setsid nohup /home/ly/vision-cu128/bin/python /home/ly/video_recorder.py \
  --out "$OUT" --fps 10 --seconds "$DUR" > /tmp/video_recorder.log 2>&1 < /dev/null &
echo "RECORDER_STARTED $OUT"

sleep $((DUR + 8))
bash /home/ly/wildos_server_stop.sh >/dev/null 2>&1 || true
pkill -9 -f 'BinLinux/BeamNG.tech.x64' 2>/dev/null || true
echo "DONE $OUT"
ls -la "$OUT" 2>/dev/null || echo NO_VIDEO
