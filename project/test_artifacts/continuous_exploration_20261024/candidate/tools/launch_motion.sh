#!/usr/bin/env bash
set +u
pkill -9 -f 'run_motion_video.sh' 2>/dev/null || true
pkill -9 -f 'video_recorder.py' 2>/dev/null || true
pkill -9 -f 'watchdog_nizhen.sh' 2>/dev/null || true
sleep 2
DUR=240 SEED_DISTANCE=15.0 MOTION_SPEED=1.2 setsid bash /home/ly/run_motion_video.sh > /tmp/motion_run.log 2>&1 < /dev/null &
sleep 4
if pgrep -f 'run_motion_video.sh' >/dev/null 2>&1; then
  echo MOTION_LAUNCHED
  pgrep -af 'run_motion_video.sh' | grep -v pgrep
else
  echo MOTION_NOT_RUNNING
  tail -n 20 /tmp/motion_run.log
fi
