#!/usr/bin/env bash
set +u
pkill -9 -f 'run_cruise_video.sh' 2>/dev/null || true
sleep 2
DUR=70 DIST=115.0 SPEED=3.0 setsid bash /home/ly/run_cruise_video.sh > /tmp/cruise_run.log 2>&1 < /dev/null &
sleep 5
if pgrep -f 'run_cruise_video.sh' >/dev/null 2>&1; then
  echo CRUISE_LAUNCHED
  pgrep -af 'run_cruise_video.sh' | grep -v pgrep
else
  echo CRUISE_NOT_RUNNING
  tail -n 20 /tmp/cruise_run.log
fi
