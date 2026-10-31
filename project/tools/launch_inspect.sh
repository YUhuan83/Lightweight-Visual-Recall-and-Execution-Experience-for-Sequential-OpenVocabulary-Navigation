#!/usr/bin/env bash
set +u
pkill -9 -f 'official_wildos_window_demo.sh' 2>/dev/null || true
pkill -9 -f 'run_motion_video.sh' 2>/dev/null || true
sleep 2
setsid bash /home/ly/launch_nizhen_inspect.sh > /tmp/nizhen_inspect3.log 2>&1 < /dev/null &
sleep 5
if pgrep -f 'official_wildos_window_demo.sh' >/dev/null 2>&1; then
  echo INSPECT_LAUNCHED
  pgrep -af 'official_wildos_window_demo.sh' | grep -v pgrep
else
  echo INSPECT_NOT_RUNNING
  tail -n 15 /tmp/nizhen_inspect3.log
fi
