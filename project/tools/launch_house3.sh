#!/usr/bin/env bash
set +u
pkill -9 -f 'run_house3_video.sh' 2>/dev/null || true
sleep 2
DUR=75 setsid bash /home/ly/run_house3_video.sh > /tmp/house3_run.log 2>&1 < /dev/null &
sleep 5
pgrep -af 'run_house3_video.sh' | grep -v pgrep || echo NOT_RUNNING
