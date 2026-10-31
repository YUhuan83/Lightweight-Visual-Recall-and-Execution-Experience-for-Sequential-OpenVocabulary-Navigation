#!/usr/bin/env bash
set +u
pkill -9 -f 'run_cruise_only.sh' 2>/dev/null || true
sleep 2
DUR=70 DIST=115.0 SPEED=3.0 setsid bash /home/ly/run_cruise_only.sh > /tmp/cruise_run.log 2>&1 < /dev/null &
sleep 5
pgrep -af 'run_cruise_only.sh' | grep -v pgrep || echo NOT_RUNNING
