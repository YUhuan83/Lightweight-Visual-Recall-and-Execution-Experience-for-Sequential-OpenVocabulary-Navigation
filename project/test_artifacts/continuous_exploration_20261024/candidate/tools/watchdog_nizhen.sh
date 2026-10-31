#!/usr/bin/env bash
# Supervises the headless nizhen_countryside WildOS inspection run.
# When BeamNG exits (known renderer crash on this map), it restarts the chain.
set +u
LOG=/tmp/nizhen_inspect2.log
while true; do
  if ! pgrep -f 'BeamNG.tech.x64 -nosteam' >/dev/null 2>&1; then
    echo "$(date '+%F %T') watchdog: BeamNG not running, restarting pipeline" >> "$LOG"
    pkill -9 -f 'official_wildos_window_demo.sh' 2>/dev/null || true
    pkill -9 -f 'launch_nizhen_inspect.sh' 2>/dev/null || true
    sleep 2
    setsid bash /home/ly/launch_nizhen_inspect.sh >> "$LOG" 2>&1 < /dev/null &
  fi
  sleep 15
done
