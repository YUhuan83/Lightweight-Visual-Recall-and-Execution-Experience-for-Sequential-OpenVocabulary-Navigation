#!/usr/bin/env bash
set +u
pkill -9 -f 'rviz2' 2>/dev/null || true
sleep 3
setsid bash /home/ly/restart_rviz.sh > /tmp/official_rviz.log 2>&1 < /dev/null &
sleep 12
if pgrep -f 'rviz2 -d' >/dev/null 2>&1; then
  echo RVIZ_ALIVE
  pgrep -af 'rviz2 -d'
else
  echo RVIZ_DEAD
  tail -n 15 /tmp/official_rviz.log
fi
