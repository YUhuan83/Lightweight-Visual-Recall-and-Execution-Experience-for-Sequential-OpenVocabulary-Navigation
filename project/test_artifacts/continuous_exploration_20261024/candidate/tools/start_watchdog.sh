#!/usr/bin/env bash
set +u
pkill -9 -f 'watchdog_nizhen.sh' 2>/dev/null || true
sleep 1
setsid bash /home/ly/watchdog_nizhen.sh > /tmp/watchdog_nizhen.log 2>&1 < /dev/null &
sleep 3
if pgrep -f 'watchdog_nizhen.sh' >/dev/null 2>&1; then
  echo WATCHDOG_ALIVE
  pgrep -af 'watchdog_nizhen.sh' | grep -v pgrep
else
  echo WATCHDOG_DEAD
fi
