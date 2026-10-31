#!/usr/bin/env bash
set +u
setsid bash /home/ly/start_mjpeg.sh > /tmp/mjpeg_stream.log 2>&1 < /dev/null &
sleep 8
if pgrep -f 'mjpeg_stream.py' >/dev/null 2>&1; then
  echo MJPEG_ALIVE
  pgrep -af 'mjpeg_stream.py' | grep -v pgrep
  timeout 5 curl -s -o /dev/null -w 'http_status=%{http_code}\n' http://127.0.0.1:8080/ || true
else
  echo MJPEG_DEAD
  tail -n 20 /tmp/mjpeg_stream.log
fi
