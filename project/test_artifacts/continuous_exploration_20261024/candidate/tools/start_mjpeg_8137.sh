#!/usr/bin/env bash
set +u
echo "== listening ports =="
ss -ltn 2>/dev/null | grep -E ':8080|:8137|:8099|:8199' || echo none_of_those
echo "== start mjpeg on 8137 =="
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
pkill -9 -f 'mjpeg_stream.py' 2>/dev/null || true
sleep 1
setsid nohup /home/ly/vision-cu128/bin/python /home/ly/mjpeg_stream.py --host 127.0.0.1 --port 8137 > /tmp/mjpeg_stream.log 2>&1 < /dev/null &
sleep 8
if pgrep -f 'mjpeg_stream.py' >/dev/null 2>&1; then
  echo MJPEG_ALIVE
  timeout 5 curl -s -o /dev/null -w 'root_http=%{http_code}\n' http://127.0.0.1:8137/ || true
else
  echo MJPEG_DEAD
  tail -n 20 /tmp/mjpeg_stream.log
fi
