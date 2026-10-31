#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
echo "=== bridge packages ==="
for p in foxglove_bridge rosbridge_server rosbridge_suite rviz2 webviz; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "OK   $p -> $(ros2 pkg prefix $p)"; else echo "MISS $p"; fi
done
echo "=== python bridge modules ==="
python3 - <<'PY'
for m in ("foxglove_bridge","rosbridge_server","roslibpy"):
    try:
        __import__(m); print("OK  ",m)
    except Exception as e:
        print("MISS",m,type(e).__name__)
PY
echo "=== rviz marker/viz topics referenced in wildos ==="
grep -rn -E "_topic:|topic:" /home/ly/beamng_ros2_0394_ws/src/beamng_wildos_adapter/config/beamng_wildos.yaml 2>/dev/null
echo "=== display ==="
echo "DISPLAY=$DISPLAY"
who 2>/dev/null | head
ls /tmp/.X11-unix 2>/dev/null
