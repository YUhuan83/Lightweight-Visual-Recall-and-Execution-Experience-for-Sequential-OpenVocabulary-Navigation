#!/usr/bin/env bash
# Bring up a scene that actually contains objects, then test ExploRFM queries.
set +u
D=/home/ly
WS=$D/beamng_ros2_0394_ws
SCEN=$WS/src/beamng-ros2-integration/beamng_ros2/config/scenarios/wildos_ar.json

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

echo "== 1. cleanup =="
for pat in beamng_bridge beamng_agent cmd_vel_to_control state_odom cloud_filter \
           planner_server controller_server lifecycle_manager path_planner_bridge \
           goal_adapter perception_node 'BinLinux/BeamNG.tech.x64'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 5

echo "== 2. ensure american_road mod in bridge user =="
mkdir -p "$D/.local/share/BeamNG/BeamNG.tech/current/mods"
if [ ! -f "$D/.local/share/BeamNG/BeamNG.tech/current/mods/american_road_0.39.zip" ]; then
  cp "$D/beamng-american-user/current/mods/american_road_0.39.zip" \
     "$D/.local/share/BeamNG/BeamNG.tech/current/mods/"
fi
ls -lh "$D/.local/share/BeamNG/BeamNG.tech/current/mods/american_road_0.39.zip"

echo "== 3. write american_road scenario =="
python3 - "$SCEN" <<'PY'
import json, sys
scene = {
    "version": 1.0,
    "level": "american_road",
    "name": "wildos_ar",
    "vehicles": [{
        "name": "ego",
        "model": "pickup",
        "color": "red",
        "pos": [3694.54, 2421.90, -419.70],
        "rot": [0.0, 0.0, 0.0, 1.0],
        "sensors": [
            {"name": "electrics", "type": "Electrics"},
            {"name": "front_cam", "type": "Camera.default",
             "pos": [-0.3, -2.1, 1.0], "resolution": [960, 640],
             "field_of_view_y": 60.0, "is_using_shared_memory": False,
             "is_streaming": False, "requested_update_time": 0.5},
            {"name": "front_lidar", "type": "Lidar.default",
             "pos": [0, 0, 1.7], "dir": [0, -1, 0], "up": [0, 0, 1],
             "is_360_mode": True, "vertical_resolution": 8, "density": 30,
             "max_distance": 40.0, "requested_update_time": 0.5,
             "is_using_shared_memory": False, "is_visualised": False},
        ],
    }],
}
json.dump(scene, open(sys.argv[1], "w", encoding="utf-8"), indent=2)
print("wrote", sys.argv[1])
PY

echo "== 3. xvfb + beamng =="
pgrep -f "Xvfb :99" >/dev/null || (DISPLAY=:99 nohup Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render >/tmp/xvfb99.log 2>&1 &)
sleep 2
cd "$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0" || exit 1
DISPLAY=:99 nohup ./BinLinux/BeamNG.tech.x64 -headless -tcom -tport 25252 -tcom-listen-ip 127.0.0.1 > /tmp/beamng.log 2>&1 &
for _ in $(seq 1 90); do ss -ltn 2>/dev/null | grep -q 25252 && break; sleep 1; done
ss -ltn 2>/dev/null | grep -q 25252 || { echo BEAMNG_PORT_FAIL; exit 1; }
echo BEAMNG_OK

echo "== 4. bridge =="
nohup ros2 run beamng_ros2 beamng_bridge --ros-args -p host:=127.0.0.1 -p port:=25252 -p launch:=false > /tmp/bridge.log 2>&1 &
sleep 8
grep -c 'Started beamng_bridge' /tmp/bridge.log

echo "== 5. load scenario =="
ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '$SCEN'}"
sleep 30

echo "== 6. sensors =="
python3 "$D/ros2_probe.py" 6

echo "== 7. frame dump =="
python3 "$D/ros2_frame_dump.py" /tmp/scene_frame.png

echo "== 8. perception queries =="
for q in building house tree road; do
  pkill -9 -f perception_node 2>/dev/null
  sleep 2
  nohup ros2 run wildos_ros2_adapter perception_node --ros-args \
    -p query:=$q \
    -p worker_python:=$D/vision-cu128/bin/python \
    -p worker_script:=$D/wildos-opt/vocabulary_query/vision_worker.py \
    -p checkpoint_root:=$D/ckpts \
    -p source_root:=$D/wildos-sim/wildos_port/repro_check/src/nebula2-wildos \
    -p output_dir:=$D/wildos-perception-runtime \
    > /tmp/perception_$q.log 2>&1 &
  sleep 25
  printf '%s -> ' "$q"
  grep -o '"mask_pixels": [0-9]*' /tmp/perception_$q.log | sort -u | tail -1
  pkill -9 -f perception_node 2>/dev/null
  sleep 1
done
echo SCENE_TEST_DONE
