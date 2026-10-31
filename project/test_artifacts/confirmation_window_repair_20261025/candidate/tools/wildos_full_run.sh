#!/usr/bin/env bash
# Full WildOS / BeamNG / ROS2 / Nav2 closed-loop bring-up and single-goal run.
# Safe defaults: limited speed, gear 2 (measured Drive), brake on timeout.
set +u
D=/home/ly
WS=$D/beamng_ros2_0394_ws
SCEN_SOURCE=$WS/src/beamng-ros2-integration/beamng_ros2/config/scenarios/example_tech_ground_lidar_wildos.json
SCEN=$(mktemp /tmp/wildos_scenario_XXXXXX.json)

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

echo "== 1. cleanup =="
for pat in beamng_bridge beamng_agent cmd_vel_to_control state_odom cloud_filter \
           planner_server controller_server lifecycle_manager path_planner_bridge \
           goal_adapter nav2_control.launch perception_node sensor_adapter \
           'ros2 topic pub' 'BinLinux/BeamNG.tech.x64'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 5
pgrep -af 'BeamNG.tech|beamng_bridge|beamng_agent|cloud_filter|planner_server' || echo CLEAN_OK

echo "== 2. lighter LiDAR scenario =="
python3 - "$SCEN_SOURCE" "$SCEN" <<'PY'
import json, sys
source_path, output_path = sys.argv[1:3]
data = json.load(open(source_path, encoding="utf-8"))
# Avoid collision with the level's built-in example_tech_ground prefab, which
# otherwise makes BeamNG reject our prefab and silently use default sensors.
data["name"] = "wildos_tech_ground"
for vehicle in data["vehicles"]:
    for sensor in vehicle.get("sensors", []):
        if sensor.get("name") == "front_cam":
            sensor.update({
                "resolution": [480, 320],
                "field_of_view_y": 60.0,
                "requested_update_time": 0.5,
                "is_using_shared_memory": False,
                "is_streaming": False,
            })
        if sensor.get("name") == "front_lidar":
            sensor.update({
                "is_360_mode": True,
                "vertical_resolution": 8,
                "density": 30,
                "max_distance": 30.0,
                "requested_update_time": 0.5,
                "is_using_shared_memory": False,
                "is_visualised": False,
            })
    vehicle["rot"] = [0.0, 0.0, 0.0, 1.0]
json.dump(data, open(output_path, "w", encoding="utf-8"), indent=2)
print("scenario lidar reduced ->", output_path)
PY

echo "== 3. Xvfb + BeamNG =="
pgrep -f "Xvfb :99" >/dev/null || (DISPLAY=:99 nohup Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX +render >/tmp/xvfb99.log 2>&1 &)
sleep 2
cd "$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0" || exit 1
DISPLAY=:99 nohup ./BinLinux/BeamNG.tech.x64 -headless -tcom -tport 25252 -tcom-listen-ip 127.0.0.1 > /tmp/beamng.log 2>&1 &
for _ in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q 25252 && break; sleep 1; done
ss -ltn 2>/dev/null | grep -q 25252 || { echo BEAMNG_PORT_FAIL; exit 1; }
echo BEAMNG_OK

echo "== 4. bridge =="
nohup ros2 run beamng_ros2 beamng_bridge --ros-args -p host:=127.0.0.1 -p port:=25252 -p launch:=false > /tmp/bridge.log 2>&1 &
sleep 8
grep -c 'Started beamng_bridge' /tmp/bridge.log

echo "== 5. scenario =="
ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '$SCEN'}"
sleep 15

echo "== 6. sensor check =="
python3 "$D/ros2_probe.py" 6

echo "== 7. state_odom =="
nohup ros2 run wildos_nav2_adapter state_odom --ros-args \
  -p state_topic:=/vehicles/ego/sensors/state -p odom_topic:=/wildos/odom \
  -p odom_frame:=odom -p base_frame:=base_link -p use_sim_truth:=true \
  > /tmp/state_odom.log 2>&1 &
sleep 4

echo "== 8. agent =="
nohup ros2 run beamng_agent beamng_agent --ros-args \
  -p host:=127.0.0.1 -p port:=25252 -p vehicle_id:=ego \
  -p driving_mode:=keyboard -p cmd_vel_topic:=/wildos/unused_cmd_vel \
  > /tmp/agent.log 2>&1 &
sleep 6
tail -n 3 /tmp/agent.log

echo "== 9. nav2 planner+controller =="
nohup ros2 launch wildos_nav2_adapter nav2_control.launch.py > /tmp/nav2_control.log 2>&1 &
sleep 12
# nav2_lifecycle_manager drops change_state responses on this loaded host, so
# activate the nodes directly with retries instead.
pkill -9 -f lifecycle_manager 2>/dev/null
sleep 1
python3 "$D/nav2_activate.py"

echo "== 10. bridges and limited controller =="
nohup ros2 run wildos_nav2_adapter path_planner_bridge --ros-args -p follow_path:=true > /tmp/plan_bridge.log 2>&1 &
nohup ros2 run wildos_nav2_adapter goal_adapter > /tmp/goal_adapter.log 2>&1 &
nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p cmd_vel_topic:=/wildos/cmd_vel -p control_topic:=/control \
  -p enable:=true -p max_speed:=0.0 -p max_steering:=0.6 \
  -p throttle_gain:=0.20 -p max_throttle:=0.35 -p min_throttle:=0.06 \
  -p throttle_integral_gain:=0.05 -p throttle_rise_rate:=0.3 \
  -p brake_gain:=0.5 \
  -p command_timeout:=0.5 -p gear_forward:=2 \
  -p overspeed_limit:=0.0 \
  -p steer_sign:=-1.0 \
  > /tmp/ctrl.log 2>&1 &
sleep 5
tail -n 2 /tmp/ctrl.log

echo "== 11. pose before =="
python3 "$D/ros2_probe.py" 4

echo "== 12. goal 10m ahead =="
ros2 topic pub --once /wildos/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -8.0, z: 0.0}, orientation: {w: 1.0}}}"
sleep 35

echo "== 13. pose after =="
python3 "$D/ros2_probe.py" 5

echo "== 14. logs =="
tail -n 6 /tmp/plan_bridge.log
tail -n 3 /tmp/ctrl.log
grep -c 'Failed to make progress' /tmp/nav2_control.log
echo ALL_DONE
