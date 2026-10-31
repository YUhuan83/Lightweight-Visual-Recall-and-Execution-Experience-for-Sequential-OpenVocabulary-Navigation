#!/usr/bin/env bash
# Visible American Road WildOS/ROS2/Nav2 demo. The vehicle is enabled only
# after every upstream health gate succeeds.
set +u
D=/home/ly
WS=$D/beamng_ros2_0394_ws
BNG=$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0
SCEN=$WS/src/beamng-ros2-integration/beamng_ros2/config/scenarios/wildos_american_window.json
DISPLAY_ID=:3
AUTH=/run/user/1011/gdm/Xauthority
CONTROL_ENABLED=0

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
export DISPLAY=$DISPLAY_ID
export XAUTHORITY=$AUTH

stop_on_failure() {
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "WINDOW_DEMO_FAILED rc=$rc"
    tail -n 20 /tmp/wildos_window_bridge.log 2>/dev/null
    tail -n 20 /tmp/wildos_window_nav2.log 2>/dev/null
    tail -n 20 /tmp/wildos_window_perception.log 2>/dev/null
    if [ -x "$D/wildos_server_stop.sh" ]; then
      bash "$D/wildos_server_stop.sh"
    fi
  fi
}
trap stop_on_failure EXIT

wait_port() {
  for _ in $(seq 1 90); do
    ss -ltn 2>/dev/null | grep -q ':25252 ' && return 0
    sleep 1
  done
  return 1
}

wait_log() {
  file=$1
  pattern=$2
  attempts=$3
  for _ in $(seq 1 "$attempts"); do
    grep -q "$pattern" "$file" 2>/dev/null && return 0
    sleep 1
  done
  return 1
}

echo "== 1. preflight =="
test -x "$BNG/BinLinux/BeamNG.tech.x64" || { echo BEAMNG_MISSING; exit 1; }
test -f "$AUTH" || { echo XAUTHORITY_MISSING; exit 1; }
DISPLAY=$DISPLAY_ID XAUTHORITY=$AUTH xdpyinfo >/dev/null 2>&1 || { echo DISPLAY_UNAVAILABLE; exit 1; }
test -f "$D/beamng-maps/american_road_0.39.zip" || { echo MAP_MISSING; exit 1; }
test -x "$D/vision-cu128/bin/python" || { echo VISION_PYTHON_MISSING; exit 1; }
test -f "$D/nav2_activate.py" || { echo NAV2_ACTIVATE_MISSING; exit 1; }
test -x "$D/wildos_server_stop.sh" || { echo STOP_SCRIPT_MISSING; exit 1; }
ros2 pkg prefix beamng_ros2 >/dev/null 2>&1 || { echo BEAMNG_ROS2_MISSING; exit 1; }
ros2 pkg prefix wildos_ros2_adapter >/dev/null 2>&1 || { echo WILDOS_ROS2_ADAPTER_MISSING; exit 1; }
ros2 pkg prefix wildos_nav2_adapter >/dev/null 2>&1 || { echo WILDOS_NAV2_ADAPTER_MISSING; exit 1; }
grep -q 'declare_parameter("update_sec", 0.2)' "$WS/build/beamng_ros2/beamng_ros2/vehicle.py" || {
  echo BRIDGE_RATE_PATCH_MISSING; exit 1;
}
grep -q '_attempt_reconnect' "$WS/build/beamng_ros2/beamng_ros2/vehicle.py" || {
  echo BRIDGE_RECONNECT_PATCH_MISSING; exit 1;
}
grep -q 'Refusing duplicate StartScenario' "$WS/build/beamng_ros2/beamng_ros2/beamng.py" || {
  echo BRIDGE_LIFECYCLE_PATCH_MISSING; exit 1;
}
"$D/vision-cu128/bin/python" -s -c 'import torch; assert torch.cuda.is_available()' || {
  echo CUDA_VISION_UNAVAILABLE; exit 1;
}
echo PREFLIGHT_OK

echo "== 2. stop old stack =="
if pgrep -f cmd_vel_to_control >/dev/null 2>&1; then
  bash "$D/wildos_server_stop.sh" || true
fi
for pat in beamng_bridge beamng_agent cmd_vel_to_control state_odom cloud_filter \
           planner_server controller_server lifecycle_manager path_planner_bridge \
           goal_adapter perception_node sensor_adapter 'ros2 topic pub' \
           'BinLinux/BeamNG.tech.x64'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 5
rm -f /tmp/wildos_window_*.log

echo "== 3. install American Road into bridge user =="
mkdir -p "$D/.local/share/BeamNG/BeamNG.tech/current/mods"
src_mod=$D/beamng-maps/american_road_0.39.zip
dst_mod=$D/.local/share/BeamNG/BeamNG.tech/current/mods/american_road_0.39.zip
cmp -s "$src_mod" "$dst_mod" 2>/dev/null || cp "$src_mod" "$dst_mod"

echo "== 4. scenario from real spawns_sawmill transform =="
"$D/venvs/beamng-039/bin/python" - "$src_mod" "$SCEN" <<'PY'
import json, sys, time, zipfile
import numpy as np
from scipy.spatial.transform import Rotation

archive_path, output = sys.argv[1], sys.argv[2]
spawn = None
with zipfile.ZipFile(archive_path) as archive:
    for member in archive.infolist():
        if not member.filename.endswith("items.level.json"):
            continue
        for line in archive.read(member).decode("utf-8-sig").splitlines():
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get("name") == "spawns_sawmill" and row.get("class") == "SpawnSphere":
                matrix = np.asarray(row.get("rotationMatrix", np.eye(3).reshape(-1)), float).reshape(3, 3)
                spawn = {"pos": [float(v) for v in row["position"]],
                         "rot": Rotation.from_matrix(matrix).as_quat().tolist()}
                break
        if spawn:
            break
if spawn is None:
    raise RuntimeError("spawns_sawmill not found")
scene = {
    "version": 1.0, "level": "american_road",
    "name": "wildos_ar_window_%d" % int(time.time()), "time_of_day": 0.0,
    "vehicles": [{
        "name": "ego", "model": "sbr",
        "part_config": "vehicles/sbr/TT_AWD_S2_DCT.pc", "color": "red",
        "pos": spawn["pos"], "rot": spawn["rot"],
        "sensors": [
            {"name": "electrics", "type": "Electrics"},
            {"name": "front_cam", "type": "Camera.default",
             "pos": [0.0, -1.5, 2.35], "dir": [0.0, -1.0, 0.0],
             "up": [0.0, 0.0, 1.0], "resolution": [960, 640],
             "field_of_view_y": 60.0, "requested_update_time": 0.2,
             "is_using_shared_memory": False, "is_streaming": False,
             "is_visualised": False},
            {"name": "front_lidar", "type": "Lidar.default",
             "pos": [0.0, 0.0, 3.6], "dir": [0.0, -1.0, -0.45],
             "up": [0.0, 0.0, 1.0], "vertical_resolution": 16,
             "vertical_angle": 60.0, "max_distance": 40.0,
             "density": 30, "is_360_mode": True,
             "requested_update_time": 0.2, "is_using_shared_memory": False,
             "is_visualised": False},
        ],
    }],
}
with open(output, "w", encoding="utf-8") as handle:
    json.dump(scene, handle, indent=2)
print("scenario", output, "spawn", spawn)
PY

echo "== 5. visible BeamNG on $DISPLAY_ID =="
cd "$BNG" || exit 1
DISPLAY=$DISPLAY_ID XAUTHORITY=$AUTH nohup ./BinLinux/BeamNG.tech.x64 \
  -tcom -tport 25252 -tcom-listen-ip 127.0.0.1 \
  > /tmp/wildos_window_beamng.log 2>&1 &
wait_port || { echo BEAMNG_PORT_FAIL; exit 1; }
beamng_pid=$(pgrep -n -f 'BinLinux/BeamNG.tech.x64')
test -n "$beamng_pid" || { echo BEAMNG_PID_MISSING; exit 1; }
tr '\0' ' ' < "/proc/$beamng_pid/cmdline" | grep -q -- '-headless' && {
  echo BEAMNG_UNEXPECTED_HEADLESS; exit 1;
}
echo BEAMNG_WINDOW_OK

echo "== 6. bridge and scenario =="
nohup ros2 run beamng_ros2 beamng_bridge --ros-args \
  -p host:=127.0.0.1 -p port:=25252 -p launch:=false \
  > /tmp/wildos_window_bridge.log 2>&1 &
wait_log /tmp/wildos_window_bridge.log 'Started beamng_bridge' 30 || {
  echo BRIDGE_START_FAILED; exit 1;
}
scenario_result=$(ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '$SCEN'}")
echo "$scenario_result"
echo "$scenario_result" | grep -q 'success=True' || { echo SCENARIO_FAILED; exit 1; }

echo "== 7. wait for real sensor messages =="
sensor_ok=0
for _ in $(seq 1 12); do
  probe=$(python3 "$D/ros2_probe.py" 4 2>&1)
  echo "$probe"
  echo "$probe" | grep -Eq "'state': [1-9]" && \
  echo "$probe" | grep -Eq "'camera': [1-9]" && \
  echo "$probe" | grep -Eq "'lidar': [1-9]" && { sensor_ok=1; break; }
  sleep 3
done
test "$sensor_ok" = 1 || { echo SENSOR_HEALTH_FAILED; exit 1; }

echo "== 8. odometry and agent =="
nohup ros2 run wildos_nav2_adapter state_odom --ros-args \
  -p state_topic:=/vehicles/ego/sensors/state -p odom_topic:=/wildos/odom \
  -p odom_frame:=odom -p base_frame:=base_link -p use_sim_truth:=true \
  > /tmp/wildos_window_odom.log 2>&1 &
sleep 3
odom_probe=$(python3 "$D/ros2_probe.py" 4 2>&1)
echo "$odom_probe"
echo "$odom_probe" | grep -Eq "'odom': [1-9]" || { echo ODOM_FAILED; exit 1; }
echo "$odom_probe" | grep -q "'odom->base_link': 1" || { echo ODOM_TF_FAILED; exit 1; }
nohup ros2 run beamng_agent beamng_agent --ros-args \
  -p host:=127.0.0.1 -p port:=25252 -p vehicle_id:=ego \
  -p driving_mode:=keyboard -p cmd_vel_topic:=/wildos/unused_cmd_vel \
  > /tmp/wildos_window_agent.log 2>&1 &
wait_log /tmp/wildos_window_agent.log 'Successfully connected to vehicle client with id ego' 30 || {
  echo AGENT_FAILED; exit 1;
}

echo "== 9. Nav2 planner and controller =="
nohup ros2 launch wildos_nav2_adapter nav2_control.launch.py \
  > /tmp/wildos_window_nav2.log 2>&1 &
sleep 12
pkill -9 -f lifecycle_manager 2>/dev/null
activate_result=$(python3 "$D/nav2_activate.py" 2>&1)
echo "$activate_result"
echo "$activate_result" | grep -q ACTIVATE_OK || { echo NAV2_ACTIVATE_FAILED; exit 1; }

echo "== 10. start perception and planning with brakes held =="
nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p cmd_vel_topic:=/wildos/cmd_vel -p control_topic:=/control \
  -p enable:=false -p gear_forward:=2 \
  > /tmp/wildos_window_control.log 2>&1 &
nohup ros2 run wildos_nav2_adapter goal_adapter --ros-args \
  -p target_standoff:=4.0 > /tmp/wildos_window_goal.log 2>&1 &
nohup ros2 run wildos_nav2_adapter path_planner_bridge --ros-args \
  -p follow_path:=true -p min_goal_distance:=3.0 \
  > /tmp/wildos_window_plan.log 2>&1 &
nohup ros2 run wildos_ros2_adapter perception_node --ros-args \
  -p query:=house -p process_period:=2.0 -p sync_slop:=0.25 \
  -p target_confirmations:=3 -p target_max_jump:=8.0 \
  -p confirmed_miss_tolerance:=10 \
  -p worker_python:=$D/vision-cu128/bin/python \
  -p worker_script:=$D/wildos-opt/vocabulary_query/vision_worker.py \
  -p checkpoint_root:=$D/ckpts \
  -p source_root:=$D/wildos-sim/wildos_port/repro_check/src/nebula2-wildos \
  -p output_dir:=$D/wildos-window-perception \
  > /tmp/wildos_window_perception.log 2>&1 &
wait_log /tmp/wildos_window_perception.log 'WildOS read-only perception started' 300 || {
  echo PERCEPTION_START_FAILED; exit 1;
}
wait_log /tmp/wildos_window_plan.log 'plan status:.*\(planned\|following\)' 120 || {
  echo FIRST_PLAN_FAILED; exit 1;
}

echo "== 11. release limited motion =="
pkill -9 -f cmd_vel_to_control 2>/dev/null
sleep 1
nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p cmd_vel_topic:=/wildos/cmd_vel -p control_topic:=/control \
  -p enable:=true -p max_speed:=0.0 -p max_steering:=0.6 \
  -p throttle_gain:=0.20 -p max_throttle:=0.35 -p min_throttle:=0.06 \
  -p throttle_integral_gain:=0.05 -p throttle_rise_rate:=0.3 \
  -p brake_gain:=0.5 \
  -p command_timeout:=0.5 -p state_timeout:=0.5 \
  -p overspeed_limit:=0.0 -p gear_forward:=2 -p steer_sign:=-1.0 \
  > /tmp/wildos_window_control.log 2>&1 &
wait_log /tmp/wildos_window_control.log 'adapter ENABLED' 20 || {
  echo CONTROL_ENABLE_FAILED; exit 1;
}
CONTROL_ENABLED=1
trap - EXIT

echo "== 12. window demo running =="
echo "Visible BeamNG should show SBR driving in American Road."
echo "Perception mode=explore until house is confirmed; then mode=target."
echo "NavigationGraph mapper is not active; this demo uses learned frontier goals plus Nav2 rolling LiDAR costmaps."
echo "Emergency stop: bash /home/ly/wildos_server_stop.sh"
echo "Logs: /tmp/wildos_window_*.log"
echo WINDOW_DEMO_RUNNING
