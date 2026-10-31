#!/usr/bin/env bash
# Visible BeamNG demo using upstream WildOS_Nav, graphnav_mapper, graphnav_planner
# and path_follower. Project code is limited to BeamNG message/control adapters.
set +u
D=/home/ly
WS=$D/beamng_ros2_0394_ws
SRC=$D/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
BNG=$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0
SCEN=$WS/src/beamng-ros2-integration/beamng_ros2/config/scenarios/wildos_official_window.json
FACE_HOUSE=${FACE_HOUSE:-0}
READ_ONLY=${READ_ONLY:-0}
WILDOS_CONFIG=${WILDOS_CONFIG:-beamng_wildos}
ENABLE_MOTION=${ENABLE_MOTION:-1}
MAP_MODE=${MAP_MODE:-american_road}
OBJECT_SEARCH=${OBJECT_SEARCH:-true}
BNG_USER=${BNG_USER:-}
BEAMNG_HEADLESS=${BEAMNG_HEADLESS:-0}
ENABLE_RVIZ=${ENABLE_RVIZ:-1}
ENABLE_RECORDER=${ENABLE_RECORDER:-1}
INSPECT_ONLY=${INSPECT_ONLY:-0}
SEED_DISTANCE=${SEED_DISTANCE:-25.0}
MOTION_SPEED=${MOTION_SPEED:-1.2}
RVIZ_CONFIG=${RVIZ_CONFIG:-$D/nizhen_wildos.rviz}
RECORD_DIR=${RECORD_DIR:-/home/ly/nizhen_viz_capture}
DISPLAY=:3
XAUTHORITY=/run/user/1011/gdm/Xauthority

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
export DISPLAY XAUTHORITY
export PYTHONPATH=$SRC:$PYTHONPATH

fail() {
  echo "OFFICIAL_DEMO_FAILED: $1"
  bash "$D/wildos_server_stop.sh" 2>/dev/null || true
  tail -n 20 /tmp/official_mapper.log 2>/dev/null
  tail -n 20 /tmp/official_wildos.log 2>/dev/null
  tail -n 40 /tmp/official_graph_health.log 2>/dev/null
  tail -n 40 /tmp/official_scored_health.log 2>/dev/null
  tail -n 40 /tmp/official_path_health.log 2>/dev/null
  exit 1
}

wait_log() {
  file=$1; pattern=$2; attempts=$3
  for _ in $(seq 1 "$attempts"); do
    grep -q "$pattern" "$file" 2>/dev/null && return 0
    sleep 1
  done
  return 1
}

echo "== official preflight =="
DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY xdpyinfo >/dev/null 2>&1 || fail DISPLAY
for pkg in beamng_ros2 beamng_wildos_adapter graphnav_mapper graphnav_planner visual_navigation wildos_nav2_adapter; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || fail "package $pkg"
done
test -x "$D/vision-cu128/bin/python" || fail VISION_PYTHON
PYTHONPATH=$SRC:$PYTHONPATH "$D/vision-cu128/bin/python" -c \
  'import rclpy, torch, visual_navigation.wildos.nav; assert torch.cuda.is_available()' \
  || fail VISION_ROS_CUDA
test -f "$WS/install/visual_navigation/share/visual_navigation/configs/beamng_wildos.yaml" || fail WILDOS_CONFIG
echo OFFICIAL_PREFLIGHT_OK

echo "== stop old processes =="
bash "$D/wildos_server_stop.sh" 2>/dev/null || true
for pat in beamng_bridge beamng_agent cmd_vel_to_control planner_server controller_server \
  lifecycle_manager graphnav_mapper graphnav_planner path_follower visual_navigation.wildos.nav obj_mask_triangulation \
  beamng_wildos_adapter 'BinLinux/BeamNG.tech.x64'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 5

echo "== prepare map and scenario =="
if test -n "$BNG_USER"; then
  mkdir -p "$BNG_USER/current/mods" "$BNG_USER/mods"
  if test "$MAP_MODE" = american_road; then
    cp -f "$D/beamng-maps/american_road_0.39.zip" "$BNG_USER/current/mods/" 2>/dev/null || true
  elif test "$MAP_MODE" = nizhen_countryside; then
    cp -f "$D/beamng-maps/nizhen_countryside_standalone.zip" "$BNG_USER/current/mods/" 2>/dev/null || true
    cp -f "$D/beamng-maps/nizhen_countryside_standalone.zip" "$BNG_USER/mods/" 2>/dev/null || true
  fi
else
  mkdir -p "$D/.local/share/BeamNG/BeamNG.tech/current/mods"
  if test "$MAP_MODE" = american_road; then
    cp -f "$D/beamng-maps/american_road_0.39.zip" "$D/.local/share/BeamNG/BeamNG.tech/current/mods/"
  elif test "$MAP_MODE" = nizhen_countryside; then
    cp -f "$D/beamng-maps/nizhen_countryside_standalone.zip" "$D/.local/share/BeamNG/BeamNG.tech/current/mods/"
  fi
fi
"$D/venvs/beamng-039/bin/python" - "$D/beamng-maps/american_road_0.39.zip" "$SCEN" "$FACE_HOUSE" "$MAP_MODE" <<'PY'
import json,math,sys,time,zipfile,numpy as np
from scipy.spatial.transform import Rotation
spawn=None; houses=[]
if sys.argv[4] == 'tech_ground':
  spawn=([0.0, 0.0, 0.65],[0.0, 0.0, 0.0, 1.0])
elif sys.argv[4] == 'nizhen_countryside':
  spawn=([-40.0, -80.0, 10.65],Rotation.from_matrix(np.array([[0,1,0],[-1,0,0],[0,0,1]],float)).as_quat().tolist())
else:
  with zipfile.ZipFile(sys.argv[1]) as z:
    for f in z.infolist():
      if not f.filename.endswith('items.level.json'): continue
      for line in z.read(f).decode('utf-8-sig').splitlines():
        try: row=json.loads(line.strip())
        except Exception: continue
        if row.get('name')=='spawns_sawmill' and row.get('class')=='SpawnSphere':
          m=np.asarray(row.get('rotationMatrix',np.eye(3).reshape(-1)),float).reshape(3,3)
          spawn=([float(v) for v in row['position']],Rotation.from_matrix(m).as_quat().tolist())
        asset=' '.join(str(row.get(k,'')) for k in ('name','internalName','shapeName','shapeFile','fileName')).lower()
        if row.get('class')=='TSStatic' and 'house' in asset and isinstance(row.get('position'),list):
          houses.append(([float(v) for v in row['position']],asset))
if not spawn: raise RuntimeError('spawn missing')
if sys.argv[3]=='1' and sys.argv[4] == 'american_road':
  target=min(houses,key=lambda h:math.dist(spawn[0],h[0]))
  dx,dy=target[0][0]-spawn[0][0],target[0][1]-spawn[0][1]
  # BeamNG applies the scenario quaternion with the inverse world yaw convention.
  yaw=-math.atan2(dx,-dy)
  spawn=(spawn[0],Rotation.from_euler('z',yaw).as_quat().tolist())
  print('FACING_HOUSE',target[0],math.hypot(dx,dy),target[1])
level='tech_ground' if sys.argv[4] == 'tech_ground' else ('nizhen_countryside' if sys.argv[4] == 'nizhen_countryside' else 'american_road')
scene={'version':1.0,'level':level,'name':'official_wildos_%d'%int(time.time()),'time_of_day':0.0,
 'vehicles':[{'name':'ego','model':'sbr','part_config':'vehicles/sbr/TT_AWD_S2_DCT.pc','pos':spawn[0],'rot':spawn[1],
 'sensors':[{'name':'electrics','type':'Electrics'},
 {'name':'front_cam','type':'Camera.default','pos':[0,-1.5,2.35],'dir':[0,-1,0],'up':[0,0,1],
  'resolution':[960,640],'field_of_view_y':60.0,'is_streaming':False,'is_using_shared_memory':False},
 {'name':'front_lidar','type':'Lidar.default','pos':[0,0,3.6],'dir':[0,-1,-0.45],'up':[0,0,1],
  'vertical_resolution':16,'vertical_angle':60.0,'max_distance':40.0,'density':30,'is_360_mode':True,
  'is_using_shared_memory':False,'is_visualised':False}]}]}
json.dump(scene,open(sys.argv[2],'w'),indent=2)
PY

echo "== visible BeamNG + bridge =="
cd "$BNG" || fail BEAMNG_DIR
bng_args=(-nosteam -tcom -tport 25252 -tcom-listen-ip 127.0.0.1)
if test -n "$BNG_USER"; then bng_args+=(-userpath "$BNG_USER"); fi
if test "$BEAMNG_HEADLESS" = 1; then bng_args+=(-headless); fi
nohup ./BinLinux/BeamNG.tech.x64 "${bng_args[@]}" > /tmp/official_beamng.log 2>&1 &
for _ in $(seq 1 120); do ss -ltn 2>/dev/null|grep -q ':25252 ' && break;sleep 1;done
ss -ltn 2>/dev/null|grep -q ':25252 ' || fail BEAMNG_PORT
nohup ros2 run beamng_ros2 beamng_bridge --ros-args -p host:=127.0.0.1 -p port:=25252 -p launch:=false > /tmp/official_bridge.log 2>&1 &
wait_log /tmp/official_bridge.log 'Started beamng_bridge' 30 || fail BRIDGE
result=$(ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario "{path_to_scenario_definition: '$SCEN'}")
echo "$result"; echo "$result"|grep -q 'success=True' || fail SCENARIO

echo "== official input adapter and graph stack =="
nohup ros2 run wildos_nav2_adapter graph_path_bridge --ros-args \
-p follow_path:=false -p carrier_model:=ackermann -p planner_id:=CarLike \
-p minimum_turning_radius:=8.0 > /tmp/official_local_plan.log 2>&1 &
nohup ros2 launch beamng_wildos_adapter official_graph_stack.launch.py \
  planner_goal_topic:=/spot1/graphnav_destination carrier_model:=ackermann \
  vehicle_front_extent:=3.4 state_topic:=/vehicles/ego/sensors/state \
  > /tmp/official_mapper.log 2>&1 &
ros2 run beamng_wildos_adapter official_health_check --ros-args \
  -p mode:=graph -p timeout:=60.0 -p output_path:=/tmp/official_graph_health.json \
  > /tmp/official_graph_health.log 2>&1 || fail NAV_GRAPH_CONTENT

echo "== official WildOS and triangulator =="
nohup "$D/vision-cu128/bin/python" -m visual_navigation.wildos.nav --config "$WILDOS_CONFIG" --do_object_search "$OBJECT_SEARCH" --memory_in_process_only > /tmp/official_wildos.log 2>&1 &
nohup "$D/vision-cu128/bin/python" -m visual_navigation.explorfm_triangulation.obj_mask_triangulation --config beamng_triangulation.yaml > /tmp/official_triangulation.log 2>&1 &
wait_log /tmp/official_wildos.log 'Finished initializing models' 300 || fail WILDOS_MODEL
ros2 run beamng_wildos_adapter official_health_check --ros-args \
  -p mode:=scored -p timeout:=300.0 -p output_path:=/tmp/official_scored_health.json \
  > /tmp/official_scored_health.log 2>&1 || fail SCORED_GRAPH_CONTENT

echo "== RViz on display :3 and matplotlib recorder =="
if test "$ENABLE_RVIZ" = 1; then
  test -f "$RVIZ_CONFIG" || fail RVIZ_CONFIG
  pkill -9 -f 'rviz2' 2>/dev/null || true
  sleep 1
  DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY nohup rviz2 -d "$RVIZ_CONFIG" > /tmp/official_rviz.log 2>&1 &
  sleep 6
  if pgrep -f 'rviz2' >/dev/null 2>&1; then
    echo OFFICIAL_RVIZ_STARTED
  else
    echo OFFICIAL_RVIZ_MAYBE_FAILED
    tail -n 20 /tmp/official_rviz.log 2>/dev/null
  fi
fi
if test "$ENABLE_RECORDER" = 1; then
  test -f "$D/viz_recorder.py" || fail RECORDER_SCRIPT
  mkdir -p "$RECORD_DIR"
  pkill -9 -f 'viz_recorder.py' 2>/dev/null || true
  sleep 1
  nohup "$D/vision-cu128/bin/python" "$D/viz_recorder.py" --out "$RECORD_DIR" --period 2.0 > /tmp/official_recorder.log 2>&1 &
  echo OFFICIAL_RECORDER_STARTED
fi

if test "$READ_ONLY" = 1; then
  echo "== read-only object-search observation =="
  if timeout 60 ros2 topic echo --once /spot1/object_mask > /tmp/official_object_mask.txt 2>&1; then
    echo OFFICIAL_OBJECT_MASK_RECEIVED
  else
    echo OFFICIAL_OBJECT_MASK_NOT_RECEIVED
  fi
  bash "$D/wildos_server_stop.sh"
  echo OFFICIAL_WILDOS_READ_ONLY_COMPLETE
  exit 0
fi

if test "$INSPECT_ONLY" = 1; then
  echo OFFICIAL_WILDOS_INSPECT_READY
  echo "RViz is on display $DISPLAY. Recorder writes to $RECORD_DIR."
  echo "Mapper/WildOS stay live; nothing is stopped. Use: bash $D/wildos_server_stop.sh"
  while true; do sleep 30; done
fi

echo "== Nav2 external local execution, brakes held =="
ACKERMANN_PARAMS=$WS/install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/nav2_ackermann_control.yaml
test -f "$ACKERMANN_PARAMS" || fail ACKERMANN_PARAMS
nohup ros2 launch wildos_nav2_adapter nav2_official_control.launch.py \
  params_file:="$ACKERMANN_PARAMS" \
  footprint_front_extent:=3.4 footprint_rear_extent:=1.05 \
  footprint_half_width:=1.05 filter_footprint_returns:=true \
  > /tmp/official_nav2.log 2>&1 &
sleep 12
python3 "$D/nav2_activate.py" | grep -q ACTIVATE_OK || fail NAV2_ACTIVE
nohup ros2 run beamng_wildos_adapter official_health_check --ros-args \
  -p mode:=path -p timeout:=90.0 -p output_path:=/tmp/official_path_health.json \
  > /tmp/official_path_health.log 2>&1 &
path_health_pid=$!
sleep 1
nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args -p enable:=false -p gear_forward:=2 -p allow_reverse:=true > /tmp/official_control.log 2>&1 &

echo "== official target waypoint or explicit seed waypoint =="
seed_used=0
if test "$OBJECT_SEARCH" != true; then
  ros2 run beamng_wildos_adapter seed_goal --ros-args -p distance:=$SEED_DISTANCE > /tmp/official_seed_goal.log 2>&1 || fail SEED_GOAL
  seed_used=1
elif timeout 20 ros2 topic echo --once /spot1/raw_object_waypoint > /tmp/official_object_goal.txt 2>&1; then
  echo OFFICIAL_OBJECT_GOAL_RECEIVED
else
  echo OFFICIAL_FRONTIER_EXPLORATION_WITHOUT_OBJECT_WAYPOINT
fi
wait_log /tmp/official_local_plan.log 'plan status:.*planned' 60 || fail LOCAL_PLAN
wait "$path_health_pid" || echo "OFFICIAL_PATH_HEALTH_INCOMPLETE (continuing; usable Nav2 path already present)"
pkill -9 -f cmd_vel_to_control; sleep 1
if test "$ENABLE_MOTION" != 1; then
  echo OFFICIAL_WILDOS_PATH_VALIDATED_MOTION_DISABLED
  bash "$D/wildos_server_stop.sh"
  exit 0
fi
nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p enable:=true -p allow_reverse:=true -p max_steering:=0.6 \
  -p throttle_gain:=0.20 -p max_throttle:=0.35 -p min_throttle:=0.06 \
  -p throttle_integral_gain:=0.05 -p throttle_rise_rate:=0.3 \
  -p brake_gain:=0.5 \
  -p steer_sign:=-1.0 -p gear_forward:=2 -p command_timeout:=0.5 \
  -p state_timeout:=0.5 -p overspeed_limit:=0.0 \
  > /tmp/official_control.log 2>&1 &
wait_log /tmp/official_control.log 'adapter ENABLED' 20 || fail CONTROL
ros2 param set /wildos_graph_path_bridge follow_path true || fail FOLLOW_PATH_ENABLE
wait_log /tmp/official_local_plan.log 'plan status:.*following' 60 || fail FOLLOW_PATH_START

echo OFFICIAL_WILDOS_WINDOW_RUNNING
echo "Emergency stop: bash $D/wildos_server_stop.sh"
if test "$seed_used" = 1; then
  echo "Seed goal is experiment orchestration only; mapper/scoring/Dijkstra/path follower are upstream WildOS."
else
  echo "Object goal came from upstream WildOS object-mask triangulation; no seed was used."
fi
echo "Health reports: /tmp/official_graph_health.json /tmp/official_scored_health.json /tmp/official_path_health.json"
echo "Official topics: /spot1/nav_graph /spot1/scored_nav_graph /spot1/imgnav_waypoint /spot1/graphnav_planner/path /spot1/goal_pose"
