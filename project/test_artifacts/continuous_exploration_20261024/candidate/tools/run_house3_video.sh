#!/usr/bin/env bash
set +u
DUR=${DUR:-75}
TS=$(date +%Y%m%d_%H%M%S)
OUT=/home/ly/nizhen_runs/nizhen_house3_$TS.mp4
mkdir -p /home/ly/nizhen_runs

source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

pkill -9 -f 'drive_look.py|cruise_cmdvel.py|cmd_vel_to_control|video_recorder.py|watchdog_nizhen.sh' 2>/dev/null || true
bash /home/ly/cleanup_all.sh >/dev/null 2>&1
sleep 4

setsid bash /home/ly/launch_nizhen_inspect.sh > /tmp/nizhen_inspect_house3.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  grep -q OFFICIAL_WILDOS_INSPECT_READY /tmp/nizhen_inspect_house3.log 2>/dev/null && break
  sleep 10
done
grep -q OFFICIAL_WILDOS_INSPECT_READY /tmp/nizhen_inspect_house3.log 2>/dev/null || { echo PIPELINE_NOT_READY; tail -n 20 /tmp/nizhen_inspect_house3.log; exit 1; }
echo "OUT=$OUT"
echo PIPELINE_READY

setsid nohup /home/ly/vision-cu128/bin/python /home/ly/video_recorder.py \
  --out "$OUT" --fps 10 --seconds "$DUR" > /tmp/video_recorder.log 2>&1 < /dev/null &
echo RECORDER_STARTED
sleep 4

setsid nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p enable:=true -p max_speed:=2.5 -p max_steering:=0.6 -p steer_sign:=-1.0 \
  -p gear_forward:=2 -p command_timeout:=0.5 -p state_timeout:=0.5 -p overspeed_limit:=4.0 \
  > /tmp/cruise_control.log 2>&1 < /dev/null &
sleep 3

/home/ly/vision-cu128/bin/python /home/ly/drive_look.py \
  --ros-args -p target_x:=46.0 -p approach_speed:=2.0 -p turn_rate:=0.55 -p turn_degrees:=85.0 -p timeout:=80.0 \
  > /tmp/drive_look.log 2>&1
echo "DRIVE_LOOK_DONE rc=$?"

timeout 4 ros2 topic pub --rate 20 /control beamng_msgs/msg/VehicleControl \
  "{steering: 0.0, throttle: 0.0, brake: 1.0, parkingbrake: 1.0, clutch: 0.0, gear: 2}" > /dev/null 2>&1 || true
pkill -9 -f 'cmd_vel_to_control' 2>/dev/null || true

echo "== odom after =="
timeout 5 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null
sleep 8
echo "DONE $OUT"
ls -la "$OUT" 2>/dev/null || echo NO_VIDEO
