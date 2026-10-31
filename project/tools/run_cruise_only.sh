#!/usr/bin/env bash
set +u
DUR=${DUR:-70}
DIST=${DIST:-115.0}
SPEED=${SPEED:-3.0}
TS=$(date +%Y%m%d_%H%M%S)
OUT=/home/ly/nizhen_runs/nizhen_drive_$TS.mp4
mkdir -p /home/ly/nizhen_runs

source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

pkill -9 -f 'cruise_cmdvel.py|cmd_vel_to_control|video_recorder.py' 2>/dev/null || true
sleep 2

echo "== odom before =="
timeout 5 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null

echo "== OUT=$OUT =="
setsid nohup /home/ly/vision-cu128/bin/python /home/ly/video_recorder.py \
  --out "$OUT" --fps 10 --seconds "$DUR" > /tmp/video_recorder.log 2>&1 < /dev/null &
sleep 4

setsid nohup ros2 run wildos_nav2_adapter cmd_vel_to_control --ros-args \
  -p enable:=true -p max_speed:=$SPEED -p max_steering:=0.4 -p steer_sign:=-1.0 \
  -p gear_forward:=2 -p command_timeout:=0.5 -p state_timeout:=0.5 -p overspeed_limit:=4.0 \
  > /tmp/cruise_control.log 2>&1 < /dev/null &
sleep 3

/home/ly/vision-cu128/bin/python /home/ly/cruise_cmdvel.py \
  --ros-args -p speed:=$SPEED -p distance:=$DIST -p timeout:=60.0 > /tmp/cruise_cmdvel.log 2>&1
echo "CRUISE_DONE rc=$?"

timeout 4 ros2 topic pub --rate 20 /control beamng_msgs/msg/VehicleControl \
  "{steering: 0.0, throttle: 0.0, brake: 1.0, parkingbrake: 1.0, clutch: 0.0, gear: 2}" > /dev/null 2>&1 || true
pkill -9 -f 'cmd_vel_to_control' 2>/dev/null || true

echo "== odom after =="
timeout 5 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null
sleep 6
echo "DONE $OUT"
ls -la "$OUT" 2>/dev/null || echo NO_VIDEO
