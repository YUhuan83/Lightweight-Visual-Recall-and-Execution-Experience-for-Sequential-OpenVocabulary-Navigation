#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Send a brake command and pause BeamNG before terminating the control stream.
bash "$SCRIPT_DIR/wildos_server_stop.sh" || true
pkill -9 -f 'run_motion_video.sh' 2>/dev/null || true
pkill -9 -f 'official_wildos_window_demo.sh' 2>/dev/null || true
pkill -9 -f 'launch_nizhen_inspect.sh' 2>/dev/null || true
pkill -9 -f 'watchdog_nizhen.sh' 2>/dev/null || true
pkill -9 -f 'video_recorder.py|viz_recorder.py|mjpeg_stream.py' 2>/dev/null || true
# wildos_server_stop.sh owns the simulator/Nav2 process list and applies the
# brake before stopping it. Leave unrelated ROS, RViz, and crash-report tools alone.
sleep 5
echo "=== remaining ==="
pgrep -af 'BeamNG.tech.x64|nav2|graphnav|visual_navigation|beamng_bridge|wildos' | grep -v pgrep | head -n 20 || echo ALL_CLEAN
