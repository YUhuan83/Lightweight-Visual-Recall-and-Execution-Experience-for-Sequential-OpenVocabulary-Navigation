#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/assets/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export SDF_PATH="$ROOT/assets/models${SDF_PATH:+:$SDF_PATH}"
case "${1:-server}" in
  lane) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/lane_sample.sdf" ;;
  lane-sensors) exec ign gazebo -r -s --headless-rendering -v 3 "$ROOT/assets/worlds/lane_sensor_sample.sdf" ;;
  lane-sensors-desktop) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/lane_sensor_sample.sdf" ;;
  lane-check) exec ign sdf -k "$ROOT/assets/worlds/lane_sensor_sample.sdf" ;;
  lane-bridge) exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$ROOT/config/lane_bridge.yaml" ;;
  lane-adapter) exec python3 "$ROOT/tools/sensor_adapter.py" --ros-args -p use_sim_time:=true ;;
  lane-health) exec python3 "$ROOT/tools/validate_sensor_sample.py" --live --seconds 15 ;;
  nav2-goals) exec python3 "$ROOT/tools/nav2_goal_adapter.py" --ros-args -p use_sim_time:=true ;;
  server) exec ign gazebo -r -s --headless-rendering "$ROOT/assets/worlds/wildos_outdoor.sdf" ;;
  physics) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/wildos_physics_only.sdf" ;;
  samples) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/asset_samples.sdf" ;;
  samples-check) exec ign sdf -k "$ROOT/assets/worlds/asset_samples.sdf" ;;
  diverse) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/diverse_preview.sdf" ;;
  render-probe) exec ign gazebo -r -s -v 3 "$ROOT/assets/worlds/render_probe.sdf" ;;
  diverse-check) python3 "$ROOT/tools/validate_diverse_world.py"; exec ign sdf -k "$ROOT/assets/worlds/diverse_preview.sdf" ;;
  gui) exec ign gazebo -r "$ROOT/assets/worlds/wildos_outdoor.sdf" ;;
  bridge) exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$ROOT/config/bridge.yaml" ;;
  check) exec ign sdf -k "$ROOT/assets/worlds/wildos_outdoor.sdf" ;;
  *) echo 'Usage: bash deploy/run.sh {lane|lane-sensors|lane-sensors-desktop|lane-check|lane-bridge|lane-adapter|lane-health|nav2-goals|server|physics|samples|samples-check|diverse|diverse-check|render-probe|gui|bridge|check}' >&2; exit 2 ;;
esac
