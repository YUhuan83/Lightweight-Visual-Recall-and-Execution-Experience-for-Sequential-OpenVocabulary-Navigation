#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v35_confirmed_vehicle/run_full.sh")
export ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_vehicle_observation_candidate.py" v35_confirmed_vehicle > "$base/v35_confirmed_vehicle/weak_preservation.log" 2>&1
