#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v53_confirmed_graph_destination/run_full.sh")
ros2 param get /beamng_vehicle_goal_standoff graph_destination_approach
