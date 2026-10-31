#!/usr/bin/env bash
set -eo pipefail
BASE=/home/ly/passability_repair_20261019/candidate7
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/goal_release_fence_20261019/install/local_setup.bash
source "$BASE/install/graphnav_mapper/share/graphnav_mapper/local_setup.bash"
source "$BASE/install/graphnav_planner/share/graphnav_planner/local_setup.bash"
export ROS_DOMAIN_ID=200 ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=$BASE/install/graphnav_planner/lib:$BASE/install/graphnav_mapper/lib:/home/ly/frontier_cost_repair_20261016/install/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/home/ly/candidate7_review_20261019/candidate/nav2_adapter:/home/ly/candidate7_review_20261019/candidate/official_adapter:$BASE/tools:${PYTHONPATH}
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "/home/ly/candidate7_review_20261019/replay_actual_mapper.py" > "/home/ly/candidate7_review_20261019/mapper_replay.log" 2>&1
