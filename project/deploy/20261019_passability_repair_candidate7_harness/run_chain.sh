#!/usr/bin/env bash
set -eo pipefail
BASE=/home/ly/passability_repair_20261019/candidate7
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
source /home/ly/goal_release_fence_20261019/install/local_setup.bash
source "$BASE/install/graphnav_mapper/share/graphnav_mapper/local_setup.bash"
source "$BASE/install/graphnav_planner/share/graphnav_planner/local_setup.bash"
export ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=$BASE/install/graphnav_planner/lib:$BASE/install/graphnav_mapper/lib:/home/ly/frontier_cost_repair_20261016/install/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=$BASE/tools:$BASE/nav2_adapter:$BASE/official_adapter:${PYTHONPATH}
cmake -S "$BASE/harness" -B "$BASE/fixture_build" -DMAPPER_PREFIX="$BASE/install/graphnav_mapper" -DCMAKE_BUILD_TYPE=RelWithDebInfo > "$BASE/fixture_build.log" 2>&1
cmake --build "$BASE/fixture_build" -j2 >> "$BASE/fixture_build.log" 2>&1
"$BASE/fixture_build/fixture_export" "$BASE/fixtures_v2" > "$BASE/fixture_export.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$BASE/harness/probe_chain.py" > "$BASE/chain.log" 2>&1
