#!/usr/bin/env bash
set -eo pipefail
label=${1:?baseline or candidate3}
[[ "$label" == baseline || "$label" =~ ^candidate[0-9]+$ ]] || exit 2
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
source /home/ly/goal_release_fence_20261019/install/local_setup.bash
root=/home/ly/physical_evidence_repair_20261019
source "$root/$label/install/local_setup.bash"
export ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$root/$label/candidate/tools:$root/$label/candidate/nav2_adapter:$root/$label/candidate/official_adapter:${PYTHONPATH}
cmake -S "$root" -B "$root/$label/fixture_build_v2" -DMAPPER_PREFIX="$root/$label/install/graphnav_mapper" -DCMAKE_BUILD_TYPE=RelWithDebInfo > "$root/$label/fixture_build_v2.log" 2>&1
cmake --build "$root/$label/fixture_build_v2" -j2 >> "$root/$label/fixture_build_v2.log" 2>&1
"$root/$label/fixture_build_v2/fixture_export" "$root/$label/fixtures_v2" > "$root/$label/fixture_export_v2.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$root/probe_chain.py" "$label" > "$root/$label/chain_v2.log" 2>&1
