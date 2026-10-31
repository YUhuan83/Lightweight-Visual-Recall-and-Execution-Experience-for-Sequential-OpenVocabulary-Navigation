#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v43_abort_recovery v45_approach_contract; do
  (
    source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
    export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
    export PYTHONPATH="$base/$version/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
    for run in 1 2; do
      if test -f "$base/$version/instance_route_replay_$run.json" && ! test -f "$base/$version/initial_wallclock_replay_$run.json"; then
        cp "$base/$version/instance_route_replay_$run.json" "$base/$version/initial_wallclock_replay_$run.json"
      fi
      if test -f "$base/$version/instance_route_replay_$run.json" && ! test -f "$base/$version/initial_default_bridge_replay_$run.json"; then
        cp "$base/$version/instance_route_replay_$run.json" "$base/$version/initial_default_bridge_replay_$run.json"
      fi
      /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_instance_route.py" "$version" >"$base/$version/instance_route_replay_$run.log" 2>&1
      cp "$base/$version/instance_route_replay.json" "$base/$version/instance_route_replay_$run.json"
    done
    cmp "$base/$version/instance_route_replay_1.json" "$base/$version/instance_route_replay_2.json"
    tail -100 "$base/$version/instance_route_replay_2.log"
  )
done
