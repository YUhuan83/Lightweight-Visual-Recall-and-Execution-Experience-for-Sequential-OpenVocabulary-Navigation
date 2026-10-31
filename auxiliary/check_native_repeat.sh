#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for repeat in 1 2 3; do
  for version in v40_observation_heading v41_general_observation; do
    (
      source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
      export ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1
      export PYTHONPATH="$base/$version/candidate/nav2_adapter/test:$PYTHONPATH"
      /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/check_native_repeat.py" >"$base/$version/native_repeat_$repeat.log" 2>&1
    ) || true
    echo "$version repeat=$repeat"
    grep -e NATIVE_MEASUREMENT -e NATIVE_SOURCE -e '^Ran ' -e '^OK$' -e '^FAILED' "$base/$version/native_repeat_$repeat.log"
  done
done
