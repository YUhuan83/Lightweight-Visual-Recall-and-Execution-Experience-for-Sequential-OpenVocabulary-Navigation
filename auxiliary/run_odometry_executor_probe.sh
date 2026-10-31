#!/usr/bin/env bash
set -eo pipefail
root=$1
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source "$root/install/local_setup.bash"
export PYTHONPATH=/tmp:$PYTHONPATH
export ROS_DOMAIN_ID=207 ROS_LOCALHOST_ONLY=1
for repeat in 1 2; do
  if /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest -v \
      test_odometry_executor_probe.NativeStatusExecutorTests.test_forward_odometry_is_fresh_at_navigation_callback_handoff \
      test_odometry_executor_probe.NativeStatusExecutorTests.test_reverse_stop_pose_uses_current_odometry_after_navigation_work \
      > "$root/odometry_before_$repeat.log" 2>&1; then
    tail -n 5 "$root/odometry_before_$repeat.log"
  else
    tail -n 9 "$root/odometry_before_$repeat.log"
  fi
done
