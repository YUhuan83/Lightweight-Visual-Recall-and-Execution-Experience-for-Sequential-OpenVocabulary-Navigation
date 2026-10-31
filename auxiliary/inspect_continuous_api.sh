#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v35_confirmed_vehicle/run_full.sh)
ros2 pkg prefix nav2_regulated_pure_pursuit_controller
ros2 pkg prefix nav2_controller
RPP_PREFIX=$(ros2 pkg prefix nav2_regulated_pure_pursuit_controller)
CONTROL_PREFIX=$(ros2 pkg prefix nav2_controller)
find "$RPP_PREFIX/include" -name '*.hpp' | head -8
find "$CONTROL_PREFIX/include" -name '*goal_checker*'
find /home/ly -name controller_server.cpp -o -name regulated_pure_pursuit_controller.cpp -o -name simple_action_server.hpp 2>/dev/null | head -15
cat "$RPP_PREFIX/include/nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
cat "$CONTROL_PREFIX/include/nav2_controller/plugins/stopped_goal_checker.hpp"
cat "$CONTROL_PREFIX/include/nav2_controller/plugins/simple_goal_checker.hpp"
