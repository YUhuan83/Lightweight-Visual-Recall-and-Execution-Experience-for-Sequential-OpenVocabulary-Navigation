#!/usr/bin/env bash
# Build upstream WildOS graph packages plus the BeamNG input adapter.
set +u
D=/home/ly
WS=$D/beamng_ros2_0394_ws
SRC=$D/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
ADAPTER=$WS/src/beamng_wildos_adapter
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"

test -d "$SRC/visual_navigation" || { echo WILDOS_SOURCE_MISSING; exit 1; }
test -f "$ADAPTER/package.xml" || { echo OFFICIAL_ADAPTER_MISSING; exit 1; }
test -f "$D/prepare_official_wildos.py" || { echo PREP_SCRIPT_MISSING; exit 1; }
test -f "$REPO_ROOT/ackermann_rpp/package.xml" || { echo ACKERMANN_CONTROLLER_SOURCE_MISSING; exit 1; }
if test -e "$WS/src/wildos_ackermann_rpp" && ! test -L "$WS/src/wildos_ackermann_rpp"; then
  echo ACKERMANN_CONTROLLER_SOURCE_CONFLICT; exit 1
fi
ln -sfn "$REPO_ROOT/ackermann_rpp" "$WS/src/wildos_ackermann_rpp"
test -f "$REPO_ROOT/ackermann_smac/package.xml" || { echo ACKERMANN_PLANNER_SOURCE_MISSING; exit 1; }
if test -e "$WS/src/wildos_ackermann_smac" && ! test -L "$WS/src/wildos_ackermann_smac"; then
  echo ACKERMANN_PLANNER_SOURCE_CONFLICT; exit 1
fi
ln -sfn "$REPO_ROOT/ackermann_smac" "$WS/src/wildos_ackermann_smac"

python3 "$D/prepare_official_wildos.py" --source-root "$SRC" --adapter-root "$ADAPTER" || exit 1
ln -sfn "$SRC" "$WS/src/nebula2-wildos"

if ! ros2 pkg prefix grid_map_ros >/dev/null 2>&1 && ! test -d "$WS/src/grid_map"; then
  git clone --depth 1 --branch humble https://github.com/ANYbotics/grid_map.git "$WS/src/grid_map" || exit 1
fi

echo "== dependency visibility =="
for pkg in grid_map_msgs grid_map_ros pose_graph_msgs graphnav_msgs nav2_controller; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 && echo "OK $pkg" || echo "MISSING $pkg"
done

cd "$WS" || exit 1
colcon build --symlink-install --event-handlers console_direct+ --packages-up-to \
  graphnav_mapper graphnav_planner visual_navigation beamng_wildos_adapter \
  wildos_nav2_adapter || exit 1
source "$WS/install/setup.bash"

echo "== official executables =="
ros2 pkg executables graphnav_mapper | grep -q mapper_node || exit 1
ros2 pkg executables graphnav_planner | grep -q planner_node || exit 1
ros2 pkg executables graphnav_planner | grep -q path_follower_node || exit 1
ros2 pkg executables visual_navigation | grep -q 'visual_navigation wildos' || exit 1
ros2 pkg executables visual_navigation | grep -q obj_mask_triangulation || exit 1
ros2 pkg executables beamng_wildos_adapter | grep -q official_health_check || exit 1
echo OFFICIAL_BUILD_OK
