#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=175
cd /home/ly/connector_audit_20261017
python3 -m zipfile -e complete_path_sources.zip complete_sources
for mode in baseline candidate; do
  colcon --log-base "log_$mode" build --base-paths "complete_sources/$mode/ackermann_smac" --build-base "build_$mode" --install-base "install_$mode" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "build_${mode}.log" 2>&1
  (
    source "install_$mode/setup.bash"
    mkdir -p "results_$mode"
    cd "results_$mode"
    ln -sf ../input*.txt .
    sha256sum ../install_$mode/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so > library_identity.txt
    ldd ../build/recorded_planner > linked_libraries.txt
    for index in $(seq 0 18); do
      GEOMETRY_CASE=$index GEOMETRY_SMOOTH=1 ../build/recorded_planner --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit > "case${index}.log" 2>&1
      grep 'GEOMETRY_PLAN' "case${index}.log"
    done
  )
done
