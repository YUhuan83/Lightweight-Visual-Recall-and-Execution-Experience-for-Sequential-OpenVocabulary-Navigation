#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
cd /home/ly/exploration_efficiency_audit_20261016/frontier_alternatives
export GEOMETRY_SMOOTH=1
unset GEOMETRY_FREE
for candidate in 20 21 22 23 24 25 26 27; do
  export GEOMETRY_CASE=$candidate
  /home/ly/town_memory_validation_20261014/dds_transport_candidate/geometry_replay1/build_repaired/recorded_planner --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit > case${candidate}.log 2>&1
  grep GEOMETRY_PLAN case${candidate}.log
done
