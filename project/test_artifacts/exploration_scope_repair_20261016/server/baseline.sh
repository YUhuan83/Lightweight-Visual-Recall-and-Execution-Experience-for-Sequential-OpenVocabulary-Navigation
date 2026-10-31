#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
audit_root=/home/ly/exploration_scope_repair_20261016
cmake -S "$audit_root/baseline" -B "$audit_root/baseline/build" >"$audit_root/baseline_configure.log" 2>&1
cmake --build "$audit_root/baseline/build" -j2 >"$audit_root/baseline_build.log" 2>&1
set +e
"$audit_root/baseline/build/test_scope" --gtest_filter=ExplorationBackoff.LocalAttemptDoesNotExcludeOtherDestinationsOrInferBlockedEdges >"$audit_root/cpp_before.log" 2>&1
result=$?
set -e
test "$result" -eq 1
cat "$audit_root/cpp_before.log"
