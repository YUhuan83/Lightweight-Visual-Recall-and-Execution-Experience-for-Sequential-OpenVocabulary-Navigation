#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
root="$base/frontier_endpoint_contract_review"
if [ "$1" = before ]; then
 test ! -e "$root"
 mkdir "$root"
 python3 -m zipfile -e "$base/review_before.zip" "$root"
fi
cmake -S "$root" -B "$root/build" >"$root/configure_$1.log" 2>&1
cmake --build "$root/build" -j2 >"$root/build_$1.log" 2>&1
for run in 1 2; do
 set +e
 "$root/build/endpoint_tests" --gtest_output="xml:$root/$1_$run.xml" >"$root/$1_$run.log" 2>&1
 code=$?
 set -e
 echo "$1 run $run exit=$code"
 tail -12 "$root/$1_$run.log"
 if [ "$1" = after ]; then test "$code" = 0; else test "$code" = 1; fi
done
