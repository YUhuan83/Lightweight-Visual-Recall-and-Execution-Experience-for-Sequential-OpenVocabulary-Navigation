#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/incoming/compile_candidate_span.py
export CANDIDATE_FIXTURE=/home/ly/execution_continuity_review_20261022/incoming/early_shunts_probe.txt
for repeat in 1 2; do
  /home/ly/execution_continuity_review_20261022/v4/candidate_span --gtest_filter=ActualPlanner.RecordedCandidateSpan > /home/ly/execution_continuity_review_20261022/v9/early_shunts_probe_$repeat.log 2>&1
  grep -E '^(SPAN|WORD|FORWARD) ' /home/ly/execution_continuity_review_20261022/v9/early_shunts_probe_$repeat.log
done
