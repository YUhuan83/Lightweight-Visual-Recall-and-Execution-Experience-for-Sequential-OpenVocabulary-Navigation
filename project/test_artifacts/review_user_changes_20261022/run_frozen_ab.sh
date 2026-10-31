#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v8/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/incoming/audit_v7_route_recovery.py /home/ly/execution_continuity_review_20261022/v8/full/trial_20261023_085412 8.014 18.087 > /home/ly/execution_continuity_review_20261022/v8/first_route_audit.log
export CANDIDATE_FIXTURE=/home/ly/execution_continuity_review_20261022/incoming/candidate_span_repeat.txt
for repeat in 1 2; do
  /home/ly/execution_continuity_review_20261022/v4/candidate_span --gtest_filter=ActualPlanner.RecordedCandidateSpan > /home/ly/execution_continuity_review_20261022/v8/candidate_span_repeat_$repeat.log 2>&1
  grep '^SPAN ' /home/ly/execution_continuity_review_20261022/v8/candidate_span_repeat_$repeat.log
done
