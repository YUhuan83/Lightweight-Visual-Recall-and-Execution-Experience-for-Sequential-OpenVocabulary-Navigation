#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v7/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/incoming/compile_candidate_span.py
export CANDIDATE_FIXTURE=/home/ly/execution_continuity_review_20261022/incoming/candidate_span.txt
/home/ly/execution_continuity_review_20261022/v4/candidate_span --gtest_filter=ActualPlanner.RecordedCandidateSpan > /home/ly/execution_continuity_review_20261022/v7/candidate_span.log 2>&1
grep '^SPAN ' /home/ly/execution_continuity_review_20261022/v7/candidate_span.log
