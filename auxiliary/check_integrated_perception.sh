#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v33_integrated_progress/run_full.sh")
export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PWD/candidate/tools:$PYTHONPATH
/home/ly/vision-cu128/bin/python "$base/validate_v33_perception.py" > "$base/v33_integrated_progress/perception_review.log" 2>&1
