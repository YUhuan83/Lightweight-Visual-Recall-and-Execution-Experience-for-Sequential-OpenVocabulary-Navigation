#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
sed -e 's/v40_observation_heading/v41_general_observation/g' -e 's/freeze_observation_heading.py/freeze_general_observation.py/g' "$base/build_observation_heading.sh" >"$base/build_general_observation_actual.sh"
bash "$base/build_general_observation_actual.sh"
