set -eo pipefail
R=/home/ly/planning_phase_physical_20261018
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$R/install/setup.bash"
python3 "$R/field/verify_before.py" > "$R/post_run_verification.json"
python3 "$R/analyze_closed_bound.py" > "$R/closed_analysis.log"
python3 "$R/review_field.py" > "$R/closed_summary.log"
python3 "$R/analyze_source_closed.py" > "$R/source_analysis.log"
python3 "$R/review_pursuit_closed.py" > "$R/pursuit_analysis.log"
python3 "$R/motion_metrics.py" > "$R/motion_metrics.log"
python3 "$R/verify_video.py" > "$R/video_check.log"
python3 "$R/seal_field.py" > "$R/archive.log"
