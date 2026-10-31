set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/visible_pursuit_repair_20261017/install/setup.bash
export ROS_DOMAIN_ID=176
cd /home/ly/retry_admission_review_20261017
python3 -m zipfile -e component.zip isolation
python3 server_component.py
