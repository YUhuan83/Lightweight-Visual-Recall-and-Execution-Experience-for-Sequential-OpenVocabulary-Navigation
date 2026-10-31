set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
export ROS_DOMAIN_ID=178
cd /home/ly/motion_intent_handoff_repair_20261017
python3 -m zipfile -e component.zip isolation
python3 server_component.py
