set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=179
cd /home/ly/motion_handoff_field_20261018
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 -m zipfile -e harness.zip .
cp official_wildos_beamng_bundle.manifest.json manifest.json
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 installed_component.py >installed_component.log 2>&1
python3 field/verify_before.py >prelaunch_verification.json
