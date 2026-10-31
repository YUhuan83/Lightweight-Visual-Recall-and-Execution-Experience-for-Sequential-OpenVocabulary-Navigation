set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/visible_pursuit_repair_20261017/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=177
cd /home/ly/full_vehicle_acceptance_20261017
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 -m zipfile -e harness.zip .
cp official_wildos_beamng_bundle.manifest.json manifest.json
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/ackermann_smac --build-base build --install-base install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON >build.log 2>&1
source install/setup.bash
colcon --log-base test_log test --base-paths candidate/ackermann_smac --build-base build --install-base install --event-handlers console_direct+ >cpp_tests.log 2>&1
colcon test-result --test-result-base build --verbose >cpp_summary.log
python3 installed_component.py >installed_component.log 2>&1
python3 bind_harness.py >binding.log
python3 field/verify_before.py >prelaunch_verification.json
