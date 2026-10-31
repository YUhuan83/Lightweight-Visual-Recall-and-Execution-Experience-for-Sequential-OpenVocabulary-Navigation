set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/motion_handoff_field_20261018/install/setup.bash
source /home/ly/controller_source_contract_20261018/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=179
cd /home/ly/controller_source_field_20261018
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 -m zipfile -e harness.zip .
cp official_wildos_beamng_bundle.manifest.json manifest.json
python3 - <<'PY'
import hashlib,json
from pathlib import Path
old=Path("/home/ly/controller_source_contract_20261018")
for n in ('ackermann_rpp/src/controller.cpp','ackermann_rpp/CMakeLists.txt','ackermann_rpp/package.xml'):
 assert (Path('candidate')/n).read_bytes()==(old/'candidate'/n).read_bytes(),n
lib=old/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
assert hashlib.sha256(lib.read_bytes()).hexdigest()=='d2fd9cee5ecabb49e8be06a24079813eb46a9e77872ef6982d1ed547bb04d846'
PY
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 installed_component.py >installed_component.log 2>&1
python3 field/verify_before.py >prelaunch_verification.json
