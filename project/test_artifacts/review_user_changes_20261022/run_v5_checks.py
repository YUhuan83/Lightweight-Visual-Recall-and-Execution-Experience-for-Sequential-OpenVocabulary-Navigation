from pathlib import Path
import shutil,subprocess
root=Path('/home/ly/execution_continuity_review_20261022')
build=root/'v5/build.sh'
text=build.read_text()
text=text.replace('cd '+str(root/'v5')+'\n',
    'cd '+str(root/'v5')+'\nexport PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH\n')
shutil.copy2(root/'v5/build.log',root/'v5/build_environment_error.log')
build.write_text(text)
replay=root/'v4/run_observation_replay.sh'
replay.write_text('''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v4/run_full.sh)
export ROS_DOMAIN_ID=206 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 /home/ly/execution_continuity_review_20261022/incoming/write_observation_fixture.py
/usr/bin/python3 /home/ly/execution_continuity_review_20261022/incoming/compile_observation_replay.py
export OBSERVATION_FIXTURE=/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180945/observation_replay/request59_public_grid.txt
/home/ly/execution_continuity_review_20261022/v4/observation_replay --gtest_filter=ActualPlanner.RecordedObservationEndpointProbe
''')
first=subprocess.Popen(['bash',str(build)])
with (root/'v4/observation_replay.log').open('w') as log:
    second=subprocess.run(['bash',str(replay)],stdout=log,stderr=subprocess.STDOUT)
print('observation_replay_exit',second.returncode,flush=True)
print('v5_checks_exit',first.wait(),flush=True)
