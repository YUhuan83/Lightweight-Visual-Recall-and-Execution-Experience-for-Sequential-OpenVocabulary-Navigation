"""Reuse the closed 215010 field harness; change only the tested recovery sources."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path(__file__).resolve().parent
parent = Path('/home/ly/graph_start_attachment_repair_20261020')
full = root/'full'; full.mkdir(exist_ok=False)
full.joinpath('candidate').symlink_to(root/'candidate', target_is_directory=True)
files = ('driver_full.py', 'console_graph_capture.py', 'console_full.py', 'preflight_full.py',
         'scene_left90.py', 'vision_identity.py', 'model_identity_helpers.py', 'score_observer.py', 'graph_pair_observer.py')
for name in files:
    shutil.copy2(parent/'full'/name, full/name)
shutil.copy2(root/'frozen_source_manifest.json', full/'frozen_source_manifest.json')
config = json.loads((parent/'full/config.json').read_text())
config['setups'].append(str(root/'install/local_setup.bash'))
config['python_paths'].insert(0, str(full/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['output_root'] = str(full/'sessions')
(full/'config.json').write_text(json.dumps(config, indent=2)+'\n')
shell = '''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/graph_start_attachment_repair_20261020/run_full.sh)
source ROOT/install/local_setup.bash
export PYTHONPATH=ROOT/full/candidate/tools:ROOT/full/candidate/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}
cd ROOT
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile full/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python full/driver_full.py
'''.replace('ROOT', str(root))
(root/'run_full.sh').write_text(shell)
(root/'field_preparation.json').write_text(json.dumps(dict(
    field_budget_s=config['goal_timeout_s'], video=False, unchanged_libraries=config['library_sha256'],
    harness_members={n: hashlib.sha256((full/n).read_bytes()).hexdigest() for n in files},
    complete_navigation_passed=False), indent=2)+'\n')
print(json.dumps(dict(prepared=True, video=False, budget=config['goal_timeout_s'])))
