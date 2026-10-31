"""Freeze short physical trial; preserve parent engine/vehicle/control settings."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path(__file__).resolve().parent
parent = Path('/home/ly/confirmed_approach_recovery_20261020/v2')
full = root/'full'
full.mkdir()
(full/'candidate').symlink_to(root/'candidate', target_is_directory=True)
files = ('driver_full.py', 'console_graph_capture.py', 'console_full.py', 'preflight_full.py',
         'scene_left90.py', 'vision_identity.py', 'model_identity_helpers.py',
         'score_observer.py', 'graph_pair_observer.py')
for name in files:
    shutil.copy2(parent/'full'/name, full/name)
shutil.copy2(root/'frozen_source_manifest.json', full/'frozen_source_manifest.json')
config = json.loads((parent/'full/config.json').read_text())
config['setups'].append(str(root/'install/local_setup.bash'))
config['python_paths'].insert(0, str(full/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['output_root'] = str(full/'sessions')
config['goal_timeout_s'] = 120
(full/'config.json').write_text(json.dumps(config, indent=2)+'\n')
instrumentation = root/'instrumentation'; instrumentation.mkdir()
shutil.copy2(root/'motion_trace.py', instrumentation/'motion_trace.py')
old = Path('/home/ly/native_delay_20261018/instrumentation/sitecustomize.py').read_text()
(instrumentation/'sitecustomize.py').write_text(old+'''\n
if Path(sys.argv[0]).name == 'graph_path_bridge' and os.environ.get('WILDOS_MOTION_TRACE_DIR'):
    import motion_trace
    motion_trace.install()
''')
p = full/'console_graph_capture.py'
s = p.read_text()
needle = "        env.pop('WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT',None)"
assert needle in s
s = s.replace(needle, needle+"\n        env['PYTHONPATH']=str(ROOT.parent/'instrumentation')+os.pathsep+env['PYTHONPATH']\n        env['WILDOS_MOTION_TRACE_DIR']=str(self.output/'motion_trace')")
p.write_text(s)
shell = '''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' PARENT/run_full.sh)
source ROOT/install/local_setup.bash
export PYTHONPATH=ROOT/full/candidate/tools:ROOT/full/candidate/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}
cd ROOT
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile full/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python full/driver_full.py
'''.replace('PARENT', str(parent)).replace('ROOT', str(root))
(root/'run_full.sh').write_text(shell)
(root/'field_preparation.json').write_text(json.dumps(dict(budget=120,video=False,
    harness_members={n:hashlib.sha256((full/n).read_bytes()).hexdigest() for n in files},
    libraries=config['library_sha256']),indent=2)+'\n')
print(json.dumps(dict(prepared=True,budget=120,video=False)))
