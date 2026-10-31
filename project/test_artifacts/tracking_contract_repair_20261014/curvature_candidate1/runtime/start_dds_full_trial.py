"""Detach one bounded full trial so SSH observation loss cannot restart it."""
import hashlib,json,os,subprocess
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1')
assert json.loads((BASE/'source_identity.json').read_text())['sha256']=='71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660'
session=BASE/'green_house_session1'
assert json.loads((session/'dds_owners.json').read_text())['passed']
vision=int((session/'vision.pid').read_text());tokens=Path(f'/proc/{vision}/cmdline').read_bytes().split(b'\0')
assert b'visual_navigation.wildos.nav' in tokens
config=b'/home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/share/beamng_wildos_adapter/config/beamng_wildos.yaml'
assert config in tokens,'Model not using the frozen explicit configuration'
out=BASE/'green_house_trial1';assert not out.exists() and not (BASE/'green_house_trial1.pid').exists()
argv=['/home/ly/beamng_ros2_0394_ws/.venv/bin/python',str(BASE/'bundle/tools/town_physical_single_target_trial.py'),
    '--session-dir',str(session),'--arm',str(BASE/'bundle'),'--output',str(out),
    '--query','green residential house','--timeout','420']
with (BASE/'green_house_trial1.log').open('x') as log:
    p=subprocess.Popen(argv,cwd=BASE/'bundle',stdin=subprocess.DEVNULL,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(BASE/'green_house_trial1.pid').write_text(str(p.pid)+chr(10))
identity={'trial_pid':p.pid,'argv':argv,'scope':'Full single target; final visual and physical checks required',
    'script_sha256':hashlib.sha256((BASE/'bundle/tools/town_physical_single_target_trial.py').read_bytes()).hexdigest()}
(BASE/'green_house_trial1_driver.json').write_text(json.dumps(identity,indent=2));print(json.dumps(identity))
