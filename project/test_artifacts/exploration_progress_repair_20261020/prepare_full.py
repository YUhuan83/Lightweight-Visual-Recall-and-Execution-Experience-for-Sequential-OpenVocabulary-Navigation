"""Freeze the current connected candidate and field-only instrumentation; no production edits."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parent
parent=Path('/home/ly/frontier_absence_repair_20261020')
manifest=json.loads((parent/'frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((parent/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
field=ROOT/'full';field.mkdir(exist_ok=False)
shutil.copytree(parent/'candidate',field/'candidate')
shutil.copy2(parent/'frozen_source_manifest.json',field/'frozen_source_manifest.json')
config=json.loads(Path('/home/ly/smoothing_boundary_repair_20261018/field_native_20261018_164222/config.json').read_text())
config['setups'] += ['/home/ly/vehicle_profile_repair_20261019/install/local_setup.bash',
                     '/home/ly/continuous_goal_repair_20261020/install/local_setup.bash',
                     str(parent/'install/local_setup.bash')]
config['python_paths']=[str(field/'candidate/wildos_port/repro_check/src/nebula2-wildos'),*config['python_paths']]
config.update(output_root=str(field/'sessions'),goal_timeout_s=420,domain=88,rviz=True)
identity=json.loads(Path('/home/ly/continuous_goal_repair_20261020/physical/verified_install_sources.json').read_text())
identity['libraries']['libgraphnav_planner_component.so']=hashlib.sha256(
    (parent/'install/graphnav_planner/lib/libgraphnav_planner_component.so').read_bytes()).hexdigest()
config['library_sha256']=identity['libraries']
(field/'config.json').write_text(json.dumps(config,indent=2)+'\n')
# Preserve the authorized left-90-degree field orientation from the last full
# test. Only this diagnostic scene differs from the frozen console scene.
scene=(field/'candidate/tools/wildos_console_scene.py').read_text()
needle='rot_quat=(0., 0., .7658087977066911, .6430683364581344)'
assert scene.count(needle)==1
scene=scene.replace(needle,'rot_quat=(0.0, 0.0, 0.08679061247481905, 0.9962265754266173)')
(field/'scene_left90.py').write_text(scene)
for name in ('console_full.py','vision_identity.py','model_identity_helpers.py','driver_full.py','score_observer.py'):
    shutil.copy2(ROOT/name,field/name)
preflight="""import sys
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'candidate/tools'))
import server_test_preflight as module
module.ROLES['vision']=(str(root/'vision_identity.py'),)
raise SystemExit(module.main())
"""
(field/'preflight_full.py').write_text(preflight)
files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir() if p.is_file()}
(field/'harness_identity.json').write_text(json.dumps(dict(source_members=len(manifest['members']),
    files=files,scope=__doc__,memory_priors='production defaults, required active by model identity probe',
    model_and_physics_unmodified=True,video=False),indent=2)+'\n')
print(json.dumps(dict(output=str(field),source_members=len(manifest['members']),libraries=identity['libraries']),indent=2))
