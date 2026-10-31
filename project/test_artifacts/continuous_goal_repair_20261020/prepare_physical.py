"""Freeze the prior full physical candidate plus the tested Smac delta, without editing either parent."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
parent = Path('/home/ly/vehicle_profile_repair_20261019')
for name in ('test_planner','test_connector','test_path_contract'):
    result = ET.parse(root/f'after_{name}.xml').getroot()
    assert int(result.attrib['failures']) == int(result.attrib['errors']) == 0
manifest = json.loads((parent/'frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((parent/'candidate'/n).read_bytes()).hexdigest()==h
           for n,h in manifest['members'].items())
physical = root/'physical'
physical.mkdir(exist_ok=False)
shutil.copytree(parent/'candidate',physical/'candidate')
for name in ('src/planner.cpp','test/test_planner.cpp','test/fixtures/reverse_goal_public.txt'):
    shutil.copy2(root/'candidate/ackermann_smac'/name, physical/'candidate/ackermann_smac'/name)
members = dict(manifest['members'])
for name in ('ackermann_smac/src/planner.cpp','ackermann_smac/test/test_planner.cpp',
             'ackermann_smac/test/fixtures/reverse_goal_public.txt'):
    members[name] = hashlib.sha256((physical/'candidate'/name).read_bytes()).hexdigest()
(physical/'frozen_source_manifest.json').write_text(json.dumps(dict(
    scope=__doc__,parent_manifest_sha256=hashlib.sha256((parent/'frozen_source_manifest.json').read_bytes()).hexdigest(),
    members=members),indent=2)+'\n')
identity = json.loads((parent/'verified_install_sources.json').read_text())
identity['libraries']['libwildos_ackermann_smac.so'] = hashlib.sha256(
    (root/'install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so').read_bytes()).hexdigest()
(physical/'verified_install_sources.json').write_text(json.dumps(identity,indent=2)+'\n')
for source,destination in [('run_short_component_v2.py','run_short_component.py'),
                           ('execution_stack_only.py','execution_stack_only.py'),
                           ('direct_execution_probe.py','direct_execution_probe.py')]:
    shutil.copy2(parent/source,physical/destination)
report = dict(source_members=len(members), libraries=identity['libraries'],
              motion_goals=0, complete_navigation_passed=False)
(root/'physical_preparation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
