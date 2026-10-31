"""Freeze a two-file implementation/test delta; retain the prior full candidate."""
import hashlib,json,sys,zipfile
from pathlib import Path
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'tools'))
from package_official_bundle import write_bundle
old=json.loads((root/'deploy/20261018_planning_phase_candidate/official_wildos_beamng_bundle.manifest.json').read_text())
restored=[]
with zipfile.ZipFile(root/'deploy/20261018_planning_phase_candidate/official_wildos_beamng_bundle.zip') as frozen:
    name='captures/20261008_B_green_house_station_trial12_repeat_geometry/stationary_sensor_calibration.json'
    if not (root/name).is_file():
        data=frozen.read(name);assert hashlib.sha256(data).hexdigest()==old['members'][name]
        (root/name).parent.mkdir(parents=True,exist_ok=True);(root/name).write_bytes(data);restored.append(name)
changed=[n for n,h in old['members'].items() if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
assert set(changed)=={'ackermann_smac/src/planner.cpp','ackermann_smac/test/test_planner.cpp'},changed
out=root/'deploy/20261018_smoothing_boundary_candidate';out.mkdir(exist_ok=False)
manifest=write_bundle(root,out/'official_wildos_beamng_bundle.zip',set(old['members']))
report=dict(previous=old['sha256'],candidate=manifest['sha256'],members=len(manifest['members']),changed=changed,restored_unchanged_calibration_assets=restored,field_acceptance=False)
(root/'test_artifacts/vehicle_interface_audit_20261018/repair_freeze.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
