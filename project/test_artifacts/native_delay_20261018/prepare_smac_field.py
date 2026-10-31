"""Freeze a field harness against the preserved two-file Smac repair candidate."""
import hashlib
import json
from pathlib import Path
root=Path('/home/ly/smoothing_boundary_repair_20261018')
diag=Path('/home/ly/native_delay_20261018')
field=root/'field_native_investigation';field.mkdir(exist_ok=False)
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text())
assert manifest['sha256']=='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
for name,want in manifest['members'].items():
    assert hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()==want,name
config=json.loads(Path('/home/ly/planning_phase_physical_20261018/field/sensor_full_console.json').read_text())
config['setups'].append(str(root/'install/local_setup.bash'))
config['output_root']=str(root/'native_investigation_results')
config['goal_timeout_s']=420
library=root/'install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so'
digest=hashlib.sha256(library.read_bytes()).hexdigest()
assert digest=='fc149c802333ebbea9504e175ae92050821a86a2debb31aaed276fbf1d711a33'
config['library_sha256']['libwildos_ackermann_smac.so']=digest
(field/'config.json').write_text(json.dumps(config,indent=2)+'\n')
wrapper=(diag/'timing_console.py').read_text().replace('/home/ly/planning_phase_physical_20261018/candidate/tools',str(root/'candidate/tools'))
wrapper=wrapper.replace("'/home/ly/planning_phase_physical_20261018/field/audit_preflight.py'",str(repr(str(field/'audit_preflight.py'))))
(field/'timing_console.py').write_text(wrapper)
preflight=Path('/home/ly/planning_phase_physical_20261018/field/audit_preflight.py').read_text().replace('/home/ly/planning_phase_physical_20261018/candidate/tools',str(root/'candidate/tools'))
(field/'audit_preflight.py').write_text(preflight)
driver=(diag/'drive_dynamic.py').read_text()
driver=driver.replace("out=root/('dynamic_'", "out=Path('/home/ly/smoothing_boundary_repair_20261018')/('field_native_'" )
driver=driver.replace("Path('/home/ly/planning_phase_physical_20261018/field/sensor_full_console.json')", "Path('/home/ly/smoothing_boundary_repair_20261018/field_native_investigation/config.json')")
driver=driver.replace('goal_timeout_s=90','goal_timeout_s=420')
driver=driver.replace("str(root/'timing_console.py')", "'/home/ly/smoothing_boundary_repair_20261018/field_native_investigation/timing_console.py'")
driver=driver.replace("wait_for(lambda:(session/'goal_001/result.json').exists(),150)","wait_for(lambda:(session/'goal_001/result.json').exists(),540)")
driver=driver.replace('90s common-stack Native diagnosis, original planner retained, no video.', '420s frozen Smac repair full search and Native timing investigation; no video.')
driver=driver.replace("report['passed']=child.returncode==0", "report['component_closed']=child.returncode==0;report['passed']=False")
driver=driver.replace("raise SystemExit(0 if report['passed'] else 1)","raise SystemExit(0 if report.get('component_closed') else 1)")
(field/'driver.py').write_text(driver)
sources={str(p.relative_to(field)):hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir() if p.is_file()}
record=dict(candidate=manifest['sha256'],members=len(manifest['members']),smac_library=digest,
            field_files=sources,native_production_modified=False,native_qos='RELIABLE depth1',
            smac_native_merge=False,scope='Single 420s baseline search; three additional memory priors remain disabled; actual physical and visual arrival require separate review.')
(field/'harness_identity.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
