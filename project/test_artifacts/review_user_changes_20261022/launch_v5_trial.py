from pathlib import Path
import subprocess,json,hashlib,sys
root=Path('/home/ly/execution_continuity_review_20261022/v5')
pre=subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],
                   check=True,capture_output=True,text=True,timeout=20)
(root/'pretrial_check.json').write_text(pre.stdout)
assert not json.loads(pre.stdout)['processes'],pre.stdout
assert not Path('/tmp/.X148-lock').exists(),'display still owned'
tests=json.loads((root/'real_ros_report.json').read_text())
assert all(r['passed'] for r in tests)
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==s for n,s in manifest['members'].items())
assert 'ROUND_TIMEOUT_SEC=180.0' in (root/'full/driver_full.py').read_text()
config_path=root/'full/config.json';config=json.loads(config_path.read_text())
config['goal_timeout_s']=180;config_path.write_text(json.dumps(config,indent=2))
print(json.dumps({'precheck':json.loads(pre.stdout),'real_ros_tests':sum(r['tests'] for r in tests),
                  'round_timeout_sec':180}),flush=True)
raise SystemExit(subprocess.call(['bash',str(root/'run_full.sh')]))
