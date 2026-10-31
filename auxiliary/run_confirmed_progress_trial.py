import json,subprocess,time,hashlib,sys
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022')
root=base/(sys.argv[1] if len(sys.argv)>1 else 'v31_confirmed_progress')
assert root.name in ('v31_confirmed_progress','v32_retry_admission','v33_integrated_progress','v34_vehicle_observation','v35_confirmed_vehicle','v36_weak_observation','v37_full_path','v38_rolling_observation','v39_observation_session','v40_observation_heading','v41_general_observation','v42_retirement_boundary','v43_abort_recovery','v44_instance_handoff','v45_approach_contract','v46_approach_feedback','v47_depth_geometry','v48_range_candidate','v49_localization_range','v50_confirmed_continuation','v51_confirmed_action_handoff','v52_approach_local_map','v53_confirmed_graph_destination','v54_confirmed_graph_handoff','v55_frontier_endpoint','v56_confirmed_local_continuity','v57_upstream_target_approach')
report=json.loads((root/'real_ros_report.json').read_text())
assert all(r['passed'] for r in report),report
pre=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],capture_output=True,text=True,check=True,timeout=20).stdout)
assert not pre['processes'] and not pre['display148_lock'],pre
(root/'pretrial_check.json').write_text(json.dumps(pre,indent=2))
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
previous=set((root/'full').glob('trial_*'))
print(json.dumps(dict(event='launch',version=root.name,precheck=pre,real_ros_tests=sum(r['tests'] for r in report))),flush=True)
with (root/'field_runner.log').open('w') as log:
    run=subprocess.run(['bash',str(root/'run_full.sh')],stdout=log,stderr=subprocess.STDOUT)
created=set((root/'full').glob('trial_*'))-previous
assert len(created)==1,created
trial=next(iter(created));result=json.loads((trial/'result.json').read_text())
post=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True,timeout=20).stdout)
receipt=dict(returncode=run.returncode,trial=str(trial),result=result,postcheck=post)
assert result['closed'] and not post['processes'] and not post['display148_lock'],receipt
with (root/'field_metrics.log').open('w') as log:
    subprocess.run(['bash','/tmp/audit_closed_v19.sh',str(trial)],check=True,stdout=log,stderr=subprocess.STDOUT)
receipt['fluency']=json.loads((trial/'fluency_summary.json').read_text())
receipt['attribution']=json.loads((trial/'control_stop_attribution.json').read_text())
(root/'field_result.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2),flush=True)
