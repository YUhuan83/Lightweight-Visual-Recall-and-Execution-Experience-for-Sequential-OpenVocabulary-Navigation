import hashlib,json,math
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/observation_heading_20261026'
scope=json.loads((root/'source_scope.json').read_text(encoding='utf8'))
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in scope['changed'].items())
for i in (1,2):
    log=(root/f'before_{i}.log').read_text(encoding='utf8')
    assert 'Ran 4 tests' in log and 'FAILED (failures=3)' in log
    log=(root/f'related_{i}.log').read_text(encoding='utf8')
    assert 'Ran 99 tests' in log and '\nOK\n' in log
assert '"tests": 828' in (root/'offline_full.log').read_text(encoding='utf8')
assert '"passed": true' in (root/'offline_full.log').read_text(encoding='utf8')
ros=json.loads((root/'field/real_ros_report.json').read_text(encoding='utf8'))
assert all(r['passed'] for r in ros)
r=json.loads((root/'field/field_result.json').read_text(encoding='utf8'))
a=json.loads((root/'field/continuous_path_audit.json').read_text(encoding='utf8'))
p=json.loads((root/'field/field_phase_audit.json').read_text(encoding='utf8'))
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
assert not a['curvature_bound_violations'] and not a['in_place_rotation_commands']
assert set(a['execution_modes'])=={'continuous_full_path_v2'}
assert (root/'field/displacement_audit_1.json').read_bytes()==(root/'field/displacement_audit_2.json').read_bytes()
rows=[]
for parent,field in [('full_path_control_20261026','field'),('full_path_control_20261026','field_repeat'),('rolling_observation_20261026','field'),('observation_session_20261026','field'),('observation_heading_20261026','field')]:
    q=repo/'test_artifacts'/parent/field;receipt=json.loads((q/'field_result.json').read_text(encoding='utf8'))
    phase=json.loads((q/'field_phase_audit.json').read_text(encoding='utf8'))
    f=receipt['fluency'];rows.append(dict(trial=Path(receipt['trial']).name,parent=parent,
        near_stop_pct=f['near_stop_pct'],longest_near_stop_s=f['longest_near_stop_s'],net_m=f['net_displacement_m'],path_m=f['sampled_distance_m'],
        masks=phase['masks'],valid_localizations=phase['valid_localizations'],confirmed=phase['confirmed'],full_pass=receipt['result']['complete_navigation_passed']))
report=dict(candidate='v40_observation_heading',changed=scope['changed'],related_tests=99,related_repeats=2,
    counterexample_tests=4,before_failures_per_run=3,real_ros_tests=sum(x['tests'] for x in ros),original_offline_tests=828,
    actual_cpp_plugin_tests=26,cpp_repeats=2,field_trials=rows,
    main_runtime_overwritten=False,commit_created=False,full_acceptance=False,promoted=False,
    acceptance_reasons=['Complete search timed out','Corrected confirmed-recovery branch not exercised in v40 field','One fluency result does not establish stable non-regression or causality'],
    metrics_scope='Historical whole Native sampling window includes a few prequery seconds; same algorithm for every row. Driver v38+ actual query search budget 180 seconds.',
    current_remaining=['Sparse reliable target geometry','Intermediate observation/failed approach handoff','Serial candidate planning stops','Collision-triggered recovery and route variance'])
(root/'validation_summary.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
