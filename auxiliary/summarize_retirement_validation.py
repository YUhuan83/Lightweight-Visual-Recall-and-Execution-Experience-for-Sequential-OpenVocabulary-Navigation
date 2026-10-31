import hashlib,json
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/retirement_boundary_20261026'
scope=json.loads((root/'source_scope.json').read_text(encoding='utf8'))
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in scope['changed'].items())
for i in (1,2):
    assert 'FAILED (failures=3)' in (root/f'before_{i}.log').read_text(encoding='utf8')
    assert 'Ran 50 tests' in (root/f'related_{i}.log').read_text(encoding='utf8')
    assert '\nOK\n' in (root/f'related_{i}.log').read_text(encoding='utf8')
    assert '[  FAILED  ] 3 tests,' in (root/f'cpp_before_{i}.log').read_text(encoding='utf8')
    assert '[  PASSED  ] 32 tests.' in (root/f'cpp_after_{i}.log').read_text(encoding='utf8')
    ros=json.loads((root/f'real_ros_report_{i}.json').read_text(encoding='utf8'));assert all(r['passed'] for r in ros)
assert '"tests": 828' in (root/'offline_full.log').read_text(encoding='utf8')
assert '"passed": true' in (root/'offline_full.log').read_text(encoding='utf8')
rows=[]
for f in ('field','field_repeat'):
    field=root/f;r=json.loads((field/'field_result.json').read_text(encoding='utf8'))
    a=json.loads((field/'retirement_boundary_audit.json').read_text(encoding='utf8'))
    c=json.loads((field/'continuous_path_audit.json').read_text(encoding='utf8'))
    p=json.loads((field/'field_phase_audit.json').read_text(encoding='utf8'))
    assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
    assert not a['retained_old_future_branch_violations'] and not c['curvature_bound_violations'] and not c['in_place_rotation_commands']
    assert (field/'displacement_audit_1.json').read_bytes()==(field/'displacement_audit_2.json').read_bytes()
    rows.append(dict(trial=r['trial'],near_stop_pct=r['fluency']['near_stop_pct'],max_stop_s=r['fluency']['longest_near_stop_s'],
        path_m=r['fluency']['sampled_distance_m'],net_m=r['fluency']['net_displacement_m'],masks=p['masks'],valid_localizations=p['valid_localizations'],
        retirements=a['retirement_messages'],old_future_branch_violations=0,complete=r['result']['complete_navigation_passed']))
out=dict(candidate='v42_retirement_boundary',changed=scope['changed'],related_tests=50,repeats=2,actual_cpp_tests=32,real_ros_tests=sum(x['tests'] for x in ros),original_offline_tests=828,
    retirement_contract_field_supported=True,field_trials=rows,full_acceptance=False,promoted=False,commit_created=False,main_runtime_overwritten=False,
    acceptance_reasons=['Both complete searches timed out','No reliable target geometry in either round','23.91/16.74% near stop and 4.36/10.25s longest pauses still variable','Second round mostly loops around one exploration owner despite cumulative movement'],
    metrics_scope='Historical Native whole sampling window; v38+ driver query_sent-based search180s. No attribution of entire single-run differences to repair.')
(root/'validation_summary.json').write_text(json.dumps(out,indent=2),encoding='utf8');print(json.dumps(out,indent=2))
