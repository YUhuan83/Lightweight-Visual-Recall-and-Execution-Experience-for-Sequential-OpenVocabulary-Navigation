import hashlib,json,re
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/full_path_control_20261026'
scope=json.loads((root/'source_scope.json').read_text(encoding='utf8'))
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in scope['changed'].items())
for name in ('related_1.log','related_2.log'):
    text=(root/name).read_text(encoding='utf8');assert 'Ran 70 tests' in text and '\nOK\n' in text
for name in ('offline_full_1.log','offline_full_2.log'):
    text=(root/name).read_text(encoding='utf8');assert '"tests": 828' in text and '"passed": true' in text
for name in ('cpp_after_1.log','cpp_after_2.log'):
    assert '[  PASSED  ] 26 tests.' in (root/'field'/name).read_text(encoding='utf8')
ros=json.loads((root/'field/real_ros_report.json').read_text(encoding='utf8'))
assert all(r['passed'] for r in ros)
fields=[]
for name in ('field','field_repeat'):
    r=json.loads((root/name/'field_result.json').read_text(encoding='utf8'))
    audit=json.loads((root/name/'continuous_path_audit.json').read_text(encoding='utf8'))
    assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
    assert not audit['curvature_bound_violations'] and not audit['in_place_rotation_commands']
    assert set(audit['execution_modes'])=={'continuous_full_path_v2'}
    assert (root/name/'displacement_audit_1.json').read_bytes()==(root/name/'displacement_audit_2.json').read_bytes()
    fields.append(dict(trial=r['trial'],closed=True,complete_navigation_passed=False,
        near_stop_pct_legacy=r['fluency']['near_stop_pct'],longest_near_stop_s=r['fluency']['longest_near_stop_s'],
        net_m=r['fluency']['net_displacement_m'],path_m=r['fluency']['sampled_distance_m'],
        internal_direction_transitions=audit['internal_direction_transitions']))
result=dict(candidate='v37_full_path',old_versions_preserved=True,main_runtime_overwritten=False,
    related_tests=70,related_repeats=2,original_offline_tests=828,offline_repeats=2,
    actual_cpp_plugin_tests=26,cpp_repeats=2,real_ros_tests=sum(r['tests'] for r in ros),
    field_trials=fields,full_acceptance=False,commit_created=False,
    search_timing_limit='Driver starts 180s at console input; actual query delay remains to correct; query-window audit saved separately')
(root/'validation_summary.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
