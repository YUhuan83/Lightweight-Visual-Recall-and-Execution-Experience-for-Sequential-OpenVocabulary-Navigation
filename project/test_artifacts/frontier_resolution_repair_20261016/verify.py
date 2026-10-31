from pathlib import Path
import collections,hashlib,json
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
delta=json.loads((out/'delta.json').read_text(encoding='utf8'))
m=json.loads((root/'deploy/20261016_frontier_resolution_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
assert delta['candidate']==m['sha256']
for name in delta['changed']:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==m['members'][name],name
offline=json.loads((out/'offline_report.json').read_text(encoding='utf8'))
counts=dict(collections.Counter(row['status'] for row in offline['checks']))
assert counts==dict(PASS=50,WARN=6)
evidence=json.loads((out/'field_archive_validation.json').read_text(encoding='utf8'))
assert evidence['crc_passed'] and evidence['verified_members']==109
server=out/'server_field'
identity=json.loads((server/'validation_identity.json').read_text(encoding='utf8'))
assert identity['candidate']==m['sha256'] and identity['production_unchanged']
loaded=json.loads((server/'field/loaded_runtime.json').read_text(encoding='utf8'))
assert loaded['expected_source']==m['sha256']
assert loaded['mapped_libraries']['libgraphnav_planner_component.so']['sha256']==identity['library_sha256']
field=json.loads((server/'field/baseline_summary.json').read_text(encoding='utf8'))
assert field['goal']['state']=='timeout' and field['masks']==0 and field['confirmations']==0
assert not field['session_errors']
assert not (out/'posttest_processes.txt').read_text(encoding='utf8').strip()
result=dict(candidate=m['sha256'],changed=delta['changed'],offline_counts=counts,callback_tests=447,
    server_ros_tests=51,cpp_route_tests=29,dds_tests=dict(existing=9,scope=2,resolution=3),
    library_sha256=identity['library_sha256'],runtime_identity_verified=True,
    evidence=evidence,field=dict(result='FAILED',reason='Search timeout without visual target arrival',
        native_distance_m=field['native_distance_m'],max_departure_m=field['max_departure_m'],
        longest_stop_s=field['longest_stationary_s'],stops=len(field['stationary_intervals_ge5s']),
        retirement_reasons=field['retirement_reasons'],masks=field['masks'],confirmations=field['confirmations']),
    production_unchanged=True,commit_created=False,whole_navigation_acceptance=False)
(out/'verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,indent=2))
