"""Adapt owned diagnostic entry points to the independently frozen candidate."""
from pathlib import Path
root=Path(__file__).parent
out=root/'curvature_candidate1/runtime';out.mkdir(parents=True,exist_ok=True)
old='/home/ly/town_memory_validation_20261014/dds_transport_candidate'
new='/home/ly/town_memory_validation_20261014/curvature_contract_candidate1'
expected='71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660'
verification="""identity=json.loads((BASE/'installation_identity.json').read_text())
        assert json.loads((BASE/'source_identity.json').read_text())['sha256']=='EXPECTED'
        assert identity['files']['libwildos_ackermann_smac.so']==hashlib.sha256((BASE/'install/lib/libwildos_ackermann_smac.so').read_bytes()).hexdigest()
        from wildos_nav2_adapter import graph_route
        assert Path(graph_route.__file__).resolve().is_relative_to(BASE/'install')
        assert identity['files']['graph_route.py']==hashlib.sha256(Path(graph_route.__file__).read_bytes()).hexdigest()""".replace('EXPECTED',expected)
for name in ('prepare_dds_component.py','prepare_dds_search.py','run_dds_probe.py',
             'capture_dds_owners.py','start_dds_full_trial.py','close_dds_trial.py',
             'record_execution_contract.py'):
    text=(root/name).read_text().replace(old,new).replace('official_wildos_beamng_bundle.zip','candidate.zip')
    text=text.replace('06bb47afd4d32df40afd85fef7aa0f11c97e89efff0c9dcf1e120916da4b64d0',expected)
    if name.startswith('prepare_'):
        text=text.replace("assert json.loads((BASE/'installation_identity.json').read_text())['passed']",verification)
    if name=='start_dds_full_trial.py':
        text=text.replace("assert json.loads((BASE/'installation_identity.json').read_text())['passed']",
            "assert json.loads((BASE/'source_identity.json').read_text())['sha256']=='"+expected+"'")
    if name=='capture_dds_owners.py':
        text=text.replace("report={'passed':not errors", """for row in owned:
    if any(Path(t).name=='planner_server' for t in row['tokens']):
        maps=(Path('/proc')/str(row['pid'])/'maps').read_text()
        if str(BASE/'install/lib/libwildos_ackermann_smac.so') not in maps:
            errors.append('Actual planner did not load frozen candidate library')
    if any(Path(t).name=='controller_server' for t in row['tokens']):
        maps=(Path('/proc')/str(row['pid'])/'maps').read_text()
        if '/home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/lib/libwildos_ackermann_rpp.so' not in maps:
            errors.append('Actual controller library identity differs')
report={'passed':not errors""")
    compile(text,name,'exec')
    (out/name).write_bytes(text.encode())
text=(root/'dds_env.sh').read_text().replace(old,new)
text=text.replace('export PYTHONPATH="', 'export PYTHONPATH="$TASK/bundle/tools:')
(out/'dds_env.sh').write_bytes(text.encode())
text=(root/'retire_dds_component.py').read_text().replace(old,new).replace('physical_component_session1','physical_component_session2')
text=text.replace('try:b.control.pause()',"""from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
try:
    snapshot=read_physics_snapshot(b)
    assert not physical_snapshot_errors(snapshot)
    assert snapshot['speed']<.05,'Vehicle not stationary; refuse cleanup'
    b.control.pause()""")
(out/'retire_component.py').write_bytes(text.encode())
print(out)
