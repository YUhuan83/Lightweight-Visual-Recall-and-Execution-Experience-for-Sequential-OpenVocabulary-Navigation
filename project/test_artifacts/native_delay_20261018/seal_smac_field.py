"""Seal only the closed Smac field; keep raw DBs remote with explicit hashes."""
import hashlib,json,subprocess,time,zipfile
from pathlib import Path
root=Path('/home/ly/smoothing_boundary_repair_20261018')
driver=root/'field_native_20261018_164222'
result=json.loads((driver/'result.json').read_text());session=Path(result['session'])
assert (session/'session_result.json').is_file() and result['component_closed']
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text())
for name,want in manifest['members'].items():assert hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()==want,name
inventory=subprocess.run(['python3','/home/ly/native_delay_20261018/domain_inventory.py'],capture_output=True,text=True,check=True)
processes=json.loads(inventory.stdout);assert not processes,processes
(driver/'poststop_inventory.json').write_text(inventory.stdout)
payload={}
for base,prefix in [(driver,'driver'),(session,'session'),(root/'field_native_investigation','field')]:
    for path in base.rglob('*'):
        if path.is_file() and path.suffix.lower() in ('.json','.jsonl','.log','.yaml','.yml','.png','.py','.sh'):
            if '__pycache__' not in path.parts:payload[prefix+'/'+str(path.relative_to(base))]=path
for name in ('ackermann_smac/src/planner.cpp','ackermann_smac/test/test_planner.cpp',
             'nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py',
             'nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
             'nav2_adapter/wildos_nav2_adapter/visible_approach.py'):
    payload['candidate/'+name]=root/'candidate'/name
for name in ('official_wildos_beamng_bundle.manifest.json','planner_tests.log','path_contract_tests.log','connector_tests.log','library_identity.txt','full_query_review.log','positive_source_recovery.log'):
    payload['build/'+name]=root/name
diag=Path('/home/ly/native_delay_20261018')
for name in ('sitecustomize.py',):payload['diagnostic/'+name]=diag/'instrumentation'/name
for name in ('trace_native_udp.c','libnative_rx_trace.so','udp_tracer_validation_v2.log','review_full_smac.py','pair_udp_timing.py','audit_unmatched_positive.py','verify_running_smac.py','prepare_smac_field.py','run_smac_field.sh'):
    payload['diagnostic/'+name]=diag/name
raw=[]
for path in session.rglob('*.db3'):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        while chunk:=handle.read(8*1024*1024):h.update(chunk)
    raw.append(dict(path=str(path),bytes=path.stat().st_size,sha256=h.hexdigest(),scope='Closed raw interfaces DB kept on server; not included in local core archive.'))
index=dict(scope=__doc__,candidate=manifest['sha256'],field_acceptance=False,native_repair_verified=False,
           files={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in payload.items()},
           raw_interfaces=raw,video=False)
archive=root/'smac_full_field_evidence_20261018.zip';assert not archive.exists()
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
    for name,path in payload.items():z.write(path,name)
    z.writestr('evidence_manifest.json',json.dumps(index,indent=2)+'\n')
identity=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payloads=len(payload),raw_interfaces=raw)
(root/'smac_full_field_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(identity,indent=2))
