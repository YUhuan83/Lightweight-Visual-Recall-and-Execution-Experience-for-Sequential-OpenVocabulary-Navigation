from pathlib import Path
import hashlib,json,zipfile,os
r=Path('/home/ly/exploration_owner_repair_20261016');prod=Path('/home/ly/wildos-console')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text());z=r/'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(z.read_bytes()).hexdigest()==m['sha256']
compiled_source=r/'frozen_da6aeea0'
for n,h in m['members'].items():
 if n!='docs/SELF_SERVICE_CONSOLE.md':assert hashlib.sha256((compiled_source/n).read_bytes()).hexdigest()==h,n
identity=json.loads((prod/'source_identity.json').read_text());assert identity['sha256']=='9bcaf8700786cbaf2ab40a22405121f20288fae5e8f51d6d777fbf5eb3d87997'
for n,h in identity['members'].items():assert hashlib.sha256((prod/'bundle'/n).read_bytes()).hexdigest()==h,n
processes=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:args=(p/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
 if any(Path(arg).name in ('planner_node','rviz2','BeamNG.tech.x64','wildos_console.py') for arg in args[:3]):processes.append(dict(pid=p.name,args=args))
assert not processes,processes
report=dict(frozen_source=m['sha256'],member_count=len(m['members']),component_test_source='da6aeea0a0f486c7ce7349d405efeca9f4eb02409756abf39a03f04c7ef5c078',only_difference='docs/SELF_SERVICE_CONSOLE.md',all_production_and_test_source_bytes_match=True,production_identity=identity['sha256'],production_members_verified=len(identity['members']),production_unchanged=True,residual_project_processes=processes,scope='Independent installed GraphNav + Bridge, synthetic ROS graph and odometry. No BeamNG run or search acceptance.')
(r/'freeze_attestation.json').write_text(json.dumps(report,indent=2))
files=[p for p in r.iterdir() if p.is_file() and p.suffix in ('.py','.sh','.json','.log')]
files += [r/'final_build/graphnav_planner/test_results/graphnav_planner/test_route_mechanism.gtest.xml']
manifest={p.relative_to(r).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(r/'component_evidence.zip','w',zipfile.ZIP_DEFLATED) as archive:
 for p in files:archive.write(p,p.relative_to(r).as_posix())
 archive.writestr('EVIDENCE_MANIFEST.json',json.dumps(manifest,indent=2))
meta=dict(sha256=hashlib.sha256((r/'component_evidence.zip').read_bytes()).hexdigest(),members=len(manifest)+1)
(r/'component_evidence_meta.json').write_text(json.dumps(meta,indent=2));print(meta,report)
