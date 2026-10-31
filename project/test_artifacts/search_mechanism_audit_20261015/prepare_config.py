from pathlib import Path
import json,hashlib
r=Path('/home/ly/wildos-console');c=json.loads((r/'console.json').read_text())
c.update(display=':148',rviz=True,goal_timeout_s=120,output_root=str(r/'audit_results'))
assert not (r/'audit_console.json').exists();(r/'audit_console.json').write_text(json.dumps(c,indent=2))
prefix='wildos_port/repro_check/src/nebula2-wildos/'
base=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate/bundle')
files=[*list((r/'bundle'/prefix/'graphnav_planner').rglob('*.cpp')),*list((r/'bundle'/prefix/'graphnav_planner').rglob('*.hpp'))]
diff=[]
for p in files:
 old=base/p.relative_to(r/'bundle')
 if not old.is_file() or old.read_bytes()!=p.read_bytes():diff.append(str(p.relative_to(r/'bundle')))
report=dict(base_source=str(base),current_source=str(r/'bundle'),graphnav_source_files=len(files),different_files=diff)
Path('/home/ly/search_mechanism_audit_20261015/graphnav_source_comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
