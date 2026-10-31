from pathlib import Path
import json,subprocess,zipfile,hashlib

r=Path('/home/ly/exploration_scope_repair_20261016');old=Path('/home/ly/exploration_regression_audit_20261016')
replay=r/'replay';replay.mkdir(exist_ok=True)
cmake=(old/'CMakeLists.txt').read_text().replace('/home/ly/wildos-console/bundle',str(r/'candidate')).replace('/home/ly/frontier_cost_repair_20261016/install',str(r/'install'))
(replay/'CMakeLists.txt').write_text(cmake)
(replay/'backoff_replay.cpp').write_bytes((old/'backoff_replay.cpp').read_bytes())
subprocess.run(['cmake','-S',str(replay),'-B',str(replay/'build')],check=True,stdout=open(r/'replay_configure.log','w'),stderr=subprocess.STDOUT)
subprocess.run(['cmake','--build',str(replay/'build'),'-j2'],check=True,stdout=open(r/'replay_build.log','w'),stderr=subprocess.STDOUT)
result=subprocess.run([str(replay/'build/backoff_replay')],input=(old/'replay_commands.txt').read_text(),text=True,capture_output=True,check=True)
(r/'replay_after.log').write_text(result.stdout)
(r/'replay_after_stderr.log').write_text(result.stderr)
report=json.loads((old/'closed_audit.json').read_text());bytime={s['received_ns']:s for s in report['snapshots']}
samples=[]
for line in result.stdout.splitlines():
 p=line.split()
 if not p or p[0]!='P':continue
 v=[list(map(int,x.split(':'))) for x in p[3:]]
 s=bytime[int(p[1])]
 samples.append(dict(elapsed_s=s['elapsed_s'],old=s['replay_counts'],
  new=dict(eligible=len(v),postponed=sum(x[1] for x in v),remaining_known=sum(not x[1] and x[5] for x in v))))
assert len(samples)==62
key=min(samples,key=lambda s:abs(s['elapsed_s']-240.4));assert key['new']['remaining_known']>0
final=samples[-1];assert final['new']['remaining_known']>0
summary=dict(scope='Same closed recorded graphs and observed rejection sequence; counterfactual filtering comparison, not a new physical trajectory or vehicle reachability proof.',samples=samples,
 old_all_postponed=sum(s['old']['region_blocked']==s['old']['eligible'] for s in samples),
 new_all_postponed=sum(s['new']['postponed']==s['new']['eligible'] for s in samples),
 representative=[key,final])
(r/'replay_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='samples'},indent=2))
