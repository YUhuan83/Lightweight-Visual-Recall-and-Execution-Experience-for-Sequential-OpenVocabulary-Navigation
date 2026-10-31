"""Audit the closed trial and collect compact evidence, leaving raw bags intact."""
from pathlib import Path
import hashlib
import json
import sqlite3
import subprocess
import tarfile
import sys

root = Path(sys.argv[1] if len(sys.argv)>1 else '/home/ly/execution_continuity_review_20261022/v1')
trial = Path(sys.argv[2]) if len(sys.argv)>2 else root/'full/trial_20261022_155227'
sessions=list((trial/'sessions').glob('session_*'));assert len(sessions)==1
session = sessions[0]
goal = session / 'goal_001'
assert json.loads((trial/'result.json').read_text())['closed']
audit = trial/'fluency_audit'
audit.mkdir(exist_ok=True)
for window in (180,420):
    subprocess.run(['python3', str(root.parent/'incoming/analyze_closed_trial.py'),
                    str(goal), str(audit/f'summary_{window}.json'), '--window',str(window),
                    '--stack',str(session/'stack.log')],check=True,capture_output=True,text=True)
bags=[]
topics=[]
for bag in goal.rglob('*.db3'):
    with sqlite3.connect(bag.resolve().as_uri()+'?mode=ro',uri=True) as db:
        topics.extend(db.execute('select t.name,t.type,count(*) from messages m join topics t on t.id=m.topic_id group by t.id').fetchall())
    bags.append({'path':str(bag),'bytes':bag.stat().st_size,'sha256':hashlib.sha256(bag.read_bytes()).hexdigest()})
(audit/'bags_and_topics.json').write_text(json.dumps({'bags':bags,'topics':topics},indent=2))
names=[root/'deployment_receipt.json',root/'installed_identity.json',root/'real_ros_report.json',
       root/'build.log',root/'smac_tests.log',root/'full/frozen_source_manifest.json']
names += list(root.glob('mapper*.log'))
names += [p for p in trial.rglob('*') if p.is_file() and p.suffix in ('.json','.jsonl','.log','.png')]
names=sorted(p for p in set(names) if p.is_file())
inventory={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in names}
(audit/'compact_manifest.json').write_text(json.dumps(inventory,indent=2))
archive=root/(trial.name+'_compact.tar.gz')
with tarfile.open(archive,'w:gz') as z:
    for p in names+[audit/'compact_manifest.json']:
        z.add(p,arcname=str(p.relative_to(root)))
receipt={'path':str(archive),'bytes':archive.stat().st_size,
         'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'members':len(names)+1}
(root/'compact_receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt))
print(json.dumps({'graph_topics':[x for x in topics if 'graphnav' in x[1] or '/graph' in x[0]]},indent=2))
