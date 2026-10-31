"""Seal terminal approach-recovery tests, source identities and field evidence."""
from pathlib import Path
import hashlib
import json
import sqlite3
import zipfile

root = Path(__file__).resolve().parent
trial = root/'full/trial_20261020_224538'
driver = json.loads((trial/'result.json').read_text()); assert driver['closed']
session = Path(driver['session'])
assert (session/'goal_001/interfaces/metadata.yaml').is_file()
assert not json.loads((root/'poststart.json').read_text())['processes']
db = session/'goal_001/interfaces/interfaces_0.db3'
con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
topic_counts = dict(con.execute('select t.name,count(m.id) from topics t left join messages m on t.id=m.topic_id group by t.id'))
con.close()
vision = (session/'vision.log').read_text()
changed = "Object-search target changed: 'door' -> 'green residential house' (encoded text features)."
assert changed in vision
active = vision.split(changed, 1)[1]
visual = dict(topic_counts=topic_counts, query_encoded=True,
              received_callbacks_after_query=active.count('Received callback'),
              tf_found_after_query=active.count('TF found'),
              no_object_after_query=active.count('No object detected'),
              reason='No target output; visibility/model correctness not established by this count',
              approach_recovery_exercised_in_field=False,
              complete_navigation_passed=False)
(root/'field_visual_summary.json').write_text(json.dumps(visual, indent=2)+'\n')

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        while data := f.read(4*1024*1024): h.update(data)
    return h.hexdigest()
out = root/'sealed'; out.mkdir(exist_ok=False)
files = {}; references = []
def add(path, name=None):
    if path.is_file(): files[name or str(path.relative_to(root))] = path
for p in root.iterdir():
    if p.is_file() and p.suffix in ('.py','.sh','.json','.log'): add(p)
parent = root.parent
for p in parent.iterdir():
    if p.is_file() and p.suffix in ('.py','.json','.log'): add(p, 'initial_candidate/'+p.name)
for p in (root/'before').rglob('*.py'): add(p)
frozen = json.loads((root/'frozen_source_manifest.json').read_text())
for name, digest in frozen['members'].items():
    p = root/'candidate'/name; assert sha(p) == digest, name; add(p)
for path in sorted(root.glob('dds_*')):
    if path.is_dir():
        for p in path.rglob('*'):
            if p.is_file(): add(p)
excluded = {'native_timing.jsonl', 'native_udp.jsonl', 'sensor_timing.jsonl', 'native_samples.jsonl'}
for p in trial.rglob('*'):
    if not p.is_file(): continue
    if p.suffix == '.db3' or p.name in excluded or 'graph_pairs' in p.parts:
        # Large originals remain on the server with a content identity.
        references.append(dict(path=str(p), size=p.stat().st_size, sha256=sha(p))); continue
    if p.suffix in ('.json','.jsonl','.log','.png','.yaml'): add(p)
for p in (root/'full').iterdir():
    if p.is_file() and p.suffix in ('.py','.json'): add(p)
for original in (Path('/home/ly/confirmed_approach_recovery_20261020.zip'),
                 Path('/home/ly/confirmed_approach_recovery_20261020_v2.zip')):
    references.append(dict(path=str(original), size=original.stat().st_size, sha256=sha(original)))
manifest = dict(scope=__doc__, files={n: dict(size=p.stat().st_size, sha256=sha(p)) for n,p in files.items()},
                references=references, complete_navigation_passed=False)
(out/'evidence_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
archive = out/'confirmed_recovery_evidence.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
    for name, path in files.items(): z.write(path, name)
    z.write(out/'evidence_manifest.json', 'evidence_manifest.json')
report = dict(size=archive.stat().st_size, sha256=sha(archive), payloads=len(files), references=len(references))
(out/'archive.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report))
