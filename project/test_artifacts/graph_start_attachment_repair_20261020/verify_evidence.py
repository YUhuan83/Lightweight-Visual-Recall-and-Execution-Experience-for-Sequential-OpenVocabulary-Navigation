"""Verify repair source, old/new real tests, closed field identity and physical stop."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile

root=Path(__file__).resolve().parent;repo=root.parents[1]
identity=json.loads((root/'sealed/archive.json').read_text());archive=root/'sealed/graph_start_evidence.zip'
assert archive.stat().st_size==identity['size'] and hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('evidence_manifest.json'))
    for name,item in manifest['files'].items():
        data=z.read(name);assert len(data)==item['size'] and hashlib.sha256(data).hexdigest()==item['sha256'],name
    frozen=json.loads(z.read('frozen_source_manifest.json'))
    for name,digest in frozen['members'].items():
        assert hashlib.sha256(z.read('candidate/'+name)).hexdigest()==digest,name
        assert hashlib.sha256((repo/name).read_bytes()).hexdigest()==digest,name
    before=ET.fromstring(z.read('baseline.xml'));after=ET.fromstring(z.read('after.xml'))
    assert (int(before.attrib['tests']),int(before.attrib['failures']),int(before.attrib['errors']))==(30,2,0)
    assert (int(after.attrib['tests']),int(after.attrib['failures']),int(after.attrib['errors']))==(30,0,0)
    trial='full/trial_20261020_215010';session=trial+'/sessions/session_20261020_215010'
    result=json.loads(z.read(trial+'/result.json'));assert result['closed'] and not result['errors'] and not result['session_result']['errors']
    assert result['goal_result']['state']=='timeout' and not result['harness_arrival']
    runtime=json.loads(z.read(trial+'/runtime_identity.json'));config=json.loads(z.read(trial+'/config.json'))
    assert all(runtime['libraries'][n]['sha256']==digest for n,digest in config['library_sha256'].items())
    assert all(runtime['model']['stores'].values())
    graph=json.loads(z.read('graph_progress_final.json'));assert len(graph['snapshots'])==171 and not graph['observer_errors']
    assert all(s['physical_reachable']>1 for s in graph['snapshots'])
    audit=json.loads(z.read(trial+'/closed_audit.json'));assert 87<audit['net_displacement_m']<89 and 145<audit['traveled_distance_m']<147
    stop=json.loads(z.read(session+'/goal_001/stop.json'));physics=stop['physics']
    assert len(stop['samples'])>=5 and all(s['speed']<.03 for s in stop['samples'])
    assert physics['nodeCount']==723 and physics['bodyCollisionNodeCount']==467 and not physics['collisionOverrideLoaded'] and not physics['configKinematic']
    assert physics['speed']<.03 and physics['wheelCount']==4 and all(w['collisionNodeCount']==32 for w in physics['wheels'])
    image=z.read(session+'/goal_001/failure_last_rgb.png');(root/'failure_trial215010.png').write_bytes(image)
    report=dict(verified=True,payloads=len(manifest['files']),source_members=len(frozen['members']),
                archive_sha256=identity['sha256'],baseline_tests=30,baseline_failures=2,after_tests=30,after_failures=0,
                exact_graph_snapshots=171,isolated_start_snapshots=0,net_displacement_m=audit['net_displacement_m'],
                traveled_distance_m=audit['traveled_distance_m'],stop_speed_mps=physics['speed'],
                field_state='timeout',complete_navigation_passed=False)
(root/'local_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
