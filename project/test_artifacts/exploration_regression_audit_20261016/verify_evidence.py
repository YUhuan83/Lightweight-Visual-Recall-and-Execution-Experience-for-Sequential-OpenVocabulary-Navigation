from pathlib import Path
from collections import Counter
import hashlib,json,math,zipfile

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
meta=json.loads((OUT/'archive_meta.json').read_text(encoding='utf8'))
archive=OUT/'closed_audit_evidence.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==meta['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('archive_manifest.json'))
    assert set(z.namelist())==set(manifest)|{'archive_manifest.json'}
    for name,row in manifest.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        blob=z.read(name)
        assert len(blob)==row['size'] and hashlib.sha256(blob).hexdigest()==row['sha256']
    z.extractall(OUT/'server')
report=json.loads((OUT/'server/closed_audit.json').read_text(encoding='utf8'))
for name,digest in report['critical_runtime_source_hashes'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest
assert report['production_source_sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
assert report['decision_crosscheck']['selected_goal_blocked_in_reconstruction']==0
snapshots=report['snapshots']; q=snapshots[0]['received_ns']-round(snapshots[0]['elapsed_s']*1e9)
decisions=report['decisions']
gaps=sorted([dict(start_s=(a['stamp_ns']-q)/1e9,end_s=(b['stamp_ns']-q)/1e9,
    gap_s=(b['stamp_ns']-a['stamp_ns'])/1e9) for a,b in zip(decisions,decisions[1:])],
    key=lambda x:x['gap_s'],reverse=True)
events=[json.loads(l) for l in (ROOT/'test_artifacts/frontier_cost_repair_20261016/server/wildos-console/audit_results/session_20261016_145143/goal_001/events.jsonl').read_text(encoding='utf8').splitlines()]
correlations=[]
for rejection in report['accepted_rejections']:
    choices=[e for e in events if e.get('exploration_id')==rejection['identity'] and e.get('status') in
        ('follow_failed','no_vehicle_path','no_free_graph_candidate','recovery_started')]
    event=min(choices,key=lambda e:abs(e['stamp_ns']-rejection['now_ns']))
    assert abs(event['stamp_ns']-rejection['now_ns'])<50_000_000
    correlations.append(dict(identity=rejection['identity'],elapsed_s=(rejection['now_ns']-q)/1e9,
        status=event['status'],reason=event.get('reason'),detail=event.get('detail'),
        goal_distance_from_failure_m=math.dist(rejection['goal'],rejection['failure_position']),
        goal_distance_from_rejected_endpoint_m=math.dist(rejection['goal'],rejection['endpoint'])))
zero=[s for s in snapshots if s['replay_counts']['region_blocked']==s['replay_counts']['eligible']]
summary=dict(archive_sha256=meta['sha256'],archive_members_verified=len(manifest),
    critical_sources_verified=len(report['critical_runtime_source_hashes']),
    cpp_decision_crosscheck=report['decision_crosscheck'],exact_topology_joins=len(snapshots),
    unmatched_topologies=report['unmatched_topology_samples'],all_region_blocked_samples=len(zero),
    heading_gate_veto_node_frame_counts=report['aggregate_node_frame_counts']['low_confidence_heading_veto'],
    total_node_frames=report['aggregate_node_frame_counts']['frontiers'],
    edge_backoff_disconnected_known_targets=sum(s['replay_counts']['disconnected_by_edge_backoff'] for s in snapshots),
    first_rejection_expiry_s=(report['accepted_rejections'][0]['now_ns']-q)/1e9+120.,
    longest_route_publication_gap=gaps[0],rejection_correlations=correlations,
    representative_snapshots=[{k:s[k] for k in ('index','elapsed_s','counts','replay_counts')}
        for s in (min(snapshots,key=lambda s:abs(s['elapsed_s']-240.4)),snapshots[-1])],
    scope='Audit reproduction only. Complete graph paths do not prove physically feasible vehicle trajectories; not a new simulator trial or repaired-system acceptance.')
(OUT/'verification.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k!='rejection_correlations'},indent=2))
