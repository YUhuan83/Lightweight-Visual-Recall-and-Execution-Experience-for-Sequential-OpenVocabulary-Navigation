"""Review the closed brake-only contact recorder, including sensor presence."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
import sys

run = Path(sys.argv[1]); result = json.loads((run/'result.json').read_text())
assert result['passed'] and result['errors'] == []
probe = json.loads((run/'contact_result.json').read_text())
nodes = [n for s in probe['samples'] for n in (s['contacts']['nodes'] or [])]
positions = [n['peak_position'] for n in nodes if n.get('peak_position')]
normals = [n['peak_normal'] for n in nodes if n.get('peak_normal')]
body = [n for n in nodes if n['class'] == 'body']
wheel = [n for n in nodes if n['class'] == 'wheel']
traffic = json.loads((run/'traffic.json').read_text())
watchdogs = [v for v in traffic['native'] if v['watchdog_stop']]
timing = [json.loads(line) for line in (run/'native_timing.jsonl').read_text().splitlines()]
callback_age = sorted((r['callback_wall_ns']-r['issued_wall_ns'])/1e6 for r in timing if r.get('issued_wall_ns'))
db = next((run/'interfaces').glob('*.db3'))
con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
try:
    counts = {name: n for name, n in con.execute('SELECT topics.name,count(messages.id) FROM topics LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id')}
finally: con.close()
report = dict(scope=__doc__, source_run=str(run), component_passed=probe['passed'],
    fixture_scope='11 Lua 5.3 interface/ownership tests; separate from this real BeamNG lifecycle',
    event_totals=probe['event_totals'], samples=len(probe['samples']),
    unload=probe.get('unload'), maximum_speed=probe['maximum_native_speed'],
    positive_drive_commands=probe['positive_drive_commands'], watchdog_samples=probe['watchdog_samples'],
    observer_watchdog_frames=[dict(command_receipt_age=v.get('command_receipt_age'),
                                  receipt_present=v.get('source_receipt') is not None,
                                  telemetry_ok=v.get('telemetry_ok'), telemetry_age_sec=v.get('telemetry_age_sec'),
                                  actuator_ok=v.get('actuator_ok'), control_acknowledged=v.get('control_acknowledged'),
                                  command_age=v.get('command_age'),
                                  sent=v['sent']) for v in watchdogs],
    native_callback_source_age_ms=dict(n=len(callback_age),p95=callback_age[int(.95*len(callback_age))],maximum=callback_age[-1]) if callback_age else None,
    body_node_samples=len(body), wheel_node_samples=len(wheel),
    body_max_normal_force=max([n['max_normal_force'] for n in body], default=None),
    wheel_max_normal_force=max([n['max_normal_force'] for n in wheel], default=None),
    raw_callback_position_ranges={a:[min(p[a] for p in positions), max(p[a] for p in positions)] for a in ('x','y','z')} if positions else None,
    raw_callback_normal_z_range=[min(p['z'] for p in normals), max(p['z'] for p in normals)] if normals else None,
    sim_window=[probe['samples'][0]['contacts']['begin_sim_time'],probe['samples'][-1]['contacts']['end_sim_time']],
    recorded_topic_counts=counts, database_bytes=db.stat().st_size,
    measurement_clock_mapping_topic_recorded=counts.get('/spot1/measurement_timing',0)>0,
    cross_stream_clock_mapping_verified=False,
    database_sha256=hashlib.sha256(db.read_bytes()).hexdigest(),
    model_mass_kg=probe['physics_after']['massKg'],
    body_collision_nodes=probe['physics_after']['bodyCollisionNodeCount'],
    loaded_library_identity=json.loads((run/'running_identity.json').read_text()),
    complete_navigation_passed=False, historical_stall_reproduced=False,
    contact_object_identity_proven=False, runtime_vector_frame_proven=False)
for topic in ('/vehicles/ego/sensors/front_lidar','/spot1/ouster/front/points_filtered',
              '/wildos/points','/wildos/clearing_points','/wildos/diagnostic/contact_snapshot'):
    assert counts.get(topic,0)>0, 'Recorder missing '+topic
assert probe['passed'] and probe['unload']['unloaded'] is True
assert probe['positive_drive_commands']==0 and probe['maximum_native_speed']<.05
(run/'review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='loaded_library_identity'},indent=2))
