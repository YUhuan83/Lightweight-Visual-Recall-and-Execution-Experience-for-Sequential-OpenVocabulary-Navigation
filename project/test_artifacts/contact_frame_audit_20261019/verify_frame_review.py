"""Evidence coverage/identity checks; failed execution remains failed."""
from pathlib import Path
import json
import sys

run=Path(sys.argv[1]);r=json.loads((run/'frame_review.json').read_text())
probe=json.loads((run/'contact_result.json').read_text())
events=[json.loads(s) for s in (run/'contact_result.events.jsonl').read_text().splitlines()]
checks={}
checks['failed_execution_kept_failed']=(r['outcome']=='execution_stalled' and r['complete_navigation_passed'] is False)
c=r['contact_coordinates']
checks['independent_body_coordinate_agreement']=(c['body_nodes']>0 and c['body_world_errors_m']['maximum']<.005 and not c['reference_errors'] and all(n['relative_hypothesis_error_m']>100 for n in c['body_records']))
checks['same_callback_sim_readings']=(c['event_reference_time_errors_sec']['minimum']==c['event_reference_time_errors_sec']['maximum']==0)
checks['explicit_bridge_anchor_recorded']=(type(r['bridge_clock']['anchor_ros_ns']) is int and 0<r['bridge_clock']['state_engine_seconds_range'][0]<r['bridge_clock']['state_engine_seconds_range'][1]<100)
checks['true_acquisition_time_not_claimed']=(r['bridge_clock']['lidar_true_acquisition_time_verified'] is False)
checks['five_actual_filter_replays_match']=len(r['cloud_snapshots'])==5 and all(s[k]['shape_equal'] and s[k]['max_error']==0 for s in r['cloud_snapshots'] for k in ('marking_replay','clearing_replay'))
checks['independently_located_early_returns_removed_at_height_gate']=all(s['adapted']['count']==s['outside_footprint']['count']==116 and s['after_height']['count']==s['marking']['count']==0 and s['clearing']['count']>0 for s in r['cloud_snapshots'][:2])
w=r['stall_window']
checks['actual_drive_receipts_and_no_braking']=(w['n']==w['source_matches'] and w['n']>0 and w['brake']['maximum']==0 and w['gears']=={'D':w['n']} and w['watchdog']==0)
new=r['repeated_raw_payload_new_header']
checks['duplicate_payload_new_header_while_moving_recorded']=(new['moving_pairs']>0 and all(p['previous_source_ns']!=p['current_source_ns'] and p['sim_header_advance_sec']>0 for p in new['records']))
accepted=next(e for e in events if e['kind']=='follow_accepted')
ready=next(e for e in events if e['kind']=='enable_current_owner_confirmed')
checks['current_enable_generation_confirmed_before_action']=(ready['wall_ns']<accepted['wall_ns'] and ready['control']['source_control']['generation']==ready['native']['source_receipt']['generation']==r['enable_barrier']['expected_generation'] and ready['control']['source_ack_ready'] is True)
checks['safe_stop_and_unload']=(r['unload']['unloaded'] is True and r['stop']['physics']['speed']<.05 and probe['errors']==[] and probe['protocol_closed'])
out=dict(scope=__doc__,checks=checks,passed=all(checks.values()),complete_navigation_passed=False)
(run/'frame_review_consistency.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2));raise SystemExit(not out['passed'])
