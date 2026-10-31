"""Consistency gates for recorded evidence, not a navigation success test."""
import json
from pathlib import Path
import sys

run=Path(sys.argv[1]);report=json.loads((run/'review_recorded.json').read_text())
probe=json.loads((run/'contact_result.json').read_text())
checks={}
checks['failed_execution_not_claimed_as_success']=(report['outcome']=='execution_stalled' and report['execution_passed'] is False and report['complete_navigation_passed'] is False)
w=report['stall_window']
checks['actual_actuation_with_exact_source_receipts']=(w['samples']==100 and w['source_matches']==100 and not w['invalid'] and w['brake']['maximum']==0 and w['gears']=={'D':100} and w['watchdogs']==0)
checks['body_contact_started_after_arm']=(report['body_contacts']['before_motion_events']==0 and report['body_contacts']['events']>0)
checks['bounded_observer_windows']=(
    all(s['contacts']['invalid_events']==0 and s['contacts']['truncated_nodes']==0
        and s['contacts']['dropped_distinct_events']==0 for s in probe['contacts'])
    and [s['contacts']['window_sequence'] for s in probe['contacts']]
        ==list(range(1,len(probe['contacts'])+1)))
c=report['clock_mapping']
checks['recorded_state_to_adapter_clock_mapping']=(c['state_records']==c['state_source_recorded']==c['mapped_odom_recorded']==c['offset_equal'] and c['state_records']>0 and c['resets']==0)
checks['no_unproven_sim_or_collision_identity_claim']=(c['exact_moving_sdk_payload_anchor'] is None and report['body_contacts']['object_identity_proven'] is False and report['body_contacts']['full_engine_vector_contract_verified'] is False)
checks['six_actual_filter_replays_match_recorded_output']=all(s[k]['exact_shape'] and s[k]['maximum_point_error']==0 for s in report['snapshots'] for k in ('marking_replay','clearing_replay')) and len(report['snapshots'])==6
early=report['snapshots'][:2]
checks['earlier_returns_lost_at_height_gate']=all(s['roi_filter_stages']['before_footprint']==s['roi_filter_stages']['after_footprint']==116 and s['roi_filter_stages']['after_body_height']==0 and s['roi_filter_stages']['body_z']['maximum']<.1 and s['roi_filter_stages']['passing_ground_clearance_if_height_gate_removed']==0 and s['marking']['within_xy_40cm']==0 and s['clearing']['within_xy_40cm']>0 for s in early)
checks['public_maps_allow_prefix_while_body_contact_exists']=all(not value['footprint_blockers'] and value['forward_prefix_issue'] is None for s in report['snapshots'] for value in s['public_maps'].values())
checks['bridge_and_visual_not_part_of_this_replay']=(report['visual_handoff_present'] is False and report['graph_bridge_follow_path_enabled'] is False)
checks['source_identity_and_closed_stop']=(all(row['equals_frozen_member'] for row in report['analysis_source_identity'].values()) and report['unloaded'] is True and report['stop_speed']<.05 and len(probe['stop']['samples'])>=2 and probe['stop']['samples'][-1]['elapsed']-probe['stop']['samples'][0]['elapsed']>=.5)
result=dict(scope=__doc__,passed=all(checks.values()),checks=checks,complete_navigation_passed=False)
(run/'review_consistency.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2));raise SystemExit(not result['passed'])
