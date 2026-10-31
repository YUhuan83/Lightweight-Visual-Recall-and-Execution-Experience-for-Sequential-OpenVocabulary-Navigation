"""Summarize recorded search evidence without issuing simulator/ROS commands."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import yaml

ROOT=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
OUT=ROOT/'green_house_trial1'


def main():
    full=json.loads((OUT/'full_test.json').read_text())
    events=[json.loads(x) for x in (OUT/'events.jsonl').read_text().splitlines() if x]
    native=[json.loads(x) for x in (OUT/'native_samples.jsonl').read_text().splitlines() if x]
    native=[x for x in native if x.get('phase')=='single_target_search' and x.get('position')]
    statuses=[x for x in events if x.get('kind')=='plan_status']
    failures=[x for x in statuses if x.get('status') in ('follow_failed','no_vehicle_path',
        'execution_unavailable','execution_hold','replanning_handoff','input_unavailable',
        'graph_reset_failed','follow_accept_timeout','follow_feedback_timeout')]
    progress=[x for x in events if x.get('kind')=='search_progress']
    summary={k:full.get(k) for k in ('passed','automatic_arrival','visual_review_pending',
        'failure_reason','search_elapsed_s','travelled_m','stationary_after_stop',
        'simulation_paused','shutdown_errors','final_status','failure_image')}
    summary['event_kind_counts']=dict(Counter(x.get('kind') for x in events))
    summary['plan_status_counts']=dict(Counter(x.get('status') for x in statuses))
    summary['failure_status_events']=failures
    summary['maximum_mask_count']=max((x.get('mask_count',0) for x in progress),default=0)
    summary['native_samples']=len(native)
    summary['maximum_world_departure_m']=max((math.dist(x['position'][:2],(734.294,-11.946))
        for x in native),default=0)
    summary['native_center_travelled_m']=sum(math.dist(a['position'][:2],b['position'][:2])
        for a,b in zip(native,native[1:]))
    summary['maximum_speed_m_s']=max((x.get('speed',0) for x in native),default=0)
    stops=[]; start=None; duration=0
    for a,b in zip(native,native[1:]):
        if a.get('speed',99)<.05:
            dt=b['wall_time']-a['wall_time']
            if 0<=dt<1:
                duration+=dt
                if start is None: start=a['wall_time']
        elif start is not None:
            if duration>=5: stops.append({'start_wall_time':start,'duration_s':duration})
            start=None; duration=0
    if start is not None and duration>=5: stops.append({'start_wall_time':start,'duration_s':duration})
    summary['stationary_intervals_at_least_5s']=stops
    timing=[json.loads(x) for x in (OUT/'sensor_timing.jsonl').read_text().splitlines() if x]
    ages={}
    for x in timing:
        if x.get('stamp_clock')=='ros' and isinstance(x.get('ros_header_age_ns'),(int,float)):
            ages.setdefault(x['topic'],[]).append(x['ros_header_age_ns']/1e9)
    summary['source_age_ros_s']={topic:{'count':len(v),'median':statistics.median(v),
        'p95':sorted(v)[int(.95*(len(v)-1))],'max':max(v),'min':min(v)} for topic,v in ages.items()}
    metadata=yaml.safe_load((OUT/'interfaces/metadata.yaml').read_text())['rosbag2_bagfile_information']
    summary['recorded_topic_message_counts']={x['topic_metadata']['name']:x['message_count']
        for x in metadata['topics_with_message_count']}
    summary['final_physics']={k:full.get('physics_after',{}).get(k) for k in ('speed','position',
        'nodeCount','fixedNodeCount','bodyCollisionNodeCount','wheelCount','wheels','mission')}
    text=(ROOT/'green_house_session1/stack.log').read_text(errors='replace')
    summary['controller_log_counts']={phrase:text.count(phrase) for phrase in (
        'Ackermann lookahead would reverse','Failed to make progress','collision ahead',
        'Control loop missed its desired rate','Feedback received','Received a goal',
        'Reached the goal','excess_curvature')}
    summary['production_candidate_sha256']=hashlib.sha256(Path(
        '/home/ly/town_memory_validation_20261014/dds_transport_candidate/official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()
    (OUT/'analysis_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    display={k:v for k,v in summary.items() if k not in ('failure_status_events','source_age_ros_s',
        'recorded_topic_message_counts','event_kind_counts','final_physics','final_status')}
    display['first_failure_status']=failures[0] if failures else None
    print(json.dumps(display,indent=2),flush=True)


if __name__=='__main__': main()
