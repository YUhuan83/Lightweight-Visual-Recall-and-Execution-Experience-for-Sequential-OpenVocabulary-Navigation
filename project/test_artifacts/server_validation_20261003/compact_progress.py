"""Show current phase, motion, and distinct execution failures."""
from pathlib import Path
import json
import sys
ROOT=Path('/home/ly/town_memory_validation_20261003')
out=ROOT/sys.argv[1]
events=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
def compact(e):
    result={k:v for k,v in e.items() if k not in ('graph_route','candidate_goals','confirmation','candidates','path','points','payload')}
    if isinstance(result.get('status'),dict):
        result['status']={k:v for k,v in result['status'].items() if k not in ('graph_route','candidate_goals','confirmation','candidates')}
    return result
selected=[e for e in events if e['kind'] in ('plan_status','search_progress','return_progress','house_search_finished',
                                           'return_finished','test_error','query_sent','station_memory_established','memory_selection')]
print(json.dumps({'recent':[compact(e) for e in selected[-6:]],
                  'confirmations':[{'elapsed':e['elapsed'],'query':e['query']} for e in events if e['kind']=='object_confirmation'],
                  'failures':{s:sum(e.get('status')==s for e in events) for s in
                              ('stale_odom','stale_costmap','follow_failed','no_vehicle_path')},
                  'vision_pid':int((ROOT/'vision.pid').read_text())},indent=2))
if (out/'full_test.json').exists():
    result=json.loads((out/'full_test.json').read_text())
    print(json.dumps({'final':{k:v for k,v in result.items() if k not in
                              ('events','station_views_before_departure','collision_profile_before_sequence')}},indent=2))
