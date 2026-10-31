from pathlib import Path
import json
import sys
import time

root = Path('/home/ly/town_memory_validation_20261003')
trial = root/sys.argv[1]
if '--snapshot' in sys.argv:
    import cv2
    for _ in range(20):
        frame = cv2.imread(str(trial/'traffic_light_search_latest.png'))
        if frame is not None:
            cv2.imwrite(str(trial/'camera_review.png'), frame)
            print('snapshot_saved')
            break
        time.sleep(.05)
report = trial/'full_test.json'
if report.exists():
    result = json.loads(report.read_text())
    print(json.dumps({k:v for k,v in result.items() if k not in (
        'events','collision_profile_before_sequence','calibration','topics_recorded')}))
else:
    events = trial/'events.jsonl'
    lines = events.read_text().splitlines() if events.exists() else []
    values = []
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if 'status' in value and isinstance(value['status'], dict):
            status = value['status']
            value['status'] = {k:v for k,v in status.items() if k in (
                'status', 'reason', 'pending', 'following', 'query', 'active_segment',
                'candidate', 'candidate_index', 'candidate_count', 'target_confirmed',
                'target_distance', 'query_generation', 'last_plan_status', 'motion_direction')}
        values.append(value)
    progress = [v for v in values if v.get('kind') == 'search_progress']
    transitions = [v for v in values if v.get('kind') == 'plan_status']
    print(json.dumps(progress[-2:]+transitions[-2:]))
    review = [v for v in values if v.get('kind') == 'awaiting_first_target_visual_review']
    if review:
        print(json.dumps(review[-1]))
bridge = root/'graph_bridge.log'
print('\n'.join(bridge.read_text().splitlines()[-12:]))
