"""Replay a stopped ROS bag to compare latest-slot and observation joins."""
import json
import sqlite3
import sys
from collections import deque, Counter
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import (
    arrival_status_matches, mask_matches_confirmation, matched_arrival_evidence)


def replay(folder):
    statuses, confirmations, masks = (deque(maxlen=128) for _ in range(3))
    query, started, latest_confirmation, latest_mask, mask_count = '', 0, None, None, 0
    results = dict(latest_slot=[], observation_join=[])
    selected = ''
    for db in sorted(Path(folder).rglob('*.db3')):
        connection = sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True)
        topics = {i: (name, get_message(kind)) for i, name, kind in connection.execute(
            'SELECT id,name,type FROM topics') if name in (
                '/spot1/object_search/query', '/spot1/object_mask',
                '/spot1/object_confirmation', '/wildos/plan_status', '/wildos/memory_selection')}
        for tid, ns, raw in connection.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
            if tid not in topics:
                continue
            name, cls = topics[tid]
            msg = deserialize_message(raw, cls)
            now = ns/1e9
            if name.endswith('/query'):
                query, started = msg.data, ns
                statuses.clear(); confirmations.clear(); masks.clear()
                latest_confirmation = latest_mask = None
                mask_count, selected = 0, ''
            elif name.endswith('/object_mask') and msg.query == query:
                mask_count += 1
                latest_mask = dict(query=query, at=now, pixels=sum(msg.object_mask.data),
                    count=mask_count, observation_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
                masks.append(latest_mask)
            elif name.endswith('/object_confirmation'):
                data = json.loads(msg.data)
                if data.get('query') == query:
                    latest_confirmation = dict(data, received_at=now)
                    confirmations.append(latest_confirmation)
            elif name.endswith('/plan_status'):
                data = json.loads(msg.data)
                if data.get('status') == 'object_found':
                    statuses.append(dict(status=data, received_at=now))
            elif name.endswith('/memory_selection'):
                data = json.loads(msg.data)
                if data.get('status') == 'memory_candidate_selected':
                    selected = data.get('view_id', '')
            old = None
            if (latest_confirmation and latest_mask
                    and latest_confirmation.get('search_confirmed') is True
                    and 0 <= now-latest_confirmation['received_at'] < 5.
                    and latest_mask['count'] >= 2 and latest_mask['pixels'] > 0
                    and 0 <= now-latest_mask['at'] < 5.
                    and mask_matches_confirmation(query, latest_mask['observation_stamp_ns'], latest_confirmation)):
                old = next((v for v in reversed(statuses) if now-v['received_at'] < 5.
                    and arrival_status_matches(v['status'],query,latest_confirmation)),None)
            new = matched_arrival_evidence(list(statuses), list(confirmations), list(masks),
                                          query, started, ns, now, 2)
            for label, item in (('latest_slot', old), ('observation_join', new)):
                if item is None:
                    continue
                stamp = item['status']['confirmation']['observation_stamp_ns']
                if not any(x['query']==query and x['observation_stamp_ns']==stamp for x in results[label]):
                    s=item['status']
                    results[label].append(dict(query=query, observation_stamp_ns=stamp,
                        memory_view_id=s.get('memory_view_id'), selected_view_id=selected,
                        standoff_distance_m=s['standoff_distance_m'], object_distance_m=s['object_distance_m']))
        connection.close()
    results['scope']='Stopped bag replay using recorded delivery order; not a new simulator run'
    return results


if __name__ == '__main__':
    result = replay(sys.argv[1])
    Path(sys.argv[2]).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
