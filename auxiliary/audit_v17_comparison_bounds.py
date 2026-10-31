"""Read closed bags; estimate decision-preserving pruning without executing motion."""
import bisect
import json
import math
import sqlite3
import sys
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def audit(trial):
    assert json.loads((trial / 'result.json').read_text())['closed']
    goal = next(trial.glob('sessions/*/goal_001'))
    assert (goal / 'stop.json').exists()
    db = sqlite3.connect(next((goal / 'interfaces').glob('*.db3')).as_uri() + '?mode=ro', uri=True)

    def rows(topic, cls):
        entry = db.execute('select id from topics where name=?', (topic,)).fetchone()
        if not entry:
            return []
        return [(t, deserialize_message(b, cls)) for t, b in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp', (entry[0],))]

    routes = {}
    for t, msg in rows('/spot1/graphnav_planner/route_status', String):
        d = json.loads(msg.data)
        if d.get('vehicle_options'):
            routes.setdefault(d.get('exploration_id'), d)
    plans = [json.loads(m.data) for t, m in rows('/wildos/plan_status', String)]
    odom = [(t, (m.pose.pose.position.x, m.pose.pose.position.y))
            for t, m in rows('/spot1/odom', Odometry)]
    odom_t = [t for t, p in odom]
    groups = []
    for identity, route in routes.items():
        events = [p for p in plans if p.get('exploration_id') == identity]
        evaluations = [p for p in events if p['status'] == 'exploration_cost_evaluated']
        choices = [p for p in events if p['status'] == 'frontier_selection_waiting']
        starts = [p for p in events if p['status'] == 'planning']
        if not evaluations or not choices or not starts:
            continue
        i = bisect.bisect_right(odom_t, starts[0]['stamp_ns']) - 1
        if i < 0:
            continue
        origin = odom[i][1]
        options = route['vehicle_options']
        index = {o['uuid']: j for j, o in enumerate(options)}
        best = -math.inf
        for event in evaluations:
            j = index[event['candidate_uuid']]
            score = options[j]['benefit'] - .004 * event['vehicle_cost_m']
            best = max(best, score)
            # Actual path accepts <=0.05 m start and <=1 m goal deviation.
            # Extra 0.25 m makes this bag-only estimate conservative for odom timing.
            upper = [(o['uuid'], o['benefit'] - .004 * max(0.,
                      math.dist(origin, o['goal'][:2]) - 1.30)) for o in options[j+1:]]
            possible = bool(upper) and all(value < best - 1e-9 for uid, value in upper)
            if possible:
                groups.append(dict(identity=identity, origin=origin, options=options,
                    best_score=best, remaining_upper=upper,
                    after_uuid=event['candidate_uuid'], choice=choices[0]['candidate_uuid'],
                    remaining_wait_s=(choices[0]['stamp_ns']-event['stamp_ns'])/1e9,
                    subsequent_events=[p for p in events if event['stamp_ns'] < p['stamp_ns'] <= choices[0]['stamp_ns']]))
                break
    db.close()
    return dict(trial=str(trial), comparisons_with_evaluations=sum(
        any(p.get('exploration_id') == identity and p['status']=='exploration_cost_evaluated' for p in plans)
        for identity in routes), prunable=groups)


if __name__ == '__main__':
    result = [audit(Path(name)) for name in sys.argv[1:]]
    print(json.dumps(result, indent=2))
