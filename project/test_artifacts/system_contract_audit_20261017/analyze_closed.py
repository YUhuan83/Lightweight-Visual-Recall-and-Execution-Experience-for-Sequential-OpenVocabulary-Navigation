"""Audit frozen field records without executing or changing navigation."""
from pathlib import Path
import bisect
import collections
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent
CLOSED = ROOT / 'test_artifacts/full_vehicle_acceptance_20261017/closed_evidence'
SESSION = CLOSED / 'field_results/session_20261017_172053'


def main():
    review = json.loads((CLOSED/'pursuit_closed_review.json').read_text(encoding='utf8'))
    lo = review['query_wall']
    hi = lo + review['goal']['elapsed_s']
    control, native, twists, statuses = {}, [], [], []
    observer = SESSION/'execution_observer/messages.jsonl'
    with observer.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            t, topic, message = row['received_ros_ns']/1e9, row['topic'], row['message']
            if topic in ('/wildos/control_status', '/wildos/native_control_status', '/wildos/plan_status'):
                data = json.loads(message['data'])
                if topic == '/wildos/control_status':
                    source = data.get('source_control', {})
                    key = tuple(source.get(k) for k in ('sender_id', 'generation', 'sequence'))
                    control[key] = (t, data)
                elif topic == '/wildos/native_control_status' and lo <= t <= hi:
                    native.append((t, data))
                elif topic == '/wildos/plan_status' and data.get('stamp_ns', 0) > 0:
                    statuses.append((data['stamp_ns']/1e9, data))
            elif topic == '/wildos/cmd_vel':
                twists.append((t, message))
    statuses.sort(key=lambda x: x[0])
    native.sort(key=lambda x: x[0])
    stamps, twist_stamps = [x[0] for x in statuses], [x[0] for x in twists]
    by_reason, cross, classes = [collections.defaultdict(float) for _ in range(3)]
    velocity_age, exact, stopped, covered = [], 0, 0, 0.
    for (t, data), (u, _) in zip(native, native[1:]):
        dt = min(.2, max(0., u-t))
        covered += dt
        if data['speed'] >= .05:
            continue
        stopped += 1
        receipt = data.get('source_receipt') or {}
        key = tuple(receipt.get(k) for k in ('sender_id', 'generation', 'sequence'))
        matched = control.get(key)
        reason = (matched[1]['reason'] if matched and receipt.get('applied') is True
                  else 'unmatched_or_not_applied')
        exact += bool(matched and receipt.get('applied') is True)
        by_reason[reason] += dt
        i = bisect.bisect_right(stamps, t)-1
        state = statuses[i][1]['status'] if i >= 0 else 'unknown'
        cross[state+' / '+reason] += dt
        sent = data['sent']
        category = ('positive_throttle' if sent['throttle'] > .1 and sent['brake'] <= .01
                    else 'braking' if sent['brake'] > .1 else 'coasting_or_low_torque')
        classes[category] += dt
        j = bisect.bisect_right(twist_stamps, t)-1
        if state == 'following' and j >= 0:
            velocity_age.append(t-twist_stamps[j])
    cancels = []
    for line in (SESSION/'stack.log').read_text(encoding='utf8').splitlines():
        if 'Cancelling follow_path action' in line:
            found = re.search(r'\[(\d+\.\d+)\]', line)
            if found and lo <= float(found[1]) <= hi:
                cancels.append(dict(t=float(found[1])-lo, line=line))
    report = dict(candidate=review['candidate'], observer_sha256=hashlib.sha256(observer.read_bytes()).hexdigest(),
        scope='Closed goal window; dt capped .2s. Source reason uses EXACT applied receipt identity; '
              'public plan status association is approximate. Sent/SDK ACK does not establish physical force.',
        native_samples=len(native), covered_seconds=covered, stopped_samples=stopped,
        stopped_applied_identity_matches=exact, stopped_seconds_by_source_reason=dict(by_reason),
        stopped_seconds_by_sent_control=dict(classes),
        stopped_cross_status_source_reason=dict(sorted(cross.items(), key=lambda x: -x[1])),
        following_stopped_latest_twist_receive_age=dict(count=len(velocity_age), max=max(velocity_age, default=0),
            over_half_sec=sum(x > .5 for x in velocity_age)), bridge_follow_cancels=cancels)
    (OUT/'closed_trace.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
