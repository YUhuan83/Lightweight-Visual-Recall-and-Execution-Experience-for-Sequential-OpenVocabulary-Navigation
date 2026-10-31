"""Read-only 180s trial audit; stdlib only, no ROS or simulator connection.

Reads SQLite rosbag2 std_msgs/String CDR, writes a separate summary. Source
timestamps label state dwell; they do not measure planner CPU or network delay.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import struct


def cdr_string(blob):
    if len(blob) < 9 or blob[:2] not in (b'\x00\x01', b'\x00\x00'):
        raise ValueError('unsupported std_msgs/String CDR encapsulation')
    size = struct.unpack_from('<I' if blob[1] == 1 else '>I', blob, 4)[0]
    if not size or 8 + size > len(blob) or blob[7 + size] != 0:
        raise ValueError('invalid CDR string length/terminator')
    return blob[8:7 + size].decode('utf8')


def point(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        p = tuple(map(float, value[:2]))
        return p if all(map(math.isfinite, p)) else None
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path, help='trial/goal/evidence directory containing closed *.db3')
    parser.add_argument('output', type=Path)
    parser.add_argument('--window', type=float, default=180.)
    parser.add_argument('--query', default='green residential house')
    parser.add_argument('--stack', type=Path, help='optional stack.log, count timestamped requests in query window')
    args = parser.parse_args()
    bags = sorted(args.trial.rglob('*.db3'))
    if not bags:
        parser.error('No SQLite bag found; MP4/events alone cannot yield this Native speed audit')
    # Prevent silently combining trials copied below a parent evidence folder.
    if len({b.parent for b in bags}) != 1:
        parser.error('Choose one trial/goal; bags from multiple directories found')
    topics = Counter()
    rows = []
    errors = []
    wanted = ('/spot1/object_search/query', '/wildos/plan_status',
              '/wildos/control_status', '/wildos/native_control_status',
              '/spot1/visible_target_approach', '/spot1/object_confirmation')
    for bag in bags:
        uri = bag.resolve().as_uri() + '?mode=ro'
        with sqlite3.connect(uri, uri=True) as db:
            for name, typ, count in db.execute('select t.name,t.type,count(*) from messages m join topics t on t.id=m.topic_id group by t.id'):
                topics[name] += count
            marks = ','.join('?' for _ in wanted)
            for name, typ, ns, blob in db.execute('select t.name,t.type,m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name in (' + marks + ') order by m.timestamp', wanted):
                try:
                    if typ != 'std_msgs/msg/String':
                        raise ValueError('expected String, got ' + typ)
                    s = cdr_string(blob)
                    data = s if name.endswith('/query') else json.loads(s)
                    rows.append((ns / 1e9, name, data))
                except (ValueError, UnicodeError) as exc:
                    errors.append(dict(topic=name, bag_ns=ns, error=str(exc)))
    rows.sort(key=lambda r: r[0])
    starts = [t for t, name, data in rows if name == wanted[0] and data == args.query]
    if not starts:
        parser.error('Exact query String not present in bag; specify --query, do not guess query start')
    q0 = starts[0]
    q1 = q0 + args.window
    next_query = next((t for t, name, data in rows if name == wanted[0] and t > q0 and data != args.query), None)
    if next_query is not None:
        q1 = min(q1, next_query)

    def sources(topic, key, seconds=False):
        dedup = {}
        for bag_t, name, data in rows:
            if name != topic or not isinstance(data, dict):
                continue
            try:
                t = float(data[key]) / (1. if seconds else 1e9)
                if not math.isfinite(t):
                    continue
            except (KeyError, TypeError, ValueError):
                continue
            dedup[t] = dict(data, _bag_time=bag_t)
        return sorted(dedup.items())

    native = sources(wanted[3], 'wall_time', True)
    control = sources(wanted[2], 'stamp_ns')
    plan = sources(wanted[1], 'stamp_ns')
    timelines = (native, control, plan)
    times = [[t for t, _ in seq] for seq in timelines]

    def latest(i, t):
        index = bisect.bisect_right(times[i], t) - 1
        return timelines[i][index] if index >= 0 else (None, {})

    cuts = sorted({q0, q1, *[t for seq in timelines for t, _ in seq if q0 < t < q1],
                   *[t + .5 for t, _ in native if q0 < t + .5 < q1]})
    covered = stopped = 0.
    stop_control, stop_plan, dwell, cross = Counter(), Counter(), Counter(), Counter()
    runs = []
    for a, b in zip(cuts, cuts[1:]):
        nt, n = latest(0, a)
        _, c = latest(1, a)
        pt, p = latest(2, a)
        if pt is not None:
            dwell[p.get('status', 'unknown')] += b - a
        if nt is None or a - nt >= .5:
            continue
        try:
            speed = float(n['speed'])
        except (ValueError, TypeError, KeyError):
            continue
        if not math.isfinite(speed):
            continue
        covered += b - a
        if abs(speed) >= .1:
            continue
        dt = b - a
        stopped += dt
        reason, state = c.get('reason', 'unknown'), p.get('status', 'unknown')
        stop_control[reason] += dt
        stop_plan[state] += dt
        cross[(reason, state)] += dt
        if runs and abs(runs[-1]['end_s'] - (a - q0)) < 1e-6:
            run = runs[-1]
            run['end_s'] = b - q0
        else:
            run = dict(start_s=a - q0, end_s=b - q0)
            runs.append(run)
    for run in runs:
        run['duration_s'] = run['end_s'] - run['start_s']

    ns = [(t, n) for t, n in native if q0 <= t <= q1]
    gears = []
    for t, n in ns:
        gear = n.get('sent', {}).get('gear')
        if gear is not None and (not gears or gears[-1]['gear'] != gear):
            gears.append(dict(source_s=t - q0, gear=gear))
    positions = [(t, point(n.get('position'))) for t, n in ns]
    positions = [(t, p) for t, p in positions if p is not None]
    movement = None
    if len(positions) >= 2:
        movement = dict(first_native_s=positions[0][0] - q0, last_native_s=positions[-1][0] - q0,
                        start=positions[0][1], end=positions[-1][1],
                        net_displacement_m=math.dist(positions[0][1], positions[-1][1]),
                        max_displacement_m=max(math.dist(positions[0][1], p) for _, p in positions),
                        sampled_travel_m=sum(math.dist(pa, pb) for (ta, pa), (tb, pb) in zip(positions, positions[1:]) if tb - ta <= .5))

    ps = [(t, p) for t, p in plan if q0 <= t <= q1]
    counts = Counter(p.get('status', 'unknown') for _, p in ps)
    failures = []
    proposals = [(t, name, d) for t, name, d in rows if name in wanted[4:] and isinstance(d, dict)]
    for t, failure in ps:
        state = failure.get('status')
        if state != 'visible_target_no_vehicle_path' and not (state == 'confirmed_approach_failed' and failure.get('reason') == 'no_vehicle_path'):
            continue
        failed = point(failure.get('goal'))
        visible = state == 'visible_target_no_vehicle_path'
        radius = 2. if visible else 1.2
        evidence = []
        if failed is not None:
            for rt, name, d in proposals:
                goal = point(d.get('point') if name == wanted[4] else d.get('standoff'))
                if (failure.get('_bag_time', t) <= rt <= min(q1, t + 15)
                        and d.get('query') == failure.get('query') and goal is not None
                        and (visible and name == wanted[4] or not visible and d.get('approach_refresh') is True)):
                    evidence.append(dict(received_s=rt - q0, goal=goal,
                        distance_from_failed_m=math.dist(failed, goal), different_outside_rejection=math.dist(failed, goal) >= radius,
                        observation_stamp_ns=d.get('observation_stamp_ns'), instance_id=d.get('instance_id'),
                        approach_refresh=d.get('approach_refresh', False)))
        accepted = [dict(source_s=tt - q0, status=p.get('status'), goal=p.get('goal'), source=p.get('source'))
                    for tt, p in ps if t < tt <= min(q1, t + 15)
                    and p.get('query') == failure.get('query')
                    and p.get('status') in ('visible_target_approach', 'planned', 'following')
                    and point(p.get('goal')) is not None and failed is not None
                    and math.dist(point(p['goal']), failed) >= radius]
        failures.append(dict(source_s=t - q0, status=state, failed_goal=failed,
                             proposals=evidence, subsequent_other_goal_states=accepted[:20]))

    stack = None
    if args.stack:
        scoped = []
        for line in args.stack.read_text(encoding='utf8', errors='replace').splitlines():
            match = re.search(r'\[(\d{10}\.\d+)\]', line)
            if match and q0 <= float(match[1]) <= q1:
                scoped.append(line)
        stack = dict(path=str(args.stack.resolve()), timestamped_window_lines=len(scoped),
                     compute_path_requests=sum('ComputePath request sent:' in s for s in scoped),
                     empty_path=sum('ComputePath candidate failed:' in s and 'reason=empty_path' in s for s in scoped),
                     tile_recoveries=sum('Tiled object search recovered' in s for s in scoped),
                     lidar_locks=sum('Object LOCK using LiDAR' in s for s in scoped),
                     validated_multiview=sum('Validated multi-view' in s for s in scoped))

    assert abs(sum(stop_control.values()) - stopped) < 1e-6
    assert abs(sum(stop_plan.values()) - stopped) < 1e-6
    report = dict(scope=__doc__, query=args.query, query_start_bag_wall=q0,
        window_s=q1 - q0, topics=dict(topics), decode_errors=errors,
        bag_files=[dict(path=str(b.resolve()), sha256=hashlib.sha256(b.read_bytes()).hexdigest()) for b in bags],
        native=dict(samples=len(ns), covered_s=covered, uncovered_s=(q1 - q0) - covered,
                    maximum_source_gap_s=max((b - a for (a, _), (b, _) in zip(ns, ns[1:])), default=None),
                    source_rejections=dict(Counter(str(n.get('source_rejection')) for _, n in ns)),
                    telemetry_not_true=sum(n.get('telemetry_ok') is not True for _, n in ns)),
        near_stop=dict(threshold_abs_speed_mps=.1, seconds=stopped if covered else None,
                       fraction_of_covered=stopped / covered if covered else None,
                       fraction_of_window=stopped / (q1 - q0) if covered else None,
                       by_control_s=dict(stop_control), by_plan_s=dict(stop_plan),
                       cross_s=[dict(control=c, plan=p, seconds=s) for (c, p), s in cross.most_common()],
                       longest_s=max((r['duration_s'] for r in runs), default=None),
                       runs_over_1s=[r for r in runs if r['duration_s'] >= 1.]),
        movement=movement, sent_gear_runs=gears, sent_gear_switches=max(0, len(gears) - 1),
        plan_status_dwell_s=dict(dwell), plan_status_events=dict(counts), stack_window_counts=stack,
        observation_failure_chains=failures,
        limitations=[
            'near_stop is unavailable if Native speed absent; following is not proof of motion',
            'plan dwell uses source timestamps and latest state; not CPU time or ComputePath call duration',
            'goal changes after failure are correlation; inspect query, identity and actual route before asserting recovery success',
            'no arrival inference: use harness result plus image/pose evidence separately',
            'net displacement differs from travel distance; sampled_travel omits source gaps over .5s',
            'query start is bag receipt time; source chronology may differ by DDS/recorder latency',
            'last state can persist to window end after early shutdown; check actual result elapsed_s and use --window accordingly'])
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / 'closed_trial_summary.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps(dict(output=str(target), native=report['native'], near_stop_s=stopped,
                         near_stop_fraction=report['near_stop']['fraction_of_covered'], movement=movement,
                         plan_status_dwell_s=dict(dwell), observation_failure_chains=len(failures)), ensure_ascii=False))


if __name__ == '__main__':
    main()
