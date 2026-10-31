"""Join recorded stall events to measured actuator and wheel evidence."""
import argparse
import collections
import json
from pathlib import Path


def audit(root):
    events = [json.loads(s) for s in (root/'events.jsonl').read_text().splitlines()]
    samples = [json.loads(s) for s in (root/'native_samples.jsonl').read_text().splitlines()]
    samples.sort(key=lambda s: s['wall_time'])
    report = json.loads((root/'full_test.json').read_text())
    return events, samples, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    events, samples, report = audit(args.root)
    stalls = [e for e in events if e.get('reason') == 'execution_stalled' and e.get('status') == 'follow_failed']
    joins = [e for e in events if e.get('status') == 'replanning_after_maneuver']
    driving = [s for s in samples if s.get('phase') == 'green_house_search']
    stationary = [s for s in driving if s.get('speed', 0) < .1 and s.get('sent', {}).get('throttle', 0) > .05]
    contacts = collections.Counter((s.get('wheel_contact') or {}).get('diagnosis', {}).get('classification', 'unavailable') for s in stationary)
    evidence = dict(stalls=len(stalls), stall_labels=dict(collections.Counter(e['stall_class'] for e in stalls)),
        maneuver_replans=len(joins), joins=[{k:e.get(k) for k in ('elapsed','position_error','heading_error','reason')} for e in joins],
        stationary_throttle_samples=len(stationary), stationary_contact_labels=dict(contacts),
        stationary_watchdog_samples=sum(bool(s.get('watchdog_stop')) for s in stationary),
        stationary_brake_samples=sum(s['sent'].get('brake',0)>.05 for s in stationary),
        stationary_actual_throttle_range=[min((s.get('actual',{}).get('throttle') or 0) for s in stationary), max((s.get('actual',{}).get('throttle') or 0) for s in stationary)] if stationary else [],
        collision_profile=report.get('collision_profile_before_sequence'),
        note='Sample counts are correlated telemetry, not independent incidents. Fallback stall labels do not prove collision.')
    (args.root/'stall_physics_audit.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    print(json.dumps(evidence,indent=2))


if __name__ == '__main__':
    main()
