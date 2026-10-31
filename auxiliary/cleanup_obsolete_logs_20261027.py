"""Explicit, closed-trial cleanup. Plan first; apply only saved, revalidated files."""
import collections
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path('/home/ly/execution_continuity_review_20261022')
PLAN = ROOT / 'cleanup_plan_20261027.json'
RECEIPT = ROOT / 'cleanup_receipt_20261027.json'

def checked(value):
    p = Path(value)
    assert p.is_absolute() and p.is_file() and not p.is_symlink(), str(p)
    assert p.resolve() == p and p.is_relative_to(ROOT), str(p)
    return p

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def trial_proof(p):
    candidates = [a for a in p.parents if a.name.startswith('trial_') and a.parent.name == 'full']
    assert len(candidates) == 1
    proof = checked(candidates[0] / 'result.json')
    assert json.loads(proof.read_text())['closed'] is True
    return str(proof)

def main():
    state = json.loads(subprocess.check_output(['python3', '/home/ly/planning_phase_physical_20261018/prestart.py'], text=True))
    assert state['processes'] == [] and state['display148_lock'] is False
    if sys.argv[1] == 'plan':
        rows = json.loads((ROOT / 'cleanup_log_inventory_20261027.json').read_text())['files']
        selected, skipped = [], []
        loggroups = collections.defaultdict(list)
        for r in rows:
            p = Path(r['path'])
            rel = p.relative_to(ROOT)
            match = re.match(r'^v(\d+)(?:_|$)', rel.parts[0])
            if not match or not (2 <= int(match[1]) <= 27) or int(match[1]) == 25:
                continue
            p = checked(p)
            if r['kind'] == 'recording':
                try:
                    proof = trial_proof(p)
                    assert p.parent.name == 'interfaces' and p.suffix in ('.db3', '.mcap', '.bag')
                except (AssertionError, KeyError, FileNotFoundError) as e:
                    skipped.append(dict(path=str(p), reason='no verified closed trial / unexpected layout'))
                    continue
                selected.append(dict(path=str(p), kind='obsolete_recording', proof=proof))
            elif r['kind'] == 'log' and p.suffix == '.log' and p.stat().st_size > 0:
                # Only byte-identical logs within the same directory; preserve all unique evidence.
                loggroups[(str(p.parent), p.stat().st_size)].append(p)
        for paths in loggroups.values():
            if len(paths) < 2:
                continue
            hashes = collections.defaultdict(list)
            for p in sorted(paths):
                hashes[digest(p)].append(p)
            for sha, group in hashes.items():
                for p in group[1:]:
                    selected.append(dict(path=str(p), kind='duplicate_log', kept=str(group[0]), sha256=sha))
        for r in selected:
            s = checked(r['path']).stat()
            r.update(bytes=s.st_size, mtime_ns=s.st_mtime_ns)
        plan = dict(root=str(ROOT), policy='Delete closed obsolete v2-v27 raw recordings except v25; keep v1,v25,v28+ and all unique diagnostics. Deduplicate identical sibling logs only.', files=selected, skipped=skipped)
        PLAN.write_text(json.dumps(plan, indent=2))
        print(json.dumps(dict(counts=dict(collections.Counter(r['kind'] for r in selected)), bytes=sum(r['bytes'] for r in selected), versions=sorted(set(Path(r['path']).relative_to(ROOT).parts[0] for r in selected)), skipped=skipped), indent=2))
        return
    assert sys.argv[1] == 'apply' and not RECEIPT.exists()
    plan = json.loads(PLAN.read_text())
    assert plan['root'] == str(ROOT)
    rows = plan['files']
    for r in rows:
        p = checked(r['path'])
        s = p.stat()
        assert s.st_size == r['bytes'] and s.st_mtime_ns == r['mtime_ns']
        if r['kind'] == 'obsolete_recording':
            assert trial_proof(p) == r['proof']
        else:
            assert r['kind'] == 'duplicate_log'
            assert digest(p) == digest(checked(r['kept'])) == r['sha256']
    receipt = dict(policy=plan['policy'], deleted=[], total_bytes=0, precheck=state)
    RECEIPT.write_text(json.dumps(receipt, indent=2))
    for r in rows:
        p = checked(r['path'])
        if r['kind'] == 'obsolete_recording':
            note = p.parent / 'REMOVED_RECORDINGS_20261027.json'
            note.write_text(json.dumps(dict(reason='User authorized obsolete raw recording cleanup; full sensor replay no longer available. Results, events, images and diagnostics retained.', files=[x for x in rows if x['kind'] == 'obsolete_recording' and Path(x['path']).parent == p.parent]), indent=2))
        p.unlink()
        assert not p.exists()
        receipt['deleted'].append(r)
        receipt['total_bytes'] += r['bytes']
        RECEIPT.write_text(json.dumps(receipt, indent=2))
    retained = [r for r in json.loads((ROOT / 'cleanup_log_inventory_20261027.json').read_text())['files'] if r['kind'] == 'recording' and r['path'] not in {x['path'] for x in rows}]
    assert all(checked(r['path']).stat().st_size == r['bytes'] for r in retained)
    receipt['retained_recording_count'] = len(retained)
    receipt['verified'] = True
    RECEIPT.write_text(json.dumps(receipt, indent=2))
    print(json.dumps(dict(deleted=len(rows), bytes=receipt['total_bytes'], retained_recordings=len(retained), verified=True)))

if __name__ == '__main__':
    main()
