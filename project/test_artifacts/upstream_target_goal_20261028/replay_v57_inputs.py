import json
from pathlib import Path

root = Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/full/trial_20261028_091404/sessions/session_20261028_091404/graph_pairs')
origin_ns = 1793150163973234019 - round(74.29697823524475 * 1e9)
records = [json.loads(line) for line in (root / 'snapshots.jsonl').read_text().splitlines()]
pairs = [r for r in records if r.get('kind') == 'pair' and all(r.get('contract', {}).values())]
selected = []
for target in (75, 80, 89, 93, 119, 144, 170):
    row = min(pairs, key=lambda r: abs((r['ros_ns'] - origin_ns) / 1e9 - target))
    if row not in selected:
        selected.append(row)
for row in selected:
    print(root / row['raw']['file'], root / row['scored']['file'],
          749.1316533905641, -130.08572630437436, 60.12138368819478, row['ros_ns'])
    print('selected', round((row['ros_ns'] - origin_ns) / 1e9, 3), row['scored']['file'], file=__import__('sys').stderr)
