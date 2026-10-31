import json
from pathlib import Path
root=Path('/home/ly/wildos-console')
for pattern in ('field_lifecycle_*','field_search_*'):
    paths=sorted(root.glob(pattern))
    if paths:
        path=paths[-1];print(str(path))
        if (path/'console.log').exists():print('\n'.join((path/'console.log').read_text(errors='replace').splitlines()[-6:]))
        if (path/'result.json').exists():print((path/'result.json').read_text())
paths=sorted((root/'results').glob('session_*'))
if paths:
    session=paths[-1];print('LATEST SESSION',session)
    for name in ('scene.json','sensor.log','vision.log','triangulation.log','preflight.log','session_result.json'):
        path=session/name
        if path.exists():
            print(name, '\n'.join(path.read_text(errors='replace').splitlines()[-5:])[:1500])
    for goal in sorted(session.glob('goal_*')):
        print('GOAL',goal.name)
        if (goal/'result.json').exists():print((goal/'result.json').read_text()[:2500])
        if (goal/'events.jsonl').exists():print('\n'.join((goal/'events.jsonl').read_text().splitlines()[-3:])[:2000])
