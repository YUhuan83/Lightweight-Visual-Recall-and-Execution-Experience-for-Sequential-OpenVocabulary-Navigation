from pathlib import Path,PurePosixPath
import json,tarfile,shutil,sys
base=Path(__file__).parent
trial=sys.argv[1] if len(sys.argv)>1 else 'trial_20261022_212511'
version='v6' if trial=='trial_20261022_215520' else 'v5'
out=base/(version+'_'+trial);out.mkdir(exist_ok=True)
with tarfile.open(base/(trial+'_core.tar.gz')) as archive:
    for member in archive.getmembers():
        name=PurePosixPath(member.name)
        assert member.isfile() and not name.is_absolute() and '..' not in name.parts
        target=out.joinpath(*name.parts);target.parent.mkdir(parents=True,exist_ok=True)
        with archive.extractfile(member) as source,target.open('wb') as dest:shutil.copyfileobj(source,dest)
goal=next(out.glob('sessions/*/goal_001'))
rows=[json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
for row in rows:
    if row.get('kind')=='plan_status' and row.get('status') not in ('planning','exploration_cost_evaluated'):
        print(json.dumps({k:row[k] for k in ('elapsed','status','goal','graph_goal','reason','motion_sections_remaining','candidate_index','background','continuous','previous_goal') if k in row}))
