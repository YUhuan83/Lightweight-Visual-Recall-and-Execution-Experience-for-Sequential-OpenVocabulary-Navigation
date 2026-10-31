from pathlib import Path
import shutil, json
base=Path('/home/ly/execution_continuity_review_20261022')
src=base/'v15_short_reverse_handoff'
dst=base/'v16_explore_during_reobserve'
if dst.exists():
    raise SystemExit(f'refusing to overwrite {dst}')
dst.mkdir(parents=True)
shutil.copytree(src/'candidate', dst/'candidate')
full=dst/'full'; full.mkdir()
for p in (src/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):
        shutil.copy2(p, full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):
    shutil.copy2(src/name, dst/name)
old='execution_continuity_review_20261022/v15_short_reverse_handoff'
new='execution_continuity_review_20261022/v16_explore_during_reobserve'
for p in [dst/'launch_trial.py',dst/'run_full.sh',dst/'build.sh',dst/'real_ros_review.py',*full.glob('*.py'),full/'config.json']:
    text=p.read_text(encoding='utf8')
    p.write_text(text.replace(old,new),encoding='utf8')
print(json.dumps({'source':str(src),'candidate':str(dst/'candidate'),'full_files':sorted(p.name for p in full.iterdir()),'root':str(dst)}))
