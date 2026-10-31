import hashlib,json,shutil,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');source=base/'v37_full_path';target=base/'v38_rolling_observation'
assert not target.exists()
with zipfile.ZipFile(base/'rolling_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    for n,h in expected.items():assert hashlib.sha256((source/'candidate'/n).read_bytes()).hexdigest()==h,n
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate',ignore=shutil.ignore_patterns('__pycache__'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    for n in names:
        p=target/'candidate'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for n in ('launch_trial.py','run_full.sh','real_ros_review.py'):shutil.copy2(source/n,target/n)
for p in [target/'launch_trial.py',target/'run_full.sh']+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
run=target/'run_full.sh';text=run.read_text();needle=f'source {target}/install/local_setup.bash'
assert text.count(needle)==1;run.write_text(text.replace(needle,f'source {source}/install/local_setup.bash\n'+needle))
config_path=full/'config.json';config=json.loads(config_path.read_text());i=config['setups'].index(str(target/'install/local_setup.bash'))
config['setups'].insert(i,str(source/'install/local_setup.bash'));config_path.write_text(json.dumps(config,indent=2))
review=target/'real_ros_review.py';review.write_text(review.read_text().replace('modules = (',"modules = ('test_rolling_observation_ros', ",1))
driver=full/'driver_full.py';text=driver.read_text()
old="""    GOAL_STARTED_AT=time.monotonic()
    ROUND_DEADLINE=GOAL_STARTED_AT+ROUND_TIMEOUT_SEC
    report['startup_elapsed_sec']=GOAL_STARTED_AT-ROUND_STARTED_AT
    report['search_timeout_sec']=ROUND_TIMEOUT_SEC
    report['startup_outside_search_budget']=True
    send('green residential house')
"""
new="""    send('green residential house')
    query_events=session/'goal_001/events.jsonl'
    def issued_query():
        if not query_events.exists():return None
        for line in query_events.read_text().splitlines():
            try:event=json.loads(line)
            except json.JSONDecodeError:continue
            if event.get('kind')=='query_sent':return event
        return None
    wait(lambda:issued_query() is not None,30.)
    issued=issued_query();GOAL_STARTED_AT=issued['monotonic_ns']/1e9
    assert 0<=time.monotonic()-GOAL_STARTED_AT<2.
    ROUND_DEADLINE=GOAL_STARTED_AT+ROUND_TIMEOUT_SEC
    report['startup_elapsed_sec']=GOAL_STARTED_AT-ROUND_STARTED_AT
    report['search_timeout_sec']=ROUND_TIMEOUT_SEC
    report['startup_outside_search_budget']=True
    report['query_event_is_deadline_origin']=True
"""
assert text.count(old)==1;driver.write_text(text.replace(old,new))
manifest=json.loads((full/'frozen_source_manifest.json').read_text())
for n in names:manifest['members'][n]=hashlib.sha256((target/'candidate'/n).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
(target/'receipt.json').write_text(json.dumps(dict(base=str(source),target=str(target),changed={n:manifest['members'][n] for n in names}),indent=2))
print(target)
