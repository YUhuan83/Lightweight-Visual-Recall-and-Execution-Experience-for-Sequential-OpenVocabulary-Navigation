"""Apply the user's search-only 180s budget to the isolated field driver."""
from pathlib import Path
import ast,hashlib,json,shutil
root=Path('/home/ly/execution_continuity_review_20261022/v7')
driver=root/'full/driver_full.py';before=driver.read_text();after=before
assert 'ROUND_TIMEOUT_SEC=180.0' in before
after=after.replace('ROUND_DEADLINE=ROUND_STARTED_AT+ROUND_TIMEOUT_SEC',
    'STARTUP_TIMEOUT_SEC=300.0\nROUND_DEADLINE=ROUND_STARTED_AT+STARTUP_TIMEOUT_SEC\nGOAL_STARTED_AT=None')
after=after.replace("    send('green residential house')", "    GOAL_STARTED_AT=time.monotonic()\n    ROUND_DEADLINE=GOAL_STARTED_AT+ROUND_TIMEOUT_SEC\n    report['startup_elapsed_sec']=GOAL_STARTED_AT-ROUND_STARTED_AT\n    report['search_timeout_sec']=ROUND_TIMEOUT_SEC\n    report['startup_outside_search_budget']=True\n    send('green residential house')")
after=after.replace('finally:\n    halt=False',
    "finally:\n    SHUTDOWN_STARTED_AT=time.monotonic()\n    if GOAL_STARTED_AT is not None:report['search_elapsed_sec']=SHUTDOWN_STARTED_AT-GOAL_STARTED_AT\n    halt=False")
after=after.replace("    report['closed']=child is not None and child.poll() is not None", 
    "    report['closed']=child is not None and child.poll() is not None\n    report['shutdown_elapsed_sec']=time.monotonic()-SHUTDOWN_STARTED_AT")
assert after!=before and after.count('ROUND_DEADLINE=GOAL_STARTED_AT+ROUND_TIMEOUT_SEC')==1
ast.parse(after)
shutil.copy2(driver,root/'driver_before_search_budget.py');driver.write_text(after)
config_path=root/'full/config.json';config=json.loads(config_path.read_text());config['goal_timeout_s']=180
config_path.write_text(json.dumps(config,indent=2))
receipt=dict(search_timeout_sec=180,startup_timeout_sec=300,startup_outside_search_budget=True,
    before_sha256=hashlib.sha256(before.encode()).hexdigest(),after_sha256=hashlib.sha256(after.encode()).hexdigest())
(root/'search_budget_receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
