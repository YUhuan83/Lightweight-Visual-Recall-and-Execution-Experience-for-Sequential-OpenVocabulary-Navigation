import hashlib,json,shutil,subprocess
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v37_full_path'
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed']
assert not receipt['postcheck']['processes'] and not receipt['postcheck']['display148_lock']
saved=root/'round1_closed';saved.mkdir(exist_ok=False)
for name in ('field_result.json','field_phase_audit.json','closed_arrival_audit.json','continuous_path_audit.json','trial_frozen.json'):
    shutil.copy2(root/name,saved/name)
frozen=json.loads((root/'trial_frozen.json').read_text())
for name,h in frozen['full_wrapper_sha256'].items():assert hashlib.sha256((root/'full'/name).read_bytes()).hexdigest()==h,name
assert hashlib.sha256((root/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so').read_bytes()).hexdigest()==frozen['new_rpp_sha256']
subprocess.run(['python3',str(base/'run_confirmed_progress_trial.py'),'v37_full_path'],check=True)
