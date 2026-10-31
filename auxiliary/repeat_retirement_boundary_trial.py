import hashlib,json,shutil,subprocess
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v42_retirement_boundary'
receipt=json.loads((root/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes'] and not receipt['postcheck']['display148_lock']
saved=root/'round1_closed';saved.mkdir(exist_ok=False)
for name in ('field_result.json','field_phase_audit.json','closed_arrival_audit.json','continuous_path_audit.json','retirement_boundary_audit.json','trial_frozen.json'):
    shutil.copy2(root/name,saved/name)
frozen=json.loads((root/'trial_frozen.json').read_text())
for name,h in frozen['frozen_wrappers'].items():
    p=root/name if name=='run_full.sh' else root/'full'/name
    assert hashlib.sha256(p.read_bytes()).hexdigest()==h,name
assert hashlib.sha256((root/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so').read_bytes()).hexdigest()==frozen['rpp_sha256']
subprocess.run(['python3',str(base/'run_confirmed_progress_trial.py'),root.name],check=True)
