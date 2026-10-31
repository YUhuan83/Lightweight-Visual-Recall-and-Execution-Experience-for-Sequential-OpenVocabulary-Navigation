"""Repeat preserved field versions without editing source/config or old results."""
import hashlib
import json
import subprocess
import time
from pathlib import Path

base=Path('/home/ly/execution_continuity_review_20261022')
out=base/('continuity_recheck_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
records=[]
for version in ('v28_intent_handoff',):
    root=base/version
    pre=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True,timeout=20).stdout)
    assert not pre['processes'] and not pre['display148_lock'],pre
    manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
    assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
    assert 'ROUND_TIMEOUT_SEC=180.0' in (root/'full/driver_full.py').read_text()
    config=(root/'full/config.json').read_bytes()
    source_before={n:hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest() for n in manifest['members']}
    previous=set((root/'full').glob('trial_*'))
    print(json.dumps(dict(event='launch',version=version,output=str(out),precheck=pre)),flush=True)
    with (out/(version+'.log')).open('w') as log:
        result=subprocess.run(['bash',str(root/'run_full.sh')],stdout=log,stderr=subprocess.STDOUT)
    created=set((root/'full').glob('trial_*'))-previous
    assert len(created)==1,created
    trial=next(iter(created))
    post=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True,timeout=20).stdout)
    record=dict(version=version,trial=str(trial),returncode=result.returncode,postcheck=post,result=json.loads((trial/'result.json').read_text()))
    assert record['result']['closed'] and not post['processes'] and not post['display148_lock'],record
    assert (root/'full/config.json').read_bytes()==config
    assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in source_before.items())
    record['source_and_config_unchanged']=True
    with (out/(version+'_audit.log')).open('w') as log:
        subprocess.run(['bash','/tmp/audit_closed_v19.sh',str(trial)],check=True,stdout=log,stderr=subprocess.STDOUT)
    record['fluency']=json.loads((trial/'fluency_summary.json').read_text())
    record['attribution']=json.loads((trial/'control_stop_attribution.json').read_text())
    records.append(record)
    (out/'results.json').write_text(json.dumps(records,indent=2))
    print(json.dumps(dict(event='closed',output=str(out),trial=str(trial),fluency=record['fluency'],attribution=record['attribution'])),flush=True)
