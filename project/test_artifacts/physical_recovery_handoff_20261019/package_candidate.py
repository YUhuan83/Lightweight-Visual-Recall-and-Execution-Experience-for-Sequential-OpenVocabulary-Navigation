"""Freeze a bounded delta over the preserved 6153 candidate for isolated tests."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

out=Path(__file__).resolve().parent;root=out.parents[1]
baseline='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
files=['nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
       'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
       'nav2_adapter/wildos_nav2_adapter/execution_recovery.py',
       'nav2_adapter/test/test_navigation_transactions_ros.py',
       'test_artifacts/physical_recovery_handoff_20261019/run_regression.py']
manifest=dict(baseline_bundle=baseline,members={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},
              production_deployment=False,field_acceptance=False)
with zipfile.ZipFile(out/'candidate_delta.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(root/p,p)
    z.writestr('candidate_delta.json',json.dumps(manifest,indent=2)+'\n')
(out/'candidate_delta.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
shutil.copyfile(root/'test_artifacts/offline_navigation/callback_report.json',out/'full_callback_report_final.json')
print(json.dumps(manifest,indent=2))
