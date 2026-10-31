"""Package only the reset-intent implementation/test delta; preserve baseline archive."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile
import argparse

parser=argparse.ArgumentParser();parser.add_argument('--portable',action='store_true');args=parser.parse_args()
out=Path(__file__).resolve().parent;root=out.parents[1]
files=['nav2_adapter/wildos_nav2_adapter/navigation_transactions.py',
       'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
       'nav2_adapter/test/test_navigation_transactions_ros.py',
       'test_artifacts/reset_intent_repair_20261019/run_regression.py']
original=json.loads((out/'before/manifest.json').read_text())
manifest=dict(baseline_bundle='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b',
    before=original,after={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},
    production_deployment=False,field_acceptance=False)
suffix='.portable' if args.portable else ''
with zipfile.ZipFile(out/('reset_intent_delta'+suffix+'.zip'),'x',zipfile.ZIP_DEFLATED) as zf:
    for f in files:zf.write(root/f,f)
    zf.writestr('delta_manifest.json',json.dumps(manifest,indent=2)+'\n')
(out/('delta_manifest'+suffix+'.json')).write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
shutil.copyfile(root/'test_artifacts/offline_navigation/callback_report.json',out/'full_callback_report.json')
print(json.dumps(manifest,indent=2))
