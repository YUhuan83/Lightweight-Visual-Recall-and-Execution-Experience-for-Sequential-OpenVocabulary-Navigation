from pathlib import Path
import json,hashlib,subprocess,shutil
root=Path('/home/ly/execution_continuity_review_20261022/v5')
report=json.loads((root/'real_ros_report.json').read_text())
print(json.dumps({'real_ros_tests':sum(r['tests'] for r in report),
    'failures':sum(r['failures'] for r in report),'errors':sum(r['errors'] for r in report),
    'native_status_tests':[r for r in report if r['suite']=='test_native_status_executor_ros']}),flush=True)
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
bad=[name for name,sha in manifest['members'].items()
     if hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()!=sha]
assert not bad,bad
print(json.dumps({'frozen_members_verified':len(manifest['members']),
    'display148_lock':Path('/tmp/.X148-lock').exists(),
    'disk_free_gb':shutil.disk_usage(root).free/1e9}),flush=True)
print((root/'full/config.json').read_text(),flush=True)
print((root/'full/driver_full.py').read_text(),flush=True)
