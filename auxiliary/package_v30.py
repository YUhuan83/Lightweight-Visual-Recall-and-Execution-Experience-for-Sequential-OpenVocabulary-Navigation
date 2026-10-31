import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\confirmation_window_repair_20261025')
base=json.loads((root/'base_hashes.json').read_text());c=root/'candidate'
changed={n for n,h in base.items() if hashlib.sha256((c/n).read_bytes()).hexdigest()!=h}
assert changed=={'official_adapter/launch/official_graph_stack.launch.py'}
added={'nav2_adapter/test/test_confirmation_profile_ros.py','nav2_adapter/test/fixtures/v29_r2_sparse_localizations.json'}
with zipfile.ZipFile(root/'confirmation_profile_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed}))
    for n in sorted(changed|added):z.write(c/n,n)
print(json.dumps(dict(changed=sorted(changed),added=sorted(added))))
