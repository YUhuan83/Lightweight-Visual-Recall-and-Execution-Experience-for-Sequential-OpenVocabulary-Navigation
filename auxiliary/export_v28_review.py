import difflib,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
out=root/'execution_handoff_repair_20261025'
base=root/'arrival_decoupling_20261025/candidate';candidate=out/'candidate'
scope=json.loads((out/'production_scope_verified.json').read_text())
names=set(scope['changed_members'])|{scope['new_test']}
before={};diff=[]
with zipfile.ZipFile(out/'v28_delta_from_v26.zip','w',zipfile.ZIP_DEFLATED) as z:
    for n in sorted(names):
        old=(base/n).read_text(encoding='utf8') if (base/n).exists() else ''
        new=(candidate/n).read_text(encoding='utf8')
        if (base/n).exists():before[n]=hashlib.sha256((base/n).read_bytes()).hexdigest()
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
            fromfile='v26/'+n if old else '/dev/null',tofile='v28/'+n))
        z.write(candidate/n,n)
    z.writestr('base_hashes.json',json.dumps(before,indent=2))
(out/'v26_to_v28.diff').write_text(''.join(diff),encoding='utf8')
print(json.dumps(dict(files=len(names),archive_sha256=hashlib.sha256((out/'v28_delta_from_v26.zip').read_bytes()).hexdigest())))
