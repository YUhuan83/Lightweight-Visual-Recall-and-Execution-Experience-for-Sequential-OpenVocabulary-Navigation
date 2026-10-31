import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\fresh_bearing_repair_20261025')
candidate=root/'candidate'
visual='wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
names=[visual+'/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py',
       visual+'/test/test_fresh_bearing.py']
hashes=json.loads((root/'base_hashes.json').read_text())
base=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\arrival_decoupling_20261025\candidate')
# v26's added arrival regression is not in the older exported manifest.
extra='nav2_adapter/test/test_body_arrival_ros.py'
if not (candidate/extra).exists():
    (candidate/extra).write_bytes((base/extra).read_bytes())
hashes[extra]=hashlib.sha256((base/extra).read_bytes()).hexdigest()
(root/'base_hashes.json').write_text(json.dumps(hashes,indent=2))
with zipfile.ZipFile(root/'fresh_bearing_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:hashes[n] for n in names if n in hashes}))
    for n in names:z.write(candidate/n,n)
print(json.dumps(dict(changed=names,sha256=hashlib.sha256((root/'fresh_bearing_delta.zip').read_bytes()).hexdigest())))
