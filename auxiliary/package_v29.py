import hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\target_identity_repair_20261025');c=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text())
changed={n for n,h in base.items() if hashlib.sha256((c/n).read_bytes()).hexdigest()!=h}
v='wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
expected={v+'/visual_navigation/explorfm_triangulation/localization_geometry.py',
 v+'/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py',
 v+'/test/test_localization_geometry.py','nav2_adapter/test/test_surface_association_ros.py'}
assert changed==expected,changed
added={v+'/test/test_range_consistency.py',v+'/test/test_depth_reference.py',v+'/test/fixtures/v28_localization_callback.json'}
added|={p.relative_to(c).as_posix() for p in (c/v/'test/fixtures').glob('depth_fixture_*.npz')}
with zipfile.ZipFile(root/'range_consistency_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed}))
    for n in sorted(changed|added):z.write(c/n,n)
receipt=dict(changed={n:hashlib.sha256((c/n).read_bytes()).hexdigest() for n in sorted(changed|added)},
             archive_sha256=hashlib.sha256((root/'range_consistency_delta.zip').read_bytes()).hexdigest())
(root/'delta_receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(dict(files=len(changed|added),sha256=receipt['archive_sha256'])))
