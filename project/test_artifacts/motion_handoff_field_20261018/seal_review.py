from pathlib import Path
import hashlib,json,zipfile
out=Path(__file__).parent
root=out.parents[1]
files=[p for p in out.iterdir() if p.is_file() and p.suffix in ('.py','.json','.log')]
files += [root/'docs/WHOLE_FLOW_REPAIR_20261018.md',root/'docs/WHOLE_FLOW_AUDIT_20261017.md',
    root/'nav2_adapter/wildos_nav2_adapter/continuous_path.py',
    root/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    root/'nav2_adapter/test/test_motion_intent_handoff_ros.py']
files=sorted(set(files))
manifest={p.relative_to(root).as_posix():dict(bytes=p.stat().st_size,
    sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files}
archive=out/'review_supplement.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(p,p.relative_to(root).as_posix())
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name,item in manifest.items():assert hashlib.sha256(z.read(name)).hexdigest()==item['sha256']
identity=dict(payloads=len(manifest),bytes=archive.stat().st_size,
    sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),crc_and_members_verified=True)
(out/'supplement_identity.json').write_text(json.dumps(identity,indent=2)+'\n',encoding='utf8')
print(json.dumps(identity,indent=2))
