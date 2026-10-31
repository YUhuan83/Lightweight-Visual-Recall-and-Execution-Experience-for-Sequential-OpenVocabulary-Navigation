import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/wildos-console');bundle=root/'bundle'
archive=Path('/home/ly/wildos_console_21fd3518.zip')
expected='21fd35186143d128d24e01106653632c9a8bd0e47fe965b32dfd00eafeeb76d4'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
old=json.loads((root/'source_identity.json').read_text())
assert old['sha256']=='17adb4ae32ece1415ac83e27bbf19976a3c4d359cfba861a0635a49422167fea'
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
    changed={n for n in set(hashes)|set(old['members']) if hashes.get(n)!=old['members'].get(n)}
    assert changed=={'tools/wildos_console_scene.py','tools/test_wildos_console.py'},changed
    bundle.rename(root/'bundle_17adb4ae');bundle.mkdir()
    z.extractall(bundle)
    assert all(hashlib.sha256((bundle/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
(root/'source_identity.json').rename(root/'source_identity_17adb4ae.json')
(root/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes,changed=sorted(changed),installation_unchanged=True),indent=2))
print(json.dumps(dict(sha256=expected,changed=sorted(changed),installation_unchanged=True)))
