import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/wildos-console');bundle=root/'bundle'
archive=Path('/home/ly/wildos_console_ff0fa6e5.zip')
expected='ff0fa6e512f21a67e1605ea960df15148edac385db6b2a55c84cb1f5f8785a43'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
old=json.loads((root/'source_identity.json').read_text())
assert old['sha256']=='21fd35186143d128d24e01106653632c9a8bd0e47fe965b32dfd00eafeeb76d4'
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
    changed={n for n in set(hashes)|set(old['members']) if hashes.get(n)!=old['members'].get(n)}
    assert changed=={'tools/wildos_console.py','tools/test_wildos_console.py','tools/test_town_trial_harness.py'},changed
    bundle.rename(root/'bundle_21fd3518');bundle.mkdir()
    z.extractall(bundle)
    assert all(hashlib.sha256((bundle/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
(root/'source_identity.json').rename(root/'source_identity_21fd3518.json')
(root/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes,changed=sorted(changed),installation_unchanged=True),indent=2))
print(json.dumps(dict(sha256=expected,changed=sorted(changed),installation_unchanged=True)))
