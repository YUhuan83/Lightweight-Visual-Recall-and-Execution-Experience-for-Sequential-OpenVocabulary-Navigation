import hashlib,json,subprocess,time,zipfile
from pathlib import Path
root=Path('/home/ly/wildos-console')
archive=Path('/home/ly/wildos_console_5c6e501c.zip')
expected='5c6e501c457ab59b70637e1e15c499b01f4af5825ba32261168b6edf6f2374ae'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
root.mkdir(exist_ok=False)
bundle=root/'bundle';bundle.mkdir()
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and len(z.namelist())==430
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in z.namelist())
    z.extractall(bundle)
    hashes={n:hashlib.sha256((bundle/n).read_bytes()).hexdigest() for n in z.namelist()}
(root/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes),indent=2))
with (root/'build.log').open('xb') as log:
    child=subprocess.Popen(['bash',str(bundle/'tools/setup_wildos_console.sh'),str(root),
        '/home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash'],
        cwd=bundle,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'build.pid').write_text(str(child.pid))
print(json.dumps(dict(build_pid=child.pid,sha256=expected,log=str(root/'build.log'))))
