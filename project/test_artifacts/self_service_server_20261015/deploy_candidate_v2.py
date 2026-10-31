import hashlib,json,subprocess,zipfile
from pathlib import Path
root=Path('/home/ly/wildos-console')
archive=Path('/home/ly/wildos_console_17adb4ae.zip')
expected='17adb4ae32ece1415ac83e27bbf19976a3c4d359cfba861a0635a49422167fea'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
assert not (root/'install').exists() and not (root/'console.json').exists()
(root/'bundle').rename(root/'bundle_5c6e501c')
(root/'source_identity.json').rename(root/'source_identity_5c6e501c.json')
(root/'build.log').rename(root/'build_colcon_missing.log')
bundle=root/'bundle';bundle.mkdir()
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and len(z.namelist())==431
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in z.namelist())
    z.extractall(bundle)
    hashes={n:hashlib.sha256((bundle/n).read_bytes()).hexdigest() for n in z.namelist()}
(root/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes),indent=2))
test=subprocess.run(['bash',str(bundle/'tools/test_console_installer.sh')],capture_output=True)
(root/'installer_contract.log').write_bytes(test.stdout+test.stderr)
assert test.returncode==0,test.stdout+test.stderr
with (root/'build.log').open('xb') as log:
    child=subprocess.Popen(['bash',str(bundle/'tools/setup_wildos_console.sh'),str(root),
        '/home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash'],
        cwd=bundle,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'build.pid').write_text(str(child.pid))
print(json.dumps(dict(build_pid=child.pid,sha256=expected,installer_contract='passed')))
