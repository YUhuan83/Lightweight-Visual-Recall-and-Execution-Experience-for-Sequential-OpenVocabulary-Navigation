"""Check dependency closure using Linux CPython 3.10 markers, not host markers."""
from pathlib import Path
import email
import hashlib
import json
import zipfile
from pip._vendor.packaging.requirements import Requirement
from pip._vendor.packaging.utils import canonicalize_name

root = Path(__file__).resolve().parent / 'wheelhouse-linux310'
env = dict(os_name='posix', sys_platform='linux', platform_machine='x86_64',
           platform_system='Linux', python_version='3.10', python_full_version='3.10.12',
           implementation_name='cpython', implementation_version='3.10.12',
           platform_python_implementation='CPython', platform_release='', platform_version='')
packages = {}
for path in root.glob('*.whl'):
    with zipfile.ZipFile(path) as archive:
        meta = email.message_from_bytes(archive.read(next(n for n in archive.namelist() if n.endswith('.dist-info/METADATA'))))
    packages[canonicalize_name(meta['Name'])] = (meta, path)
queue = [Requirement('beamngpy==1.36')]
seen = set()
missing = []
while queue:
    req = queue.pop()
    name = canonicalize_name(req.name)
    key = (name, tuple(sorted(req.extras)))
    if name not in packages:
        missing.append(str(req))
        continue
    meta, path = packages[name]
    if not req.specifier.contains(meta['Version'], prereleases=True):
        missing.append(f'{req}: available {meta["Version"]}')
    if key in seen:
        continue
    seen.add(key)
    for dependency in meta.get_all('Requires-Dist', []):
        child = Requirement(dependency)
        if child.marker is None or any(child.marker.evaluate(dict(env, extra=extra)) for extra in ('', *req.extras)):
            queue.append(child)
if missing:
    raise SystemExit('Missing: ' + '\n'.join(sorted(set(missing))))
report = dict(target='Linux x86_64 CPython 3.10.12', dependency_closure_passed=True,
              runtime_import_tested=False, files=[dict(file=p.name, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for _,p in packages.values()])
(root/'manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(f'PASS dependency closure: {len(packages)} wheels, {sum(p.stat().st_size for _,p in packages.values())/1e6:.1f} MB. Linux runtime not tested.')
