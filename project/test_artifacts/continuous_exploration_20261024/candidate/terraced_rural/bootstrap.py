"""One-time exact copies of original workspace-authored helpers; no source writes."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path(__file__).resolve().parent
source = root.parent / 'standalone_rural'
copies = {'mesh.py': 'mesh.py', 'build.py': 'base.py', 'install.py': 'install.py',
          'run.py': 'run.py', 'verify.py': 'verify.py', 'test_standalone.py': 'test_original.py'}
records = {}
for src, dst in copies.items():
    target = root / dst
    if target.exists():
        raise ValueError('Refusing existing file: ' + str(target))
    shutil.copyfile(source / src, target)
    records[dst] = {'source': 'standalone_rural/' + src,
                    'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                    'license': 'Original workspace-authored content, copied with user permission'}
(root / 'helper_provenance.json').write_text(json.dumps(records, indent=2), encoding='ascii')
