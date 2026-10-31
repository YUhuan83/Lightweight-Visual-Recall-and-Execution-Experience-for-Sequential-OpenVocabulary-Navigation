"""Build and verify a source-only upload ZIP. No dependencies beyond stdlib."""
import hashlib
import json
from pathlib import Path
import zipfile


def main():
    here = Path(__file__).resolve().parent
    root = here.parent
    paths = {
        'beamng_pointgoal.py': root/'tools/beamng_pointgoal.py',
        'test_beamng_pointgoal.py': root/'tools/test_beamng_pointgoal.py',
        'README.md': here/'README.md',
        '明日验收.md': here/'明日验收.md',
        'requirements.json': here/'requirements.json',
    }
    data = {name: path.read_bytes() for name, path in paths.items()}
    manifest = dict(format=1, source_only=True, runtime_verified=False,
                    files={name: dict(bytes=len(blob), sha256=hashlib.sha256(blob).hexdigest())
                           for name, blob in data.items()})
    encoded = (json.dumps(manifest, indent=2)+'\n').encode()
    (here/'manifest.json').write_bytes(encoded)
    archive = here/'pointgoal_kit.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as z:
        for name, blob in {**data, 'manifest.json': encoded}.items():
            info = zipfile.ZipInfo('pointgoal_kit/'+name, date_time=(2026, 9, 11, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, blob)
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:
            raise RuntimeError('ZIP integrity failure')
        for name, expected in manifest['files'].items():
            blob = z.read('pointgoal_kit/'+name)
            if hashlib.sha256(blob).hexdigest() != expected['sha256']:
                raise RuntimeError('ZIP source hash mismatch: '+name)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (here/'pointgoal_kit.zip.sha256').write_text(digest+'  pointgoal_kit.zip\n', encoding='ascii')
    print('Verified source-only ZIP:', archive)
    print('SHA256:', digest)


if __name__ == '__main__':
    main()
