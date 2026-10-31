"""Build a source/asset-only migration archive with a strict file allowlist."""
import hashlib
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
FILES = ['run.py', 'run.sh', 'run.ps1', 'install_mod.py', 'requirements.txt',
         'course_config.json', 'README.md', '迁移与验收.md',
         'generated/map_objects.json', 'generated/build_manifest.json',
         'generated/nizhen_rural.zip', 'generated/asset_provenance.json',
         'generated/route_samples.json', 'generated/RUNTIME_API.md']


def build():
    for name in FILES:
        path = ROOT/name
        if not path.is_file() or path.is_symlink():
            raise ValueError(f'Missing or redirected package file: {name}')
    out = ROOT/'dist'
    out.mkdir(exist_ok=True)
    target = out/'rural_scene_portable.zip'
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        for name in FILES:
            archive.write(ROOT/name, 'rural_scene/'+name)
    with zipfile.ZipFile(target) as archive:
        assert archive.testzip() is None
        assert set(archive.namelist()) == {'rural_scene/'+name for name in FILES}
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    (out/'rural_scene_portable.sha256').write_text(f'{digest}  {target.name}\n',encoding='ascii')
    print(f'Created {target}, {target.stat().st_size} bytes, SHA256 {digest}')


if __name__ == '__main__':
    build()
