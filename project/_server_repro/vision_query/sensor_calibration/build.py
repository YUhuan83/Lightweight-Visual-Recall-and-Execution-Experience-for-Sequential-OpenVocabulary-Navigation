"""Build own native mod and source-only migration kit, without launching BeamNG."""
import argparse
import hashlib
import json
import zipfile
from PIL import Image, ImageDraw
from fixture import ROOT, ART, MOD, geometry, scenario
from inspect_local import inspect_home


# Explicit first-party source allowlist; never recursively package a workspace.
SOURCE_FILES = (
    'build.py', 'fixture.py', 'inspect_local.py', 'install.py', 'preview.py', 'run.py',
    'summarize_capture.py', 'start_preview.ps1', 'README.md', 'README_操作入口.md',
    'requirements.txt', 'requirements-visual.txt', 'test_empty_frames.py',
    'test_fixture.py', 'test_imu.py', 'test_preview.py', 'test_summary.py',
)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def build(home):
    evidence = inspect_home(home)
    mesh, manifest = geometry()
    art = ROOT/'generated/mod'/ART.lstrip('/')
    art.mkdir(parents=True, exist_ok=True)
    mesh.write(art/'fixture.dae')
    materials = {}
    colors = dict(neutral=(205,205,205), red=(180,35,25), blue=(25,65,185), green=(35,140,55))
    for name in (*colors, 'checker'):
        if name == 'checker':
            im = Image.new('RGB', (2048,2048), (225,225,225))
            draw = ImageDraw.Draw(im)
            for y in range(7):
                for x in range(10):
                    if (x+y)%2 == 0:
                        draw.rectangle((round(x*2048/10), round(y*2048/7),
                                        round((x+1)*2048/10)-1, round((y+1)*2048/7)-1), fill=(18,18,18))
        else:
            im = Image.new('RGB', (16,16), colors[name])
        im.save(art/(name+'.color.png'))
        mat = 'nzs_'+name
        materials[mat] = dict(name=mat, mapTo=mat, **{'class':'Material'}, version=1.5,
            Stages=[dict(baseColorMap=ART+name+'.color.png', baseColorFactor=[1,1,1,1],
                         metallicFactor=0, roughnessFactor=1)], doubleSided=True, groundType='ASPHALT')
    write_json(art/'main.materials.json', materials)
    write_json(ROOT/'generated/fixture.json', manifest)
    # Only hashes and structural observations, never native game source in the kit.
    write_json(ROOT/'generated/local_evidence.json', dict(smallgrid_sha256=evidence['sha256'],
               native_ground_z=0, vehicle=evidence['vehicle'], beamngpy='1.36', runtime_observed=False))
    scene, _ = scenario(manifest)
    prefab = scene._get_prefab()
    (ROOT/'generated/fixture.prefab.json').write_text(prefab, encoding='utf-8')
    (ROOT/'dist').mkdir(exist_ok=True)
    with zipfile.ZipFile(ROOT/'dist'/MOD, 'w', zipfile.ZIP_DEFLATED) as z:
        for path in sorted(art.iterdir()):
            entry = zipfile.ZipInfo(path.relative_to(ROOT/'generated/mod').as_posix(), (2026,9,11,0,0,0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(entry, path.read_bytes())
    digest = hashlib.sha256((ROOT/'dist'/MOD).read_bytes()).hexdigest()
    write_json(ROOT/'generated/package.json', dict(mod=MOD, sha256=digest, calibrated=False))
    with zipfile.ZipFile(ROOT/'dist/sensor_calibration_source.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for name in sorted(SOURCE_FILES):
            path = ROOT / name
            if path.is_symlink() or not path.is_file() or path.suffix not in ('.py', '.md', '.txt', '.ps1'):
                raise ValueError('Invalid source allowlist entry: ' + name)
            z.write(path, 'sensor_calibration/'+name)
    print('Prepared:', ROOT/'dist'/MOD)
    print('SHA256:', digest)
    print('No game connection, install or runtime calibration performed.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home', required=True)
    build(p.parse_args().home)
