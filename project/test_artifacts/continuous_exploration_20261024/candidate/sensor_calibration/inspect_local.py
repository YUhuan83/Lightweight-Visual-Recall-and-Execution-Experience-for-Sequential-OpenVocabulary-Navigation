"""Read-only inspection of native assets and installed Python APIs; no simulator."""
import argparse
import hashlib
import importlib.metadata
import inspect
import json
from pathlib import Path
import zipfile


def inspect_home(home):
    archive = Path(home) / 'content/levels/smallgrid.zip'
    with zipfile.ZipFile(archive) as z:
        evidence = {}
        for name in z.namelist():
            if name.endswith(('.json', '.mis')) and ('main/' in name or name.endswith('info.json')):
                text = z.read(name).decode('utf-8-sig')
                if 'Plane' in text or name.endswith('info.json'):
                    evidence[name] = text
        rows = [json.loads(line) for name, text in evidence.items() if 'Groundplanes/' in name
                for line in text.splitlines() if line.strip()]
        if not any(r['class'] == 'GroundPlane' and not r.get('hidden', False)
                   and float(r['position'].split()[2]) == 0 for r in rows):
            raise ValueError('smallgrid visible native GroundPlane at z=0 not verified')
    with zipfile.ZipFile(Path(home) / 'content/vehicles/pickup.zip') as z:
        if 'vehicles/pickup/d15_4wd_A.pc' not in z.namelist():
            raise ValueError('Native D-Series configuration missing')
    return dict(archive=str(archive), sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                evidence=evidence, vehicle='vehicles/pickup/d15_4wd_A.pc')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(inspect_home(a.home), indent=2))
    from beamngpy import BeamNGpy, Scenario
    from beamngpy.sensors import Camera, Lidar
    print('beamngpy=' + importlib.metadata.version('beamngpy'))
    for cls, names in ((Camera, ('__init__', 'get_position', 'get_direction', 'world_point_to_pixel',
                                'send_ad_hoc_poll_request', 'collect_ad_hoc_poll_request')),
                       (Lidar, ('__init__', 'get_position', 'get_direction', 'collect_ad_hoc_poll_request')),
                       (Scenario, ('_get_prefab',)), (BeamNGpy, ('open',))):
        for name in names:
            print(cls.__name__, name, inspect.signature(getattr(cls, name)))
