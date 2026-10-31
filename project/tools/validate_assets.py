"""Check downloaded hashes, mesh indices, normals, UVs and texture references."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NS = {'c':'http://www.collada.org/2005/11/COLLADASchema'}


def validate():
    total = 0
    for asset in ('rock_moss_set_01','dead_tree_trunk','wooden_barrels_01'):
        folder = ROOT / 'assets/models' / f'ph_{asset}'
        provenance = json.loads((folder / 'provenance.json').read_text())
        for file in provenance['files']:
            path = ROOT / 'source_assets' / asset / file['path']
            assert hashlib.sha256(path.read_bytes()).hexdigest() == file['sha256']
        dae = ET.parse(folder / 'visual.dae')
        for image in dae.findall('.//c:image/c:init_from',NS):
            assert (folder / image.text).is_file()
        count = 0
        for mesh in dae.findall('.//c:mesh',NS):
            sources = {}
            for source in mesh.findall('c:source',NS):
                array = source.find('c:float_array',NS)
                values = list(map(float,array.text.split()))
                accessor = source.find('c:technique_common/c:accessor',NS)
                stride = int(accessor.get('stride'))
                assert len(values) == int(array.get('count')) == int(accessor.get('count'))*stride
                sources[source.get('id')] = (values,stride)
            tri = mesh.find('c:triangles',NS)
            packed = list(map(int,tri.find('c:p',NS).text.split()))
            faces = int(tri.get('count'))
            assert len(packed) == faces*9
            for inp in tri.findall('c:input',NS):
                ref = inp.get('source')[1:]
                if inp.get('semantic') == 'VERTEX':
                    ref = mesh.find('c:vertices/c:input',NS).get('source')[1:]
                values,stride = sources[ref]
                indices = packed[int(inp.get('offset'))::3]
                assert min(indices)>=0 and max(indices)<len(values)//stride
                if inp.get('semantic') == 'NORMAL':
                    for i in range(0,len(values),3):
                        assert abs(sum(v*v for v in values[i:i+3])-1)<1e-4
            count += faces
        ET.parse(folder / 'model.sdf')
        assert count == json.loads((folder/'conversion.json').read_text())['triangles']
        total += count
        print(f'PASS {asset}: {count} triangles, normals, UVs, textures, source SHA256')
    ET.parse(ROOT/'assets/worlds/asset_samples.sdf')
    print(f'PASS total {total} triangles. Gazebo rendering and collision require VM verification.')


if __name__ == '__main__':
    validate()
