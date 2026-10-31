"""Import three CC0 Poly Haven samples. Source files are cached and verified."""
import hashlib
import json
import shutil
import sys
import time
import urllib.request
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.asset-tools'))
import numpy as np
import trimesh
from build_world import node, write_xml, primitive

IDS = ('rock_moss_set_01', 'dead_tree_trunk', 'wooden_barrels_01')


def fetch(url):
    for attempt in range(3):
        try:
            request = urllib.request.Request(url, headers={'User-Agent': 'WildOS-Local-Asset-Import/1.0'})
            with urllib.request.urlopen(request, timeout=120) as response:
                return response.read()
        except Exception:
            if attempt == 2:
                raise
            time.sleep(3)


def download(asset):
    folder = ROOT / 'source_assets' / asset
    folder.mkdir(parents=True, exist_ok=True)
    meta_path = folder / 'files.json'
    if not meta_path.exists():
        meta_path.write_bytes(fetch(f'https://api.polyhaven.com/files/{asset}'))
    meta = json.loads(meta_path.read_text())['gltf']['1k']['gltf']
    entries = {f'{asset}_1k.gltf': meta, **meta['include']}
    records = []
    for relative, info in entries.items():
        target = (folder / relative).resolve()
        assert target.is_relative_to(folder.resolve())
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists() or hashlib.md5(target.read_bytes()).hexdigest() != info['md5']:
            print(f'Download {asset}/{relative}', flush=True)
            data = fetch(info['url'])
            assert len(data) == info['size']
            assert hashlib.md5(data).hexdigest() == info['md5']
            target.write_bytes(data)
        records.append(dict(path=relative, url=info['url'], md5=info['md5'],
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    (folder / 'provenance.json').write_text(json.dumps(dict(
        asset=asset, source=f'https://polyhaven.com/a/{asset}', license='CC0-1.0',
        license_url='https://polyhaven.com/license', files=records), indent=2))
    return folder / f'{asset}_1k.gltf'


def convert(asset, source):
    scene = trimesh.load_scene(source, process=False)
    meshes = []
    # glTF is Y-up, Gazebo is Z-up. Bake graph transforms before recentering.
    rotate = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]], dtype=float)
    for name in scene.graph.nodes_geometry:
        transform, geometry = scene.graph[name]
        mesh = scene.geometry[geometry].copy()
        mesh.apply_transform(rotate @ transform)
        meshes.append(mesh)
    assert meshes
    bounds = np.array([m.bounds for m in meshes])
    low, high = bounds[:,0].min(axis=0), bounds[:,1].max(axis=0)
    origin = np.array([(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]])
    out = ROOT / 'assets/models' / f'ph_{asset}'
    out.mkdir(parents=True, exist_ok=True)
    dae = ET.Element('COLLADA', xmlns='http://www.collada.org/2005/11/COLLADASchema', version='1.4.1')
    asset_node = node(dae, 'asset')
    node(asset_node, 'created', '2026-09-09T00:00:00Z')
    node(asset_node, 'modified', '2026-09-09T00:00:00Z')
    node(asset_node, 'unit', name='meter', meter='1')
    node(asset_node, 'up_axis', 'Z_UP')
    images, effects, materials, geometries, scenes = [node(dae, tag) for tag in (
        'library_images','library_effects','library_materials','library_geometries','library_visual_scenes')]
    visual_scene = node(scenes, 'visual_scene', id='Scene')
    sdf = ET.Element('sdf', version='1.8')
    model = node(sdf, 'model', name=f'ph_{asset}')
    node(model, 'static', 'true')
    link = node(model, 'link', name='body')
    tri_count = 0
    for i, mesh in enumerate(meshes):
        mesh.apply_translation(-origin)
        vertices, normals, faces = mesh.vertices, mesh.vertex_normals, mesh.faces
        assert np.isfinite(vertices).all() and np.isfinite(normals).all()
        uv = mesh.visual.uv
        assert uv is not None and len(uv) == len(vertices)
        material = mesh.visual.material
        image = material.baseColorTexture
        assert image is not None, f'Missing diffuse texture: {asset}/{i}'
        texture_file = f'diffuse_{i}.png'
        image.convert('RGB').save(out / texture_file)
        node(node(images, 'image', id=f'image{i}'), 'init_from', texture_file)
        profile = node(node(effects, 'effect', id=f'effect{i}'), 'profile_COMMON')
        surface = node(node(profile, 'newparam', sid=f'surface{i}'), 'surface', type='2D')
        node(surface, 'init_from', f'image{i}')
        sampler = node(node(profile, 'newparam', sid=f'sampler{i}'), 'sampler2D')
        node(sampler, 'source', f'surface{i}')
        phong = node(node(profile, 'technique', sid='common'), 'phong')
        node(node(phong, 'diffuse'), 'texture', texture=f'sampler{i}', texcoord='UVMap')
        node(node(phong, 'specular'), 'color', '0.05 0.05 0.05 1')
        node(node(materials, 'material', id=f'mat{i}'), 'instance_effect', url=f'#effect{i}')
        geom = node(node(geometries, 'geometry', id=f'geo{i}'), 'mesh')
        for label, values, params in [('pos', vertices, 'XYZ'),('nor', normals, 'XYZ'),('uv', uv, 'ST')]:
            source_id = f'{label}{i}'
            src = node(geom, 'source', id=source_id)
            node(src, 'float_array', ' '.join(f'{v:.7g}' for v in values.ravel()), id=f'{source_id}-array', count=values.size)
            accessor = node(node(src, 'technique_common'), 'accessor', source=f'#{source_id}-array', count=len(values), stride=len(params))
            for param in params:
                node(accessor, 'param', name=param, type='float')
        node(node(geom, 'vertices', id=f'vert{i}'), 'input', semantic='POSITION', source=f'#pos{i}')
        triangles = node(geom, 'triangles', material=f'symbol{i}', count=len(faces))
        node(triangles, 'input', semantic='VERTEX', source=f'#vert{i}', offset=0)
        node(triangles, 'input', semantic='NORMAL', source=f'#nor{i}', offset=1)
        node(triangles, 'input', semantic='TEXCOORD', source=f'#uv{i}', offset=2, set=0)
        node(triangles, 'p', ' '.join(f'{v} {v} {v}' for v in faces.ravel()))
        inst = node(node(visual_scene, 'node', id=f'node{i}'), 'instance_geometry', url=f'#geo{i}')
        binding = node(node(node(inst, 'bind_material'), 'technique_common'), 'instance_material', symbol=f'symbol{i}', target=f'#mat{i}')
        node(binding, 'bind_vertex_input', semantic='UVMap', input_semantic='TEXCOORD', input_set=0)
        # Per-component boxes avoid blocking the empty gaps of a multi-object set.
        size = np.maximum(mesh.extents, .02)
        center = mesh.bounds.mean(axis=0)
        collision = node(link, 'collision', name=f'proxy{i}')
        node(collision, 'pose', ' '.join(map(str, (*center,0,0,0))))
        node(node(node(collision, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
        tri_count += len(faces)
    node(node(dae, 'scene'), 'instance_visual_scene', url='#Scene')
    write_xml(out / 'visual.dae', dae)
    node(node(node(node(link, 'visual', name='scan'), 'geometry'), 'mesh'), 'uri', f'model://ph_{asset}/visual.dae')
    write_xml(out / 'model.sdf', sdf)
    config = ET.Element('model')
    node(config, 'name', f'Poly Haven: {asset}')
    node(config, 'version', '1.0')
    node(config, 'sdf', 'model.sdf', version='1.8')
    node(config, 'description', f'CC0 Poly Haven / {asset}; diffuse conversion, simplified box collisions.')
    write_xml(out / 'model.config', config)
    shutil.copyfile(source.parent / 'provenance.json', out / 'provenance.json')
    report = dict(asset=asset,triangles=tri_count,parts=len(meshes),size_m=(high-low).tolist(),
                  texture_resolution='1k',material='diffuse only; normal and roughness retained in source files',
                  collision='per-mesh bounding boxes, not shape-accurate')
    (out / 'conversion.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report), flush=True)
    return report


def sample_world():
    sdf = ET.Element('sdf', version='1.8')
    world = node(sdf,'world',name='asset_samples')
    for short, cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        node(world,'plugin',filename=f'ignition-gazebo-{short}-system',name=f'ignition::gazebo::systems::{cls}')
    scene = node(world,'scene')
    node(scene,'ambient','.65 .65 .65 1')
    node(scene,'background','.65 .75 .85 1')
    node(scene,'shadows','false')
    light = node(world,'light',name='sun',type='directional')
    node(light,'direction','-.3 -.4 -1')
    node(light,'diffuse','.9 .9 .9 1')
    node(light,'cast_shadows','false')
    ground = node(world,'model',name='ground')
    node(ground,'static','true')
    link = node(ground,'link',name='body')
    primitive(link,'ground','box',(40,40,.2),(0,0,-.1,0,0,0),(.42,.43,.38))
    for asset, x in zip(IDS,(-7,0,7)):
        inc = node(world,'include')
        node(inc,'uri',f'model://ph_{asset}')
        node(inc,'pose',f'{x} 0 .02 0 0 0')
    write_xml(ROOT / 'assets/worlds/asset_samples.sdf', sdf)


if __name__ == '__main__':
    print('Source: Poly Haven https://polyhaven.com / CC0 https://polyhaven.com/license',flush=True)
    for asset in IDS:
        convert(asset, download(asset))
    sample_world()
