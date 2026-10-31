"""Standalone rural lane. Sources: Poly Haven (CC0); generated geometry is original.

python tools/build_lane_sample.py --download  # explicit, limited network access
python tools/build_lane_sample.py             # deterministic offline build + validation
build() returns the output Path; no robot or sensor systems are inserted.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.asset-tools'))
import trimesh

MODELS = ROOT / 'assets/models'
SOURCE = ROOT / 'source_assets/lane_textures'
WORLD = ROOT / 'assets/worlds/lane_sample.sdf'
UA = 'RuralLaneSample-LocalBuilder/1.0 (Poly Haven CC0 asset integration)'
SOURCES = {'concrete': ('concrete_floor_02', '2k'),
           'ground': ('brown_mud_leaves_01', '1k'),
           'wood': ('weathered_brown_planks', '1k')}


def node(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e


def xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def fetch(url):
    for attempt in range(3):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': UA}), timeout=90) as r:
                return r.read()
        except Exception:
            if attempt == 2:
                raise
            time.sleep(2 + attempt)


def download():
    print('Textures by Poly Haven https://polyhaven.com | CC0 https://polyhaven.com/license', flush=True)
    SOURCE.mkdir(parents=True, exist_ok=True)
    for role, (asset, resolution) in SOURCES.items():
        folder = SOURCE / asset
        folder.mkdir(exist_ok=True)
        metadata = folder / 'files.json'
        if not metadata.exists():
            metadata.write_bytes(fetch(f'https://api.polyhaven.com/files/{asset}'))
        files = json.loads(metadata.read_text())
        records = []
        # All available JPG map types at one bounded resolution, never previews.
        for channel, resolutions in files.items():
            info = resolutions.get(resolution, {}).get('jpg')
            if not info:
                continue
            target = folder / info['url'].rsplit('/', 1)[1]
            if not target.exists() or hashlib.md5(target.read_bytes()).hexdigest() != info['md5']:
                print('Download', target.name, flush=True)
                data = fetch(info['url'])
                assert len(data) == info['size'] and hashlib.md5(data).hexdigest() == info['md5']
                target.write_bytes(data)
            data = target.read_bytes()
            assert len(data) == info['size'] and hashlib.md5(data).hexdigest() == info['md5']
            with Image.open(target) as image:
                image.verify()
            records.append(dict(channel=channel, path=target.name, **info,
                                sha256=hashlib.sha256(data).hexdigest()))
        assert any(r['channel'] == 'Diffuse' for r in records)
        (folder / 'provenance.json').write_text(json.dumps(dict(
            asset=asset, role=role, source=f'https://polyhaven.com/a/{asset}',
            license='CC0-1.0', license_url='https://polyhaven.com/license',
            api_terms='https://polyhaven.com/api-terms', api_info='https://polyhaven.com/our-api',
            user_agent=UA, resolution=resolution, files=records), indent=2), encoding='utf-8')


def textures():
    out = MODELS / 'lane_materials'
    out.mkdir(parents=True, exist_ok=True)
    for role, (asset, _) in SOURCES.items():
        folder = SOURCE / asset
        provenance = json.loads((folder / 'provenance.json').read_text())
        for record in provenance['files']:
            data = (folder / record['path']).read_bytes()
            assert hashlib.md5(data).hexdigest() == record['md5']
            assert hashlib.sha256(data).hexdigest() == record['sha256']
            if record['channel'] == 'Diffuse':
                (out / f'{role}.jpg').write_bytes(data)
    rng = np.random.default_rng(80909)
    noise = rng.normal(0, 5, (512, 512, 1))
    # Original tileable brickwork; geometry provides corners, sills and lintels.
    brick = Image.new('RGB', (512, 512), (139, 128, 111))
    draw = ImageDraw.Draw(brick)
    for row in range(16):
        for col in range(-1, 9):
            x, y = col * 64 + (row % 2) * 32, row * 32
            shade = int(rng.integers(-15, 16))
            draw.rectangle((x + 2, y + 2, x + 61, y + 29), fill=(151+shade, 91+shade, 64+shade))
    Image.fromarray(np.clip(np.asarray(brick)+noise, 0, 255).astype('uint8')).save(out / 'brick.jpg', quality=94)
    for name, base in [('roof', (88, 91, 88)), ('plaster', (201, 188, 157)),
                       ('bark', (91, 76, 51)), ('leaf', (72, 99, 40)), ('metal', (135, 151, 145))]:
        yy, xx = np.mgrid[:512, :512]
        pattern = 4*np.sin(xx*.28)+3*np.sin(yy*.09)
        if name == 'roof':
            pattern -= ((yy % 64 < 4) | (xx % 128 < 3))*25
        if name == 'bark':
            pattern += 14*np.sin(xx*.15+3*np.sin(yy*.02))
        if name == 'leaf':
            pattern += 12*np.sin(xx*.09)*np.sin(yy*.08)
        arr = np.array(base)[None, None, :] + noise + pattern[:, :, None]
        Image.fromarray(np.clip(arr, 0, 255).astype('uint8')).save(out / f'{name}.jpg', quality=92)
    return {name: '../lane_materials/'+name+'.jpg' for name in
            ('concrete', 'ground', 'wood', 'brick', 'roof', 'plaster', 'bark', 'leaf', 'metal')}


class Mesh:
    def __init__(self):
        self.vertices, self.normals, self.uv, self.groups = [], [], [], {}

    def tri(self, points, material, uv=None, normals=None):
        p = np.asarray(points, dtype=float)
        n = np.cross(p[1]-p[0], p[2]-p[0])
        length = np.linalg.norm(n)
        assert length > 1e-10, points
        n /= length
        start = len(self.vertices)
        self.vertices.extend(p.tolist())
        self.normals.extend([n.tolist()]*3 if normals is None else np.asarray(normals).tolist())
        self.uv.extend(uv if uv is not None else [(0, 0), (np.linalg.norm(p[1]-p[0]), 0), (0, np.linalg.norm(p[2]-p[0]))])
        self.groups.setdefault(material, []).extend(range(start, start+3))

    def quad(self, points, material, scale=1):
        u, v = np.linalg.norm(np.subtract(points[1], points[0]))/scale, np.linalg.norm(np.subtract(points[3], points[0]))/scale
        uv = [(0, 0), (u, 0), (u, v), (0, v)]
        for ids in ((0, 1, 2), (0, 2, 3)):
            self.tri([points[i] for i in ids], material, [uv[i] for i in ids])

    def box(self, center, size, material, scale=1):
        x, y, z = center
        a, b, c = np.array(size)/2
        p = [(x-a,y-b,z-c),(x+a,y-b,z-c),(x+a,y+b,z-c),(x-a,y+b,z-c),
             (x-a,y-b,z+c),(x+a,y-b,z+c),(x+a,y+b,z+c),(x-a,y+b,z+c)]
        for ids in ((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)):
            self.quad([p[i] for i in ids], material, scale)

    def branch(self, a, b, radius, material='bark', tip=None, sides=9):
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        axis = b-a
        axis /= np.linalg.norm(axis)
        u = np.cross(axis, (0,1,0) if abs(axis[1]) < .9 else (1,0,0))
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        tip = radius if tip is None else tip
        for i in range(sides):
            d = u*math.cos(i*math.tau/sides)+v*math.sin(i*math.tau/sides)
            e = u*math.cos((i+1)*math.tau/sides)+v*math.sin((i+1)*math.tau/sides)
            self.quad([a+d*radius, a+e*radius, b+e*tip, b+d*tip], material)
            self.tri([a, a+e*radius, a+d*radius], material)
            self.tri([b, b+d*tip, b+e*tip], material)

    def save(self, path, materials):
        dae = ET.Element('COLLADA', xmlns='http://www.collada.org/2005/11/COLLADASchema', version='1.4.1')
        asset = node(dae, 'asset')
        for tag in ('created', 'modified'):
            node(asset, tag, '2026-09-09T00:00:00Z')
        node(asset, 'unit', name='meter', meter=1)
        node(asset, 'up_axis', 'Z_UP')
        images, effects, mats = [node(dae, t) for t in ('library_images','library_effects','library_materials')]
        colors = {'dark': '.075 .065 .05 1', 'glass': '.12 .22 .25 1', 'trim': '.72 .70 .59 1',
                  'rust': '.32 .17 .08 1', 'grass': '.29 .36 .15 1'}
        for name in self.groups:
            profile = node(node(effects, 'effect', id='e_'+name), 'profile_COMMON')
            if name in materials:
                node(node(images, 'image', id='i_'+name), 'init_from', materials[name])
                node(node(node(profile, 'newparam', sid='s_'+name), 'surface', type='2D'), 'init_from', 'i_'+name)
                sampler = node(node(profile, 'newparam', sid='t_'+name), 'sampler2D')
                node(sampler, 'source', 's_'+name)
                node(sampler, 'wrap_s', 'WRAP')
                node(sampler, 'wrap_t', 'WRAP')
            phong = node(node(profile, 'technique', sid='common'), 'phong')
            diffuse = node(phong, 'diffuse')
            if name in materials:
                node(diffuse, 'texture', texture='t_'+name, texcoord='UVMap')
            else:
                node(diffuse, 'color', colors[name])
            node(node(phong, 'specular'), 'color', '.035 .035 .035 1')
            node(node(phong, 'shininess'), 'float', 12)
            node(node(mats, 'material', id='m_'+name), 'instance_effect', url='#e_'+name)
        geom = node(node(node(dae, 'library_geometries'), 'geometry', id='geo'), 'mesh')
        for label, values, axes in [('pos',self.vertices,'XYZ'),('nor',self.normals,'XYZ'),('uv',self.uv,'ST')]:
            src = node(geom, 'source', id=label)
            node(src, 'float_array', ' '.join(f'{v:.7f}' for row in values for v in row), id=label+'_array', count=len(values)*len(axes))
            accessor = node(node(src, 'technique_common'), 'accessor', source='#'+label+'_array', count=len(values), stride=len(axes))
            for axis in axes:
                node(accessor, 'param', name=axis, type='float')
        node(node(geom, 'vertices', id='verts'), 'input', semantic='POSITION', source='#pos')
        for name, indices in self.groups.items():
            tri = node(geom, 'triangles', material=name, count=len(indices)//3)
            for offset, (semantic, source) in enumerate([('VERTEX','#verts'),('NORMAL','#nor'),('TEXCOORD','#uv')]):
                attrs = dict(semantic=semantic, source=source, offset=offset)
                if offset == 2:
                    attrs['set'] = 0
                node(tri, 'input', **attrs)
            node(tri, 'p', ' '.join(f'{i} {i} {i}' for i in indices))
        scene = node(node(dae, 'library_visual_scenes'), 'visual_scene', id='Scene')
        common = node(node(node(node(scene,'node',id='object'),'instance_geometry',url='#geo'),'bind_material'),'technique_common')
        for name in self.groups:
            binding = node(common, 'instance_material', symbol=name, target='#m_'+name)
            node(binding, 'bind_vertex_input', semantic='UVMap', input_semantic='TEXCOORD', input_set=0)
        node(node(dae, 'scene'), 'instance_visual_scene', url='#Scene')
        xml(path, dae)


def model(name, mesh, materials, boxes=(), collision_mesh=None, cylinders=()):
    folder = MODELS / name
    mesh.save(folder / 'visual.dae', materials)
    sdf = ET.Element('sdf', version='1.8')
    m = node(sdf, 'model', name=name)
    node(m, 'static', 'true')
    link = node(m, 'link', name='body')
    node(node(node(node(link,'visual',name='visual'),'geometry'),'mesh'),'uri',f'model://{name}/visual.dae')
    for i, (center, size) in enumerate(boxes):
        c = node(link,'collision',name=f'box_{i}')
        node(c,'pose',' '.join(map(str, (*center,0,0,0))))
        node(node(node(c,'geometry'),'box'),'size',' '.join(map(str,size)))
    for i, (center, radius, length) in enumerate(cylinders):
        c = node(link,'collision',name=f'cylinder_{i}')
        node(c,'pose',' '.join(map(str,(*center,0,0,0))))
        g = node(node(c,'geometry'),'cylinder')
        node(g,'radius',radius)
        node(g,'length',length)
    if collision_mesh is not None:
        collision_mesh.save(folder/'collision.dae', materials)
        node(node(node(node(link,'collision',name='surface'),'geometry'),'mesh'),'uri',f'model://{name}/collision.dae')
    xml(folder/'model.sdf',sdf)
    config = ET.Element('model')
    node(config,'name',name)
    node(config,'version','1.0')
    node(config,'sdf','model.sdf',version='1.8')
    node(config,'description','Original rural sample geometry; textures by Poly Haven (CC0) and original procedural textures. See lane_sample.md.')
    xml(folder/'model.config',config)


def road_z(y):
    return .12 + .025*(1-abs(y)/1.8)


def height(x, y):
    d = abs(y)
    if d <= 1.8:
        return road_z(y)
    if d < 2.65:
        return .12-(d-1.8)*.15
    if d < 4.6:
        return -.015 - .52*max(0, 1-abs(d-3.55)/.9)
    t = min(1, max(0,(d-4.6)/8))
    t = t*t*(3-2*t)
    hills = 4.2*math.exp(-((x+23)**2/150+(y-20)**2/65))
    hills += 3.3*math.exp(-((x-23)**2/180+(y+20)**2/70))
    hills += .12*math.sin(x*.5)*math.sin(y*.6)
    z = t*hills
    for cx, cy, rx, ry in [(-8,8,7,4.5),(11,-8,7,4.5),(20,6,3,3)]:
        blend = min(1, max(0,max(abs(x-cx)-rx,abs(y-cy)-ry)/2))
        z *= blend*blend*(3-2*blend)
    return z


def lane(materials):
    road, collision = Mesh(), Mesh()
    ys = np.linspace(-1.8,1.8,25)
    for slab in range(12):
        left, right = -24+4*slab+.014, -20+4*slab-.014
        # A meandering narrow recessed channel cuts the slab topology, not a decal.
        cracked = slab in (1,4,7,10)
        rows = []
        for y in ys:
            c = (left+right)/2+.25*math.sin(y*5+slab)+.08*math.sin(y*17)
            xs = [left,left+.018,right-.018,right]
            if cracked:
                xs = [left,left+.018,c-.027,c-.009,c+.009,c+.027,right-.018,right]
            row = []
            for j,x in enumerate(xs):
                drop = .009 if j in (0,len(xs)-1) else (.016 if cracked and j in (3,4) else 0)
                row.append((x,y,road_z(y)-drop))
            rows.append(row)
        for a,b in zip(rows,rows[1:]):
            for j in range(len(a)-1):
                pts = [a[j],a[j+1],b[j+1],b[j]]
                mat = 'dark' if cracked and j == 3 else 'concrete'
                for ids in ((0,1,2),(0,2,3)):
                    road.tri([pts[i] for i in ids],mat,[(pts[i][0]/2,pts[i][1]/2) for i in ids])
        road.box(((left+right)/2,0,.005),(right-left,3.6,.17),'concrete',2)
    for x in np.arange(-24,24,4):
        for y0,y1 in ((-1.8,0),(0,1.8)):
            collision.quad([(x,y0,road_z(y0)),(x+4,y0,road_z(y0)),(x+4,y1,road_z(y1)),(x,y1,road_z(y1))],'concrete',2)
    # Dark continuous bed lies below the open joints and crack bottoms.
    road.box((0,0,.045),(48,3.6,.07),'dark')
    model('lane_road',road,materials,collision_mesh=collision)
    terrain = Mesh()
    xs = np.arange(-36,36.01,.75)
    ys = sorted(set(np.arange(-26,26.01,.5).tolist()+[-4.6,-4.45,-3.55,-2.65,-1.8,1.8,2.65,3.55,4.45,4.6,26]))
    for x0,x1 in zip(xs,xs[1:]):
        for y0,y1 in zip(ys,ys[1:]):
            if -24 <= x0 and x1 <= 24 and -1.8 <= y0 and y1 <= 1.8:
                continue
            pts = [(x0,y0,height(x0,y0)),(x1,y0,height(x1,y0)),(x1,y1,height(x1,y1)),(x0,y1,height(x0,y1))]
            for ids in ((0,1,2),(0,2,3)):
                terrain.tri([pts[i] for i in ids],'ground',[(pts[i][0]/3,pts[i][1]/3) for i in ids])
    model('lane_terrain',terrain,materials,collision_mesh=terrain)


def building(name, barn, materials):
    m = Mesh()
    w,d,h = (8,5,3.4) if barn else (7,5,3)
    wall = 'wood' if barn else 'plaster'
    m.box((0,0,.2),(w+.25,d+.25,.4),'brick',2)
    m.box((0,0,h/2+.3),(w,d,h-.6),wall,2)
    m.box((0,0,.7),(w+.015,d+.015,.6),'brick',2)
    ridge = h+1.65
    for y in (-d/2,d/2):
        pts = [(-w/2,y,h),(w/2,y,h),(0,y,ridge)]
        if y > 0:
            pts.reverse()
        m.tri(pts,wall,[(0,0),(w/2,0),(w/4,.825)])
    for side in (-1,1):
        x = side*(w/2+.35)
        pts = [(0,-d/2-.4,ridge+.08),(x,-d/2-.4,h-.08),(x,d/2+.4,h-.08),(0,d/2+.4,ridge+.08)]
        if side < 0:
            pts.reverse()
        m.quad(pts,'roof',1.5)
        m.branch((x,-d/2-.4,h-.08),(x,d/2+.4,h-.08),.07,'metal',sides=10)
        m.branch((x,d/2,h-.08),(x,d/2,.15),.055,'metal')
        for y in (-d/2-.4,d/2+.4):
            m.branch((0,y,ridge),(x,y,h-.1),.07,'wood',sides=4)
    m.branch((0,-d/2-.45,ridge+.1),(0,d/2+.45,ridge+.1),.095,'roof',sides=12)
    # Front and back windows: opaque dark inset, deep jambs, mullions, lintel and sill.
    for side in (-1,1):
        y = side*(d/2+.035)
        for x in (-w*.31,w*.31):
            m.box((x,y,1.95),(1.18,.075,1.2),'glass')
            for dx in (-.64,0,.64):
                m.box((x+dx,y+side*.06,1.95),(.065,.14,1.35),'trim')
            for z in (1.29,1.95,2.61):
                m.box((x,y+side*.07,z),(1.35,.16,.075),'trim')
            m.box((x,y+side*.13,1.25),(1.48,.36,.1),'concrete')
            for dx in (-.91,.91):
                m.box((x+dx,y,1.95),(.4,.10,1.35),'wood',1)
    door_w = 2 if barn else 1.05
    m.box((0,-d/2-.06,1.2),(door_w,.13,2.3),'wood',1.6)
    for x in (-door_w/2-.07,door_w/2+.07):
        m.box((x,-d/2-.15,1.22),(.12,.17,2.5),'trim')
    m.box((0,-d/2-.15,2.46),(door_w+.25,.17,.14),'trim')
    for z in (.55,1.9):
        m.box((0,-d/2-.145,z),(door_w,.06,.10),'dark')
    m.branch((-door_w/2,-d/2-.16,.35),(door_w/2,-d/2-.16,2.2),.055,'wood',sides=4)
    m.box((door_w*.3,-d/2-.22,1.15),(.06,.08,.17),'rust')
    m.box((0,-d/2-.45,.10),(door_w+.5,.9,.2),'concrete',2)
    if not barn:
        m.box((1.6,.7,h+1.35),(.65,.7,1.85),'brick',2)
        m.box((1.6,.7,h+2.29),(.83,.88,.12),'concrete')
        m.box((1.6,.7,h+2.36),(.48,.54,.025),'dark')
        for x in (-1.2,1.2):
            m.box((x,-3.8,1.15),(.12,.12,2.3),'wood')
        m.quad([(-1.5,-4,2.35),(1.5,-4,2.35),(1.5,-2.5,2.7),(-1.5,-2.5,2.7)],'roof',1.5)
    model(name,m,materials,boxes=[((0,0,h/2),(w,d,h)),((0,-d/2-.45,.1),(door_w+.5,.9,.2))])


def tank(materials):
    m = Mesh()
    m.box((0,0,.14),(3,3,.28),'concrete',2)
    for x in (-.8,.8):
        for y in (-.8,.8):
            m.branch((x,y,.28),(x,y,1.25),.075,'metal')
    m.branch((0,0,1.2),(0,0,3.55),1.12,'metal',sides=48)
    for z in (1.3,1.55,2.15,2.75,3.4):
        # Separate narrow hoop geometry, not painted rings.
        for i in range(48):
            a,b = i*math.tau/48,(i+1)*math.tau/48
            m.branch((1.135*math.cos(a),1.135*math.sin(a),z),(1.135*math.cos(b),1.135*math.sin(b),z),.025,'rust',sides=6)
    m.branch((0,0,3.55),(0,0,3.72),.36,'metal',sides=32)
    m.branch((0,0,3.73),(0,0,3.81),.42,'dark',sides=32)
    m.branch((1.05,0,1.5),(1.5,0,1.5),.06,'metal')
    m.branch((1.5,0,1.5),(1.5,0,.35),.06,'metal')
    for y in (-.3,.3):
        m.branch((-1.28,y,.28),(-1.28,y,3.6),.025,'metal')
    for z in np.arange(.45,3.5,.28):
        m.branch((-1.3,-.3,z),(-1.3,.3,z),.02,'metal')
    m.box((0,-1.135,2.45),(.55,.035,.55),'trim')
    m.box((0,-1.16,2.45),(.08,.025,.43),'dark')
    m.box((0,-1.16,2.45),(.43,.025,.08),'dark')
    model('lane_water_tank',m,materials,boxes=[((0,0,.14),(3,3,.28))],cylinders=[((0,0,2.375),1.14,2.35)])


def tree(variant, materials):
    rng = random.Random(912+variant)
    m = Mesh()
    h = rng.uniform(5.5,7.4)
    m.branch((0,0,-.15),(.18,.07,h*.73),.24,tip=.07)
    leaf = trimesh.creation.icosphere(subdivisions=1)
    for i in range(17):
        angle = i*2.399
        reach = rng.uniform(.8,2.1)
        end = np.array((reach*math.cos(angle),reach*math.sin(angle),h*rng.uniform(.53,.95)))
        start = np.array((.1,0,h*rng.uniform(.32,.52)))
        m.branch(start,end,.065,tip=.016)
        for j in range(4):
            center = end+np.array([rng.uniform(-.65,.65),rng.uniform(-.65,.65),rng.uniform(-.25,.55)])
            m.branch(end,center,.018,tip=.008)
            scales = np.array([rng.uniform(.4,.75),rng.uniform(.4,.8),rng.uniform(.5,.9)])
            vertices = leaf.vertices*scales+center
            normals = leaf.vertex_normals/scales
            normals /= np.linalg.norm(normals,axis=1)[:,None]
            uv = np.column_stack((.5+np.arctan2(leaf.vertices[:,1],leaf.vertices[:,0])/math.tau,
                                  .5-np.arcsin(np.clip(leaf.vertices[:,2],-1,1))/math.pi))
            for face in leaf.faces:
                tex = uv[face].copy()
                if np.ptp(tex[:,0]) > .5:
                    tex[tex[:,0]<.5,0] += 1
                m.tri(vertices[face],'leaf',tex.tolist(),normals[face])
    for i in range(6):
        a = i*math.tau/6
        m.branch((0,0,.4),(.65*math.cos(a),.65*math.sin(a),0),.095,tip=.02)
    model(f'lane_tree_{variant}',m,materials,cylinders=[((0,0,h*.34),.25,h*.68)])


def details(materials):
    m = Mesh()
    # Small brick garden boundary, timber rails and a real driveway over each ditch.
    boxes = []
    for cx,side in [(-8,1),(11,-1)]:
        cy = side*3.7
        m.box((cx,cy,.025),(3.2,3.8,.19),'concrete',2)
        boxes.append(((cx,cy,.025),(3.2,3.8,.19)))
        for dx in (-1.72,1.72):
            m.box((cx+dx,cy,.12),(.18,3.6,.22),'brick',2)
        for dx in (-5,-3,3,5):
            m.box((cx+dx,side*5.4,.55),(.32,.32,1.1),'brick',2)
        for dx in (-4,4):
            m.box((cx+dx,side*5.4,.35),(1.8,.22,.7),'brick',2)
            m.box((cx+dx,side*5.4,.73),(1.9,.32,.08),'concrete')
    for x in np.arange(-22,23,2.5):
        y = -6.2
        if 3 < x < 19:
            continue
        m.box((x,y,.65),(.12,.12,1.3),'wood')
        for z in (.45,1.05):
            m.box((x+1.2,y,z),(2.5,.08,.1),'wood')
    rng = random.Random(621)
    for i in range(1800):
        x,y = rng.uniform(-33,33),rng.choice([-1,1])*rng.uniform(2.1,24)
        if any(abs(x-a)<8 and abs(y-b)<5 for a,b in [(-8,8),(11,-8),(20,6)]):
            continue
        z = height(x,y)
        for j in range(3):
            a = rng.uniform(0,math.tau)
            w,h = rng.uniform(.025,.05),rng.uniform(.15,.42)
            p = [(x-w*math.cos(a),y-w*math.sin(a),z),(x+w*math.cos(a),y+w*math.sin(a),z),(x+.08,y+.03,z+h)]
            m.tri(p,'grass')
            m.tri(p[::-1],'grass')
    model('lane_details',m,materials,boxes=boxes)


def build():
    """Rebuild only owned lane outputs, offline. Return assets/worlds/lane_sample.sdf."""
    materials = textures()
    lane(materials)
    building('lane_cottage',False,materials)
    building('lane_barn',True,materials)
    tank(materials)
    for i in range(4):
        tree(i,materials)
    details(materials)
    sdf = ET.Element('sdf',version='1.8')
    world = node(sdf,'world',name='lane_sample')
    physics = node(world,'physics',name='physics',type='ignored')
    node(physics,'max_step_size',.002)
    node(physics,'real_time_factor',1)
    node(world,'gravity','0 0 -9.81')
    for short,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        node(world,'plugin',filename=f'ignition-gazebo-{short}-system',name=f'ignition::gazebo::systems::{cls}')
    scene = node(world,'scene')
    node(scene,'ambient','.48 .49 .47 1')
    node(scene,'background','.66 .77 .84 1')
    node(scene,'shadows','true')
    sun = node(world,'light',name='sun',type='directional')
    node(sun,'pose','0 0 20 0 0 0')
    node(sun,'direction','-.45 .35 -.82')
    node(sun,'diffuse','.92 .87 .77 1')
    node(sun,'specular','.15 .15 .15 1')
    node(sun,'cast_shadows','true')

    def include(asset, name, x=0,y=0,z=0,yaw=0):
        inc = node(world,'include')
        node(inc,'uri','model://'+asset)
        node(inc,'name',name)
        node(inc,'pose',f'{x} {y} {z} 0 0 {yaw}')

    for name in ('road','terrain','details'):
        include('lane_'+name,name)
    include('lane_cottage','cottage',-8,8)
    include('lane_barn','barn',11,-8,yaw=math.pi)
    include('lane_water_tank','water_tank_target',20,6)
    locations = [(-26,9),(-21,13),(-17,8),(-12,17),(-3,16),(4,11),(9,17),(16,14),(26,10),(30,18),
                 (-27,-11),(-21,-8),(-15,-14),(-6,-10),(0,-17),(20,-15),(27,-8),(30,-19)]
    for i,(x,y) in enumerate(locations):
        include(f'lane_tree_{i%4}',f'tree_{i:02}',x,y,height(x,y),i*1.31)
    # One instance per existing scan set, never a repeated high-poly rock field.
    for asset,x,y,yaw in [('ph_rock_moss_set_01',-20,6,.4),('ph_dead_tree_trunk',-17,-7,.7),('ph_wooden_barrels_01',15,-11,0)]:
        if not (MODELS/asset/'model.sdf').exists():
            raise FileNotFoundError('Required existing reusable asset: '+asset)
        include(asset,asset,x,y,height(x,y),yaw)
    xml(WORLD,sdf)
    report = validate()
    (MODELS/'lane_materials/validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    owned = [Path(__file__),ROOT/'lane_sample.md',WORLD]
    owned += sorted(p for p in MODELS.glob('lane_*/*') if p.is_file())
    owned += sorted(p for p in SOURCE.rglob('*') if p.is_file())
    manifest = MODELS/'lane_materials/files.json'
    owned = sorted(set(owned+[manifest]))
    manifest.write_text(json.dumps([p.relative_to(ROOT).as_posix() for p in owned],indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    return WORLD


def validate():
    """Offline XML, reference, triangle, normal, UV and source integrity checks."""
    ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
    report = dict(world='lane_sample',lane_length_m=48,lane_width_m=3.6,terrain_m=[72,52],
                  road_center_z_m=.145,road_edge_z_m=.12,ditch_bottom_z_m=-.535,
                  meshes={},renderer_tested=False)
    pending = [WORLD]
    seen = set()
    while pending:
        path = pending.pop().resolve()
        if path in seen:
            continue
        assert path.is_file(),path
        seen.add(path)
        root = ET.parse(path).getroot()
        for uri in root.findall('.//uri'):
            assert uri.text.startswith('model://'),uri.text
            target = MODELS/uri.text[8:]
            if target.is_dir():
                target /= 'model.sdf'
            assert target.is_file(),target
            if target.suffix in ('.sdf','.dae'):
                pending.append(target)
        if path.suffix != '.dae':
            continue
        for image in root.findall('.//c:image/c:init_from',ns):
            target = path.parent/image.text
            assert target.is_file(),target
            with Image.open(target) as im:
                im.verify()
        if not path.parent.name.startswith('lane_'):
            continue
        geom = root.find('.//c:mesh',ns)
        arrays = {}
        for src in geom.findall('c:source',ns):
            accessor = src.find('c:technique_common/c:accessor',ns)
            values = np.fromstring(src.find('c:float_array',ns).text,sep=' ')
            assert np.isfinite(values).all()
            assert len(values) == int(accessor.attrib['count'])*int(accessor.attrib['stride'])
            arrays[src.attrib['id']] = values.reshape(-1,int(accessor.attrib['stride']))
        assert np.allclose(np.linalg.norm(arrays['nor'],axis=1),1,atol=2e-6)
        count = 0
        for tri in geom.findall('c:triangles',ns):
            semantics = {i.attrib['semantic'] for i in tri.findall('c:input',ns)}
            assert semantics == {'VERTEX','NORMAL','TEXCOORD'}
            binding = root.find(f".//c:instance_material[@symbol='{tri.attrib['material']}']",ns)
            assert binding is not None
            assert binding.find('c:bind_vertex_input',ns).attrib['input_set']=='0'
            idx = np.fromstring(tri.find('c:p',ns).text,dtype=int,sep=' ').reshape(-1,3,3)
            assert len(idx) == int(tri.attrib['count'])
            for offset,key in enumerate(('pos','nor','uv')):
                assert idx[:,:,offset].min() >= 0 and idx[:,:,offset].max() < len(arrays[key])
            p = arrays['pos'][idx[:,:,0]]
            cross = np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0])
            assert (np.linalg.norm(cross,axis=1)>1e-9).all()
            assert (np.einsum('ij,ij->i',cross,arrays['nor'][idx[:,0,1]])>0).all()
            count += len(idx)
        report['meshes'][path.relative_to(ROOT).as_posix()] = dict(triangles=count,
            bounds=[arrays['pos'].min(axis=0).tolist(),arrays['pos'].max(axis=0).tolist()],
            uv_bounds=[arrays['uv'].min(axis=0).tolist(),arrays['uv'].max(axis=0).tolist()])
    root = ET.parse(WORLD).getroot()
    assert root.find('world').attrib['name']=='lane_sample'
    assert not root.findall('.//sensor')
    assert len(root.findall('.//plugin'))==3
    assert {p.attrib['name'].rsplit('::',1)[-1] for p in root.findall('.//plugin')} == {'Physics','UserCommands','SceneBroadcaster'}
    road = report['meshes']['assets/models/lane_road/collision.dae']['bounds']
    assert np.allclose(road,[[-24,-1.8,.12],[24,1.8,.145]])
    assert abs(height(0,1.8)-road_z(1.8))<1e-9
    assert abs(height(0,3.55)+.535)<1e-9
    for name in ('road','terrain','details','cottage','barn','water_tank','tree_0','tree_1','tree_2','tree_3'):
        folder = MODELS/('lane_'+name)
        config = ET.parse(folder/'model.config').getroot()
        assert (folder/config.find('sdf').text).is_file()
    report['resolved_xml_mesh_files'] = len(seen)
    report['source_files_verified'] = 0
    for provenance in SOURCE.glob('*/provenance.json'):
        for record in json.loads(provenance.read_text())['files']:
            data = (provenance.parent/record['path']).read_bytes()
            assert len(data)==record['size']
            assert hashlib.md5(data).hexdigest()==record['md5']
            assert hashlib.sha256(data).hexdigest()==record['sha256']
            report['source_files_verified'] += 1
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download',action='store_true')
    parser.add_argument('--validate-only',action='store_true')
    args = parser.parse_args()
    if args.download:
        download()
    if args.validate_only:
        print(json.dumps(validate(),indent=2))
    else:
        build()
