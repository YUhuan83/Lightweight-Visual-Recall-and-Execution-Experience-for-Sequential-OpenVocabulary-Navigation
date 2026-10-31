"""Original fixture geometry, native scenario and camera diagnostic mathematics."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
ART = '/levels/smallgrid/art/shapes/nz_sensor_calibration/'
MOD = 'nz_sensor_calibration.zip'


class Mesh:
    def __init__(self):
        self.points, self.normals, self.uvs, self.groups = [], [], [], {}

    def quad(self, points, material):
        a, b, c, d = np.asarray(points, dtype=float)
        normal = np.cross(b-a, c-a)
        normal /= np.linalg.norm(normal)
        for ids in ((0, 1, 2), (0, 2, 3)):
            for i in ids:
                self.groups.setdefault(material, []).append(len(self.points))
                self.points.append(points[i])
                self.normals.append(normal)
                self.uvs.append(((0, 0), (1, 0), (1, 1), (0, 1))[i])

    def box(self, center, size, material):
        points = [tuple(center[i]+s[i]*size[i]/2 for i in range(3)) for s in
                  ((-1,-1,-1),(1,-1,-1),(-1,1,-1),(1,1,-1),
                   (-1,-1,1),(1,-1,1),(-1,1,1),(1,1,1))]
        for ids in ((0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)):
            self.quad([points[i] for i in ids], material)

    def write(self, path):
        ns = 'http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('', ns)
        def e(parent, tag, text=None, **attrs):
            child = ET.SubElement(parent, '{'+ns+'}'+tag, {k:str(v) for k,v in attrs.items()})
            child.text = text
            return child
        root = ET.Element('{'+ns+'}COLLADA', version='1.4.1')
        asset = e(root, 'asset')
        e(asset, 'created', '2026-09-11T00:00:00Z')
        e(asset, 'modified', '2026-09-11T00:00:00Z')
        e(asset, 'unit', name='meter', meter=1)
        e(asset, 'up_axis', 'Z_UP')
        effects, materials = e(root, 'library_effects'), e(root, 'library_materials')
        for mat in self.groups:
            tech = e(e(e(effects, 'effect', id=mat+'-fx'), 'profile_COMMON'), 'technique', sid='common')
            e(e(e(tech, 'lambert'), 'diffuse'), 'color', '0.5 0.5 0.5 1')
            e(e(materials, 'material', id=mat, name=mat), 'instance_effect', url='#'+mat+'-fx')
        mesh = e(e(e(root, 'library_geometries'), 'geometry', id='geometry'), 'mesh')
        for name, values, params in (('positions',self.points,'XYZ'),('normals',self.normals,'XYZ'),('uv',self.uvs,'ST')):
            src = e(mesh, 'source', id=name)
            e(src, 'float_array', ' '.join(f'{x:.8f}' for v in values for x in v), id=name+'-array', count=len(values)*len(params))
            acc = e(e(src, 'technique_common'), 'accessor', source='#'+name+'-array', count=len(values), stride=len(params))
            for p in params:
                e(acc, 'param', name=p, type='float')
        e(e(mesh, 'vertices', id='vertices'), 'input', semantic='POSITION', source='#positions')
        for mat, indices in self.groups.items():
            tri = e(mesh, 'triangles', material=mat, count=len(indices)//3)
            for off, sem, src in ((0,'VERTEX','vertices'),(1,'NORMAL','normals'),(2,'TEXCOORD','uv')):
                e(tri, 'input', semantic=sem, source='#'+src, offset=off, **({'set':0} if off==2 else {}))
            e(tri, 'p', ' '.join(str(i) for i in indices for _ in range(3)))
        scene = e(e(root, 'library_visual_scenes'), 'visual_scene', id='Scene')
        node = e(scene, 'node', id='fixture300', name='fixture300', type='NODE')
        bind = e(e(e(node, 'instance_geometry', url='#geometry'), 'bind_material'), 'technique_common')
        for mat in self.groups:
            im = e(bind, 'instance_material', symbol=mat, target='#'+mat)
            e(im, 'bind_vertex_input', semantic='UVMap', input_semantic='TEXCOORD', input_set=0)
        e(e(root, 'scene'), 'instance_visual_scene', url='#Scene')
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def geometry():
    m = Mesh()
    # A closed slab, top z=.1, avoids z-fighting with native ground at z=0.
    m.box((0, -3, .05), (24, 30, .1), 'nzs_neutral')
    boxes = [dict(center=[-3,-6,.85], size=[1,1,1.5], material='nzs_red'),
             dict(center=[3,-8,1.1], size=[1.2,1.2,2], material='nzs_blue'),
             dict(center=[-3,-11,.6], size=[1.5,1,1], material='nzs_green')]
    for b in boxes:
        m.box(b['center'], b['size'], b['material'])
    # Front face y=-10 looks toward +Y; texture covers exactly 10 by 7 squares.
    m.box((0,-10.05,2.35), (5,.1,3.5), 'nzs_neutral')
    m.quad([(2.5,-9.999,.6),(-2.5,-9.999,.6),(-2.5,-9.999,4.1),(2.5,-9.999,4.1)], 'nzs_checker')
    points = [[-2.5+c*.5,-9.999,.6+r*.5] for r in range(1,7) for c in range(1,10)]
    return m, dict(level='smallgrid', scope='calibration_only_not_navigation',
                   spawn=dict(pos=[0,4,.65], rot_quat=[0,0,0,1]), boxes=boxes,
                   camera=dict(pos=[0,0,2.35], dir=[0,-1,0], up=[0,0,1]),
                   checker=dict(inner=[9,6], square_m=.5, points_world=points),
                   slab_top_z=.1, calibrated=False)


def scenario(manifest, name='nzs_calibration'):
    from beamngpy import Scenario, Vehicle, ScenarioObject
    scene = Scenario('smallgrid', name, description='Original sensor calibration fixture')
    car = Vehicle('nzs_ego', model='pickup', part_config='vehicles/pickup/d15_4wd_A.pc')
    scene.add_vehicle(car, pos=tuple(manifest['spawn']['pos']),
                      rot_quat=tuple(manifest['spawn']['rot_quat']))
    scene.add_object(ScenarioObject('nzs_fixture', 'nzs_fixture', 'TSStatic',
                     pos=tuple([0,0,0]), scale=tuple([1,1,1]), rot_quat=tuple([0,0,0,1]),
                     shapeName=ART+'fixture.dae', collisionType='Visible Mesh', decalType='Visible Mesh'))
    return scene, car


def coordinates(points, position, direction, up):
    f = np.asarray(direction, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    down = np.cross(f, r)
    q = (np.asarray(points)-position) @ np.stack((r, down, f)).T
    if not np.isfinite(q).all() or np.any(q[:,2] <= .05):
        raise ValueError('Invalid pose or targets behind camera')
    return q[:,:2]/q[:,2,None]


def fit_intrinsics(xy, pixels, resolution=(960,640), fov_y=60):
    xy, pixels = np.asarray(xy), np.asarray(pixels)
    if not np.isfinite(pixels).all() or xy.shape != pixels.shape or len(xy)<12:
        raise ValueError('Invalid correspondences')
    train = np.arange(len(xy)) % 3 != 0
    parameters, predicted = [], np.empty_like(pixels, dtype=float)
    for axis in range(2):
        design = np.column_stack((xy[:,axis], np.ones(len(xy))))
        params, _, rank, _ = np.linalg.lstsq(design[train], pixels[train,axis], rcond=None)
        if rank != 2:
            raise ValueError('Degenerate target distribution')
        parameters.append(params.tolist())
        predicted[:,axis] = design @ params
    error = np.linalg.norm(predicted-pixels, axis=1)
    nominal = resolution[1]/(2*np.tan(np.deg2rad(fov_y)/2))
    fx, cx = parameters[0]
    fy, cy = parameters[1]
    passed = (max(error[~train]) <= 2 and abs(fx/nominal-1)<.03 and abs(fy/nominal-1)<.03
              and abs(cx-resolution[0]/2)<3 and abs(cy-resolution[1]/2)<3)
    return dict(fx=fx, fy=fy, cx=cx, cy=cy, nominal_focal=nominal,
                train_indices=np.flatnonzero(train).tolist(), holdout_indices=np.flatnonzero(~train).tolist(),
                holdout_max_px=float(max(error[~train])), passed=bool(passed)), predicted
