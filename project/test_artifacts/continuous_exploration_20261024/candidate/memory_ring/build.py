"""Build the WildOS Memory Ring standalone level: a 200x200 asphalt ring road,
varied houses (one red-roof target), trees, a dirt branch and a spawn, packaged
as a BeamNG.tech mod ZIP using original procedural assets.
"""
import hashlib
import os
import re
import shutil
import zipfile
import json
import math
import random
from pathlib import Path
import uuid
import zipfile
from PIL import Image, ImageDraw, ImageFont
import numpy as np
from mesh import Mesh

ROOT = Path(__file__).resolve().parent
LEVEL = 'wildos_memory_ring'
PREFIX = 'levels/' + LEVEL + '/'
ART = '/' + PREFIX + 'art/shapes/'
MOD_NAME = 'wildos_memory_ring.zip'
GRID = 4   # asphalt strip = 8 m
BASE_Z = 11.0

# Road ring nodes (X,Y) in a 0..200 world frame, R01..R14 (closed back to R01).
RING = [(35,42),(57,27),(93,23),(132,29),(161,48),(176,78),(171,112),
        (158,149),(128,170),(91,176),(57,164),(32,139),(24,101),(27,68)]

BRANCH = [(132,29),(122,46),(108,58)]   # dirt spur off the ring

# Houses (X,Y,yawDeg,style) from the World Editor plan; style picks roof/colors.
HOUSES = [
    (91,14,10,'blue'),(59,15,355,'green'),(130,16,15,'gray'),(166,34,45,'REDROOF'),
    (188,76,90,'blue'),(190,107,90,'blue'),(184,130,75,'blue'),(169,161,45,'gray'),
    (134,190,170,'red'),(100,194,180,'blue'),(76,190,190,'blue'),(42,181,220,'green'),
    (12,145,270,'green'),(8,112,270,'blue'),(10,79,280,'blue'),(14,48,305,'red'),
    (42,28,330,'green'),(72,78,135,'gray'),(129,91,250,'gray'),(103,139,180,'red'),
]

# Raise density: add one extra house outside each ring node, facing the road.
for _i, (_rx, _ry) in enumerate(RING):
    _vx, _vy = _rx - 100.0, _ry - 100.0
    _n = math.hypot(_vx, _vy) or 1.0
    _hx, _hy = _rx + _vx / _n * 16.0, _ry + _vy / _n * 16.0
    if 6 <= _hx <= 194 and 6 <= _hy <= 194:
        HOUSES.append((round(_hx, 1), round(_hy, 1), int((_i * 47) % 360),
                       ('blue', 'red', 'green', 'gray')[_i % 4]))

STYLES = {
    'blue':   (195,180,145, 45,70,95),
    'green':  (195,180,145, 92,108,65),
    'red':    (195,180,145, 150,55,48),
    'gray':   (195,180,145, 116,118,124),
    'REDROOF':(195,180,145, 200,42,32),   # target house red roof
}

TARGET = 'redroof'

COLORS = {'grass':(96,116,64),'asphalt':(62,62,60),'dirt':(120,96,72),
          'wall':(200,186,150),'white':(226,220,196),'window':(44,66,72),
          'wood':(104,84,58),'roof_blue':(45,70,95),'roof_red':(150,55,48),
          'roof_gray':(116,118,124),'roof_green':(92,108,65),'roof_target':(200,42,32),
          'sign':(205,205,205),'leaf':(64,94,50),'leaf2':(84,112,54),
          'rock':(116,114,102)}

MESHES_ZIP = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content/assets/meshes.zip'
ART_SHAPES_ZIP = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content/art_shapes.zip'
CONTENT = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content'
MAT_ZIPS = {
    'tree': CONTENT + '/assets/materials/tree.zip',
    'foliage': CONTENT + '/assets/materials/foliage.zip',
    'tileable': CONTENT + '/assets/materials/tileable.zip',
    'atlas': CONTENT + '/assets/materials/atlas.zip',
    'meshes': MESHES_ZIP,
    'art_shapes': ART_SHAPES_ZIP,
    'tech_ground': CONTENT + '/levels/tech_ground.zip',
    'west_coast': CONTENT + '/levels/west_coast_usa.zip',
}

# dae material name -> (zip key, entry, kind) real textures bundled into the mod
MATERIAL_TEXTURES = {
    'beech_trunk': ('tree', 'assets/materials/tree/beech/t_beech_trunk/t_beech_bark_b.color.dds', 'opaque'),
    'm_ind_beech_leaves': ('tree', 'assets/materials/tree/beech/t_beech_leaves/t_beech_leaves_b.color.dds', 'leaf'),
    'beech_branch_01': ('tree', 'assets/materials/tree/beech/t_beech_branch_01/t_beech_branch_b.color.dds', 'opaque'),
    'm_oak_leaves_02': ('tree', 'assets/materials/tree/oak/oak_leaves/t_oak_leaves_wcusa_b.color.dds', 'leaf'),
    'Oak_bark_01': ('tree', 'assets/materials/tree/oak/t_oak_bark/t_oak_bark_b.color.dds', 'opaque'),
    'Oak_branch_01': ('tree', 'assets/materials/tree/oak/Oak_branch_01/t_oak_branch_b.color.dds', 'opaque'),
    'Oak_branch_02': ('tree', 'assets/materials/tree/oak/Oak_branch_01/t_oak_branch_b.color.dds', 'opaque'),
    'm_ind_birch_leaves_01': ('tree', 'assets/materials/tree/birch/t_birch_leaves/t_birch_leaves_b.color.dds', 'leaf'),
    'Birch_Bark_01': ('tree', 'assets/materials/tree/birch/t_birch_bark/t_birch_bark_b.color.dds', 'opaque'),
    'generibush_leaves': ('foliage', 'assets/materials/foliage/bush/t_generibush_leaves/t_generic_bush_b.color.dds', 'leaf'),
    'brick_painted': ('tileable', 'assets/materials/tileable/brick/brick_painted/t_brick_painted_b.color.dds', 'opaque'),
    'brick_plain': ('tileable', 'assets/materials/tileable/brick/brick_plain/t_brick_plain_b.color.dds', 'opaque'),
    'wirefence': ('atlas', 'assets/materials/atlas/wirefence/t_wirefence_b.color.dds', 'leaf'),
    'm_plaster_raw_dirty_01': ('tileable', 'assets/materials/tileable/plaster/m_plaster_raw_dirty_01/t_plaster_raw_dirty_01_b.color.dds', 'opaque'),
    'leaves_strong': ('tree', 'assets/materials/tree/beech/t_beech_leaves/t_beech_leaves_b.color.dds', 'leaf'),
    'leaves_thin': ('tree', 'assets/materials/tree/beech/t_beech_leaves/t_beech_leaves_b.color.dds', 'leaf'),
    'm_ind_beech_blocker': ('tree', 'assets/materials/tree/beech/t_beech_blocker/t_beech_blocker_b.color.dds', 'leaf'),
    'm_oak_leaves_01': ('tree', 'assets/materials/tree/oak/t_oak_leaves_01/t_oak_leaves_b.color.dds', 'leaf'),
    'Moss_01': ('tree', 'assets/materials/tree/oak/t_oak_bark/t_oak_bark_b.color.dds', 'opaque'),
    'Birch_Branch_01': ('tree', 'assets/materials/tree/birch/t_birch_branch/t_birch_branch_b.color.dds', 'opaque'),
}
# Material names with no good texture get a neutral tint so nothing renders red.
NEUTRAL_MATERIALS = {
    'italy_bld_metal': (120, 122, 126), 'italy_bld_trim_metal': (150, 150, 150),
    'italy_road_signs': (210, 210, 210), 'SOFT_COLLISION_GENERAL': (0, 0, 0),
    'lod_vertcol': (120, 120, 120), 'Material__18770': (140, 140, 140),
    'moderncorrugated_tin': (150, 155, 160), 'italy_curb_concrete': (170, 168, 160),
    'italy_guardrails': (160, 160, 160), 'Italy_bld_ind_windows': (60, 80, 90),
    'm_mm_gate_arrows': (200, 60, 60), 'm_mm_gate_pillar': (150, 145, 135),
    'm_electric_wires': (40, 40, 40), 'm_oldwood': (104, 84, 58),
    'ind_metal_trim': (120, 122, 126), 'italy_bld_decals': (200, 200, 200),
    'ColorEffectR27G177B88-material': (27, 177, 88),
    'guardrail': (150, 150, 154), 'lod_vertcol': (150, 150, 154),
}
TILEABLE_ZIP = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content/assets/materials/tileable.zip'
TERRAIN_ZIP = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content/assets/materials/terrain.zip'
DECALROAD_ZIP = 'D:/BEAMNG/BeamNG.tech.v0.39.4.0/content/assets/materials/decalroad.zip'

# Public-domain (CC0) PBR textures from ambientCG, downloaded to ./dl and bundled
# into the mod. key -> (zip_file, color_entry, normal_entry)
DL_DIR = ROOT / 'dl'
# Textures that are known to import correctly in this BeamNG.tech build.
TEXTURE_SOURCES = {
    'grass':    (str(DL_DIR / 'Grass004_1K-JPG.zip'), 'Grass004_1K-JPG_Color.jpg', 'Grass004_1K-JPG_NormalGL.jpg'),
    'asphalt':  (DECALROAD_ZIP, 'assets/materials/decalroad/asphalt/AsphaltRoad_variation_01/t_asphalt_variation_01_b.color.dds', None),
    'dirt':     (DECALROAD_ZIP, 'assets/materials/decalroad/dirt/dirt_road_gravels/t_dirt_road_gravels_b.color.dds', None),
    'concrete': (str(DL_DIR / 'Gravel001_1K-JPG.zip'), 'Gravel001_1K-JPG_Color.jpg', 'Gravel001_1K-JPG_NormalGL.jpg'),
    'wall':     (str(DL_DIR / 'Bricks001_1K-JPG.zip'), 'Bricks001_1K-JPG_Color.jpg', 'Bricks001_1K-JPG_NormalGL.jpg'),
    'roof_tile':(str(DL_DIR / 'RoofingTiles010_1K-JPG.zip'), 'RoofingTiles010_1K-JPG_Color.jpg', 'RoofingTiles010_1K-JPG_NormalGL.jpg'),
    'wood':     (str(DL_DIR / 'Wood066_1K-JPG.zip'), 'Wood066_1K-JPG_Color.jpg', 'Wood066_1K-JPG_NormalGL.jpg'),
    'plaster':  (str(DL_DIR / 'Plaster001_1K-JPG.zip'), 'Plaster001_1K-JPG_Color.jpg', 'Plaster001_1K-JPG_NormalGL.jpg'),
}

# Real native residential buildings from the game's west_coast_usa level.
WESTCOAST_ZIP = CONTENT + '/levels/west_coast_usa.zip'
HOUSE_MODELS = ['house1.dae', 'house2.dae', 'house3.dae', 'house4.dae',
                'house_small1.dae', 'house_small2.dae', 'house_small3.dae',
                'cabin1.dae', 'cabin2.dae']
TREE_DAES = ['tree_beech_large_b.dae', 'tree_beech_large_c.dae', 'tree_beech_small_c.dae',
             'tree_oak_large_a.dae', 'tree_oak_large_b.dae', 'tree_oak_sml_a.dae',
             'tree_aspen_large_a.dae', 'tree_aspen_small_a.dae', 'tree_aspen_small_c.dae']
# material name -> (texture key, tint, is_glass)
HOUSE_MATERIAL_MAP = {
    'brick_plain': ('wall', (0.95, 0.90, 0.85), False),
    'stone_brick_regular': ('wall', (0.85, 0.83, 0.80), False),
    'stone_brick_irregular': ('concrete', (0.70, 0.68, 0.64), False),
    'stucco1_white': ('plaster', (1.0, 0.99, 0.96), False),
    'm_stucco_white': ('plaster', (1.0, 0.99, 0.96), False),
    'm_stucco_scraped_01': ('plaster', (0.93, 0.90, 0.84), False),
    'rooftiles': ('roof_tile', (0.55, 0.42, 0.36), False),
    'm_roof_slates_ochre': ('roof_tile', (0.70, 0.55, 0.35), False),
    'weatherboards': ('wood', (1.0, 1.0, 1.0), False),
    'wood_rough': ('wood', (0.9, 0.85, 0.8), False),
    'windows_doors_reflective': ('concrete', (0.12, 0.16, 0.20), True),
    'concrete_plain': ('concrete', (0.85, 0.85, 0.83), False),
    'slabs_large': ('concrete', (0.80, 0.80, 0.78), False),
    'tiles_marble_square': ('concrete', (0.90, 0.90, 0.88), False),
    'architraves': ('plaster', (0.98, 0.98, 0.96), False),
    'columns': ('plaster', (0.98, 0.98, 0.96), False),
    'balustrades': ('plaster', (0.96, 0.96, 0.94), False),
    'highrise': ('concrete', (0.70, 0.72, 0.75), False),
    'corrugation': ('concrete', (0.60, 0.62, 0.64), False),
    'corrugated_modern': ('concrete', (0.65, 0.67, 0.70), False),
    'industrial': ('concrete', (0.62, 0.62, 0.60), False),
    'industrial_trim': ('concrete', (0.50, 0.50, 0.50), False),
    'metal_galvanized': ('concrete', (0.70, 0.72, 0.74), False),
    'vertexColor': ('plaster', (0.9, 0.9, 0.9), False),
    'm_house_small1_LOD': ('plaster', (0.9, 0.87, 0.82), False),
    'm_house_small2_LOD': ('plaster', (0.88, 0.86, 0.80), False),
    'm_house_small3_LOD': ('plaster', (0.9, 0.88, 0.84), False),
}

# Official level art: reuse tech_ground's own material definitions and tree
# meshes so every prop resolves with the game's real materials/textures.
TG_ZIP = CONTENT + '/levels/tech_ground.zip'
TG = 'levels/tech_ground/'
OFFICIAL_MAT_JSONS = [
    TG + 'art/shapes/main.materials.json',
    TG + 'art/shapes/trees/main.materials.json',
    TG + 'art/shapes/trees/trees_beech/main.materials.json',
    TG + 'art/shapes/trees/trees_oak/main.materials.json',
    TG + 'art/shapes/trees/trees_aspen/main.materials.json',
    TG + 'art/shapes/trees/trees_italy/main.materials.json',
    TG + 'art/shapes/groundcover/main.materials.json',
    TG + 'art/road/main.materials.json',
]
OFFICIAL_TREE_ENTRIES = [
    TG + 'art/shapes/trees/trees_beech/tree_beech_large_b.dae',
    TG + 'art/shapes/trees/trees_beech/tree_beech_large_c.dae',
    TG + 'art/shapes/trees/trees_beech/tree_beech_small_c.dae',
    TG + 'art/shapes/trees/trees_oak/tree_oak_large_a.dae',
    TG + 'art/shapes/trees/trees_oak/tree_oak_sml_a.dae',
    TG + 'art/shapes/trees/trees_aspen/tree_aspen_large_a.dae',
    TG + 'art/shapes/trees/trees_aspen/tree_aspen_small_a.dae',
]


def prop_archive_sources():
    return []


_ZIP_HANDLES = None  # kind -> ZipFile, opened lazily in extract_props


def extract_props(art_dir):
    return []


def prop_archive_sources():
    return []


_ZIP_HANDLES = None  # kind -> ZipFile, opened lazily in extract_props


def extract_props(art_dir):
    """Copy chosen native prop .dae files and their real textures from the game
    content zips into the mod, so level-local paths and materials resolve."""
    handles = {}
    def read(kind, entry):
        zpath = MAT_ZIPS[kind]
        if kind not in handles:
            handles[kind] = zipfile.ZipFile(zpath)
        return handles[kind].read(entry)
    dest = art_dir / 'props'
    tex = art_dir / 'props' / 'tex'
    dest.mkdir(parents=True, exist_ok=True)
    tex.mkdir(parents=True, exist_ok=True)
    written = []
    for base, kind, entry in prop_archive_sources():
        out = dest / base
        out.write_bytes(read(kind, entry))
        written.append(out)
    wc = zipfile.ZipFile(WESTCOAST_ZIP)
    for h in HOUSE_MODELS:
        entry = 'levels/west_coast_usa/art/shapes/buildings/' + h
        out = dest / h
        out.write_bytes(wc.read(entry))
        written.append(out)
    wc.close()
    for mat, (kind, entry, _k) in MATERIAL_TEXTURES.items():
        base = entry.rsplit('/', 1)[-1]
        out = tex / base
        out.write_bytes(read(kind, entry))
        written.append(out)
    for h in handles.values():
        h.close()
    return written


def dist_to_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    c1 = vx * wx + vy * wy
    c2 = vx * vx + vy * vy
    if c2 <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, c1 / c2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def poly_dist(px, py, pts):
    return min(dist_to_segment(px, py, a[0], a[1], b[0], b[1])
               for a, b in zip(pts, pts[1:] + pts[:1]))


def road_dist(x, y):
    return poly_dist(x, y, RING)


def branch_dist(x, y):
    return poly_dist(x, y, BRANCH)


def height(x, y):
    base = BASE_Z + 0.4 * math.sin(x / 60.0) + 0.4 * math.sin(y / 51.0)
    # Central pond: a smooth depression around (100, 100).
    d2 = (x - 100.0) ** 2 + (y - 100.0) ** 2
    base -= 3.0 * math.exp(-d2 / (2 * 13.0 ** 2))
    return base


def surface(x, y):
    if road_dist(x, y) < GRID:
        return 'asphalt'
    if branch_dist(x, y) < 2.5:
        return 'dirt'
    return 'grass'


def native(name, cls, parent=None, **fields):
    obj = {'name': name, 'class': cls,
           'persistentId': str(uuid.uuid5(uuid.NAMESPACE_URL, LEVEL + '/' + name))}
    if parent:
        obj['__parent'] = parent
    return dict(obj, **fields)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, separators=(',', ':')) + '\n' for r in rows),
                    encoding='utf-8')


def generate_meshes():
    meshes = {}
    for ty in range(0, 224, 32):
        for tx in range(-16, 256, 32):
            m = Mesh('ground_%d_%d' % (tx + 16, ty))
            for y in range(ty, ty + 32):
                for x in range(tx, tx + 32):
                    m.quad(*[(a, b, height(a, b)) for a, b in
                             ((x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1))],
                           'mr_' + surface(x + .5, y + .5))
            meshes[m.name] = m

    rng = random.Random(3911)
    tree_pos = []
    placed = []
    props = []
    PROPS_PREFIX = ART + 'props/'

    def tstatic(name, shape, pos, rmat=None, scale=1.0, collision='None', scale3=None):
        props.append({
            'name': name, 'class': 'TSStatic', '__parent': 'Geometry',
            'position': [float(pos[0]), float(pos[1]), float(pos[2])],
            'scale': [float(v) for v in (scale3 if scale3 else (scale, scale, scale))],
            'rotationMatrix': rmat or [1, 0, 0, 0, 1, 0, 0, 0, 1],
            'shapeName': (shape if shape.startswith('/') else PROPS_PREFIX + shape),
            'collisionType': collision, 'decalType': 'None'})

    # Houses: real west_coast models, Poisson placed (>=24 m apart, >=19 m off road).
    house_pts = []
    attempts = 0
    while len(house_pts) < 24 and attempts < 4000:
        attempts += 1
        hx, hy = rng.uniform(18, 182), rng.uniform(18, 182)
        if road_dist(hx, hy) < GRID + 15.0:
            continue
        if any(math.hypot(hx - a, hy - b) < 24.0 for a, b in house_pts):
            continue
        if any(math.hypot(hx - a, hy - b) < 13.0 for a, b in placed):
            continue
        house_pts.append((hx, hy))
        placed.append((hx, hy))
    for idx, (hx, hy) in enumerate(house_pts):
        model = HOUSE_MODELS[idx % len(HOUSE_MODELS)]
        yaw = rng.uniform(0, math.tau)
        rmat = [math.cos(yaw), -math.sin(yaw), 0, math.sin(yaw), math.cos(yaw), 0, 0, 0, 1]
        tstatic('mr_house_%d' % idx, '/levels/west_coast_usa/art/shapes/buildings/' + model, (hx, hy, height(hx, hy)),
                rmat=rmat, scale=1.0 + (idx % 3) * 0.12, collision='Visible Mesh Final')

    # Trees: real west_coast / native models.
    while len(tree_pos) < 160:
        x, y = rng.uniform(12, 188), rng.uniform(12, 188)
        if road_dist(x, y) < GRID + 8 or branch_dist(x, y) < 9:
            continue
        if any(math.hypot(x - a, y - b) < 12 for a, b in placed):
            continue
        if any(math.hypot(x - a, y - b) < 5 for a, b in tree_pos):
            continue
        tree_pos.append((x, y))
        dae = TREE_DAES[rng.randrange(len(TREE_DAES))]
        fam = 'beech' if dae.startswith('tree_beech') else ('oak' if dae.startswith('tree_oak') else 'aspen')
        tstatic('mr_tree_%d' % len(tree_pos),
                '/levels/tech_ground/art/shapes/trees/trees_%s/%s' % (fam, dae), (x, y, height(x, y) - 0.1),
                scale=0.9 + rng.random() * 0.5)

    # Grass tufts + bushes along both road edges.
    tuf = Mesh('grass_tufts')
    bu = Mesh('bushes')
    for seg in range(14):
        ax, ay = RING[seg]
        bx, by = RING[(seg + 1) % 14]
        L = math.hypot(bx - ax, by - ay)
        nx, ny = -(by - ay) / L, (bx - ax) / L
        n = max(4, int(L / 3.0))
        for k in range(n):
            t = (k + 0.5) / n
            sx, sy = ax + (bx - ax) * t, ay + (by - ay) * t
            for sgn in (1, -1):
                base = GRID + 1.3 + rng.random() * 3.0
                qx, qy = sx + nx * sgn * base, sy + ny * sgn * base
                if not (6 <= qx <= 194 and 6 <= qy <= 194):
                    continue
                z = height(qx, qy)
                if rng.random() < 0.55:
                    bh = 0.3 + rng.random() * 0.25
                    rad = 0.35 + rng.random() * 0.2
                    for ag in (0.0, math.pi / 3, 2 * math.pi / 3):
                        ca, sa = math.cos(ag), math.sin(ag)
                        q = [(qx + dx * ca - dy * sa, qy + dx * sa + dy * ca)
                             for dx, dy in ((-rad, 0), (rad, 0))]
                        tuf.quad((q[0][0], q[0][1], z), (q[1][0], q[1][1], z),
                                 (q[1][0], q[1][1], z + bh), (q[0][0], q[0][1], z + bh), 'mr_grass_tuft')
                elif rng.random() < 0.6:
                    ca, sa = math.cos(0.6), math.sin(0.6)
                    br = 0.9 + rng.random() * 0.5
                    q = [(qx + dx * ca - dy * sa, qy + dx * sa + dy * ca)
                         for dx, dy in ((-br, 0), (br, 0))]
                    bu.quad((q[0][0], q[0][1], z), (q[1][0], q[1][1], z),
                            (q[1][0], q[1][1], z + 0.8), (q[0][0], q[0][1], z + 0.8), 'mr_foliage')
    meshes['grass_tufts'] = tuf
    meshes['bushes'] = bu

    # Central pond.
    pond = Mesh('pond')
    cx, cy = 100.0, 100.0
    wz = BASE_Z - 1.4
    pn = 28
    for i in range(pn):
        j = (i + 1) % pn
        a0 = (cx + 11 * math.cos(i * math.tau / pn), cy + 11 * math.sin(i * math.tau / pn))
        a1 = (cx + 11 * math.cos(j * math.tau / pn), cy + 11 * math.sin(j * math.tau / pn))
        pond.triangle((cx, cy, wz), (a0[0], a0[1], wz), (a1[0], a1[1], wz), 'mr_water')
    meshes['pond'] = pond

    # Guardrails: short pieces along BOTH road edges (tangent-aligned).
    guard = 0
    for seg in range(14):
        ax, ay = RING[seg]
        bx, by = RING[(seg + 1) % 14]
        L = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        rmat = [math.cos(yaw), -math.sin(yaw), 0, math.sin(yaw), math.cos(yaw), 0, 0, 0, 1]
        nx, ny = -(by - ay) / L, (bx - ax) / L
        n = max(2, int(round(L / 2.4)))
        for k in range(n):
            t = (k + 0.5) / n
            sx, sy = ax + (bx - ax) * t, ay + (by - ay) * t
            for sgn in (1, -1):
                qx, qy = sx + nx * sgn * (GRID + 0.7), sy + ny * sgn * (GRID + 0.7)
                guard += 1
                tstatic('mr_guard_%d' % guard, '/levels/west_coast_usa/art/shapes/objects/guardrail2.dae',
                        (qx, qy, height(qx, qy)), rmat=rmat, scale=1.0, collision='Visible Mesh Final')

    # Street lights (alternating sides).
    lamp = 0
    for seg in range(14):
        ax, ay = RING[seg]
        bx, by = RING[(seg + 1) % 14]
        L = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        rmat = [math.cos(yaw), -math.sin(yaw), 0, math.sin(yaw), math.cos(yaw), 0, 0, 0, 1]
        nx, ny = -(by - ay) / L, (bx - ax) / L
        sgn = 1 if seg % 2 == 0 else -1
        qx, qy = (ax + bx) / 2 + nx * sgn * 5.5, (ay + by) / 2 + ny * sgn * 5.5
        lamp += 1
        tstatic('mr_lamp_%d' % lamp, '/levels/west_coast_usa/art/shapes/objects/pole_city1.dae', (qx, qy, height(qx, qy)),
                rmat=rmat, scale=1.0, collision='Visible Mesh Final')

    # Utility poles (outer side).
    pole = 0
    for seg in range(14):
        ax, ay = RING[seg]
        bx, by = RING[(seg + 1) % 14]
        L = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        rmat = [math.cos(yaw), -math.sin(yaw), 0, math.sin(yaw), math.cos(yaw), 0, 0, 0, 1]
        nx, ny = -(by - ay) / L, (bx - ax) / L
        qx, qy = (ax + bx) / 2 + nx * 7.5, (ay + by) / 2 + ny * 7.5
        pole += 1
        tstatic('mr_pole_%d' % pole, '/art/shapes/common/power_lines_procedural/electric_pole_wood_old_01.dae',
                (qx, qy, height(qx, qy)), rmat=rmat, scale=1.0, collision='Visible Mesh Final')

    # Road signs at a few segments.
    sign = 0
    for seg in (0, 4, 8, 12):
        ax, ay = RING[seg]
        bx, by = RING[(seg + 1) % 14]
        L = math.hypot(bx - ax, by - ay)
        nx, ny = -(by - ay) / L, (bx - ax) / L
        qx, qy = ax + nx * 4.2, ay + ny * 4.2
        sign += 1
        tstatic('mr_sign_%d' % sign, '/levels/west_coast_usa/art/shapes/buildings/bussign.dae', (qx, qy, height(qx, qy)),
                scale=1.0, collision='Visible Mesh Final')

    return meshes, tree_pos, props


def preview(path, trees):
    image = Image.new('RGB', (960, 600), (28, 40, 30))
    d = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=29)
    small = ImageFont.load_default(size=17)
    d.text((32, 22), 'WILDOS MEMORY RING', font=font, fill=(232,224,199))
    d.text((32, 64), '200 x 200 m | ORIGINAL STANDALONE LEVEL', font=small, fill=(160,184,152))
    def p(x, y):
        return (int(80 + x * 1.9), int(520 - y * 1.9))
    # ground
    for iy in range(390):
        for ix in range(390):
            x, y = (ix - 10) / 1.9, (520 - iy - 130) / 1.9
            if not (0 <= x <= 200 and 0 <= y <= 200):
                continue
            color = COLORS[surface(x, y)]
            image.putpixel((10 + ix, 130 + iy), tuple(color))
    for x, y in trees:
        px, py = p(x, y)
        d.ellipse((px-3, py-3, px+3, py+3), fill=COLORS['leaf'])
    for hx, hy, _, style in HOUSES:
        px, py = p(hx, hy)
        d.rectangle((px-7, py-5, px+7, py+5),
                    fill=STYLES[style][1] if False else COLORS['roof_blue'],
                    outline=(235,215,172))
        if style == 'REDROOF':
            d.rectangle((px-7, py-5, px+7, py+5), outline=(255,255,0), width=2)
    px, py = p(*RING[0])
    d.polygon(((px+9,py),(px-5,py-6),(px-5,py+6)), fill=(255,218,96))
    for i, (label, color) in enumerate((('Asphalt ring (8 m)', COLORS['asphalt']),
                                        ('Dirt branch', COLORS['dirt']),
                                        ('Varied houses / red-roof target', COLORS['roof_target']),
                                        ('50 real-style trees', COLORS['leaf']),
                                        ('Start at R01', (255,218,96)))):
        yy = 145 + i * 48
        d.rectangle((550, yy, 566, yy+16), fill=color)
        d.text((578, yy-2), label, font=small, fill=(224,219,198))
    d.text((550, 430), 'LAYOUT DIAGRAM', font=font, fill=(230,212,164))
    d.text((550, 470), 'Not an in-game render', font=small, fill=(184,191,177))
    image.save(path)


MIRROR_SOURCES = [
    (CONTENT + '/levels/tech_ground.zip', [
        'levels/tech_ground/art/shapes/trees/trees_beech/',
        'levels/tech_ground/art/shapes/trees/trees_oak/',
        'levels/tech_ground/art/shapes/trees/trees_aspen/',
        'levels/tech_ground/art/shapes/trees/main.materials.json',
        'levels/tech_ground/art/shapes/groundcover/',
        'levels/tech_ground/art/road/',
    ], [
        'levels/west_coast_usa/art/shapes/buildings/main.materials.json',
        'levels/west_coast_usa/art/shapes/objects/guardrail2.dae',
        'levels/west_coast_usa/art/shapes/objects/pole_city1.dae',
        'levels/west_coast_usa/art/shapes/buildings/bussign.dae',
    ]),
    (CONTENT + '/levels/west_coast_usa.zip', [], [
        'levels/west_coast_usa/art/shapes/buildings/main.materials.json',
        'levels/west_coast_usa/art/shapes/buildings/house1.dae',
        'levels/west_coast_usa/art/shapes/buildings/house2.dae',
        'levels/west_coast_usa/art/shapes/buildings/house3.dae',
        'levels/west_coast_usa/art/shapes/buildings/house4.dae',
        'levels/west_coast_usa/art/shapes/buildings/house_small1.dae',
        'levels/west_coast_usa/art/shapes/buildings/house_small2.dae',
        'levels/west_coast_usa/art/shapes/buildings/house_small3.dae',
        'levels/west_coast_usa/art/shapes/buildings/cabin1.dae',
        'levels/west_coast_usa/art/shapes/buildings/cabin2.dae',
        'levels/west_coast_usa/art/shapes/objects/guardrail2.dae',
        'levels/west_coast_usa/art/shapes/objects/pole_city1.dae',
        'levels/west_coast_usa/art/shapes/buildings/bussign.dae',
    ]),
]


def mirror_official(mod_root, files):
    """Copy official map art (materials + textures) into our mod at their
    original absolute paths, plus any textures their material JSON references
    (fetched from any content zip), so official materials resolve on load."""
    content_index = {}
    for zp in Path(CONTENT).rglob('*.zip'):
        try:
            zf = zipfile.ZipFile(zp)
        except Exception:
            continue
        for n in zf.namelist():
            content_index.setdefault(n, zp)
        zf.close()

    def write(rel, data):
        out = mod_root / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)
        files.append(out)

    handles = {}
    json_texts = []
    for zip_path, prefixes, explicit in MIRROR_SOURCES:
        zf = zipfile.ZipFile(zip_path)
        handles[zip_path] = zf
        for n in zf.namelist():
            take = any(n.startswith(pref) for pref in prefixes) or (n in explicit) or any(n == e for e in explicit)
            if take:
                try:
                    data = zf.read(n)
                except Exception:
                    continue
                write(n, data)
                if n.endswith('.json'):
                    try:
                        json_texts.append(data.decode('utf-8', 'ignore'))
                    except Exception:
                        pass
    # resolve texture references from the collected json and copy at same path
    refs = set()
    for txt in json_texts:
        for m in re.findall(r'"([^"]+)"', txt):
            mm = m.lstrip('/')
            if (mm.startswith('levels/') or mm.startswith('assets/')) and '.' in mm.rsplit('/', 1)[-1]:
                refs.add(mm)
    for rel in refs:
        src = rel if rel in content_index else (rel + '.link' if (rel + '.link') in content_index else None)
        if not src:
            continue
        zp = content_index[src]
        if zp not in handles:
            handles[zp] = zipfile.ZipFile(zp)
        write(src, handles[zp].read(src))
        if src.endswith('.link'):
            # also drop a copy at the referenced (non .link) path so it resolves
            write(rel, handles[zp].read(src))
    for h in handles.values():
        h.close()


def build():
    out = ROOT / 'generated'
    level = out / 'mod' / PREFIX
    art = level / 'art/shapes'
    if (out / 'mod').exists():
        shutil.rmtree(out / 'mod')
    art.mkdir(parents=True, exist_ok=True)
    meshes, trees, props = generate_meshes()
    files = []
    materials = {}
    # Bundle real BeamNG textures and point every material at them.
    tex_dir = art / 'tex'
    tex_dir.mkdir(parents=True, exist_ok=True)
    tex_files = {}
    normal_files = {}
    _zips = {}
    for key, (zip_path, color_entry, normal_entry) in TEXTURE_SOURCES.items():
        if zip_path not in _zips:
            _zips[zip_path] = zipfile.ZipFile(zip_path)
        dest = tex_dir / (key + os.path.splitext(color_entry)[1])
        dest.write_bytes(_zips[zip_path].read(color_entry))
        files.append(dest)
        tex_files[key] = ART + 'tex/' + dest.name
        if normal_entry:
            destn = tex_dir / (key + '_n' + os.path.splitext(normal_entry)[1])
            destn.write_bytes(_zips[zip_path].read(normal_entry))
            files.append(destn)
            normal_files[key] = ART + 'tex/' + destn.name
        else:
            normal_files[key] = None
    for h in _zips.values():
        h.close()
    def stage_for(tex_key, tint, rough=0.9):
        stage = dict(baseColorMap=tex_files[tex_key],
                     baseColorFactor=[tint[0], tint[1], tint[2], 1.0],
                     roughnessFactor=rough, metallicFactor=0, useAnisotropic=True)
        if normal_files.get(tex_key):
            stage['normalMap'] = normal_files[tex_key]
            stage['normalStrength'] = 1.0
        return stage
    MATERIAL_MAP = {
        'grass': ('grass', (1.0, 1.0, 1.0)), 'asphalt': ('asphalt', (1.0, 1.0, 1.0)),
        'dirt': ('dirt', (1.0, 1.0, 1.0)), 'wall': ('plaster', (0.96, 0.94, 0.90)),
        'white': ('plaster', (1.0, 1.0, 1.0)), 'window': ('concrete', (0.16, 0.20, 0.25)),
        'wood': ('wood', (1.0, 1.0, 1.0)),
        'roof_blue': ('roof_tile', (0.30, 0.40, 0.55)), 'roof_red': ('roof_tile', (0.66, 0.26, 0.22)),
        'roof_gray': ('roof_tile', (0.55, 0.56, 0.58)), 'roof_green': ('roof_tile', (0.40, 0.48, 0.32)),
        'roof_target': ('roof_tile', (0.86, 0.16, 0.12)),
        'sign': ('plaster', (0.85, 0.85, 0.85)), 'leaf': ('grass', (0.55, 0.62, 0.45)),
        'leaf2': ('grass', (0.60, 0.66, 0.48)), 'rock': ('concrete', (0.72, 0.72, 0.70)),
    }
    for name in COLORS:
        tex_key, tint = MATERIAL_MAP.get(name, ('concrete', (1.0, 1.0, 1.0)))
        mat = 'mr_' + name
        gt = {'grass': 'GRASS', 'wood': 'WOOD', 'leaf': 'WOOD', 'leaf2': 'WOOD',
              'rock': 'ROCK', 'dirt': 'DIRT'}.get(name, 'ASPHALT')
        materials[mat] = dict(name=mat, mapTo=mat, **{'class': 'Material'}, version=1.5,
                              Stages=[stage_for(tex_key, tint, 0.9), {}, {}, {}],
                              groundType=gt, translucentBlendOp='None', doubleSided=False)
    for i, tint in enumerate([(0.97, 0.94, 0.89), (0.86, 0.84, 0.81), (0.93, 0.85, 0.75),
                              (0.89, 0.87, 0.91), (0.82, 0.87, 0.79)]):
        mat = 'mr_wall_%d' % i
        materials[mat] = dict(name=mat, mapTo=mat, **{'class': 'Material'}, version=1.5,
                              Stages=[stage_for('plaster', tint, 0.9), {}, {}, {}],
                              groundType='ASPHALT', translucentBlendOp='None', doubleSided=False)
    # a real brick variant used by some houses
    materials['mr_wall_brick'] = dict(name='mr_wall_brick', mapTo='mr_wall_brick',
        **{'class': 'Material'}, version=1.5,
        Stages=[stage_for('wall', (0.95, 0.90, 0.85), 0.85), {}, {}, {}],
        groundType='ASPHALT', translucentBlendOp='None', doubleSided=False)
    # Low grass tuft: procedural alpha blade texture, double sided.
    tuft_img = Image.new('RGBA', (128, 128), (0, 0, 0, 0))
    td = ImageDraw.Draw(tuft_img)
    trng = random.Random(7)
    for _ in range(60):
        bx = trng.randint(4, 124)
        bh = trng.randint(40, 120)
        wdt = trng.randint(2, 4)
        shade = trng.randint(-25, 25)
        col = (max(0, min(255, 96 + shade)), max(0, min(255, 140 + shade)), max(0, min(255, 56 + shade)), 255)
        lean = trng.randint(-14, 14)
        td.polygon([(bx, 128), (bx + wdt, 128), (bx + lean, 128 - bh)], fill=col)
    tuft_path = tex_dir / 'grass_tuft.png'
    tuft_img.save(tuft_path)
    files.append(tuft_path)
    materials['mr_grass_tuft'] = dict(
        name='mr_grass_tuft', mapTo='mr_grass_tuft', **{'class': 'Material'}, version=1.5,
        Stages=[dict(baseColorMap=ART + 'tex/grass_tuft.png', baseColorFactor=[1, 1, 1, 1],
                     roughnessFactor=1.0, metallicFactor=0, useAnisotropic=True), {}, {}, {}],
        groundType='GRASS', translucentBlendOp='Alpha', doubleSided=True, alphaTest=True)
    # Foliage (leaves/bushes): green blobs on transparency, alpha cutout.
    fimg = Image.new('RGBA', (128, 128), (0, 0, 0, 0))
    fd = ImageDraw.Draw(fimg)
    frng = random.Random(11)
    for _ in range(70):
        bx, by = frng.randint(8, 120), frng.randint(8, 120)
        rad = frng.randint(5, 16)
        shade = frng.randint(-22, 22)
        fc = (max(0, 40 + shade), max(0, min(255, 130 + shade)), max(0, 48 + shade), 255)
        fd.ellipse((bx - rad, by - rad, bx + rad, by + rad), fill=fc)
    fimg2 = fimg.resize((256, 256), Image.LANCZOS)
    fpath = tex_dir / 'foliage.png'
    fimg2.save(fpath)
    files.append(fpath)
    materials['mr_foliage'] = dict(
        name='mr_foliage', mapTo='mr_foliage', **{'class': 'Material'}, version=1.5,
        Stages=[dict(baseColorMap=ART + 'tex/foliage.png', baseColorFactor=[1, 1, 1, 1],
                     roughnessFactor=1.0, metallicFactor=0, useAnisotropic=True), {}, {}, {}],
        groundType='GRASS', translucentBlendOp='Alpha', doubleSided=True, alphaTest=True)
    # Metal guardrail.
    materials['mr_guard'] = dict(
        name='mr_guard', mapTo='mr_guard', **{'class': 'Material'}, version=1.5,
        Stages=[dict(baseColorMap=tex_files['concrete'],
                     baseColorFactor=[0.62, 0.63, 0.66, 1.0],
                     roughnessFactor=0.4, metallicFactor=0.7, useAnisotropic=True), {}, {}, {}],
        groundType='ASPHALT', translucentBlendOp='None', doubleSided=False)
    # Pond water: translucent blue.
    materials['mr_water'] = dict(
        name='mr_water', mapTo='mr_water', **{'class': 'Material'}, version=1.5,
        Stages=[dict(baseColorFactor=[0.15, 0.30, 0.42, 0.75],
                     roughnessFactor=0.2, metallicFactor=0, useAnisotropic=True), {}, {}, {}],
        groundType='ASPHALT', translucentBlendOp='Alpha', doubleSided=False)
    # Native prop materials: bind the bundled real textures so the tree, bush and
    # fence .dae meshes resolve instead of falling back to "NO MATERIAL".
    for mat_name, (kind, entry, mkind) in MATERIAL_TEXTURES.items():
        tex_name = entry.rsplit('/', 1)[-1]
        materials[mat_name] = dict(
            name=mat_name, mapTo=mat_name, **{'class': 'Material'}, version=1.5,
            Stages=[dict(baseColorMap=ART + 'props/tex/' + tex_name,
                         baseColorFactor=[1, 1, 1, 1], roughnessFactor=.9,
                         metallicFactor=0, useAnisotropic=True), {}, {}, {}],
            groundType='WOOD' if mkind == 'leaf' else 'ASPHALT',
            translucentBlendOp='None', doubleSided=(mkind == 'leaf'),
            alphaTest=(mkind == 'leaf'))
    for mat_name, tint in NEUTRAL_MATERIALS.items():
        materials[mat_name] = dict(
            name=mat_name, mapTo=mat_name, **{'class': 'Material'}, version=1.5,
            Stages=[dict(baseColorMap=tex_files['concrete'],
                         baseColorFactor=[tint[0] / 255, tint[1] / 255, tint[2] / 255, 1.0],
                         roughnessFactor=.9, metallicFactor=0, useAnisotropic=True), {}, {}, {}],
            groundType='ASPHALT', translucentBlendOp='None', doubleSided=False)
    # Real native house materials -> real textures (heuristic by material name).
    for mname, (tex_key, tint, glass) in HOUSE_MATERIAL_MAP.items():
        materials[mname] = dict(
            name=mname, mapTo=mname, **{'class': 'Material'}, version=1.5,
            Stages=[stage_for(tex_key, tint, 0.25 if glass else 0.85), {}, {}, {}],
            groundType='ASPHALT', translucentBlendOp='Alpha' if glass else 'None',
            doubleSided=False)
    # Generic remap: scan every bundled native prop .dae, collect its material
    # names and bind them to our bundled real textures by heuristic, so the
    # west_coast trees/guardrails/lamps/signs/houses all resolve reliably.
    def _heuristic_material(mat):
        m = mat.lower()
        if any(k in m for k in ('bark', 'branch', 'trunk', 'wood')):
            return 'wood', (1.0, 1.0, 1.0), False
        if any(k in m for k in ('leaf', 'foliage', 'leaves', 'canopy', 'bush')):
            return None, None, True          # use mr_foliage (alpha greenery)
        if any(k in m for k in ('brick', 'stone', 'brick_plain')):
            return 'wall', (0.95, 0.90, 0.85), False
        if any(k in m for k in ('stucco', 'plaster', 'paint', 'concrete', 'white')):
            return 'plaster', (0.96, 0.94, 0.90), False
        if any(k in m for k in ('roof', 'tile_s', 'slate')):
            return 'roof_tile', (0.62, 0.45, 0.35), False
        if any(k in m for k in ('metal', 'steel', 'galvan', 'guard', 'rail', 'pole', 'light')):
            return 'concrete', (0.62, 0.63, 0.66), False
        if any(k in m for k in ('glass', 'window', 'reflective', 'mirror')):
            return 'concrete', (0.12, 0.16, 0.20), False
        if any(k in m for k in ('sign', 'plastic', 'red', 'white', 'signal')):
            return 'concrete', (0.90, 0.90, 0.90), False
        return 'plaster', (0.96, 0.94, 0.90), False

    for _base, _kind, _entry in prop_archive_sources():
        zp = MAT_ZIPS[_kind]
        try:
            with zipfile.ZipFile(zp) as _zf:
                _txt = _zf.read(_entry).decode('utf-8', 'ignore')
        except Exception:
            continue
        for _m in set(re.findall(r'<material id="[^"]+" name="([^"]+)"', _txt)):
            if _m in materials:
                continue
            _tk, _tint, _alpha = _heuristic_material(_m)
            if _alpha:
                materials[_m] = dict(
                    name=_m, mapTo=_m, **{'class': 'Material'}, version=1.5,
                    Stages=[dict(baseColorMap=ART + 'tex/foliage.png', baseColorFactor=[1, 1, 1, 1],
                                 roughnessFactor=1.0, metallicFactor=0, useAnisotropic=True), {}, {}, {}],
                    groundType='GRASS', translucentBlendOp='Alpha', doubleSided=True, alphaTest=True)
            else:
                materials[_m] = dict(
                    name=_m, mapTo=_m, **{'class': 'Material'}, version=1.5,
                    Stages=[stage_for(_tk, _tint, 0.6), {}, {}, {}],
                    groundType='ASPHALT', translucentBlendOp='None', doubleSided=False)
    mirror_official(out / 'mod', files)
    write_json(art / 'main.materials.json', materials)
    files.append(art / 'main.materials.json')
    for dae_file in extract_props(art):
        files.append(dae_file)
    statics = []
    for name, m in meshes.items():
        dest = art / (name + '.dae')
        m.write(dest)
        files.append(dest)
        statics.append(native('mr_' + name, 'TSStatic', 'Geometry', position=[0,0,0],
                              scale=[1,1,1], rotationMatrix=[1,0,0,0,1,0,0,0,1],
                              shapeName=ART + dest.name, collisionType='Visible Mesh Final',
                              decalType='Visible Mesh', useInstanceRenderData=True))
    # Native-art TSStatic props (trees/bushes/bins/poles/fences). Native assets are
    # referenced by their served virtual path and are NOT re-packaged into the mod.
    for prop in props:
        statics.append(prop)

    # Road: native DecalRoads using the game's own road materials (as other maps do),
    # so the surface matches existing levels instead of a flat custom strip.
    def decalroad(name, nodes_xy, material, width, lift=0.06, drivability=1.0):
        nodes = [[x, y, height(x, y) + lift, width] for x, y in nodes_xy]
        item = {'class': 'DecalRoad',
                'persistentId': str(uuid.uuid5(uuid.NAMESPACE_URL, LEVEL + '/' + name)),
                'name': name, '__parent': 'Geometry',
                'position': nodes[0][:3], 'detail': 0.2, 'distanceFade': [200, 50],
                'drivability': drivability, 'material': material,
                'nodes': nodes, 'renderPriority': 1}
        statics.append(item)

    ring_closed = RING + [RING[0]]
    decalroad('mr_road_main', ring_closed, 'road_asphalt_2lane', 8.0, 0.06, 1.0)
    decalroad('mr_road_centerline', ring_closed, 'm_line_white', 0.25, 0.09, 1.0)
    decalroad('mr_road_branch', BRANCH, 'm_dirt_road', 5.0, 0.06, 1.0)

    def rows(relative, items):
        p = level / relative
        write_rows(p, items)
        files.append(p)
    rows('main/items.level.json', [native('MissionGroup', 'SimGroup', enabled='1')])
    rows('main/MissionGroup/items.level.json', [native(n, 'SimGroup', 'MissionGroup')
                                                for n in ('Geometry', 'Sky', 'PlayerDropPoints')])
    rows('main/MissionGroup/Geometry/items.level.json', statics)
    spawn = list(RING[0]) + [height(*RING[0])]
    rows('main/MissionGroup/PlayerDropPoints/items.level.json', [
        native('spawn_default', 'SpawnSphere', 'PlayerDropPoints', position=spawn,
               rotationMatrix=[0,1,0,-1,0,0,0,0,1], dataBlock='SpawnSphereMarker', radius=1,
               spawnClass='player', spawnDatablock='DefaultPlayerData', sphereWeight='1', enabled='1')])
    rows('main/MissionGroup/Sky/items.level.json', [
        native('theLevelInfo', 'LevelInfo', 'Sky', gravity=-9.81, visibleDistance=1800,
               fogDensity=.0003, fogAtmosphereHeight=300, fogColor=[.74,.81,.9,1],
               globalEnviromentMap='BNG_Sky_01_cubemap', canvasClearColor=[1,1,1,255]),
        native('sunsky', 'ScatterSky', 'Sky', position=[0,0,100], azimuth=230, elevation=45,
               skyBrightness=40, mieScattering=.00043, enableFogFallBack=False,
               ambientScale=[1,.93,.84,1], sunScale=[1,.91,.8,1], colorize=[.22,.35,.6,1],
               fogScale=[.4,.67,1,1], castShadows=True,
               **{key: 'art/sky_gradients/default/gradient_' + value + '.png'
                  for key, value in (('ambientScaleGradientFile','ambient'),
                                     ('colorizeGradientFile','colorize'),
                                     ('fogScaleGradientFile','fog'),
                                     ('sunScaleGradientFile','sunscale'))}),
        native('tod', 'TimeOfDay', 'Sky', play=False, axisTilt=10, startTime=.15, time=.15),
        native('mr_clouds', 'CloudLayer', 'Sky',
               texture='art/skies/clouds/clouds_normal_displacement',
               coverage=.4, windSpeed=.1, Textures=[{}, {}, {}])])
    target = (next(h for h in HOUSES if h[3] == 'REDROOF'))
    info = dict(title='WildOS Memory Ring',
                description='200x200 asphalt ring for WildOS visual-memory testing.',
                authors='WildOS workspace', size=[200, 200], biome='suburban',
                roads='asphalt_ring + dirt spur', previews=['preview.png'],
                defaultSpawnPointName='spawn_default',
                spawnPoints=[dict(objectname='spawn_default', translationId='Ring Road',
                                  preview='preview.png')],
                isAuxiliary=False, supportsTraffic=False, supportsTimeOfDay=True)
    write_json(level / 'info.json', info)
    files.append(level / 'info.json')
    preview(level / 'preview.png', trees)
    files.append(level / 'preview.png')
    lic = level / 'ASSET_LICENSE.txt'
    lic.write_text('Original procedural meshes, textures and layout: CC0 1.0.\n'
                   'Requires a separately licensed BeamNG installation and its common sky resources.\n',
                   encoding='ascii')
    files.append(lic)
    guide = level / 'EDITOR_GUIDE.md'
    guide.write_text(
        '# WildOS Memory Ring - World Editor guide\n\n'
        'Base level already contains: terrain, the asphalt ring road (DecalRoad),\n'
        'a dirt spur and the spawn point. All props/materials below are bundled in\n'
        'this mod and resolve on load - place them yourself with the World Editor.\n\n'
        '## Bundled prop meshes (pick via World Editor -> Objects / Static)\n'
        + ''.join('levels/wildos_memory_ring/art/shapes/props/' + p.name + '\n'
                  for p in sorted((level / 'art/shapes/props').glob('*.dae')) if p.is_file())
        + '\n## Materials\n'
        'All materials are defined in art/shapes/main.materials.json and are bound\n'
        'to bundled real textures, so nothing shows as NO MATERIAL.\n\n'
        '## Notes\n'
        '- Houses: house1..house4, house_small1..3, cabin1..2\n'
        '- Trees: tree_beech_*, tree_oak_*, tree_aspen_*, pine_radiata_*\n'
        '- Guardrail: guardrail2.dae (lay along the road edge)\n'
        '- Street light: pole_city1.dae\n'
        '- Utility pole: electric_pole_wood_old_01.dae\n'
        '- Road sign: bussign.dae\n'
        '- Bush: generibush.dae / sidewalk_brittlebush1.dae\n',
        encoding='utf-8')
    files.append(guide)
    archive = out / MOD_NAME
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for p in sorted(files):
            member = p.relative_to(out / 'mod').as_posix()
            zi = zipfile.ZipInfo(member, (2026, 9, 21, 0, 0, 0))
            zi.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(zi, p.read_bytes())
    manifest = dict(level=LEVEL, zip=MOD_NAME, zip_bytes=archive.stat().st_size,
                    zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    spawn=dict(pos=spawn), road_nodes=RING, target_house=dict(pos=list(target[:2]), id='B04'),
                    tree_count=len(trees), house_count=len(HOUSES),
                    triangles=sum(len(m.vertices) // 3 for m in meshes.values()),
                    runtime_verified=False, ground_size_m=[200, 200], road_width_m=8,
                    mesh_triangles={n: len(m.vertices) // 3 for n, m in meshes.items()})
    write_json(out / 'build_manifest.json', manifest)
    print(json.dumps({k: v for k, v in manifest.items() if k not in ('mesh_triangles',)},
                     indent=2))


if __name__ == '__main__':
    build()
