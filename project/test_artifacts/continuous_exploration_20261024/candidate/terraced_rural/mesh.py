"""Original metre/Z-up COLLADA geometry, explicit normals and material bindings."""
from collections import defaultdict
import math
import xml.etree.ElementTree as ET


class Mesh:
    def __init__(self, name):
        self.name = name
        self.vertices, self.normals, self.uvs = [], [], []
        self.groups = defaultdict(list)

    def triangle(self, a, b, c, material, uv=None):
        u = [b[i]-a[i] for i in range(3)]
        v = [c[i]-a[i] for i in range(3)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(x*x for x in n))
        if length < 1e-10:
            raise ValueError('Degenerate triangle')
        start = len(self.vertices)
        self.vertices.extend((a, b, c))
        self.normals.extend([tuple(x/length for x in n)] * 3)
        self.uvs.extend(uv or [(p[0]/3, p[1]/3) for p in (a, b, c)])
        self.groups[material].extend(range(start, start+3))

    def quad(self, a, b, c, d, material):
        self.triangle(a, b, c, material)
        self.triangle(a, c, d, material)

    def box(self, center, size, material):
        p = [tuple(center[i]+s[i]*size[i]/2 for i in range(3))
             for s in ((-1,-1,-1),(1,-1,-1),(-1,1,-1),(1,1,-1),
                       (-1,-1,1),(1,-1,1),(-1,1,1),(1,1,1))]
        for ids in ((0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)):
            self.quad(*(p[i] for i in ids), material)

    def lathe(self, center, rings, material, sides=10):
        x, y, z = center
        p = [[(x+r*math.cos(i*math.tau/sides), y+r*math.sin(i*math.tau/sides), z+h)
              for i in range(sides)] for h, r in rings]
        for a, b in zip(p, p[1:]):
            for i in range(sides):
                j = (i+1) % sides
                self.quad(a[i], a[j], b[j], b[i], material)
        for i in range(sides):
            j = (i+1) % sides
            self.triangle((x,y,z+rings[0][0]), p[0][j], p[0][i], material)
            self.triangle((x,y,z+rings[-1][0]), p[-1][i], p[-1][j], material)

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
        effects, materials = e(root,'library_effects'), e(root,'library_materials')
        for mat in self.groups:
            fx = e(effects, 'effect', id=mat+'-fx')
            tech = e(e(fx, 'profile_COMMON'), 'technique', sid='common')
            e(e(e(tech,'lambert'),'diffuse'),'color','0.5 0.5 0.5 1')
            e(e(materials,'material',id=mat,name=mat),'instance_effect',url='#'+mat+'-fx')
        geo = e(e(root,'library_geometries'),'geometry',id='geometry',name=self.name)
        mesh = e(geo,'mesh')
        for name, values, params in (('positions',self.vertices,'XYZ'),('normals',self.normals,'XYZ'),('uv',self.uvs,'ST')):
            src = e(mesh,'source',id=name)
            e(src,'float_array',' '.join(f'{x:.7f}' for v in values for x in v), id=name+'-array',count=len(values)*len(params))
            acc = e(e(src,'technique_common'),'accessor',source='#'+name+'-array',count=len(values),stride=len(params))
            for param in params:
                e(acc,'param',name=param,type='float')
        e(e(mesh,'vertices',id='vertices'),'input',semantic='POSITION',source='#positions')
        for mat, indices in self.groups.items():
            tri = e(mesh,'triangles',material=mat,count=len(indices)//3)
            for offset, semantic, source in ((0,'VERTEX','vertices'),(1,'NORMAL','normals'),(2,'TEXCOORD','uv')):
                e(tri,'input',semantic=semantic,source='#'+source,offset=offset,**({'set':0} if offset==2 else {}))
            e(tri,'p',' '.join(str(i) for i in indices for _ in range(3)))
        scene = e(e(root,'library_visual_scenes'),'visual_scene',id='Scene')
        node = e(scene,'node',id=self.name+'300',name=self.name+'300',type='NODE')
        instance = e(node,'instance_geometry',url='#geometry')
        binding = e(e(instance,'bind_material'),'technique_common')
        for mat in self.groups:
            im = e(binding,'instance_material',symbol=mat,target='#'+mat)
            e(im,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set=0)
        e(e(root,'scene'),'instance_visual_scene',url='#Scene')
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
