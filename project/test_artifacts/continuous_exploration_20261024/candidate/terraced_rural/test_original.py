"""Offline checks, including the actual installed BeamNGpy prefab serializer."""
from collections import Counter, deque
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile
from PIL import Image
import build
import install
import run


class StandaloneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest=install.validate()
        cls.meshes,cls.trees=build.generate_meshes()
        cls.level=build.ROOT/'generated/mod'/build.PREFIX

    def test_zip_integrity_and_independence(self):
        with zipfile.ZipFile(build.ROOT/'generated'/build.MOD_NAME) as z:
            for name in z.namelist():
                self.assertTrue(name.startswith(build.PREFIX))
                if name.endswith(('.json','.dae','.txt')):
                    text=z.read(name).decode('utf-8').lower()
                    for forbidden in ('italy','rural_scene','gazebo','.sdf','tech.key'):
                        self.assertNotIn(forbidden,text)
        for filename in ('build.py','mesh.py','run.py','install.py'):
            self.assertNotIn('italy',(build.ROOT/filename).read_text(encoding='utf-8').lower())

    def test_native_hierarchy_spawn_no_duplicate_car(self):
        rows=[]
        for p in (self.level/'main').rglob('items.level.json'):
            rows += [json.loads(line) for line in p.read_text().splitlines() if line.strip()]
        names={r['name'] for r in rows}
        self.assertEqual(len(names),len(rows))
        self.assertEqual(len({r['persistentId'] for r in rows}),len(rows))
        for row in rows:
            if '__parent' in row: self.assertIn(row['__parent'],names)
            self.assertNotEqual(row['class'],'BeamNGVehicle')
            if row['class']=='TSStatic':
                self.assertTrue((build.ROOT/'generated/mod'/row['shapeName'].lstrip('/')).is_file())
                self.assertEqual(row['collisionType'],'Visible Mesh Final')
        info=json.loads((self.level/'info.json').read_text())
        self.assertFalse(info['isAuxiliary'])
        self.assertEqual(info['title'],'Nizhen Countryside')
        spawn=next(r for r in rows if r['name']==info['defaultSpawnPointName'])
        self.assertEqual(spawn['__parent'],'PlayerDropPoints')
        self.assertEqual(spawn['position'],list(build.SPAWN))
        self.assertEqual(info['spawnPoints'][0]['objectname'],spawn['name'])
        x,y,z=build.SPAWN
        self.assertAlmostEqual(z-build.height(x,y),.65)
        for dx in (-4,0,4):
            for dy in (-1.5,0,1.5): self.assertEqual(build.surface(x+dx,y+dy),'concrete')

    def test_ground_manifold_seams_and_normals(self):
        edges=Counter()
        triangles=set()
        area=0
        max_grade=0
        for name,m in self.meshes.items():
            if not name.startswith('ground_'): continue
            for i in range(0,len(m.vertices),3):
                tri=tuple(tuple(round(v,7) for v in p) for p in m.vertices[i:i+3])
                self.assertNotIn(tuple(sorted(tri)),triangles)
                triangles.add(tuple(sorted(tri)))
                n=m.normals[i]
                self.assertGreater(n[2],0)
                max_grade=max(max_grade,math.hypot(n[0],n[1])/n[2])
                a,b,c=tri
                area+=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
                for j in range(3): edges[tuple(sorted((tri[j],tri[(j+1)%3])))]+=1
        self.assertEqual(area,256*256)
        self.assertLess(max_grade,.14)
        for (a,b),count in edges.items():
            if count==1:
                self.assertTrue(any(abs(a[k])==128 and a[k]==b[k] for k in (0,1)),(a,b))
            else: self.assertEqual(count,2)
        self.assertEqual(sum(v==1 for v in edges.values()),4*256)

    def test_road_connected_loop_branch_clearance(self):
        cells={(x,y) for x in range(-128,128) for y in range(-128,128)
               if build.surface(x+.5,y+.5)!='grass'}
        seen={(-40,-80)}
        queue=deque(seen)
        while queue:
            x,y=queue.popleft()
            for q in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                if q in cells and q not in seen: seen.add(q);queue.append(q)
        self.assertEqual(seen,cells)
        for p in ((0,8),(0,80),(-80,0),(80,0),(0,-80)):
            self.assertIn(p,seen)
        for x,y in self.trees:
            self.assertGreaterEqual(build.road_distance(x,y),10)
        for x,y in build.HOUSES:
            for dx in (-5.5,5.5):
                for dy in (-4.5,4.5): self.assertEqual(build.surface(x+dx,y+dy),'grass')

    def test_cobbles_positive_relief_and_seams(self):
        count=0
        for name,m in self.meshes.items():
            if not name.startswith('cobbles_'): continue
            for x,y,z in m.vertices:
                self.assertGreaterEqual(z,build.height(x,y))
                self.assertLessEqual(z-build.height(x,y),.02500001)
                self.assertGreaterEqual(x,-24)
                self.assertLessEqual(x,24)
                self.assertEqual(build.surface(x,y),'mortar')
                count+=z>10
        self.assertGreater(count,1000)

    def test_all_dae_xml_normals_bindings(self):
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        mats=json.loads((self.level/'art/shapes/main.materials.json').read_text())
        for p in (self.level/'art/shapes').glob('*.dae'):
            root=ET.parse(p).getroot()
            ids={node.attrib['id'] for node in root.iter() if 'id' in node.attrib}
            self.assertEqual(root.find('c:asset/c:up_axis',ns).text,'Z_UP')
            for node in root.iter():
                for key in ('source','url','target'):
                    value=node.get(key,'')
                    if value.startswith('#'): self.assertIn(value[1:],ids)
            sources={n.get('id'):n for n in root.findall('.//c:mesh/c:source',ns)}
            for name,src in sources.items():
                arr=src.find('c:float_array',ns)
                values=[float(v) for v in arr.text.split()]
                self.assertEqual(len(values),int(arr.get('count')))
                self.assertTrue(all(math.isfinite(v) for v in values))
                if name=='normals':
                    for i in range(0,len(values),3):
                        self.assertAlmostEqual(sum(v*v for v in values[i:i+3]),1,places=5)
            bindings={n.get('symbol'):n.get('target')[1:] for n in root.findall('.//c:instance_material',ns)}
            for tri in root.findall('.//c:triangles',ns):
                mat=tri.get('material')
                self.assertEqual(bindings[mat],mat)
                self.assertEqual(mats[mat]['mapTo'],mat)
                indices=[int(v) for v in tri.find('c:p',ns).text.split()]
                self.assertEqual(len(indices),int(tri.get('count'))*9)
                for i in range(0,len(indices),3): self.assertEqual(indices[i:i+3],[indices[i]]*3)
                self.assertLess(max(indices),int(sources['positions'].find('c:technique_common/c:accessor',ns).get('count')))
        for mat in mats.values():
            texture=mat['Stages'][0]['baseColorMap']
            with Image.open(build.ROOT/'generated/mod'/texture.lstrip('/')) as im: im.verify()
        with Image.open(self.level/'preview.png') as im:
            self.assertEqual(im.size,(960,600))
            im.verify()

    def test_real_beamngpy_full_prefab_serializer(self):
        scene,car=run.build_scenario()
        class OfflineTransport:
            def _message(self,kind,**kwargs):
                self.payload=kwargs
                self.kind=kind
                return 'offline/scenario.json'
        transport=OfflineTransport()
        scene.make(transport)
        rows=[json.loads(line) for line in transport.payload['prefab'].splitlines() if line.strip()]
        vehicles=[r for r in rows if r['class']=='BeamNGVehicle']
        self.assertEqual(len(vehicles),1)
        self.assertFalse(any(r['class']=='TSStatic' for r in rows))
        self.assertEqual(transport.payload['level'],build.LEVEL)
        self.assertEqual(vehicles[0]['position'],list(build.SPAWN))
        self.assertEqual(vehicles[0]['jBeam'],'pickup')
        evidence=build.ROOT/'evidence'
        evidence.mkdir(exist_ok=True)
        (evidence/'offline_runtime.prefab.json').write_text(transport.payload['prefab'],encoding='utf-8')
        (evidence/'offline_scenario.json').write_text(json.dumps(transport.payload['info'],indent=2),encoding='utf-8')

    def test_installer_packed_unpacked_and_changed_refusal(self):
        for mode in ('packed','unpacked'):
            with tempfile.TemporaryDirectory(dir=build.ROOT) as tmp:
                user=Path(tmp)/'current'
                user.mkdir()
                with contextlib.redirect_stdout(io.StringIO()):
                    target=install.install(Path(tmp),mode)
                    self.assertEqual(install.installed(user),target)
                    install.install(user,mode)
                if mode=='packed': target.write_bytes(target.read_bytes()+b'changed')
                else: (target/build.PREFIX/'info.json').write_text('{}')
                with self.assertRaisesRegex(ValueError,'modified'):
                    install.install(user,mode,replace=True)

    def test_runtime_requires_explicit_consent(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): run.parse_args(['--connect'])


if __name__=='__main__':
    unittest.main(verbosity=2)
