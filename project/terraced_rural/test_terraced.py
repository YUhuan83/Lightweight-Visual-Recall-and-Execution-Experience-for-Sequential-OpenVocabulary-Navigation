"""Offline geometry, installation, and real BeamNGpy serialization regression tests."""
from collections import Counter
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import zipfile
import build
import install
import run
import test_original


class TerracedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest=install.validate()
        build.SEED=cls.manifest['seed']; build.MODE=cls.manifest['mode']
        cls.meshes,_=build.generate_meshes()
        cls.level=build.ROOT/'generated/mod'/build.PREFIX

    test_all_dae_xml_normals_bindings=test_original.StandaloneTests.test_all_dae_xml_normals_bindings
    test_real_beamngpy_full_prefab_serializer=test_original.StandaloneTests.test_real_beamngpy_full_prefab_serializer
    test_runtime_requires_explicit_consent=test_original.StandaloneTests.test_runtime_requires_explicit_consent
    test_zip_integrity_and_independence=test_original.StandaloneTests.test_zip_integrity_and_independence

    def test_ground_shared_samples_normals_continuity(self):
        edges=Counter(); area=0
        for name,m in self.meshes.items():
            if not name.startswith('ground_'): continue
            for i in range(0,len(m.vertices),3):
                tri=m.vertices[i:i+3]
                self.assertGreater(m.normals[i][2],0)
                a,b,c=tri
                area+=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
                for p in tri: self.assertEqual(p[2],build.height(*p[:2]))
                for j in range(3): edges[tuple(sorted((tri[j],tri[(j+1)%3])))]+=1
        self.assertAlmostEqual(area,256*128)
        for (a,b),count in edges.items():
            if count==1: self.assertTrue((a[0]==b[0] and abs(a[0])==128) or (a[1]==b[1] and abs(a[1])==64))
            else: self.assertEqual(count,2)
        for x in (-75,-40,40,75):
            self.assertTrue(1.2<build.height(x,7)-10<2)
            self.assertLess(build.height(x,-5),9.1)
            self.assertLess(build.height(x,-15),9.5)
            self.assertGreater(build.height(x,40),build.height(x,12))

    def test_native_spawn_and_turning_clearance(self):
        rows=[json.loads(line) for p in (self.level/'main').rglob('items.level.json') for line in p.read_text().splitlines()]
        names={r['name'] for r in rows}
        self.assertEqual(len(names),len(rows))
        self.assertEqual(len({r['persistentId'] for r in rows}),len(rows))
        for r in rows:
            if '__parent' in r: self.assertIn(r['__parent'],names)
            self.assertNotEqual(r['class'],'BeamNGVehicle')
            if r['class']=='TSStatic':
                self.assertTrue((build.ROOT/'generated/mod'/r['shapeName'].lstrip('/')).is_file())
                self.assertEqual(r['collisionType'],'None' if 'grass_patch_' in r['name'] else 'Visible Mesh Final')
        info=json.loads((self.level/'info.json').read_text())
        self.assertEqual(info['title'],'Terraced Countryside'); self.assertFalse(info['isAuxiliary'])
        spawn=next(r for r in rows if r['name']==info['defaultSpawnPointName'])
        self.assertEqual(spawn['position'],list(build.SPAWN))
        self.assertAlmostEqual(build.SPAWN[2]-build.height(*build.SPAWN[:2]),.65)
        for cx in (-103,103):
            for x in range(cx-11,cx+12):
                for y in range(-11,12):
                    self.assertEqual(build.height(x,y),10)
                    self.assertEqual(build.surface(x,y),'concrete')
        for x in range(-90,91):
            for y in (-1.69,0,1.69): self.assertEqual(build.height(x,y),10)
        for x in range(-13,14):
            for y in (-1,0,1,2,3,4,5): self.assertEqual(build.height(x,y),10)

    def test_seed_modes_and_actual_geometry_free_width(self):
        a=build.obstacles(123,'passable')
        self.assertEqual(a,build.obstacles(123,'passable'))
        self.assertNotEqual(a,build.obstacles(124,'passable'))
        for seed in (0,1,99,123,20260911):
            records=build.obstacles(seed,'passable')
            self.assertEqual({r['kind'] for r in records},{'crate','barrel','sack','rock'})
            for r in records:
                bb=build.bounds(build.obstacle_mesh(r)); self.assertEqual(bb,r['aabb'])
                for k in (0,1):
                    self.assertAlmostEqual(bb[0][k],r['center'][k]-r['size'][k]/2)
                    self.assertAlmostEqual(bb[1][k],r['center'][k]+r['size'][k]/2)
                self.assertAlmostEqual(bb[1][2]-bb[0][2],r['size'][2])
                self.assertGreater(bb[0][0],-85); self.assertLess(bb[1][0],85)
                if r['role']=='encroachment': self.assertGreaterEqual(bb[0][1]+1.7,2.2+2*.25)
                else: self.assertTrue(bb[0][1]>2.3 or bb[1][1]<-2.3)
            for i,r in enumerate(records):
                for s in records[i+1:]:
                    a,b=r['aabb'],s['aabb']
                    self.assertTrue(any(a[1][k]<=b[0][k] or b[1][k]<=a[0][k] for k in (0,1)),(r,s))
        blocked=build.obstacles(123,'blocked')
        closure=[r for r in blocked if r['role']=='intentional_closure']
        self.assertEqual(len(closure),4)
        intervals=sorted((r['aabb'][0][1],r['aabb'][1][1]) for r in closure)
        self.assertLess(max(intervals[i+1][0]-intervals[i][1] for i in range(3)),.03)

    def test_grass_geometry_and_build_budget(self):
        self.assertLess(self.manifest['triangles'],300000)
        self.assertGreater(self.manifest['grass_triangles'],40000)
        for name,m in self.meshes.items():
            if not name.startswith('grass_patch_'): continue
            for i in range(0,len(m.vertices),6):
                self.assertEqual(m.vertices[i:i+3],m.vertices[i+3:i+6][::-1])
                self.assertGreater(max(v[2] for v in m.vertices[i:i+3])-min(v[2] for v in m.vertices[i:i+3]),.2)

    def test_installer_packed_unpacked_and_modified_refusal(self):
        # Test a copied runtime root with a local verification receipt; no live profile touched.
        with tempfile.TemporaryDirectory(dir=build.ROOT) as tmp:
            root=Path(tmp); (root/'generated').mkdir()
            import shutil
            for name in (build.MOD_NAME,'build_manifest.json'):
                shutil.copyfile(build.ROOT/'generated'/name,root/'generated'/name)
            build.write_json(root/'generated/verification.json',dict(successful=True,zip_sha256=self.manifest['zip_sha256']))
            for mode in ('packed','unpacked'):
                user=root/mode; user.mkdir()
                with contextlib.redirect_stdout(io.StringIO()):
                    target=install.install(user,mode,root=root)
                    install.install(user,mode,root=root)
                if mode=='packed': target.write_bytes(target.read_bytes()+b'changed')
                else: (target/build.PREFIX/'info.json').write_text('{}')
                with self.assertRaisesRegex(ValueError,'modified'): install.install(user,mode,replace=True,root=root)


if __name__=='__main__': unittest.main(verbosity=2)
