"""Offline authored geometry, native references and packaging contract tests."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import zipfile

from rural_scene.geometry import Route, build_geometry, cross, sub, scenario_objects

ROOT = Path(__file__).resolve().parent
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT/"course_config.json").read_text())
        cls.manifest = json.loads((ROOT/"generated/map_objects.json").read_text())
        cls.route = Route(cls.manifest["route_nodes_world"], cls.config)
        cls.meshes, cls.stats = build_geometry(cls.route)

    def test_length_nodes_grade_and_height(self):
        r = self.route
        self.assertGreater(r.length, 100)
        self.assertLess(r.length, 200)
        for s, node in zip(r.stations, r.nodes):
            for a, b in zip(r.center(s), node):
                self.assertAlmostEqual(a, b, places=8)
        for i, (a, b) in enumerate(zip(r.stations, r.stations[1:])):
            lo, hi = sorted((r.nodes[i][2], r.nodes[i+1][2]))
            for j in range(101):
                s = a+(b-a)*j/100
                z = r.center(s)[2]
                self.assertGreaterEqual(z+1e-10, lo)
                self.assertLessEqual(z-1e-10, hi)
                p, q = r.center(s-.01), r.center(s+.01)
                self.assertLess(abs(q[2]-p[2])/math.dist(p[:2], q[:2]), self.config["max_grade"])
                self.assertLessEqual(r.point(s)[2]-z, .02000001)

    def test_continuity_and_width(self):
        seams = self.stats["road_seams_local"]
        self.assertEqual(seams["concrete_entry"][1], seams["cobble_bedding"][0])
        self.assertEqual(seams["cobble_bedding"][1], seams["concrete_exit"][0])
        r = self.route
        self.assertEqual(r.lift(0), 0)
        self.assertEqual(r.lift(r.length), 0)
        for s in r.stations[1:-1]:
            before = sub(r.center(s), r.center(s-.001))
            after = sub(r.center(s+.001), r.center(s))
            self.assertLess(math.dist(before, after), 1e-6)
        for i in range(1001):
            s = r.length*i/1000
            self.assertAlmostEqual(math.dist(r.point(s, -1.8), r.point(s, 1.8)), 3.6, places=9)

    def test_stones_real_relief_and_no_inverted_tops(self):
        self.assertEqual(self.stats["stone_count"], 3000)
        self.assertGreater(self.stats["stone_relief_min_m"], 0)
        self.assertLessEqual(self.stats["stone_relief_max_m"], .04)
        mesh = self.meshes["cobbles_merged"]
        self.assertEqual(len(mesh.vertices)//3, 30000)
        for i in range(0, len(mesh.vertices), 30):
            self.assertGreater(mesh.normals[i][2], .99)

    def test_all_triangles_normals_indices(self):
        for mesh in self.meshes.values():
            self.assertEqual(len(mesh.vertices), len(mesh.normals))
            self.assertEqual(len(mesh.vertices), len(mesh.uvs))
            for ids in mesh.groups.values():
                self.assertEqual(len(ids) % 3, 0)
                self.assertGreaterEqual(min(ids), 0)
                self.assertLess(max(ids), len(mesh.vertices))
            for i in range(0, len(mesh.vertices), 3):
                a, b, c = mesh.vertices[i:i+3]
                normal = mesh.normals[i]
                self.assertAlmostEqual(sum(n*n for n in normal), 1, places=9)
                face = cross(sub(b, a), sub(c, a))
                self.assertGreater(sum(x*y for x, y in zip(face, normal)), 1e-10)
                self.assertTrue(all(math.isfinite(v) for p in (a, b, c) for v in p))

    def test_local_geometry_translation_invariance(self):
        shifted = [[p[0]+10000, p[1]-20000, p[2]+500, p[3]] for p in self.manifest["route_nodes_world"]]
        r = Route(shifted, self.config)
        for s in (0, 20, 60, 100, self.route.length):
            self.assertLess(math.dist(r.point(s, 1.4), self.route.point(s, 1.4)), 1e-8)

    def test_native_dae_material_and_texture_contract(self):
        art = ROOT/"generated/mod/art/shapes/nizhen_rural"
        materials = json.loads((art/"main.materials.json").read_text())
        for name, mat in materials.items():
            self.assertEqual(mat["mapTo"], name)
            self.assertEqual(mat["version"], 1.5)
            self.assertEqual(mat["class"], "Material")
            texture = mat["Stages"][0]["baseColorMap"]
            self.assertTrue((ROOT/"generated/mod"/texture.lstrip("/")).is_file())
            self.assertNotIn("diffuseMap", mat["Stages"][0])
        for path in art.glob("*.dae"):
            root = ET.parse(path).getroot()
            self.assertEqual(root.find("c:asset/c:up_axis", NS).text, "Z_UP")
            self.assertEqual(root.find("c:asset/c:unit", NS).get("meter"), "1")
            for tri in root.findall(".//c:triangles", NS):
                self.assertIn(tri.get("material"), materials)
                indices = list(map(int, tri.find("c:p", NS).text.split()))
                self.assertEqual(len(indices), int(tri.get("count"))*9)
                self.assertEqual({i.get("semantic") for i in tri.findall("c:input", NS)}, {"VERTEX", "NORMAL", "TEXCOORD"})
            for arr in root.findall(".//c:float_array", NS):
                self.assertEqual(len(arr.text.split()), int(arr.get("count")))

    def test_packaging_no_game_or_secrets(self):
        report = json.loads((ROOT/"generated/build_manifest.json").read_text())
        path = ROOT/"generated/nizhen_rural.zip"
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), report["zip_sha256"])
        with zipfile.ZipFile(path) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()), set(report["packaged_files"]))
            for name in archive.namelist():
                self.assertTrue(name.startswith(("art/shapes/nizhen_rural/", "art/prefabs/nizhen_rural/")))
                self.assertNotIn("..", name)
                self.assertNotIn(".key", name)
                self.assertNotIn("levels/", name)
                self.assertNotIn("vehicles/", name)
        self.assertEqual(len(self.manifest["objects"]), 11)
        for obj in self.manifest["objects"]:
            self.assertEqual(obj["collisionType"], "Visible Mesh")
            self.assertEqual(obj["otype"], "TSStatic")

    def test_spawn_heading_and_clearance(self):
        spawn = self.manifest["course_spawn"]
        x, y, z, w = spawn["rot_quat"]
        forward = (2*z*w, -(1-2*z*z), 0)
        self.assertLess(math.dist(forward, spawn["forward_world"]), 1e-9)
        for obj in self.manifest["objects"][5:]:
            self.assertGreater(math.dist(obj["pos"], spawn["pos"]), 20)
        self.assertEqual(spawn["status"], "candidate_not_simulator_verified")

    def test_beamngpy_constructor_if_available(self):
        try:
            import beamngpy
        except ImportError:
            self.skipTest("BeamNGpy not installed in this interpreter")
        objects = scenario_objects()
        self.assertEqual(len(objects), 11)
        self.assertTrue(all(isinstance(o, beamngpy.ScenarioObject) for o in objects))


if __name__ == "__main__":
    unittest.main()
