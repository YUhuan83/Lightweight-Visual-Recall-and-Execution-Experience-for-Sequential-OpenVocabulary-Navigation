"""Offline tests. Synthetic projection data is NOT simulator evidence."""
import contextlib
import io
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zipfile
import numpy as np
from fixture import ROOT, ART, MOD, geometry, scenario, coordinates, fit_intrinsics
from install import install, user_paths
from run import parse_args
from run import camera_check
from PIL import Image


class OfflineTests(unittest.TestCase):
    def test_prefab_end_to_end_json_lists_to_tuples(self):
        _, original = geometry()
        manifest = json.loads(json.dumps(original))
        scene, car = scenario(manifest)
        self.assertIsInstance(scene.objects[0].rot, tuple)
        self.assertIsInstance(scene._vehicle_locations[car.vid][1], tuple)
        # BeamNG's native JSON parser accepts trailing commas; normalize only for stdlib validation.
        rows = [json.loads(re.sub(r',\s*([}\]])', r'\1', line)) for line in scene._get_prefab().splitlines() if line.strip()]
        obj = next(r for r in rows if r['class']=='TSStatic')
        self.assertEqual(obj['shapeName'], ART+'fixture.dae')
        self.assertEqual(obj['collisionType'], 'Visible Mesh')
        self.assertEqual(obj['rotationMatrix'], [1,0,0,0,1,0,0,0,1])
        vehicle = next(r for r in rows if r['class']=='BeamNGVehicle')
        self.assertEqual(vehicle['jBeam'], 'pickup')
        self.assertEqual(vehicle['position'], manifest['spawn']['pos'])

    def test_zip_references_materials_and_mesh(self):
        ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
        with zipfile.ZipFile(ROOT/'dist'/MOD) as z:
            names = set(z.namelist())
            self.assertTrue(all(n.startswith(ART.lstrip('/')) for n in names))
            mats = json.loads(z.read(ART.lstrip('/')+'main.materials.json'))
            mesh = ET.fromstring(z.read(ART.lstrip('/')+'fixture.dae'))
            self.assertEqual(mesh.find('.//c:up_axis',ns).text,'Z_UP')
            for tri in mesh.findall('.//c:triangles',ns):
                mat = tri.attrib['material']
                self.assertEqual(mats[mat]['mapTo'],mat)
                self.assertIn(mats[mat]['Stages'][0]['baseColorMap'].lstrip('/'),names)
                self.assertEqual({i.attrib['semantic'] for i in tri.findall('c:input',ns)}, {'VERTEX','NORMAL','TEXCOORD'})
                self.assertEqual(len(tri.find('c:p',ns).text.split()),int(tri.attrib['count'])*9)
            self.assertNotIn('AmericanRoad',str(mats))
            for source in mesh.findall('.//c:source',ns):
                values = np.fromstring(source.find('c:float_array',ns).text,sep=' ')
                self.assertTrue(np.isfinite(values).all())
                if source.attrib['id']=='normals':
                    np.testing.assert_allclose(np.linalg.norm(values.reshape(-1,3),axis=1),1,atol=1e-7)
                if source.attrib['id']=='uv':
                    self.assertEqual(set(values),{0,1})

    def test_synthetic_pinhole_holdouts(self):
        _, m = geometry()
        xy = coordinates(m['checker']['points_world'],[0,0,2.35],[0,-1,0],[0,0,1])
        focal = 640/(2*np.tan(np.pi/6))
        pixels = xy*focal+[480,320]
        result,_ = fit_intrinsics(xy,np.trunc(pixels))
        self.assertTrue(result['passed'])
        self.assertEqual(len(result['holdout_indices']),18)
        self.assertFalse(set(result['train_indices']) & set(result['holdout_indices']))
        self.assertFalse(fit_intrinsics(xy,pixels*1.2)[0]['passed'])
        with self.assertRaises(ValueError):
            coordinates([[0,1,0]],[0,0,0],[0,-1,0],[0,0,1])

    def test_projection_axis_convention(self):
        xy = coordinates([[1,-10,0],[0,-10,1]],[0,0,0],[0,-1,0],[0,0,1])
        np.testing.assert_allclose(xy,[[-.1,0],[0,-.1]])

    def test_cli_refuses_implicit_connection_and_replacement(self):
        for args in ([], ['--home','.','--user','.'], ['--home','.','--user','.','--connect']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)

    def test_install_idempotent_and_refuses_differing(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as temp:
            parent = Path(temp)
            (parent/'current').mkdir()
            self.assertEqual(user_paths(parent),user_paths(parent/'current'))
            source = parent/'source.zip'
            source.write_bytes(b'own test zip')
            with patch('install.verified_mod',return_value=source):
                target = install(parent)
                self.assertEqual(install(parent),target)
                target.write_bytes(b'existing different user file')
                with self.assertRaises(ValueError):
                    install(parent)
                self.assertEqual(target.read_bytes(),b'existing different user file')
            self.assertFalse((parent/'current/mods/db.json').exists())

    def test_source_kit_allowlist(self):
        with zipfile.ZipFile(ROOT/'dist/sensor_calibration_source.zip') as z:
            self.assertIn('sensor_calibration/run.py',z.namelist())
            for name in z.namelist():
                self.assertIn(Path(name).suffix,('.py','.md','.txt','.ps1'))
                self.assertNotIn('tech.key',name)
                self.assertNotIn('captures/',name)

    def test_synthetic_api_success_never_replaces_visual_evidence(self):
        _, manifest = geometry()
        class SyntheticCamera:
            def get_position(self):
                return (0,0,2.35)
            def get_direction(self):
                return (0,-1,0)
            def send_ad_hoc_poll_request(self):
                return 123
            def is_ad_hoc_poll_request_ready(self, request):
                return True
            def collect_ad_hoc_poll_request(self, request):
                return {'colour':Image.new('RGB',(960,640),'white')}
            def world_point_to_pixel(self, point):
                xy = coordinates([point],self.get_position(),self.get_direction(),[0,0,1])[0]
                return tuple(np.trunc(xy*(640/(2*np.tan(np.pi/6)))+[480,320]).astype(int))
        with tempfile.TemporaryDirectory(dir=ROOT) as temp:
            result = camera_check(SyntheticCamera(),manifest,Path(temp),'static',False,[0,0,1],1)
            self.assertTrue(result['api_fit']['passed'])
            self.assertFalse(result['visual']['passed'])
            self.assertFalse(result['passed'])
            self.assertIsNone(result['metadata']['simulation_timestamp'])
            self.assertTrue((Path(temp)/'static.png').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
