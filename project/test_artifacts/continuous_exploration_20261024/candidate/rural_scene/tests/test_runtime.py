import contextlib
import io
import importlib.metadata
import inspect
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zipfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import install_mod
import run


class RuntimeTests(unittest.TestCase):
    def test_unpacked_mod_and_duplicate_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            user = Path(directory).resolve()
            folder = user / 'current/mods/unpacked/nizhen_rural'
            source = run.ROOT / 'generated' / install_mod.MOD_NAME
            with zipfile.ZipFile(source) as archive:
                archive.extractall(folder)
            self.assertEqual(install_mod.installed_mod(user), folder)
            self.assertEqual(install_mod.install(user), folder)
            packed = user / 'current/mods' / install_mod.MOD_NAME
            shutil.copyfile(source, packed)
            with self.assertRaisesRegex(ValueError, 'Both packed'):
                install_mod.installed_mod(user)
            packed.unlink()
            next(folder.rglob('*.dae')).write_text('changed')
            with self.assertRaisesRegex(ValueError, 'differs'):
                install_mod.installed_mod(user)

    def test_timeout_probe_requires_available_port(self):
        args = SimpleNamespace(launch=True, port=64256)
        with patch.object(run.platform, 'system', return_value='Windows'), patch.object(run.subprocess, 'run', return_value=SimpleNamespace(stdout='')), patch.object(run.socket, 'create_connection', side_effect=TimeoutError()), patch.object(run.socket, 'socket') as sock:
            run.guard_connection(args)
            sock.return_value.__enter__.return_value.bind.assert_called_once_with(('127.0.0.1',64256))
            sock.return_value.__enter__.return_value.bind.side_effect = OSError('busy')
            with self.assertRaisesRegex(ValueError, 'unavailable'):
                run.guard_connection(args)
            args.launch = False
            with self.assertRaisesRegex(ValueError, 'probe timed out'):
                run.guard_connection(args)

    def test_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            base = ['--home', directory, '--user', directory, '--replace-current']
            args = run.parse_args(base)
            self.assertEqual(args.duration, 10)
            self.assertFalse(args.launch)
            self.assertIsNone(args.output)
            for extra in (['--duration', 'nan'], ['--duration', '0'], ['--cadence', '-1'],
                          ['--port', '65536'], ['--no-sensors', '--full-sensors'],
                          ['--launch', '--connect']):
                with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    run.parse_args(base + extra)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                run.parse_args(base[:-1])

    def test_native_objects_and_rejections(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'generated').mkdir()
            for name in ('map_objects.json', 'build_manifest.json', install_mod.MOD_NAME):
                shutil.copyfile(run.ROOT / 'generated' / name, root / 'generated' / name)
            manifest = root / 'generated/map_objects.json'
            original = json.loads(manifest.read_text())
            facts, objects = run.load_inputs(root)
            self.assertEqual(facts['spawn']['position'], original['course_spawn']['pos'])
            scene_cls, vehicle_cls, object_cls = Mock(), Mock(), Mock()
            scene, car = run.build(facts, objects, scene_cls, vehicle_cls, object_cls)
            expected = dict(objects[-1])
            for key in ('pos', 'rot_quat', 'scale'):
                expected[key] = tuple(expected[key])
            self.assertEqual(object_cls.call_args.kwargs, expected)
            scene.add_vehicle.assert_called_once()
            self.assertEqual(car, vehicle_cls.return_value)
            for change in ({'shapeName': '/art/shapes/other/wall.dae'},
                           {'shapeName': run.ASSET_ROOT + '../wall.dae'},
                           {'shapeName': run.ASSET_ROOT + 'missing.dae'},
                           {'scale': [0, 1, 1]}, {'pos': [True, 1, 2]},
                           {'rot_quat': [0, 0, 0, 0]}, {'fields': {}},
                           {'otype': 'Vehicle'}, {'collisionType': 'invented'},
                           {'groundType': 'invented'}, {'oid': 'bad id'}):
                modified = json.loads(json.dumps(original))
                modified['objects'][0].update(change)
                manifest.write_text(json.dumps(modified))
                with self.subTest(change=change), self.assertRaises(ValueError):
                    run.load_inputs(root)
            for change in ({'schema_version': 2}, {'schema_version': True},
                           {'level': 'other'}, {'asset_root': '/art/other/'},
                           {'course_spawn': {}}, {'objects': original['objects'] + [original['objects'][0]]}):
                manifest.write_text(json.dumps(dict(original, **change)))
                with self.subTest(change=change), self.assertRaises(ValueError):
                    run.load_inputs(root)
            manifest.write_text(json.dumps(original))
            with (root / 'generated' / install_mod.MOD_NAME).open('ab') as stream:
                stream.write(b'tampered')
            with self.assertRaisesRegex(ValueError, 'SHA-256'):
                run.load_inputs(root)

    def test_real_assets_and_beamngpy_constructors(self):
        from beamngpy import BeamNGpy, Scenario, ScenarioObject, Vehicle
        from beamngpy.sensors import Camera, AdvancedIMU, Lidar
        self.assertEqual(importlib.metadata.version('beamngpy'), '1.36')
        facts, objects = run.load_inputs()
        manifest_path = run.ROOT / 'generated/map_objects.json'
        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(facts['scene_sha256'], install_mod.digest(manifest_path))
        self.assertEqual(facts['schema_version'], 1)
        self.assertEqual(facts['spawn']['position'], manifest['course_spawn']['pos'])
        self.assertEqual(facts['rot_quat'], manifest['course_spawn']['rot_quat'])
        self.assertNotEqual(facts['spawn']['position'], manifest['verified_italy_spawn']['pos'])
        with patch.object(BeamNGpy, 'open', side_effect=AssertionError('No simulator allowed')):
            scene, car = run.build(facts, objects, Scenario, Vehicle, ScenarioObject)
            prefab = scene._get_prefab()
            self.assertIsInstance(prefab, str)
            self.assertIn('TSStatic', prefab)
            self.assertIn('rural_ego', prefab)
            self.assertEqual(len(scene.objects), len(objects))
            for actual, definition in zip(scene.objects, objects):
                self.assertEqual(actual.id, definition['oid'])
                self.assertEqual(actual.opts['shapeName'], definition['shapeName'])
                self.assertEqual(actual.opts.get('groundType'), definition.get('groundType'))
            args = SimpleNamespace(no_sensors=False, full_sensors=True, side_cameras=False, lidar=False, cadence=0.2)
            for cls, name, opts in run.sensor_specs(args, Camera, AdvancedIMU, Lidar):
                inspect.signature(cls).bind(name, None, car, **opts)
            inspect.signature(BeamNGpy).bind('127.0.0.1', 64256, home='home', user='user',
                                           quit_on_close=False, headless=True, gfx='vk', socket_timeout=30)

    def test_real_mod_references(self):
        _, objects = run.load_inputs()
        with zipfile.ZipFile(run.ROOT / 'generated' / install_mod.MOD_NAME) as archive:
            entries = set(archive.namelist())
            materials = json.loads(archive.read('art/shapes/nizhen_rural/main.materials.json'))
            mapped = {material['mapTo'] for material in materials.values()}
            for material in materials.values():
                for stage in material.get('Stages', []):
                    for key, value in stage.items():
                        if key.endswith('Map') and isinstance(value, str) and value:
                            self.assertIn(value.lstrip('/'), entries)
            namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
            for shape in {obj['shapeName'] for obj in objects}:
                self.assertIn(shape[1:], entries)
                mesh = ET.fromstring(archive.read(shape[1:]))
                for material in mesh.findall('.//c:library_materials/c:material', namespace):
                    self.assertIn(material.attrib['name'], mapped)
            prefab = [json.loads(line) for line in archive.read('art/prefabs/nizhen_rural/course.prefab.json').decode().splitlines() if line.strip()]
            native = {obj['name']: obj for obj in prefab if obj['class'] == 'TSStatic'}
            self.assertEqual(set(native), {obj['name'] for obj in objects})
            for obj in objects:
                for key in ('shapeName', 'collisionType', 'scale', 'groundType'):
                    self.assertEqual(native[obj['name']].get(key), obj.get(key))
                self.assertEqual(native[obj['name']]['position'], obj['pos'])

    def test_process_and_socket_guards(self):
        args = SimpleNamespace(launch=True, port=64256)
        with patch.object(run.platform, 'system', return_value='Windows'), patch.object(run.subprocess, 'run', return_value=SimpleNamespace(stdout='"BeamNG.tech.x64.exe"')) as process, patch.object(run.socket, 'create_connection') as connect:
            with self.assertRaises(ValueError):
                run.guard_connection(args)
            connect.assert_not_called()
            self.assertEqual(process.call_args.args[0][0], 'tasklist')
        with patch.object(run.platform, 'system', return_value='Linux'), patch.object(run.subprocess, 'run', return_value=SimpleNamespace(stdout='python\n')) as process, patch.object(run.socket, 'create_connection', side_effect=ConnectionRefusedError):
            run.guard_connection(args)
            self.assertEqual(process.call_args.args[0], ['ps', '-A', '-o', 'comm='])
            args.launch = False
            with self.assertRaises(ValueError):
                run.guard_connection(args)
        args.launch = True
        with patch.object(run.platform, 'system', return_value='Linux'), patch.object(run.subprocess, 'run', return_value=SimpleNamespace(stdout='')), patch.object(run.socket, 'create_connection'):
            with self.assertRaises(ValueError):
                run.guard_connection(args)

    def test_sensor_selection(self):
        args = SimpleNamespace(no_sensors=False, full_sensors=False, side_cameras=False, lidar=False, cadence=0.2)
        specs = run.sensor_specs(args, 'camera', 'imu', 'lidar')
        self.assertEqual([s[1] for s in specs], ['front', 'imu'])
        self.assertFalse(specs[0][2]['is_render_annotations'])
        self.assertFalse(specs[0][2]['is_render_instance'])
        args.full_sensors = True
        self.assertEqual(len(run.sensor_specs(args, 'camera', 'imu', 'lidar')), 5)

    def test_samples_keep_time_not_pose(self):
        reading = [{'time': 1.25, 'accRaw': [1, 2, 3], 'angVel': [0, 0, 0],
                    'pos': [123, 456, 789], 'dirX': [1, 0, 0]}]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            for index in range(2):
                run.save_sample(output, index, 'imu', reading, {'host_request_unix_ns': 123 + index})
            rows = [json.loads(line) for line in (output / 'index.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]['sensor_metadata'][0]['time'], 1.25)
            self.assertNotIn('pos', rows[0]['sensor_metadata'][0])
            self.assertNotIn('dirX', rows[0]['sensor_metadata'][0])
            for row in rows:
                self.assertTrue((output / row['files'][0]).is_file())

    def test_installer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'bundle'
            (root / 'generated').mkdir(parents=True)
            source = root / 'generated' / install_mod.MOD_NAME
            with zipfile.ZipFile(source, 'w') as archive:
                archive.writestr('art/shapes/nizhen_rural/example.txt', 'fixture')
            (root / 'generated/build_manifest.json').write_text(json.dumps({
                'zip': install_mod.MOD_NAME, 'zip_sha256': install_mod.digest(source),
                'zip_bytes': source.stat().st_size,
                'packaged_files': ['art/shapes/nizhen_rural/example.txt']}))
            user = Path(directory) / 'user'
            with patch.object(install_mod, 'ROOT', root):
                target = install_mod.install(user)
                self.assertEqual(install_mod.digest(source), install_mod.digest(target))
                self.assertEqual(install_mod.install(user), target)
                target.write_bytes(b'older different mod')
                with self.assertRaises(ValueError):
                    install_mod.install(user)
                self.assertEqual(target.read_bytes(), b'older different mod')
                install_mod.install(user, backup=True)
                backups = list(target.parent.glob('*.bak'))
                self.assertEqual(len(backups), 1)
                self.assertEqual(backups[0].read_bytes(), b'older different mod')
                self.assertEqual(install_mod.digest(source), install_mod.digest(target))

    def test_archive_allowlist_and_roots(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'generated').mkdir()
            source = root / 'generated' / install_mod.MOD_NAME
            for entry, listed in (
                    ('levels/italy/unsafe.json', ['levels/italy/unsafe.json']),
                    ('art/shapes/nizhen_rural/../unsafe.dae', ['art/shapes/nizhen_rural/../unsafe.dae']),
                    ('art/shapes/nizhen_rural/mesh.dae', ['art/shapes/nizhen_rural/missing.dae'])):
                with zipfile.ZipFile(source, 'w') as archive:
                    archive.writestr(entry, 'fixture')
                (root / 'generated/build_manifest.json').write_text(json.dumps({
                    'zip': install_mod.MOD_NAME, 'zip_sha256': install_mod.digest(source),
                    'zip_bytes': source.stat().st_size, 'packaged_files': listed}))
                with self.subTest(entry=entry), self.assertRaises(ValueError):
                    install_mod.validate_mod(root)


if __name__ == '__main__':
    unittest.main()
