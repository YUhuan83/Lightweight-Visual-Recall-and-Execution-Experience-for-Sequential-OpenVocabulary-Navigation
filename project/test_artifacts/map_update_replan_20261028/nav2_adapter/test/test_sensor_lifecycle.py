import ast
from pathlib import Path
from types import ModuleType, SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
import importlib.util

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('lifecycle_patch', ROOT/'tools/patch_beamng_sensor_lifecycle.py')
PATCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PATCH)


class SensorLifecycleTests(unittest.TestCase):
    def source(self):
        return (ROOT/'test_artifacts/server_validation_20261002/body_collision_bridge_vehicle.py').read_text(encoding='utf8')

    def test_patch_is_idempotent_on_actual_recorded_bridge_source(self):
        source = PATCH.patched_source(self.source())
        self.assertEqual(PATCH.patched_source(source), source)
        ast.parse(source)

    def test_failed_owned_sensor_cleanup_does_not_leak_other_sensors(self):
        cls = next(n for n in ast.parse(PATCH.patched_source(self.source())).body
                   if isinstance(n, ast.ClassDef))
        cls.bases = [ast.Name(id='Parent', ctx=ast.Load())]
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'destroy_node']
        parent = type('Parent', (), {'destroy_node': lambda self: True})
        ns = {'Parent': parent}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), '<patched vehicle>', 'exec'), ns)
        node = ns['VehicleNode']()
        first, second = Mock(), Mock()
        first.remove.side_effect = RuntimeError('closed socket')
        node.destroyed, node.timer, node.sensors = False, object(), [first, second]
        node._coupling_publisher, node.vehicle, node.logger = None, Mock(), Mock()
        node.destroy_timer = Mock()
        self.assertTrue(node.destroy_node())
        first.remove.assert_called_once()
        second.remove.assert_called_once()
        node.vehicle.disconnect.assert_called_once()
        self.assertFalse(node.destroy_node())
        second.remove.assert_called_once()

    def test_polling_reconnect_reuses_publishers_and_uses_only_known_listener(self):
        cls = next(n for n in ast.parse(PATCH.patched_source(self.source())).body
                   if isinstance(n, ast.ClassDef))
        cls.bases = []
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_attempt_reconnect']
        ns = {}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), '<patched reconnect>', 'exec'), ns)
        connection_module = ModuleType('beamngpy.connection')
        connection_module.Connection = Mock()
        with patch.dict('sys.modules', {'beamngpy.connection': connection_module}):
            for endpoint, expected in (({'result': 18005}, 18005), ({}, 18004)):
                with self.subTest(endpoint=endpoint):
                    connection_module.Connection.reset_mock()
                    node = ns['VehicleNode']()
                    bng = Mock()
                    bng.vehicles.start_connection.return_value = endpoint
                    node.node = S(beamng=lambda: bng, get_parameter=lambda _name: S(
                        get_parameter_value=lambda: S(string_value='127.0.0.1')))
                    node.destroyed, node.logger, node.failures = False, Mock(), 3
                    node.vehicle = Mock(connection=S(port=18004), extensions=[], vid='ego')
                    camera, lidar = Mock(), Mock()
                    node.sensors = [camera, lidar]
                    owned = node.sensors
                    node._attempt_reconnect()
                    connection_module.Connection.assert_called_once_with('127.0.0.1', expected, timeout=30.)
                    self.assertIs(node.sensors, owned)
                    camera.poll.assert_called_once()
                    lidar.poll.assert_called_once()
                    camera.remove.assert_not_called()
                    lidar.remove.assert_not_called()
                    self.assertEqual(node.failures, 0)
                    self.assertFalse(node._manual_control_enabled)

    def test_cosimulation_failure_still_closes_its_publisher(self):
        cls = next(n for n in ast.parse(PATCH.patched_source(self.source())).body
                   if isinstance(n, ast.ClassDef))
        cls.bases = [ast.Name(id='Parent', ctx=ast.Load())]
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'destroy_node']
        ns = {'Parent': type('Parent', (), {'destroy_node': lambda self: True})}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), '<patched cleanup>', 'exec'), ns)
        node = ns['VehicleNode']()
        node.destroyed, node.timer, node.sensors = False, None, []
        coupling = Mock()
        node._coupling_publisher, node.vehicle, node.logger = coupling, Mock(), Mock()
        node._stop_cosimulation = Mock(side_effect=RuntimeError('closed socket'))
        self.assertTrue(node.destroy_node())
        coupling.stop.assert_called_once()
        self.assertIsNone(node._coupling_publisher)
