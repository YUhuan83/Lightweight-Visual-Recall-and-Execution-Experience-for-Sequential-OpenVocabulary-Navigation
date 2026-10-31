"""Verify that kinematic status reports both configured and BeamNG-observed modes."""
import ast
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "wildos_nav2_adapter/kinematic_vehicle_control.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf8"))
CLASS = next(node for node in TREE.body
              if isinstance(node, ast.ClassDef) and node.name == "KinematicVehicleControl")
TICK = next(node for node in CLASS.body
            if isinstance(node, ast.FunctionDef) and node.name == "tick")
namespace = {"json": json, "math": math, "time": time}


class String:
    def __init__(self, data=""):
        self.data = data


namespace["String"] = String
exec(compile(ast.fix_missing_locations(ast.Module(body=[TICK], type_ignores=[])),
              str(SOURCE), "exec"), namespace)
PARAMETERS = next(node for node in CLASS.body
                  if isinstance(node, ast.FunctionDef) and node.name == 'on_parameters')
namespace['SetParametersResult'] = SimpleNamespace
exec(compile(ast.fix_missing_locations(ast.Module(body=[PARAMETERS], type_ignores=[])),
             str(SOURCE), 'exec'), namespace)


class RuntimeProfileStatusTests(unittest.TestCase):
    def parameter_controller(self):
        controller = SimpleNamespace(enabled=False, command=(1.2,.3), command_at=100.,
            command_lock=threading.Lock(), rpc_worker=Mock(), carrier_model='ideal_point',
            minimum_turning_radius=0., max_angular_rate=.8,
            get_parameter=lambda _: SimpleNamespace(value=False))
        controller.on_parameters = lambda parameters: namespace['on_parameters'](controller, parameters)
        return controller

    def test_cached_motion_profile_cannot_silently_accept_runtime_changes(self):
        for name, value in (('carrier_model', 'ackermann'), ('minimum_turning_radius', 8.),
                            ('max_angular_rate', 2.), ('use_sim_time', True)):
            controller = self.parameter_controller()
            result = controller.on_parameters([SimpleNamespace(name=name,value=value)])
            self.assertFalse(result.successful, name)
            self.assertEqual(controller.carrier_model, 'ideal_point')
            self.assertEqual(controller.minimum_turning_radius, 0.)

    def test_rejected_parameter_batch_does_not_partially_enable_motion(self):
        controller = self.parameter_controller()
        result = controller.on_parameters([SimpleNamespace(name='enable',value=True),
            SimpleNamespace(name='carrier_model',value='ackermann')])
        self.assertFalse(result.successful)
        self.assertFalse(controller.enabled)
        controller.rpc_worker.retry.assert_not_called()

    def test_explicit_enable_still_clears_prior_command_and_retries_transport(self):
        controller = self.parameter_controller()
        result = controller.on_parameters([SimpleNamespace(name='enable',value=True)])
        self.assertTrue(result.successful)
        self.assertTrue(controller.enabled)
        self.assertEqual(controller.command, (0.,0.))
        controller.rpc_worker.retry.assert_called_once()

    def test_status_contains_configured_and_beamng_observed_ideal_profile(self):
        class Controller:
            tick = namespace["tick"]

        controller = Controller()
        controller.command_lock = threading.Lock()
        controller.command = (0.0, 0.0)
        controller.command_at = 100.0
        controller.enabled = False
        controller.last_state_received_at = 99.9
        controller.last_state_stamp_ns = 456789
        controller.measured_velocity = [0.0, 0.0, 0.0]
        controller.carrier_model = "ideal_point"
        controller.minimum_turning_radius = 0.0
        controller.rpc_worker = Mock()
        controller.rpc_worker.snapshot.return_value = {
            "data": {
                "position": [1.0, 2.0, 3.0],
                "direction": [1.0, 0.0, 0.0],
                "up": [0.0, 0.0, 1.0],
                "carrierModel": "ideal_point",
                "minimumTurningRadius": 0.0,
                "rosYawToClusterSign": -1,
                "rotationConvention": "world-z-passive-v2",
                "visualMeshesHidden": True,
                "profile": "all-fixed-v1",
            },
            "telemetry_ok": True,
            "telemetry_age_sec": 0.0,
            "rpc_wall_time": 1.0,
            "actuator_ok": True,
            "error": None,
        }
        controller.native = Mock()
        controller.control = Mock()
        controller.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=123456789))

        controller.last_state_received_at = time.monotonic()
        controller.tick()

        controller.rpc_worker.set_active.assert_called_once_with(True)
        native = json.loads(controller.native.publish.call_args.args[0].data)
        control = json.loads(controller.control.publish.call_args.args[0].data)
        self.assertEqual(native["carrier_model"], "ideal_point")
        self.assertEqual(native["minimum_turning_radius_m"], 0.0)
        self.assertEqual(native["beamng_carrier_model"], "ideal_point")
        self.assertEqual(native["beamng_minimum_turning_radius_m"], 0.0)
        self.assertEqual(native["beamng_ros_yaw_to_cluster_sign"], -1)
        self.assertEqual(native["beamng_rotation_convention"], "world-z-passive-v2")
        self.assertTrue(native["beamng_visual_meshes_hidden"])
        self.assertEqual(control["carrier_model"], "ideal_point")
        self.assertEqual(control["minimum_turning_radius_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
