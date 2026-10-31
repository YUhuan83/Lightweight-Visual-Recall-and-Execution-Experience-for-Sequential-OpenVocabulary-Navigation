import argparse
import inspect
import json
import unittest
from unittest.mock import patch

import scene


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = scene.discover(scene.HOME)

    def test_native_vehicle_and_clearance(self):
        scenario, car = scene.build(self.data)
        self.assertEqual(car.options["partConfig"], scene.CONFIG)
        self.assertEqual(len(scenario.vehicles), 1)
        self.assertEqual(len(scenario.proc_meshes), 2)
        self.assertTrue(all(o["spawn_clearance_m"] > 15 for o in self.data["objects"]))
        lines = [json.loads(line) for line in scenario._get_prefab().splitlines() if line.strip()]
        vehicle = next(o for o in lines if o["class"] == "BeamNGVehicle")
        self.assertEqual(vehicle["position"], self.data["spawn"]["position"])
        self.assertEqual(vehicle["jBeam"], "pickup")

    def test_sensor_signatures_and_flags(self):
        for full, count in ((False, 2), (True, 5)):
            args = argparse.Namespace(no_sensors=False, full_sensors=full,
                                      side_cameras=False, lidar=False)
            specs = scene.sensor_specs(args)
            self.assertEqual(len(specs), count)
            for cls, name, options in specs:
                inspect.signature(cls).bind(name, None, None, **options)
        args.no_sensors = True
        self.assertEqual(scene.sensor_specs(args), [])

    def test_refuse_without_permission_before_connect(self):
        with patch("sys.argv", ["scene.py"]), patch.object(scene, "BeamNGpy") as bng:
            with self.assertRaises(SystemExit) as result:
                scene.main()
            self.assertEqual(result.exception.code, 2)
            bng.assert_not_called()

    def test_minimal_preview(self):
        scenario, _ = scene.build(self.data, objects=False)
        self.assertEqual(len(scenario.proc_meshes), 0)
        self.assertEqual(len(scenario.roads), 0)


if __name__ == "__main__":
    unittest.main()
