import importlib.util
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("offline_perception", HERE / "wildos_ros2_adapter" / "offline_perception.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OfflinePerceptionTests(unittest.TestCase):
    def test_identity_quaternion(self):
        np.testing.assert_allclose(MODULE.quaternion_matrix([0, 0, 0, 1]), np.eye(3))

    def test_non_unit_quaternion_is_normalized(self):
        np.testing.assert_allclose(MODULE.quaternion_matrix([0, 0, 0, 2]), np.eye(3))

    def test_localize_mask_lidar(self):
        cloud = np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 5.1]], dtype=float)
        mask = np.zeros((100, 100), dtype=bool)
        mask[45:55, 45:55] = True
        K = np.array([[10.0, 0.0, 50.0], [0.0, 10.0, 50.0], [0.0, 0.0, 1.0]])
        point = MODULE.localize_mask_lidar(cloud, mask, K, np.eye(3), np.zeros(3), 2)
        self.assertIsNotNone(point)

    def test_worker_runs_in_external_cuda_interpreter(self):
        source = (HERE / "wildos_ros2_adapter" / "offline_perception.py").read_text(encoding="utf-8")
        self.assertIn("subprocess.Popen", source)
        self.assertIn("args.worker_python", source)
        self.assertIn("args.output_dir", source)


if __name__ == "__main__":
    unittest.main()
