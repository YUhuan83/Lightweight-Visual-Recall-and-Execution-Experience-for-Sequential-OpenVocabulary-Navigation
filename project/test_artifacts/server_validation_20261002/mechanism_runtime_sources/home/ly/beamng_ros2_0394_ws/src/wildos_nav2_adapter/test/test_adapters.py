import importlib.util
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "math3d", HERE / "wildos_nav2_adapter" / "math3d.py")
MATH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MATH)


class DummyVector:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class Math3dTests(unittest.TestCase):
    def test_vector3_message_is_accepted(self):
        q = MATH.quaternion_from_forward_up(DummyVector(1, 0, 0), DummyVector(0, 0, 1))
        self.assertAlmostEqual(sum(v * v for v in q), 1.0, places=6)
        self.assertAlmostEqual(q[3], 1.0, places=6)

    def test_sequence_is_accepted(self):
        q = MATH.quaternion_from_forward_up([1, 0, 0], [0, 0, 1])
        self.assertAlmostEqual(sum(v * v for v in q), 1.0, places=6)

    def test_zero_vector_does_not_divide_by_zero(self):
        q = MATH.quaternion_from_forward_up([0, 0, 0], [0, 0, 1])
        self.assertTrue(all(isinstance(v, float) for v in q))

    def test_world_velocity_projects_to_forward_axis(self):
        velocity = MATH.world_velocity_to_body([0, -2, 0], [0, -1, 0], [0, 0, 1])
        self.assertAlmostEqual(velocity[0], 2.0)
        self.assertAlmostEqual(velocity[1], 0.0)

    def test_world_up_projects_positive(self):
        velocity = MATH.world_velocity_to_body([0, 0, 2], [1, 0, 0], [0, 0, 1])
        self.assertAlmostEqual(velocity[2], 2.0)

    def test_quaternion_matrix_identity(self):
        matrix = MATH.matrix_from_quaternion([0, 0, 0, 1])
        self.assertEqual(matrix, ((1.0, 0.0, 0.0),
                                  (0.0, 1.0, 0.0),
                                  (0.0, 0.0, 1.0)))


if __name__ == "__main__":
    unittest.main()
