"""Unit tests for the A* occupancy-grid planner."""
import unittest

import numpy as np

from wildos_lite import planner


class PlannerTests(unittest.TestCase):
    def test_empty_grid_straight_path(self):
        grid = planner.build_grid(None, (0, 0), (10, 0), half_extent=4.0, resolution=0.5)
        path = planner.astar(grid, (0, 0), (10, 0))
        self.assertTrue(path)
        self.assertAlmostEqual(path[0][0], 0.0, places=6)
        self.assertAlmostEqual(path[-1][0], 10.0, places=6)
        self.assertAlmostEqual(path[-1][1], 0.0, places=6)

    def test_wall_forces_detour(self):
        wall = [(5.0, y) for y in np.linspace(-3.0, 3.0, 40)]
        grid = planner.build_grid(wall, (0, 0), (10, 0), half_extent=5.0,
                                  resolution=0.5, inflation=1.0)
        path = planner.astar(grid, (0, 0), (10, 0))
        self.assertTrue(path)
        straight = np.linalg.norm(np.asarray(path[-1]) - np.asarray(path[0]))
        walked = sum(np.linalg.norm(np.asarray(path[i + 1]) - np.asarray(path[i]))
                     for i in range(len(path) - 1))
        self.assertGreater(walked, straight + 1.0)  # detour is longer than a straight line
        for x, y in path:
            self.assertFalse(x > 4.0 and x < 6.0 and -1.0 < y < 1.0)

    def test_grid_inflation_marks_cells(self):
        grid = planner.build_grid([(5.0, 0.0)], (0, 0), (10, 0), half_extent=5.0, inflation=1.5)
        self.assertTrue(grid.occ[grid.to_index((5.0, 0.0))])
        self.assertFalse(grid.occ[grid.to_index((0.0, 0.0))])

    def test_simplify_reduces_collinear(self):
        path = [(float(i), 0.0) for i in range(11)]
        simple = planner.simplify(path, None, obstacle_radius=1.0)
        self.assertEqual(len(simple), 2)


if __name__ == "__main__":
    unittest.main()
