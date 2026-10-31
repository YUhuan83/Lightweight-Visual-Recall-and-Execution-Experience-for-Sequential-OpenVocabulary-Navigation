import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'nav2_adapter'))
from beamng_wildos_adapter.terrain import terrain_layers, grid_map_values


class TerrainTests(unittest.TestCase):
    def test_observed_heights_keep_real_slope_and_missing_cells_unknown(self):
        points = np.array([[x, y, .2*x-.5] for x in np.arange(4.05, 8., .1)
                           for y in np.arange(2.05, 5., .1)])
        layers = terrain_layers(points)
        valid = layers['elevation_valid'] > .5
        self.assertGreater(valid.sum(), 30)
        rows, cols = np.nonzero(valid)
        design = np.c_[(cols+.5-40)*.5, (rows+.5-40)*.5, np.ones(len(rows))]
        plane = np.linalg.lstsq(design, layers['elevation'][valid], rcond=None)[0]
        self.assertAlmostEqual(plane[0], .2, places=2)
        self.assertEqual(layers['elevation_valid'][40,40], 0.)
        self.assertEqual(layers['unknown'][70,70], 1.)

    def test_single_return_and_vehicle_footprint_do_not_prove_terrain_height(self):
        layers = terrain_layers(np.array([[5., 0., -.5], [0., 0., -.5]]))
        self.assertEqual(layers['elevation_valid'].sum(), 0.)
        self.assertEqual(layers['unknown'][40,40], 0.)

    def test_raised_surface_preserves_measured_height_instead_of_flattening(self):
        points = np.array([[x,y,z] for x in np.arange(4.05,7.,.1)
                           for y in np.arange(-2.95,-1.2,.1)
                           for z in [-.5 if x<5.5 else -.1]])
        layers = terrain_layers(points)
        self.assertAlmostEqual(layers['elevation'][35,50], -.5)
        self.assertAlmostEqual(layers['elevation'][35,52], -.1)

    def test_ros_layout_reverses_both_grid_axes_once(self):
        self.assertEqual(grid_map_values(np.array([[1.,2.],[3.,4.]])), [4.,3.,2.,1.])

    def test_ideal_point_keeps_ground_returns_inside_old_car_footprint(self):
        points = np.array([[1.05, .05, -.5], [1.1, .1, -.45]])
        car = terrain_layers(points, carrier_model="ackermann")
        point = terrain_layers(points, carrier_model="ideal_point")
        self.assertEqual(car["elevation_valid"].sum(), 0.)
        self.assertEqual(point["elevation_valid"][40, 42], 1.)


if __name__ == '__main__': unittest.main()
