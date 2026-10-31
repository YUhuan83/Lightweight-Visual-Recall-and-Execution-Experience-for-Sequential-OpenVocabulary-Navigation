"""Regression for road returns classified as cross-road obstacle bands."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('scan_ground',Path(__file__).parents[1]/'wildos_nav2_adapter/pointcloud.py')
cloud=importlib.util.module_from_spec(spec);spec.loader.exec_module(cloud)

class CurvedScanGround(unittest.TestCase):
    def split(self,points):
        return cloud.split_costmap_points(points,min_height=.1,max_height=1.5,
            filter_footprint=False,front_extent=3.4,rear_extent=1.05,
            half_width=1.05,voxel=.1,max_points=20000,
            ground_plane_radius=8.,obstacle_min_height=.1)

    def scan(self):
        # Measured scan rings have radial gaps; a continuous road need not
        # produce a dense connected lattice of 10 cm returns.
        radius=np.r_[np.arange(1.,5.,.15),np.arange(5.,18.,.65)]
        theta=np.linspace(-.48,.48,180)
        r,t=np.meshgrid(radius,theta,indexing='ij')
        x,y=(r*np.cos(t)).ravel(),(r*np.sin(t)).ravel()
        return np.c_[x,y,-.15+.002*x*x]

    def test_sparse_curved_road_is_not_an_obstacle(self):
        points=self.scan()
        marked,clearing=self.split(points)
        self.assertEqual(len(marked),0)
        np.testing.assert_array_equal(clearing,cloud.voxel_downsample(points,.1,20000))

    def test_raised_object_on_curve_remains_an_obstacle(self):
        points=self.scan();patch=(points[:,0]>10)&(points[:,0]<12)&(abs(points[:,1])<.5)
        points[patch,2]+=.25
        marked,_=self.split(points)
        self.assertGreater(len(marked),0)
        self.assertTrue(np.all((marked[:,0]>10)&(marked[:,0]<12)&(abs(marked[:,1])<.5)))

    def test_step_across_road_does_not_become_smooth_ground(self):
        points=self.scan();step=points[:,0]>9
        points[step,2]+=.25
        marked,_=self.split(points)
        self.assertGreater(len(marked[marked[:,0]>9]),0)

    def test_cloud_order_does_not_change_classification(self):
        points=self.scan();first,_=self.split(points)
        other,_=self.split(points[np.random.default_rng(31).permutation(len(points))])
        def voxels(p):return np.unique(np.floor(p/.1).astype(np.int64),axis=0)
        np.testing.assert_array_equal(voxels(first),voxels(other))

    def test_recorded_scan_removes_road_bands_and_preserves_above_surface_returns(self):
        fixtures=Path(__file__).parent/'fixtures/curved_scan_ground'
        for serial in (24,34):
            with self.subTest(request=serial),np.load(fixtures/f'request_{serial}.npz') as data:
                p=data['points']
                marked,clearing=cloud.split_costmap_points(p,min_height=.1,max_height=1.5,
                    filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,
                    voxel=.2,max_points=20000,ground_plane_radius=8.,obstacle_min_height=.1)
                keys={tuple(row) for row in marked}
                still_ground=sum(tuple(row) in keys for row in p[data['old_smooth_marks']])
                self.assertLessEqual(still_ground,.1*int(data['old_smooth_marks'].sum()))
                for row in p[data['old_raised_marks']]:self.assertIn(tuple(row),keys)
                expected=cloud.filter_robot_footprint(p,3.4,1.05,1.05)
                expected=cloud.voxel_downsample(expected[expected[:,2]<=1.5],.2,20000)
                np.testing.assert_array_equal(clearing,expected)

if __name__=='__main__':unittest.main()
