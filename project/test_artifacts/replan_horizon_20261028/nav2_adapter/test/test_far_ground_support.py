"""Far ground must have its own measured support, preserving steps and gaps."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('far_ground_pointcloud',Path(__file__).parents[1]/'wildos_nav2_adapter/pointcloud.py')
cloud=importlib.util.module_from_spec(spec);spec.loader.exec_module(cloud)

class FarGroundSupport(unittest.TestCase):
    def split(self,points):
        return cloud.split_costmap_points(points,min_height=.1,max_height=1.5,
            filter_footprint=False,front_extent=3.4,rear_extent=1.05,
            half_width=1.05,voxel=.1,max_points=20000,
            ground_plane_radius=8.,obstacle_min_height=.1)

    def surface(self):
        xy=np.array([(x,y) for x in np.arange(.05,18.,.25)
                     for y in np.arange(-3.,3.01,.25)])
        return np.c_[xy,.02*xy[:,0]-.15]

    def test_supported_far_surface_is_not_marked_by_body_tilt(self):
        points=self.surface()
        marked,clearing=self.split(points)
        self.assertEqual(len(marked),0)
        self.assertGreater(len(clearing),0)

    def test_supported_far_surface_keeps_a_real_raised_patch(self):
        points=self.surface()
        patch=(points[:,0]>=14)&(points[:,0]<16)&(abs(points[:,1])<.5)
        points[patch,2]+=.16
        marked,_=self.split(points)
        self.assertEqual(len(marked),int(patch.sum()))
        self.assertTrue(np.all(marked[:,0]>=14))

    def test_isolated_raised_top_is_not_a_new_ground_reference(self):
        near=self.surface();near=near[near[:,0]<7.5]
        patch=np.array([(x,y,.02*x-.15+.16) for x in np.arange(14.,16.,.1)
                        for y in np.arange(-1.,1.,.1)])
        marked,_=self.split(np.vstack((near,patch)))
        self.assertGreater(len(marked),0)
        self.assertTrue(np.all(marked[:,0]>=14))

    def test_sparse_far_returns_keep_original_obstacle_gate(self):
        near=self.surface();near=near[near[:,0]<7.5]
        far=np.array([[15.,0.,.2],[15.,.4,.2]])
        marked,_=self.split(np.vstack((near,far)))
        self.assertEqual(len(marked),2)

    def test_no_ground_fit_keeps_original_marking(self):
        points=np.array([[15.,0.,.2],[15.,.4,.2]])
        marked,_=self.split(points)
        self.assertEqual(len(marked),2)

    def test_ground_fix_does_not_change_clearing_observations(self):
        points=self.surface()
        marked,clearing=self.split(points)
        expected=cloud.voxel_downsample(points,.1,20000)
        np.testing.assert_array_equal(clearing,expected)

    def test_far_classification_is_permutation_invariant(self):
        points=self.surface()
        patch=(points[:,0]>=14)&(points[:,0]<16)&(abs(points[:,1])<.5)
        points[patch,2]+=.16
        original,_=self.split(points)
        shuffled,_=self.split(points[np.random.default_rng(23).permutation(len(points))])
        def canonical(a):return a[np.lexsort((a[:,2],a[:,1],a[:,0]))]
        np.testing.assert_array_equal(canonical(original),canonical(shuffled))

if __name__=='__main__':unittest.main()
