import json
from pathlib import Path
import unittest
import numpy as np
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth,triangulate_bearings

FIXTURES=Path(__file__).parent/'fixtures'

class RangeConsistencyTests(unittest.TestCase):
    def test_recorded_lidar_representative_projects_inside_its_own_mask(self):
        with np.load(FIXTURES/'depth_fixture_1792915120137115425.npz') as f:
            result=foreground_depth(f['points_cam'],f['k'],f['mask'])
            self.assertIsNotNone(result)
            p=result['position'];pixel=f['k']@p;u,v=(pixel[:2]/pixel[2]).astype(int)
            self.assertTrue(f['mask'][v,u])

    def test_representative_is_an_actual_supported_return(self):
        with np.load(FIXTURES/'depth_fixture_1792915120137115425.npz') as f:
            result=foreground_depth(f['points_cam'],f['k'],f['mask'])
            self.assertLess(np.linalg.norm(f['points_cam']-result['position'],axis=1).min(),1e-10)

    def test_recorded_multiview_solutions_do_not_hide_angular_uncertainty(self):
        rows=json.loads((FIXTURES/'v28_localization_callback.json').read_text())
        for row in rows:
            if row['source']=='multiview':
                with self.subTest(stamp=row['stamp_ns']):
                    self.assertIsNone(triangulate_bearings(row['views']))

    def test_zero_intersection_residual_does_not_mean_precise_range(self):
        target=np.array([2.,20.,0.])
        views=[dict(origin=o,ray=target-np.array(o),angular_tolerance=.04)
               for o in ([0.,0.,0.],[2.,0.,0.])]
        self.assertIsNone(triangulate_bearings(views))

    def test_well_separated_views_keep_usable_geometry(self):
        target=np.array([5.,5.,2.])
        views=[dict(origin=o,ray=target-np.array(o),angular_tolerance=.04)
               for o in ([0.,0.,0.],[2.,0.,0.],[0.,2.,0.])]
        result=triangulate_bearings(views)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['position'],target,atol=1e-8)

    def test_exact_small_baseline_fixture_is_still_uncertain_with_angular_error(self):
        target=np.array([10.,2.,3.])
        views=[dict(origin=o,ray=target-np.array(o)) for o in
               ([0.,0.,0.],[2.,0.,0.],[2.,1.,0.])]
        self.assertIsNone(triangulate_bearings(views))

    def test_invalid_angular_error_metadata_cannot_create_covariance(self):
        target=np.array([5.,5.,2.])
        for tolerance in (0.,float('nan'),float('inf'),np.pi/2):
            with self.subTest(tolerance=tolerance):
                views=[dict(origin=o,ray=target-np.array(o),angular_tolerance=tolerance)
                       for o in ([0.,0.,0.],[2.,0.,0.],[0.,2.,0.])]
                self.assertIsNone(triangulate_bearings(views))

    def test_current_returns_can_retain_nearby_representative(self):
        k=np.array([[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]])
        mask=np.ones((100,100),bool)
        points=np.array([[x,y,10.] for x in np.linspace(-1.,1.,9) for y in np.linspace(-1.,1.,9)])
        reference=points[8].copy()
        result=foreground_depth(points,k,mask,reference_position=reference)
        np.testing.assert_array_equal(result['position'],reference)
        self.assertEqual(result['representative_kind'],'tracked_return')
        self.assertGreaterEqual(result['points'],20)

    def test_covariance_matches_actual_solver_angular_sensitivity(self):
        # Independent finite differences of the position solver: covariance
        # must describe that estimator, including unequal viewing distances.
        from copy import deepcopy
        target=np.array([5.,5.,1.])
        views=[dict(origin=np.array(o),ray=target-np.array(o),angular_tolerance=.04)
               for o in ([0.,0.,0.],[4.,0.,0.],[0.,4.,0.])]
        result=triangulate_bearings(views)
        self.assertIsNotNone(result)
        expected=np.zeros((3,3));epsilon=1e-6
        for i,view in enumerate(views):
            ray=view['ray']/np.linalg.norm(view['ray'])
            a=np.cross(ray,[0.,0.,1.]);a/=np.linalg.norm(a)
            for tangent in (a,np.cross(ray,a)):
                plus=deepcopy(views);minus=deepcopy(views)
                plus[i]['ray']=ray*np.cos(epsilon)+tangent*np.sin(epsilon)
                minus[i]['ray']=ray*np.cos(epsilon)-tangent*np.sin(epsilon)
                derivative=(triangulate_bearings(plus)['position']-
                            triangulate_bearings(minus)['position'])/(2*epsilon)
                expected+=np.outer(derivative,derivative)*np.tan(.04)**2
        np.testing.assert_allclose(result['covariance'],expected,rtol=1e-5,atol=1e-7)

    def test_old_reference_cannot_create_range_without_current_returns(self):
        k=np.array([[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]])
        self.assertIsNone(foreground_depth(np.empty((0,3)),k,np.ones((100,100)),reference_position=[0,0,10]))

    def test_reference_does_not_hide_multiple_surfaces(self):
        k=np.array([[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]])
        points=np.array([[x*z/100,y*z/100,z] for z in (8.,20.)
            for x in range(-10,0,2) for y in range(-10,0,2)])
        # Distinct pixel areas preserve both surfaces after nearest-return dedup.
        points[25:,:2]*=-1
        result=foreground_depth(points,k,np.ones((100,100)),reference_position=points[0])
        self.assertFalse(result['association_validated'])
        self.assertEqual(len(result['depth_candidates']),2)

if __name__=='__main__':unittest.main()
