import unittest
import numpy as np
from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore


class ExecutionPriorContracts(unittest.TestCase):
    def setUp(self):
        self.store=ExecutionExperienceStore(':memory:','test')
        self.addCleanup(self.store.close)

    def test_success_only_changes_executed_direction_and_ranking(self):
        self.store.record_feedback([1.,0.],None,np.pi,1.,1,heading_count=4,scene_heading=0.,scene_frame='odom')
        prior, similarity=self.store.retrieve_execution_prior([1.,0.],4,current_heading=0.,heading_frame='odom')
        np.testing.assert_allclose(prior,[1.,1.,1.8,1.])
        self.assertAlmostEqual(similarity,1.)
        from visual_navigation.wildos.execution_experience_store import fuse_execution_prior
        raw=np.array([[.5,.5,.5,.5],[0.,1e-4,0.,0.]],dtype=np.float32)
        fused=fuse_execution_prior(raw,prior,.5)
        self.assertGreater(fused[0,2],fused[0,0])
        np.testing.assert_array_equal(fused[1],raw[1])
        np.testing.assert_array_equal(raw[0],[.5]*4)

    def test_failure_and_neutral_reward_have_opposite_bounded_effects(self):
        self.store.record_feedback([1.,0.],None,0.,0.,2,heading_count=4,scene_heading=0.,scene_frame='odom')
        self.store.record_feedback([1.,0.],None,np.pi,.5,1,heading_count=4,scene_heading=0.,scene_frame='odom')
        prior,_=self.store.retrieve_execution_prior([1.,0.],4,current_heading=0.,heading_frame='odom')
        np.testing.assert_allclose(prior,[.2,1.,1.,1.],atol=1e-6)

    def test_legacy_rows_keep_their_original_penalty_semantics(self):
        self.store.record_feedback([1.,0.],[.4,.6],0.,.4,1)
        self.store._db.execute('UPDATE experiences SET execution_multiplier=NULL,supervised_mask=NULL')
        self.store._db.commit()
        prior,_=self.store.retrieve_execution_prior([1.,0.],2)
        np.testing.assert_allclose(prior,[.4,.6])

    def test_no_match_wrong_resolution_and_other_map_cannot_create_prior(self):
        self.store.record_feedback([1.,0.],None,0.,1.,1,heading_count=4)
        self.assertEqual(self.store.retrieve_execution_prior([0.,1.],4),(None,0.))
        self.assertEqual(self.store.retrieve_execution_prior([1.,0.],8),(None,0.))
        self.store.map_id='other'
        self.assertEqual(self.store.retrieve_execution_prior([1.,0.],4),(None,0.))

    def test_fusion_cannot_create_support_or_escape_score_range(self):
        from visual_navigation.wildos.execution_experience_store import fuse_execution_prior
        source=np.array([[0.,1e-4,.9,.5]],dtype=np.float32)
        np.testing.assert_allclose(fuse_execution_prior(source,[2.,2.,2.,0.],1.),[[0.,1e-4,1.,0.]])
        np.testing.assert_array_equal(fuse_execution_prior(source,[1.,1.,1.,1.],.5),source)
        np.testing.assert_array_equal(fuse_execution_prior(source,[np.nan]*4,.5),source)
        np.testing.assert_array_equal(fuse_execution_prior(source,[2.,2.],.5),source)

    def test_view_rotation_rotates_supported_bearing_and_requires_frame(self):
        self.store.record_feedback([1.,0.],None,0.,1.,1,heading_count=4,scene_heading=0.,scene_frame='odom')
        prior,_=self.store.retrieve_execution_prior([1.,0.],4,current_heading=np.pi/2,heading_frame='odom')
        np.testing.assert_allclose(prior,[1.,1.8,1.,1.])
        wrapped,_=self.store.retrieve_execution_prior([1.,0.],4,current_heading=5*np.pi/2,heading_frame='odom')
        np.testing.assert_allclose(prior,wrapped)
        self.assertEqual(self.store.retrieve_execution_prior([1.,0.],4),(None,0.))
        self.assertEqual(self.store.retrieve_execution_prior([1.,0.],4,current_heading=0.,heading_frame='map'),(None,0.))


if __name__=='__main__':unittest.main()
