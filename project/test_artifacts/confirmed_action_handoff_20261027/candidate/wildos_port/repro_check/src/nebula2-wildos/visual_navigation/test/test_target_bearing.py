import math
import ast
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np
from visual_navigation.wildos.target_bearing import mask_bearing, guide_frontier_scores


class TargetBearingTests(unittest.TestCase):
    def test_unknown_terrain_does_not_zero_all_frontier_headings(self):
        source = Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'
        owner = next(n for n in ast.parse(source.read_text(encoding='utf8')).body if isinstance(n, ast.ClassDef))
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name=='_append_heading_candidates')
        namespace = dict(np=np, HeadingCandidate=S)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),str(source),'exec'),namespace)
        nav = S(visual_memory_map=None, object_search_mode=False, text_queries=[], global_frame='spot1/odom',
                heading_turn_cost_weight=.15, heading_dynamic_risk_weight=.5,
                heading_low_confidence_weight=.5, object_memory_heading_weight=.2, search_coverage_weight=.2)
        assessment = S(robot_profile_id='default',physical_traversability=1.,minimum_clearance=2.,
                       dynamic_risk=0.,confidence=0.,feasible=True)
        node = S(robot_traversability=[assessment], heading_candidates=[])
        namespace['_append_heading_candidates'](nav,node,[.3,1e-4,0.,0.],0.)
        unknown = node.heading_candidates[0].predicted_utility
        self.assertGreater(unknown,0.)
        self.assertEqual(node.heading_candidates[1].predicted_utility,0.)
        assessment.confidence=1.;node.heading_candidates=[]
        namespace['_append_heading_candidates'](nav,node,[.3,1e-4,0.,0.],0.)
        self.assertGreater(node.heading_candidates[0].predicted_utility,unknown)
        assessment.feasible=False;node.heading_candidates=[]
        namespace['_append_heading_candidates'](nav,node,[.3,1e-4,0.,0.],0.)
        self.assertTrue(all(not h.traversable for h in node.heading_candidates))

    def test_elevated_mask_can_guide_ground_frontier_without_range(self):
        k = np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]])
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        mask = np.zeros((100, 100)); mask[10:20, 45:56] = 1
        bearing = mask_bearing(mask, k, rotation)
        self.assertAlmostEqual(bearing, 0.)
        scores = np.full((3, 8), .1, dtype=np.float32)
        actual = guide_frontier_scores(scores, [0., 0.], [[10., 0.], [0., 10.], [30., 0.]], bearing)
        self.assertGreater(actual[0, 0], .8)
        np.testing.assert_array_equal(actual[1:], scores[1:])
        np.testing.assert_array_equal(scores, np.full((3, 8), .1, dtype=np.float32))

    def test_no_mask_bad_calibration_and_vertical_ray_do_not_guide(self):
        self.assertIsNone(mask_bearing(np.zeros((3, 3)), np.eye(3), np.eye(3)))
        self.assertIsNone(mask_bearing(np.ones((3, 3)), np.zeros((3, 3)), np.eye(3)))
        mask = np.zeros((3, 3)); mask[0, 0] = 1
        self.assertIsNone(mask_bearing(mask, np.eye(3), np.eye(3)))

    def test_unsupported_scores_and_opposite_frontiers_are_preserved(self):
        scores = np.array([[0., .1, .1, .1], [.1, .1, .1, .1]])
        actual = guide_frontier_scores(scores, [0., 0.], [[10., 0.], [-10., 0.]], 0.)
        self.assertEqual(actual[0, 0], 0.)
        np.testing.assert_array_equal(actual[1], scores[1].astype(np.float32))
        rejected = np.full((1, 8), 1e-4, dtype=np.float32)
        np.testing.assert_array_equal(
            guide_frontier_scores(rejected, [0., 0.], [[10., 0.]], 0.), rejected)

    def test_heading_wrap_uses_same_world_direction(self):
        scores = np.full((1, 8), .1)
        actual = guide_frontier_scores(scores, [0., 0.], [[-10., -.01]], math.pi-.01)
        self.assertGreater(actual[0, 4], .8)

    def test_production_overlay_preserves_raw_scores_and_memory_statistics(self):
        source = Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_guide_visible_target_frontiers')
        module = ast.Module(body=[method], type_ignores=[])
        namespace = dict(np=np, mask_bearing=mask_bearing,
                         guide_frontier_scores=guide_frontier_scores)
        exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
        scores = [.1]*8
        prop = S(key='frontier_scores', value=list(scores))
        raw = S(key='frontier_score_raw', value=list(scores))
        node = S(properties=[prop, raw], pose=S(position=S(x=10., y=0.)),
                 heading_candidates=[S(semantic_traversability=.4)])
        model = S(cam_inverted=False, _append_heading_candidates=Mock(), get_logger=Mock())
        mask = np.zeros((1, 1, 100, 100)); mask[0, 0, 10:20, 45:56] = 1
        camera = dict(K=np.array([[100.,0.,50.],[0.,100.,50.],[0.,0.,1.]]),
                      R_wc=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]),
                      t_wc=np.zeros((3,1)))
        namespace['_guide_visible_target_frontiers'](model, S(nodes=[node]), mask, [camera], 0.)
        self.assertGreater(prop.value[0], .8)
        self.assertTrue(all(type(value) is float for value in prop.value))
        self.assertEqual(raw.value, scores)
        model._append_heading_candidates.assert_called_once()
        self.assertEqual(node.heading_candidates, [])


if __name__ == '__main__':
    unittest.main()
