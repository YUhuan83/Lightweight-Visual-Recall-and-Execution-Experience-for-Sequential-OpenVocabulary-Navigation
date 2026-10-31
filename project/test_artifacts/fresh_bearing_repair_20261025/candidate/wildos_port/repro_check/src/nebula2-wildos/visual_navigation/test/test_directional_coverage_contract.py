"""Store -> actual heading candidate method contracts, without ROS or a model."""
import ast
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from visual_navigation.wildos.graph_observation_store import GraphObservationStore
from visual_navigation.wildos.search_evidence import negative_search_strength


def heading_candidates(store, count=16, query='target'):
    source=Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'
    owner=next(n for n in ast.parse(source.read_text(encoding='utf8')).body if isinstance(n,ast.ClassDef))
    method=next(n for n in owner.body if isinstance(n,ast.FunctionDef) and n.name=='_append_heading_candidates')
    ns=dict(np=np,HeadingCandidate=S)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
    nav=S(visual_memory_map=store,object_search_mode=True,text_queries=[query],memory_map_id='map',
          uuid_to_str=lambda x:x,global_frame='odom',heading_low_confidence_weight=0.,
          heading_turn_cost_weight=0.,heading_dynamic_risk_weight=0.,
          object_memory_heading_weight=0.,search_coverage_weight=.2)
    assessment=S(robot_profile_id='default',physical_traversability=1.,minimum_clearance=2.,
                 dynamic_risk=0.,confidence=1.,feasible=True)
    node=S(uuid='node',robot_traversability=[assessment],heading_candidates=[])
    ns['_append_heading_candidates'](nav,node,[.8]*count,0.)
    return node.heading_candidates


class DirectionalCoverageContracts(unittest.TestCase):
    def setUp(self):
        self.store=GraphObservationStore(':memory:','test')
        self.addCleanup(self.store.close)
        self.store.register_map('map')

    def test_independent_strength_reaches_actual_candidate_utility(self):
        self.store.record_search_coverage('map','target','node',[0],1.)
        self.store.record_search_coverage('map','target','node',[8],.1)
        candidates=heading_candidates(self.store)
        self.assertAlmostEqual(candidates[0].predicted_utility,.6)
        self.assertAlmostEqual(candidates[8].predicted_utility,.78)
        self.assertAlmostEqual(candidates[1].predicted_utility,.8)
        self.assertTrue(all(c.traversable for c in candidates))

    def test_overlap_duplicate_and_reordered_observations_stay_per_bin(self):
        self.store.record_search_coverage('map','target','node',[0,2],.7,timestamp=120.,heading_bin_count=4)
        self.store.record_search_coverage('map','target','node',[1,2,2],.9,timestamp=121.,heading_bin_count=4)
        self.store.record_search_coverage('map','target','node',[0],.2,timestamp=119.,heading_bin_count=4)
        np.testing.assert_allclose(self.store.search_heading_penalties('map','target','node',4),[.7,.9,.9,0.])
        self.assertAlmostEqual(self.store.search_penalty('map','target','node',4),.625)

    def test_query_map_node_and_resolution_are_isolated(self):
        self.store.record_search_coverage('map','target','node',[0],1.,heading_bin_count=4)
        for args in [('map','other','node',4),('map','target','other',4),
                     ('other','target','node',4),('map','target','node',16)]:
            np.testing.assert_array_equal(self.store.search_heading_penalties(*args),np.zeros(args[-1]))

    def test_detector_rejection_does_not_become_unrelated_negative_confidence(self):
        self.assertEqual(negative_search_strength([.8],.3),0.)
        self.assertAlmostEqual(negative_search_strength([0.],.3),.3/1.3)
        for values in ([],[float('nan')],[.1,float('inf')],[2.]):
            self.assertIsNone(negative_search_strength(values,.3))

    def test_actual_negative_writer_skips_invalid_and_uses_detector_threshold(self):
        source=Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'
        owner=next(n for n in ast.parse(source.read_text(encoding='utf8')).body if isinstance(n,ast.ClassDef))
        method=next(n for n in owner.body if isinstance(n,ast.FunctionDef) and n.name=='_record_negative_search')
        ns=dict(np=np,wall_time=lambda:10.,torch=S(is_tensor=lambda _:False),
                negative_search_strength=negative_search_strength,GraphObservationStore=GraphObservationStore)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
        nav=S(visual_memory_map=self.store,object_search_mode=True,text_queries=['target'],
              last_negative_search_at=0.,negative_search_interval=0.,uuid_to_str=lambda x:x,
              mask_threshold=.3,num_angular_bins=16,num_cameras=1,memory_map_id='map')
        graph=S(current_node_idx=0,nodes=[S(uuid='node')])
        odom=S(pose=S(pose=S(orientation=S(x=0.,y=0.,z=0.,w=1.))))
        ns['_record_negative_search'](nav,graph,[{}],[np.nan],odom)
        self.assertIsNone(self.store.get_search_coverage('map','target','node'))
        ns['_record_negative_search'](nav,graph,[{}],[.8],odom)
        self.assertEqual(self.store.search_heading_penalties('map','target','node')[0],0.)
        ns['_record_negative_search'](nav,graph,[{}],[0.],odom)
        self.assertAlmostEqual(self.store.search_heading_penalties('map','target','node')[0],.3/1.3)

    def test_invalid_evidence_cannot_partially_update_coverage(self):
        for strength in (float('nan'),float('inf'),-.1,1.1):
            with self.assertRaises(ValueError):
                self.store.record_search_coverage('map','target','node',[0],strength)
        for bins in ([16],[-1],[1.5]):
            with self.assertRaises(ValueError):
                self.store.record_search_coverage('map','target','node',bins,.5)
        self.assertIsNone(self.store.get_search_coverage('map','target','node'))

    def test_clear_map_cascades_exact_evidence(self):
        self.store.record_search_coverage('map','target','node',[0],1.)
        self.store.clear_map('map');self.store.register_map('map')
        np.testing.assert_array_equal(self.store.search_heading_penalties('map','target','node'),np.zeros(16))

    def test_large_resolution_avoids_sql_bitmask_overflow(self):
        self.store.record_search_coverage('map','target','node',[0,127],.3,heading_bin_count=128)
        expected=np.zeros(128,dtype=np.float32);expected[[0,127]]=.3
        np.testing.assert_array_equal(self.store.search_heading_penalties('map','target','node',128),expected)
        self.assertEqual(heading_candidates(self.store,count=0),[])

    def test_coverage_eviction_removes_exact_rows_without_affecting_other_map(self):
        self.store.max_search_records=2
        self.store.register_map('other')
        self.store.record_search_coverage('other','target','keep',[0],.8,timestamp=0.)
        for i in range(3):
            self.store.record_search_coverage('map','query'+str(i),'node',[i],.5,timestamp=float(i))
        self.assertIsNone(self.store.get_search_coverage('map','query0','node'))
        self.assertEqual(self.store._db.execute("SELECT COUNT(*) FROM search_heading_coverage WHERE map_id='map'").fetchone()[0],2)
        self.assertAlmostEqual(self.store.search_heading_penalties('other','target','keep')[0],.8)

    def test_reopen_preserves_exact_evidence_but_does_not_invent_legacy_bins(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db')
            store=GraphObservationStore(path,'test')
            store.register_map('map')
            store._db.execute("INSERT INTO search_coverage VALUES(?,?,?,?,?,?,?,?)",
                              ('map','target','legacy',0x101,1.,2,1.,2.))
            store._db.commit()
            store.record_search_coverage('map','target','node',[8],.1)
            store.close()
            reopened=GraphObservationStore(path,'test')
            try:
                self.assertEqual(reopened.get_search_coverage('map','target','legacy')['heading_mask'],0x101)
                np.testing.assert_array_equal(reopened.search_heading_penalties('map','target','legacy'),np.zeros(16))
                self.assertAlmostEqual(reopened.search_heading_penalties('map','target','node')[8],.1)
            finally:reopened.close()


if __name__=='__main__':unittest.main()
