import unittest
from types import SimpleNamespace as NS
from wildos_nav2_adapter.scene_display import graph_primitives,ObjectLabelIndex


def point(x,y,z=52.):return NS(x=x,y=y,z=z)
def node(x,y,frontier=False):
    return NS(pose=NS(position=point(x,y)),trav_properties=[NS(is_frontier=frontier,
        frontier_points=[point(x+1,y)] if frontier else [])])


class SceneDisplayTests(unittest.TestCase):
    def labels(self):
        cache=ObjectLabelIndex();cache.set_frame('odom');cache.set_query('green house',1_000_000_000)
        return cache

    def observation(self,**changes):
        return dict(query='green house',frame='odom',localization_valid=True,
            position=[10.,20.,55.],observation_stamp_ns=2_000_000_000,**changes)

    def test_topology_preserves_xy_and_skips_invalid_edges(self):
        graph=NS(nodes=[node(734.,-12.),node(740.,-10.,True),node(float('nan'),0.)],
                 edges=[NS(from_idx=0,to_idx=1),NS(from_idx=1,to_idx=9),NS(from_idx=0,to_idx=2)])
        result=graph_primitives(graph)
        self.assertEqual(result['nodes'],[(734.,-12.),(740.,-10.)])
        self.assertEqual(result['frontiers'],[(740.,-10.)])
        self.assertEqual(result['frontier_points'],[(741.,-10.)])
        self.assertEqual(result['edges'],[(734.,-12.),(740.,-10.)])
        self.assertEqual(graph.nodes[0].pose.position.z,52.)

    def test_bearing_only_detection_never_gets_a_fake_map_position(self):
        cache=self.labels();data=self.observation();data.pop('position')
        data['observer_position']=[1.,2.,52.]
        self.assertFalse(cache.observe(data,3_000_000_000));self.assertEqual(cache.visible(3_000_000_000),())

    def test_rejects_wrong_frame_query_old_future_and_invalid_positions(self):
        for key,value in (('frame','camera'),('query','blue house'),('localization_valid',False),
                          ('observation_stamp_ns',500_000_000),('observation_stamp_ns',4_000_000_000),
                          ('position',[float('nan'),0.,0.]),('position',[0.,0.])):
            data=self.observation();data[key]=value
            with self.subTest(key=key,value=value):self.assertFalse(self.labels().observe(data,3_000_000_000))

    def test_duplicate_late_and_tentative_updates_cannot_downgrade_confirmation(self):
        cache=self.labels();data=self.observation();self.assertTrue(cache.observe(data,3_000_000_000))
        self.assertFalse(cache.observe(data,3_000_000_000))
        confirmed=dict(data,object_position=[10.,20.,55.],position_confirmed=True)
        self.assertTrue(cache.observe(confirmed,3_000_000_000,confirmed=True))
        self.assertTrue(cache.visible(20_000_000_000)[0].confirmed)
        later=dict(data,observation_stamp_ns=4_000_000_000)
        self.assertFalse(cache.observe(later,4_500_000_000))

    def test_query_switch_keeps_confirmed_labels_but_rejects_old_query(self):
        cache=self.labels();data=dict(self.observation(),object_position=[10.,20.,55.],position_confirmed=True)
        cache.observe(data,3_000_000_000,confirmed=True);cache.set_query('gas station',4_000_000_000)
        self.assertFalse(cache.observe(data,4_500_000_000,confirmed=True))
        self.assertEqual(cache.visible(20_000_000_000)[0].label,'green house')
        cache.set_frame('other_map');self.assertEqual(cache.visible(20_000_000_000),())

    def test_distinct_instances_stay_separate_and_tentative_labels_expire(self):
        cache=self.labels()
        for identity in ('a','b'):self.assertTrue(cache.observe(self.observation(object_id=identity),3_000_000_000))
        self.assertEqual(len(cache.visible(3_000_000_000)),2)
        self.assertEqual(cache.visible(8_000_000_000),())

    def test_display_cache_is_bounded(self):
        cache=self.labels();cache.capacity=2
        for identity in ('a','b','c'):cache.observe(self.observation(object_id=identity),3_000_000_000)
        self.assertEqual(len(cache.items),2)


if __name__=='__main__':unittest.main()
