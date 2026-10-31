import json
import math
import unittest
from types import SimpleNamespace as NS
from wildos_nav2_adapter.memory_candidates import decode_candidates, heading_options, path_cost


class MemoryCandidateTests(unittest.TestCase):
    def test_accepted_numeric_fields_have_one_runtime_type(self):
        view = dict(id='1', position=['2', '0', '0'], heading='0', frame='spot1/odom',
                    object_position=['20', '0', '2'], object_radius='0.5', horizontal_fov='1.2')
        _, views = decode_candidates(json.dumps(dict(query='gas station', views=[view])))
        value = views[0]
        self.assertEqual(value['object_position'], [20., 0., 2.])
        self.assertIsInstance(value['object_radius'], float)
        self.assertIsInstance(value['horizontal_fov'], float)
        self.assertEqual(math.dist(value['position'], value['object_position']), math.sqrt(328.))

    def test_object_region_is_preserved_and_invalid_radius_is_rejected(self):
        view = dict(id='1', position=[2., 0., 0.], heading=0., frame='spot1/odom',
                    object_position=[20., 0., 2.], object_radius=.5, object_id='station-1')
        query, views = decode_candidates(json.dumps(dict(query=' Gas Station ', views=[view])))
        self.assertEqual(query, 'gas station')
        self.assertEqual(views[0]['object_id'], 'station-1')
        for radius in (-1., float('nan')):
            view['object_radius'] = radius
            with self.assertRaises(ValueError):
                decode_candidates(json.dumps(dict(query=query, views=[view])))

    def test_payload_rejects_invalid_frame_and_nonfinite_pose(self):
        v={'id':'1','position':[2,0,0],'heading':0,'frame':'map'}
        with self.assertRaises(ValueError):decode_candidates(json.dumps({'query':'tree','views':[v]}))
        v['frame']='spot1/odom';v['position'][0]=float('nan')
        with self.assertRaises(ValueError):decode_candidates(json.dumps({'query':'tree','views':[v]}))

    def test_observation_heading_does_not_remove_feasible_arrival_options(self):
        self.assertEqual(heading_options((0,0),(8,0),math.pi,0),[math.pi,0])

    def test_path_cost_penalizes_detours_and_obstacle_proximity(self):
        g=NS(info=NS(width=40,height=40,resolution=1.,origin=NS(position=NS(x=-10,y=-10),orientation=NS(x=0,y=0,z=0,w=1))),data=[0]*1600)
        short=path_cost([(0,0),(6,0)],[0,0],g,0)
        detour=path_cost([(0,0),(0,6),(6,6),(6,0)],[0,0,0,0],g,0)
        self.assertLess(short,detour)
        for x in range(0,8):g.data[10*40+x+10]=80
        self.assertGreater(path_cost([(0,0),(6,0)],[0,0],g,0),short)


if __name__ == '__main__':unittest.main()
