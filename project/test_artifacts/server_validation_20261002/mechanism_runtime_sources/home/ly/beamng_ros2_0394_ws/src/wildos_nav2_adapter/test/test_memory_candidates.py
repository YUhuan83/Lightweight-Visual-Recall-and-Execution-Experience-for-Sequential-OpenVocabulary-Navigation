import json
import math
import unittest
from types import SimpleNamespace as NS
from wildos_nav2_adapter.memory_candidates import decode_candidates, heading_options, path_cost


class MemoryCandidateTests(unittest.TestCase):
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
