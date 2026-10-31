import unittest
from visual_navigation.wildos.confirmed_viewpoint_cache import ConfirmedViewpointCache


class ViewpointMemoryTests(unittest.TestCase):
    def test_distinct_observation_positions_survive_later_observations(self):
        m = ConfirmedViewpointCache()
        m.observe('gas station', 'spot1/odom', (0, 0, 0), 0, 1)
        m.observe('gas station', 'spot1/odom', (8, 0, 0), 0, 2)
        for i in range(3, 20):
            m.observe('gas station', 'spot1/odom', (8.1, 0, 0), 0, i)
        views = m.candidates(' Gas  Station ', 20)
        self.assertEqual(len(views), 2)
        self.assertEqual(views[0]['position'], [0, 0, 0])

    def test_different_viewing_bins_are_retained(self):
        m = ConfirmedViewpointCache()
        m.observe('tree', 'spot1/odom', (0, 0, 0), 0, 1)
        m.observe('tree', 'spot1/odom', (0, 0, 0), 1.57, 2)
        self.assertEqual(len(m.candidates('tree', 3)), 2)

    def test_rejected_view_cools_down_without_removing_other_views(self):
        m = ConfirmedViewpointCache()
        first = m.observe('tree', 'spot1/odom', (0, 0, 0), 0, 1)
        second = m.observe('tree', 'spot1/odom', (8, 0, 0), 0, 2)
        self.assertTrue(m.reject('tree', first['id'], 3))
        self.assertEqual([v['id'] for v in m.candidates('tree', 4)], [second['id']])
        self.assertEqual(len(m.candidates('tree', 34)), 2)

    def test_view_store_is_bounded_and_return_values_are_copies(self):
        m = ConfirmedViewpointCache(max_views=3)
        for i in range(5):m.observe('tree', 'spot1/odom', (i*3, 0, 0), 0, i)
        views=m.candidates('tree', 10)
        self.assertEqual(len(views), 3)
        views[0]['position'][0]=999
        self.assertNotEqual(m.candidates('tree', 10)[0]['position'][0], 999)

    def test_new_process_has_no_saved_observations(self):
        m=ConfirmedViewpointCache();m.observe('tree','spot1/odom',(0,0,0),0,1)
        self.assertEqual(ConfirmedViewpointCache().candidates('tree',2),[])

    def test_invalid_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            ConfirmedViewpointCache().observe('tree','spot1/odom',(float('nan'),0,0),0,1)


if __name__ == '__main__':unittest.main()
