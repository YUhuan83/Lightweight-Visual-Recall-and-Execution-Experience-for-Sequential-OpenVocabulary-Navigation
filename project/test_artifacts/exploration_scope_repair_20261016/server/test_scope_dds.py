"""Actual C++/DDS: close frontier survives refusal, moved start releases attempts."""
import unittest
import test_graphnav_owner_ros as fixtures

class ScopeContract(fixtures.OwnerContract):
 def close_graph(self):
  g=self.graph(.9,.8)
  g.nodes[2].pose.position.x=24.
  g.nodes[2].trav_properties[0].frontier_points[0].x=25.
  g.edges[1].traversability[0].traversability_cost=24.
  return g
 def test_refusal_keeps_a_close_distinct_frontier_selectable(self):
  a=self.update(self.close_graph());self.assertEqual(a['goal'],[20.,0.])
  self.rejectpub.publish(self.endpoint(a));self.pump(.15)
  b=self.update(self.close_graph());self.assertEqual(b['goal'],[24.,0.])
  self.assertNotEqual(a['exploration_id'],b['exploration_id'])
 def test_exhausted_attempts_are_released_at_a_new_measured_start(self):
  a=self.update(self.close_graph());self.rejectpub.publish(self.endpoint(a));self.pump(.15)
  b=self.update(self.close_graph());self.assertEqual(b['goal'],[24.,0.])
  self.rejectpub.publish(self.endpoint(b));self.pump(.15)
  c=self.update(self.close_graph(),x=2.)
  self.assertEqual(c['goal'],[20.,0.])
  self.assertNotEqual(c['exploration_id'],a['exploration_id'])

if __name__=='__main__':
 names=['test_refusal_keeps_a_close_distinct_frontier_selectable',
        'test_exhausted_attempts_are_released_at_a_new_measured_start']
 suite=unittest.TestSuite(ScopeContract(name) for name in names)
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 raise SystemExit(0 if result.wasSuccessful() else 1)
