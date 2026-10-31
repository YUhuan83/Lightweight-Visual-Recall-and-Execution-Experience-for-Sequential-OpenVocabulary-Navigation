"""Actual GraphNav ownership invalidation; synthetic DDS, no vehicle success claim."""
import unittest
import test_graphnav_owner_ros as fixtures

class ResolutionContract(fixtures.OwnerContract):
 def test_resolved_frontier_retires_without_observation_or_region_backoff(self):
  a=self.update(self.graph());self.assertEqual(a['goal'],[20.,0.])
  changed=self.graph();changed.nodes[1].trav_properties[0].is_frontier=False
  b=self.update(changed)
  self.assertEqual(b['goal'],[40.,0.]);self.assertNotEqual(a['exploration_id'],b['exploration_id'])
  self.assertFalse(b['endpoint_observed'])
  self.assertTrue(any(x['reason']=='frontier_resolved' and x['exploration_id']==a['exploration_id'] for x in self.retirements))
  # A new nearby frontier stays selectable; resolution is no obstacle evidence.
  changed.nodes[1].pose.position.x=22.;changed.nodes[1].trav_properties[0].is_frontier=True
  changed.nodes[1].properties[0].value=[1.]
  self.rejectpub.publish(self.endpoint(b));self.pump(.2)
  c=self.update(changed);self.assertEqual(c['goal'],[22.,0.])
 def test_late_nonfrontier_graph_does_not_cancel_current_owner(self):
  a=self.update(self.graph());old=self.graph()
  old.header.stamp.sec-=6;old.nodes[1].trav_properties[0].is_frontier=False
  self.graphpub.publish(old);self.pump(.3)
  self.assertEqual(self.status[-1]['exploration_id'],a['exploration_id'])
  b=self.update(self.graph(.1,1.,missing=True))
  self.assertEqual(b['exploration_id'],a['exploration_id'])
 def test_explicit_object_destination_is_not_frontier_retired(self):
  a=self.update(self.graph());message=self.endpoint(a)
  self.goalpub.publish(message);self.pump(.2)
  changed=self.graph();changed.nodes[1].trav_properties[0].is_frontier=False
  b=self.update(changed)
  self.assertNotIn('exploration_id',b)
  self.assertFalse(any(x['reason']=='frontier_resolved' for x in self.retirements))

if __name__=='__main__':
 names=[name for name in ResolutionContract.__dict__ if name.startswith('test_')]
 result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(ResolutionContract(name) for name in names))
 raise SystemExit(0 if result.wasSuccessful() else 1)
