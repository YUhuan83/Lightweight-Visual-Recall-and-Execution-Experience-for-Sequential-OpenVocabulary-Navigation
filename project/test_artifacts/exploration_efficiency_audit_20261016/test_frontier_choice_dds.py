"""Actual C++ GraphNav selection protocol over DDS; synthetic graph/odom only."""
import copy
import json
import unittest
from graphnav_msgs.msg import ExplorationChoice
from std_msgs.msg import String
import test_graphnav_owner_ros as fixtures


class ChoiceContract(fixtures.OwnerContract):
 def setUp(self):
  super().setUp()
  self.choicepub=self.n.create_publisher(ExplorationChoice,'/spot1/graphnav_planner/vehicle_choice',10)
  self.selection_acks=[]
  self.n.create_subscription(String,'/spot1/graphnav_planner/selection_ack',lambda m:self.selection_acks.append(json.loads(m.data)),10)
  self.pump(.3)
 def costgraph(self):
  graph=self.graph()
  graph.nodes[2].pose.position.x=22.
  graph.nodes[2].trav_properties[0].frontier_points[0].x=23.
  graph.edges[1].traversability[0].traversability_cost=22.
  return graph
 def choice(self,status,index):
  path=self.paths[status['stamp_ns']]
  message=ExplorationChoice();message.header=copy.deepcopy(path.header)
  message.exploration_id=status['exploration_id']
  message.route_endpoint=copy.deepcopy(path.poses[-1].pose.position)
  message.evaluation_start.orientation.w=1.
  message.chosen_uuid=status['vehicle_options'][index]['uuid'];message.vehicle_cost=20.
  return message
 def test_vehicle_choice_commits_new_identity_and_late_old_choice_cannot_replace_it(self):
  a=self.update(self.costgraph());self.assertEqual(len(a['vehicle_options']),2)
  message=self.choice(a,1);self.choicepub.publish(message);self.pump(.2)
  b=self.update(self.costgraph());self.assertEqual(b['goal'],[22.,0.])
  self.assertNotEqual(a['exploration_id'],b['exploration_id']);self.assertTrue(b['vehicle_cost_selected'])
  self.assertTrue(any(r['reason']=='vehicle_cost_selection' for r in self.retirements))
  self.choicepub.publish(message);self.pump(.2)
  c=self.update(self.costgraph());self.assertEqual(c['exploration_id'],b['exploration_id'])
 def test_retaining_original_choice_is_idempotent_and_does_not_mark_observed(self):
  a=self.update(self.costgraph());message=self.choice(a,0)
  for _ in range(2):self.choicepub.publish(message);self.pump(.08)
  b=self.update(self.costgraph());self.assertEqual(a['exploration_id'],b['exploration_id'])
  self.assertTrue(b['vehicle_cost_selected']);self.assertFalse(b['endpoint_observed'])
  self.assertFalse(self.retirements)
 def test_bad_origin_cost_option_and_source_are_rejected_without_changing_owner(self):
  for case in ('moved','quaternion','cost','uuid','endpoint','stamp'):
   a=self.update(self.costgraph());message=self.choice(a,1)
   if case=='moved':message.evaluation_start.position.x=1.
   if case=='quaternion':message.evaluation_start.orientation.w=0.
   if case=='cost':message.vehicle_cost=float('nan')
   if case=='uuid':message.chosen_uuid='not_offered'
   if case=='endpoint':message.route_endpoint.y=5.
   if case=='stamp':message.header.stamp.nanosec=1000000000
   self.choicepub.publish(message);self.pump(.1)
   b=self.update(self.costgraph());self.assertEqual(a['exploration_id'],b['exploration_id'],case)
   self.assertFalse(b['vehicle_cost_selected'],case);self.assertIsNone(self.proc.poll())
 def test_stale_odometry_does_not_authorize_selection(self):
  a=self.update(self.costgraph());message=self.choice(a,1)
  old=self.n.get_clock().now().to_msg();old.sec-=2
  self.odompub.publish(self.odom(0.,old));self.pump(.05)
  self.choicepub.publish(message);self.pump(.15)
  self.assertTrue(any(not a['accepted'] and a['reason']=='stale_origin' for a in self.selection_acks))
  b=self.update(self.costgraph());self.assertEqual(a['exploration_id'],b['exploration_id'])
  self.assertFalse(b['vehicle_cost_selected'])

if __name__=='__main__':unittest.main(verbosity=2)
