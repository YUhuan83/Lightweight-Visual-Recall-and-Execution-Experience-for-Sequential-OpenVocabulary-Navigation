from pathlib import Path
R=Path(r'C:\Users\Yu\Desktop\拟真'); B=R/'wildos_port/repro_check/src/nebula2-wildos'
def edit(p,a,b):
 p=Path(p);s=p.read_text(encoding='utf-8');assert a in s,(str(p),a[:80]);p.write_text(s.replace(a,b,1),encoding='utf-8')
p=R/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
edit(p,'        self.query_started_ns = 0','        self.query_started_ns = 0\n        self.object_found = False\n        self.latest_route_complete = True')
edit(p,'        self.query_started_ns = self.get_clock().now().nanoseconds',
       '        self.object_found = False\n        self.query_started_ns = self.get_clock().now().nanoseconds')
edit(p,'        if self.pending_destination is not None:\n            goal,',
'''        if self.object_found:
            return
        if self.pending_destination is not None:
            goal,''')
edit(p,'        self.reset()\n        self.memory_selecting = False\n        self.pending_destination = None',
       '        self.object_found = True\n        self.reset()\n        self.memory_selecting = False\n        self.pending_destination = None')
edit(p,"        if self.last_status.get('status') == 'object_found':",
       '        if self.object_found:')
edit(p,"            self.route_complete = metadata['complete']",
       "            incoming_complete = metadata['complete']")
edit(p,'            self.route_complete = True\n            if self.expected is not None',
       '            incoming_complete = True\n            if self.expected is not None')
edit(p,'        self.latest_route = message\n        if self.following',
       '        self.latest_route = message\n        self.latest_route_complete = incoming_complete\n        if self.following')
edit(p,'        self.route = message\n        if self.route_endpoint',
       '        self.route = message\n        self.route_complete = incoming_complete\n        if self.route_endpoint')
edit(p,'                self.route = self.latest_route\n                self.route_endpoint',
       '                self.route = self.latest_route\n                self.route_complete = self.latest_route_complete\n                self.route_endpoint')
# Keep local failures out of the permanent object unreachability cache.
p=R/'nav2_adapter/wildos_nav2_adapter/memory_selection.py'
edit(p,'        self.memory_matches = []',
       '        self.memory_matches = []\n        self.memory_deferred = []')
edit(p,"            self.memory_rejected.add(self.memory_active['id'])",
'''            self.memory_deferred.append(dict(view=deepcopy(self.memory_active),
                route=deepcopy(self.route), route_complete=getattr(self, 'route_complete', True)))
            self.memory_rejected.add(self.memory_active['id'])''')
edit(p,"""        if not self.memory_matches:
            self.release_memory_goal('no_reachable_memory_candidate')
            return""","""        if not self.memory_matches:
            if self.memory_deferred:
                blocked = self.memory_deferred[0]
                self.reset()
                self.memory_selecting = False
                self.memory_active = blocked['view']
                self.memory_goal = self.expected = tuple(self.memory_active['position'][:2])
                self.memory_heading = self.memory_active['heading']
                self.route = self.latest_route = blocked['route']
                self.route_complete = self.latest_route_complete = blocked['route_complete']
                self.route_endpoint = position(self.route.poses[-1].pose) if self.route else None
                self.relay_destination(self.make_memory_pose(self.memory_active))
                self.block_current_route()
                self.status('memory_locally_blocked', reason='candidates_temporarily_unavailable',
                            goal=list(self.memory_goal))
                return
            self.release_memory_goal('no_eligible_memory_view')
            return""")
edit(p,"                self.route_complete = best.get('route_complete', True)",
       "                self.route_complete = self.latest_route_complete = best.get('route_complete', True)")
# Height layer validity is assessed per sample, not by whether the layer exists.
p=B/'graphnav_mapper/src/mapper.cpp'
edit(p,'  bool terrain_observed = !map.exists("elevation_valid");',
       '  bool terrain_observed = true;')
edit(p,'''      const double z = terrain_observed ? map.atPosition("elevation", p) :
          std::numeric_limits<double>::quiet_NaN();''',
'''      const bool valid_height = !map.exists("elevation_valid") ||
          map.atPosition("elevation_valid", p) > 0.5f;
      terrain_observed = terrain_observed && valid_height;
      const double z = valid_height ? map.atPosition("elevation", p) :
          std::numeric_limits<double>::quiet_NaN();''')
# Preserve visited evidence for planning. Local Nav2 validation still owns execution.
p=B/'graphnav_mapper/src/navigation_graph.cpp'
edit(p,'      node_msg.trav_properties[trav_class_idx].explored_radius = node.properties.explored_radius;',
'''      node_msg.trav_properties[trav_class_idx].explored_radius = node.properties.explored_radius;
      graphnav_msgs::msg::KeyValue visited;
      visited.key = "visited";
      visited.value = {trav_prop.visited ? 1.0f : 0.0f};
      node_msg.trav_properties[trav_class_idx].properties.push_back(visited);''')
p=B/'graphnav_planner/src/planner.cpp'
edit(p,'''    if (assessment.robot_profile_id == profile_id)
      return assessment.feasible;''',
'''    if (assessment.robot_profile_id == profile_id)
      return assessment.confidence < 0.7f || assessment.observed_at.sec == 0 || assessment.feasible;''')
edit(p,'''    if (!profile_node_feasible(node, robot_profile_id_))
      continue;''',
'''    bool visited = false;
    if (trav_class_idx_ < node.trav_properties.size())
      for (const auto& property : node.trav_properties[trav_class_idx_].properties)
        if (property.key == "visited" && !property.value.empty() && property.value.front() > 0.5f)
          visited = true;
    const auto* physical = profile_node_assessment(node, robot_profile_id_);
    const double physical_age = physical ? (current_time - rclcpp::Time(physical->observed_at)).seconds() : 0.0;
    const bool fresh_blocked = !profile_node_feasible(node, robot_profile_id_) &&
        physical_age >= 0.0 && physical_age <= stale_observation_seconds_;
    if (fresh_blocked && !visited)
      continue;''')
edit(p,'if (profile_node_feasible(node, robot_profile_id_) && trav_class_idx_ < node.trav_properties.size()',
       'if (!fresh_blocked && trav_class_idx_ < node.trav_properties.size()')
edit(p,'      graph_.add_edge(id, virtual_goal, goal_cost);',
       '      if (std::isfinite(goal_cost) && goal_cost >= 0.0) graph_.add_edge(id, virtual_goal, goal_cost);')
# Legacy follower no longer emits learning labels without execution evidence.
p=B/'graphnav_planner/src/path_follower_node.cpp'
edit(p,'    this->declare_parameter("traversal_feedback_topic", "/spot1/traversal_feedback");',
'''    this->declare_parameter("traversal_feedback_topic", "/spot1/traversal_feedback");
    // This waypoint observer cannot establish that Nav2 executed the graph route.
    this->declare_parameter("publish_traversal_feedback", false);
    publish_traversal_feedback_ = this->get_parameter("publish_traversal_feedback").as_bool();''')
edit(p,'    update_traversal_feedback(odom_pose_in_path_frame, robot_pos);',
       '    if (publish_traversal_feedback_) update_traversal_feedback(odom_pose_in_path_frame, robot_pos);')
edit(p,'    double wp_fraction = -a / (b - a);',
       '    double wp_fraction = std::abs(b - a) > 1e-9 ? -a / (b - a) : 1.0;')
edit(p,'  bool has_attempt_goal_ = false;',
       '  bool publish_traversal_feedback_ = false;\n  bool has_attempt_goal_ = false;')
# Test legacy endpoint checks separately from the new mandatory status contract.
p=R/'nav2_adapter/test/test_graph_path_bridge_routing.py'
edit(p,'        self.node = GraphPathBridge()',
       '        self.node = GraphPathBridge()\n        self.node.require_route_status = False  # Existing legacy route fixtures.')
edit(p,'        self.node.path_pub.publish.assert_not_called()\n\n    def test_blocked_memory_view',
'''        self.node.path_pub.publish.assert_called_once()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses[-1].pose.position.x, 4.)

    def test_blocked_memory_view''')
edit(p,'''    def test_unreachable_memory_releases_old_graph_goal(self):''',
       '''    def test_local_memory_failure_retains_old_graph_goal(self):''')
edit(p,'''        self.assertIsNone(self.node.memory_goal)
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.graph_reset_client.call_async.call_count, 2)
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unreachable')''',
'''        self.assertEqual(self.node.memory_goal, (12., 0.))
        self.assertFalse(self.node.memory_selecting)
        self.assertEqual(self.node.graph_reset_client.call_async.call_count, 1)
        self.assertEqual(self.node.last_status['status'], 'memory_locally_blocked')
        feedback = json.loads(self.node.memory_feedback_pub.publish.call_args.args[0].data)
        self.assertEqual(feedback['outcome'], 'unavailable')''')
print('completed retention, route state, evidence handling')

