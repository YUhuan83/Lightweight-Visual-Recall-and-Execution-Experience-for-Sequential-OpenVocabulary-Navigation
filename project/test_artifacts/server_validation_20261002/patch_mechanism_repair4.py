from pathlib import Path
R=Path(r'C:\Users\Yu\Desktop\拟真'); B=R/'wildos_port/repro_check/src/nebula2-wildos'
def edit(p,a,b):
 p=Path(p);s=p.read_text(encoding='utf-8');assert a in s,(str(p),a[:80]);p.write_text(s.replace(a,b,1),encoding='utf-8')
p=B/'graphnav_planner/src/planner.cpp'
s=p.read_text()
a=s.index('    // connect nodes to goal if within goal_radius')
b=s.index('\n  }',a)
attach=s[a:b]
s=s[:a]+s[b:];a=s.index('    if (!fresh_blocked && trav_class_idx_')
s=s[:a]+attach+'\n\n'+s[a:]
s=s.replace('    trav_class_idx_ = 0;\n    return;','    trav_class_idx_ = 0;\n    unexplored_space_map_.reset();\n    frontier_scores_.clear();\n    return;',1)
p.write_text(s)
# Clear visual marker text when the remembered frontier is cleared.
edit(p,'''  // add text for time_diff
  if (latest_frontier_time_ && latest_frontier_)''',
'''  visualization_msgs::msg::Marker delete_frontier_time;
  delete_frontier_time.header.frame_id = frame_id;
  delete_frontier_time.header.stamp = stamp;
  delete_frontier_time.ns = "local_frontier_time";
  delete_frontier_time.id = 0;
  delete_frontier_time.action = visualization_msgs::msg::Marker::DELETE;
  markers.markers.push_back(delete_frontier_time);
  // add text for time_diff
  if (latest_frontier_time_ && latest_frontier_)''')
# Exact observation pose accompanies query-scoped localization/confirmation.
p=B/'visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
edit(p,'        pose = obs.odom.pose.pose.position',
       '        pose = obs.odom.pose.pose.position\n        orientation = obs.odom.pose.pose.orientation')
edit(p,"            'observer_position': [pose.x, pose.y, pose.z],",
       "            'observer_position': [pose.x, pose.y, pose.z],\n            'observer_orientation': [orientation.x, orientation.y, orientation.z, orientation.w],")
# Remove legacy unlabelled raw-waypoint subscriptions; waypoint remains a visualization topic.
p=R/'official_adapter/beamng_wildos_adapter/goal_standoff.py'
edit(p,'''        self.create_subscription(
            PoseStamped, str(self.get_parameter("input_topic").value), lambda m: None, 10)
''','')
edit(p,'        self.odom = None',
'''        if (int(self.get_parameter("confirmation_frames").value) < 3 or
                float(self.get_parameter("confirmation_radius").value) <= 0):
            raise ValueError("confirmation requires at least 3 observations and a positive radius")
        self.odom = None''')
edit(p,"            values = data['position']",
       "            values = data['position']\n            observer = data['observer_position']\n            orientation = data['observer_orientation']")
edit(p,'                    or not all(map(math.isfinite, values))):',
'''                    or not all(map(math.isfinite, values)) or len(observer) != 3
                    or not all(map(math.isfinite, observer)) or len(orientation) != 4
                    or not all(map(math.isfinite, orientation))
                    or abs(sum(v*v for v in orientation) - 1) > .1):''')
edit(p,"            if any(item['observation_stamp_ns'] == stamp for item in self.observations):",
       "            if self.odom is None or (self.observations and stamp <= self.observations[-1]['observation_stamp_ns']):\n                return\n            if any(item['observation_stamp_ns'] == stamp for item in self.observations):")
edit(p,"            'observer_position': data['observer_position'], 'radius': spread,",
'''            'observer_position': data['observer_position'],
            'observer_orientation': data['observer_orientation'], 'radius': spread,''')
# Region-bearing views are validated in the pure memory class.
p=B/'visual_navigation/visual_navigation/wildos/viewpoint_memory.py'
edit(p,'    def observe(self, query, frame, position, heading, now):',
'''    def observe(self, query, frame, position, heading, now, *,
                object_position=None, object_radius=0.0, object_id=None, anchor_node_uuid=None):''')
edit(p,"        key = self.key(query)\n        if not key:",
'''        if object_position is not None and (
                len(object_position) != 3 or not all(math.isfinite(v) for v in object_position)
                or not math.isfinite(object_radius) or object_radius < 0):
            raise ValueError('invalid object region')
        key = self.key(query)
        if not key:''')
edit(p,'        # Repeated observations at one location',
'''        if object_position is not None:
            match.update(object_position=list(object_position), object_radius=float(object_radius),
                         object_id=object_id, anchor_node_uuid=anchor_node_uuid)
        # Repeated observations at one location''')
# Nav consumes confirmed observation pose, connects memory map APIs and preserves object identity.
p=B/'visual_navigation/visual_navigation/wildos/nav.py'
s=p.read_text();a=s.index('    def object_confirmation_callback');b=s.index('    def object_search_waypoint_callback',a)
s=s[:a]+'''    def object_confirmation_callback(self, message):
        try:
            data = json.loads(message.data)
            query = self.text_queries[0] if self.text_queries else None
            now_ns = self.get_clock().now().nanoseconds
            stamp = int(data['observation_stamp_ns'])
            if (data['query'] != query or data['frame'] != self.global_frame
                    or not data.get('search_confirmed') or now_ns - stamp > 5000000000
                    or stamp > now_ns):
                return
            p = data['observer_position']
            q = data['observer_orientation']
            if len(p) != 3 or len(q) != 4 or not all(map(np.isfinite, [*p, *q])):
                return
            heading = float(np.arctan2(2 * (q[3]*q[2] + q[0]*q[1]),
                                      1 - 2 * (q[1]*q[1] + q[2]*q[2])))
            anchor = None
            graph = self.latest_navgraph
            if graph is not None and graph.nodes and graph.header.frame_id == data['frame']:
                node = min(graph.nodes, key=lambda n: np.hypot(n.pose.position.x-p[0],
                                                              n.pose.position.y-p[1]))
                if np.hypot(node.pose.position.x-p[0], node.pose.position.y-p[1]) <= 6.0:
                    anchor = self.uuid_to_str(node.uuid)
            index = round((heading % (2*np.pi))*self.num_angular_bins/(2*np.pi)) % self.num_angular_bins
            object_id = None
            if self.visual_memory_map is not None:
                standoff = data.get('standoff')
                object_id = self.visual_memory_map.observe_object(
                    self.memory_map_id, query, data['object_position'],
                    confidence=data['inliers']/data['observations'],
                    covariance=[data['radius']**2, data['radius']**2, data['radius']**2],
                    anchor_node_uuid=anchor, anchor_heading_bin=index,
                    observer_pose=[*p, heading],
                    approach_pose=([*standoff, p[2], heading] if standoff else None),
                    timestamp=stamp*1e-9)
            self.object_goal_memory.observe(query, data['frame'], p, heading, wall_time(),
                object_position=data['object_position'], object_radius=data['radius'],
                object_id=object_id, anchor_node_uuid=anchor)
            self.get_logger().info(f"Remembered confirmed {query!r} region and observation view; anchor={anchor}")
        except (ValueError, KeyError, TypeError, sqlite3.Error) as exc:
            self.get_logger().warning(f"Invalid object confirmation: {exc}")

'''+s[b:];p.write_text(s)
# Confirmation receiver bounds freshness and input shape independently.
p=R/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
edit(p,"            if data['query'] != self.query or data['observation_stamp_ns'] < self.query_started_ns:",
'''            now_ns = self.get_clock().now().nanoseconds
            stamp = int(data['observation_stamp_ns'])
            if (data['query'] != self.query or stamp < self.query_started_ns
                    or stamp > now_ns or now_ns - stamp > 5000000000):''')
edit(p,"            if self.memory_active and self.memory_active.get('object_position'):",
'''            if (len(data['object_position']) != 3 or len(data['observer_position']) != 3
                    or not all(map(math.isfinite, [*data['object_position'], *data['observer_position'], data['radius']]))
                    or data['radius'] < 0 or data['inliers'] < 3):
                return
            if self.memory_active and self.memory_active.get('object_position'):''')
# Harness accepts fresh matching object region, with separate memory execution evidence.
p=R/'test_artifacts/server_validation_20261002/full_town_memory_test.py'
edit(p,"        self.waypoint = None\n        self.query_pub",
       "        self.waypoint = None\n        self.confirmation = None\n        self.query_pub")
edit(p,"        self.create_subscription(PoseStamped, '/spot1/imgnav_waypoint', self.goal, 10)",
       "        self.create_subscription(PoseStamped, '/spot1/imgnav_waypoint', self.goal, 10)\n        self.create_subscription(String, '/spot1/object_confirmation', self.confirmed, 10)")
edit(p,'    def path(self, msg):',
'''    def confirmed(self, msg):
        data = json.loads(msg.data)
        if data.get('query') != self.query:
            return
        self.confirmation = dict(data, received_at=time.monotonic())
        self.event('object_confirmation', payload=data)

    def path(self, msg):''')
edit(p,'        self.query = query\n        self.waypoint = None',
       '        self.query = query\n        self.waypoint = None\n        self.confirmation = None')
edit(p,"              '/spot1/graphnav_planner/path', '/wildos/full_planned_path',",
       "              '/spot1/graphnav_planner/path', '/wildos/full_planned_path',\n              '/spot1/graphnav_planner/route_status', '/spot1/object_localization',\n              '/spot1/object_confirmation', '/wildos/viz/frontier_traversability',")
edit(p,"        if (target and target['query'] == 'house' and time.monotonic()-mask.get('at', 0) < 5",
       "        if (n.confirmation and n.confirmation['query'] == 'house' and\n                time.monotonic()-n.confirmation['received_at'] < 5 and\n                target and target['query'] == 'house' and time.monotonic()-mask.get('at', 0) < 5")
edit(p,"    n.phase = 'station_return'\n    n.change_query",
       "    if not house_found:\n        raise RuntimeError('House was not confirmed; full multi-goal sequence cannot proceed')\n    n.phase = 'station_return'\n    n.change_query")
a=p.read_text().index('        # Arrival is measured against the remembered station viewpoint')
b=p.read_text().index('        if time.monotonic() - last_progress > 65:',a)
s=p.read_text()
s=s[:a]+'''        confirmed = n.confirmation
        known = report['station_views_before_departure']
        matches_region = bool(confirmed and any(v.get('object_position') and math.dist(
            confirmed['object_position'][:2], v['object_position'][:2]) <=
            3.0 + confirmed['radius'] + v.get('object_radius', 0.0) for v in known))
        if (matches_region and confirmed['query'] == 'gas station'
                and time.monotonic()-confirmed['received_at'] < 5
                and mask.get('count', 0) >= 2 and mask.get('pixels', 0) > 0
                and time.monotonic()-mask.get('at', 0) < 5):
            returned = True
            report['return_confirmation'] = confirmed
            break
'''+s[b:];p.write_text(s)
edit(p,'                  memory_selection_latency=first_memory_selection, memory_path_latency=first_memory_path,',
'''                  memory_selection_latency=first_memory_selection, memory_path_latency=first_memory_path,
                  memory_navigation_exercised=first_memory_path is not None,''')
print('completed object identity integration and future harness')

