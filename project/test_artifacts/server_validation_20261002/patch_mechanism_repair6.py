from pathlib import Path
R=Path(r'C:\Users\Yu\Desktop\拟真');B=R/'wildos_port/repro_check/src/nebula2-wildos'
def edit(p,a,b):
 p=Path(p);s=p.read_text(encoding='utf-8');assert a in s,(p,a[:80]);p.write_text(s.replace(a,b,1),encoding='utf-8')
# Standoff geometry receives an already-confirmed region; no synthetic repeated measurements.
p=R/'official_adapter/beamng_wildos_adapter/goal_standoff.py'
s=p.read_text().replace('        self.targets = []\n','').replace('        self.targets.clear()\n','')
s=s.replace('        self.targets = [tuple(centre)] * frames\n','').replace('        self.on_target(target)\n','        self.publish_standoff(target)\n')
a=s.index('    def on_target(self, target):');b=s.index('        robot = self.odom.pose.pose.position',a)
s=s[:a]+'''    def publish_standoff(self, target):
        if self.odom is None:
            return
        stable = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
'''+s[b:]
s=s.replace('        point = target.pose.position\n','')
p.write_text(s,encoding='utf-8')
# Remove the old unlabelled viewpoint capture path.
p=B/'visual_navigation/visual_navigation/wildos/nav.py'
s=p.read_text();a=s.index('    def object_search_waypoint_callback');b=s.index('    def _reuse_remembered_object_goal',a)
s=s[:a]+s[b:];s=s.replace('from visual_navigation.wildos.remembered_goal import observed_viewpoint_pose\n','')
p.write_text(s,encoding='utf-8')
# Require current-query observation stamps on unlabelled standoff transport too.
p=R/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
edit(p,'''    def on_waypoint(self, message):
        point = xy(message.pose)''',
'''    def on_waypoint(self, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if (self.object_found or stamp < self.query_started_ns or stamp > now_ns
                or now_ns - stamp > 5000000000):
            return
        point = xy(message.pose)''')
p=R/'nav2_adapter/test/test_graph_path_bridge_routing.py'
edit(p,'    p.header.frame_id = "spot1/odom"',
       '    p.header.frame_id = "spot1/odom"\n    p.header.stamp = rclpy.clock.Clock().now().to_msg()')
# A fully observed edge also needs evidence across its clearance corridor.
p=B/'graphnav_mapper/src/mapper.cpp'
edit(p,'      fully_observed = fully_observed && map.atPosition("unknown", p) < 0.5f;',
'''      const double unknown_clearance = map.atPosition("unknown_sdf", p);
      fully_observed = fully_observed && std::isfinite(unknown_clearance) &&
          unknown_clearance >= robot_minimum_clearance_;''')
# Reports reference the current task and existing replay evidence.
p=R/'tools/offline_navigation_check.py'
s=p.read_text()
s=s.replace('if entry["status"] == "PASS" and entry["detail"]["configured_first_query"] != ["trash can"]:',
 'if entry["status"] == "PASS" and entry["detail"]["configured_first_query"] != ["gas station"]:')
s=s.replace("The demo's initial query is not 'trash can'. Publish the intended trash can query after Town startup, then gas station in the same process; record both query messages and remembered-object goals.",
 "The generic demo does not initialize gas-station memory at Town. Use full_town_memory_test.py for station observation, house search, then gas station in one process; do not accept an earlier-query demo as this task's result.")
s=s.replace('"Town failure raw pointcloud/costmap replay: no matching recording available",',
 '"C++ graph mapper/planner compile and test_route_mechanism in a ROS 2 build environment",\n             "Exact Town prefix/costmap fixture replay is available; full rosbag/vehicle execution still needs runtime verification",')
p.write_text(s,encoding='utf-8')
print('cleaned confirmation flow and freshness guard')

