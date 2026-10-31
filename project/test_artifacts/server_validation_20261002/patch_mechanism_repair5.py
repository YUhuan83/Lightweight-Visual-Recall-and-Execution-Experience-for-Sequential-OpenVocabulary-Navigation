from pathlib import Path
R=Path(r'C:\Users\Yu\Desktop\拟真');B=R/'wildos_port/repro_check/src/nebula2-wildos'
def edit(p,a,b):
 p=Path(p);s=p.read_text(encoding='utf-8')
 if b in s: return
 assert a in s,(p,a[:70]);p.write_text(s.replace(a,b,1),encoding='utf-8')
p=B/'visual_navigation/visual_navigation/wildos/nav.py'
edit(p,'        self.object_goal_memory = ViewpointMemory(bin_count=self.num_angular_bins)',
'''        self.object_goal_memory = ViewpointMemory(bin_count=self.num_angular_bins)
        self.object_query_started_ns = self.get_clock().now().nanoseconds''')
edit(p,'        query = " ".join(str(message.data).split())',
       '        query = " ".join(str(message.data).casefold().split())')
edit(p,'        self.text_queries = [query]\n        self.text_feats',
       '        self.object_query_started_ns = self.get_clock().now().nanoseconds\n        self.text_queries = [query]\n        self.text_feats')
edit(p,'''                    or stamp > now_ns):
                return
            p = data['observer_position']''',
'''                    or stamp > now_ns or stamp < self.object_query_started_ns):
                return
            if (len(data['object_position']) != 3
                    or not all(map(np.isfinite, [*data['object_position'], data['radius']]))
                    or data['radius'] < 0 or data['inliers'] < 3
                    or data['observations'] < data['inliers']):
                return
            p = data['observer_position']''')
edit(p,'            if len(p) != 3 or len(q) != 4 or not all(map(np.isfinite, [*p, *q])):',
       '            if (len(p) != 3 or len(q) != 4 or not all(map(np.isfinite, [*p, *q]))\n                    or abs(sum(v*v for v in q) - 1) > .1):')
p=B/'visual_navigation/visual_navigation/wildos/viewpoint_memory.py'
edit(p,"                      and math.dist(v['position'], position) < 1.0), None)",
'''                      and math.dist(v['position'], position) < 1.0
                      and (object_position is None or not v.get('object_position') or
                           math.dist(v['object_position'], object_position) <=
                           3.0 + v.get('object_radius', 0.0) + object_radius)), None)''')
p=R/'nav2_adapter/wildos_nav2_adapter/memory_selection.py'
edit(p,"        if views == self.memory_views and (views or self.memory_goal is None):",
       "        if self.object_found:\n            return\n        if views == self.memory_views and (views or self.memory_goal is None):")
p=R/'tools/offline_navigation_check.py'
edit(p,'"test_graph_snapshot_sync")]',
       '"test_graph_snapshot_sync", "test_object_memory_integration")]')
edit(p,'        assert planned >= radius',
'''        assert controller["odom_topic"] == "/spot1/odom"
        assert controller["FollowPath"]["min_lookahead_dist"] >= 3.0
        assert controller["FollowPath"]["regulated_linear_scaling_min_radius"] >= planned
        assert planned >= radius''')
# Height-valid map adapters remain reusable if a real elevation layer is provided later.
p=B/'graphnav_mapper/src/mapper.cpp'
edit(p,'if (gridmap.exists("elevation") && !gridmap.exists("elevation_valid"))',
       'if (gridmap.exists("elevation") && (!gridmap.exists("elevation_valid") ||\n          gridmap.atPosition("elevation_valid", node_pos_map) > 0.5f))')
edit(p,'if (gridmap.exists("elevation_valid"))\n      {',
       'if (gridmap.exists("elevation_valid") &&\n          gridmap.atPosition("elevation_valid", node_pos_map) <= 0.5f)\n      {')
print('completed object-memory integration validation')

