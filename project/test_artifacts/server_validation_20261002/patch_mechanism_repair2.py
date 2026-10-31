from pathlib import Path
ROOT=Path(r'C:\Users\Yu\Desktop\拟真'); BASE=ROOT/'wildos_port/repro_check/src/nebula2-wildos'
def edit(p,a,b):
 p=Path(p);s=p.read_text(encoding='utf-8')
 if b in s: return
 assert a in s,(str(p),a[:90]);p.write_text(s.replace(a,b,1),encoding='utf-8')
p=BASE/'graphnav_mapper/src/mapper.cpp'
edit(p,'if (gridmap.exists("elevation_valid")) layer_names.push_back("elevation_valid");',
 'if (std::find(map->layers.begin(), map->layers.end(), "elevation_valid") != map->layers.end())\n    layer_names.push_back("elevation_valid");')
# Exact timestamp correlation permits frontier prefixes without accepting an old goal's route.
p=ROOT/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
edit(p,'        self.latest_route = None\n        self.grid_received_at',
'''        self.latest_route = None
        self.route_complete = True
        self.require_route_status = True
        self.route_metadata = {}
        self.waiting_routes = {}
        self.create_subscription(String, "/spot1/graphnav_planner/route_status", self.on_route_status, 10)
        self.create_subscription(String, "/spot1/object_confirmation", self.on_object_confirmation, 10)
        self.query_started_ns = 0
        self.grid_received_at''')
edit(p,'        self.query_generation += 1\n        self.reset()',
'''        self.query_generation += 1
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.route_metadata = {}
        self.waiting_routes = {}
        self.reset()''')
edit(p,'    def on_route(self, message):',
'''    def on_object_confirmation(self, message):
        try:
            data = json.loads(message.data)
            if data['query'] != self.query or data['observation_stamp_ns'] < self.query_started_ns:
                return
            if data['frame'] != 'spot1/odom' or not data.get('search_confirmed'):
                return
            if self.memory_active and self.memory_active.get('object_position'):
                if distance(data['object_position'][:2], self.memory_active['object_position'][:2]) > (
                        3.0 + data['radius'] + self.memory_active.get('object_radius', 0.0)):
                    return
        except (ValueError, KeyError, TypeError):
            return
        self.reset()
        self.memory_selecting = False
        self.pending_destination = None
        self.query_reset_pending = True
        self.query_reset_inflight = False
        self.retry_graph_reset()
        self.status('object_found', query=self.query, object_position=data['object_position'],
                    observer_position=data['observer_position'], confirmation=data)

    def on_route_status(self, message):
        try:
            data = json.loads(message.data)
            stamp = int(data['stamp_ns'])
            if not isinstance(data['complete'], bool) or len(data['goal']) != 2:
                return
            if not all(map(math.isfinite, data['goal'])):
                return
        except (ValueError, KeyError, TypeError):
            return
        self.route_metadata[stamp] = data
        for old in sorted(self.route_metadata)[:-32]:
            self.route_metadata.pop(old, None)
        pending = self.waiting_routes.pop(stamp, None)
        if pending is not None:
            self.on_route(pending)

    def on_route(self, message):''')
edit(p,'        if self.memory_selecting and self.memory_active is None:\n            return',
'''        if self.last_status.get('status') == 'object_found':
            return
        if self.memory_selecting and self.memory_active is None:
            return''')
edit(p,'        # Accept nearby graph endpoints, but reject another task\'s destination.\n        if self.expected is not None and distance(endpoint, self.expected) > 7.0:\n            return  # Old graph path from the preceding query or frontier goal.',
'''        metadata = getattr(self, 'route_metadata', {}).get(stamp)
        if getattr(self, 'require_route_status', False) and metadata is None:
            self.waiting_routes[stamp] = message
            for old in sorted(self.waiting_routes)[:-8]:
                self.waiting_routes.pop(old, None)
            return
        if metadata is not None:
            if self.expected is not None and distance(metadata['goal'], self.expected) > 0.25:
                return
            self.route_complete = metadata['complete']
        else:
            self.route_complete = True
            if self.expected is not None and distance(endpoint, self.expected) > 7.0:
                return''')
edit(p,'if (self.expected is not None and distance(points[-1], self.expected) <= 7.0',
       "if (getattr(self, 'route_complete', True) and self.expected is not None and distance(points[-1], self.expected) <= 7.0")
edit(p,'        endpoint = self.expected or xy(self.route.poses[-1].pose)',
       "        endpoint = (self.expected if getattr(self, 'route_complete', True) else None) or xy(self.route.poses[-1].pose)")
edit(p,'            self.memory_full_check = distance(start, endpoint) <= self.max_segment',
       "            self.memory_full_check = getattr(self, 'route_complete', True) and distance(start, endpoint) <= self.max_segment")
edit(p,'in ("follow_failed", "stale_odom", "stale_costmap")):',
       'in ("follow_failed", "stale_odom", "stale_costmap", "memory_locally_blocked",\n                        "path_changed_before_execution")):')
edit(p,'            if endpoint is None or (self.expected is not None',
       "            if endpoint is None or (getattr(self, 'route_complete', True) and self.expected is not None")
# Query-labelled localization transports observation time, not result publication time.
p=BASE/'visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
edit(p,'import rclpy','import json\nimport rclpy')
edit(p,'        self.nav_goal_publisher = self.create_publisher(',
'''        self.localization_pub = self.create_publisher(String, '/spot1/object_localization', 10)
        self.nav_goal_publisher = self.create_publisher(''')
edit(p,'        lidar_msg = oldest_msg["lidar"]','        self.current_observation = obj_mask_msg\n        lidar_msg = oldest_msg["lidar"]')
edit(p,'        self.nav_goal_publisher.publish(nav_goal)',
'''        self.nav_goal_publisher.publish(nav_goal)
        obs = self.current_observation
        stamp_ns = obs.header.stamp.sec * 1000000000 + obs.header.stamp.nanosec
        pose = obs.odom.pose.pose.position
        self.localization_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': self.global_frame,
            'observation_stamp_ns': stamp_ns,
            'position': self.triangulated_position.tolist(),
            'observer_position': [pose.x, pose.y, pose.z],
            'source': 'lidar' if self.found_lidar_in_mask else 'multiview',
            'views': len(self.views)})))''')
# Confirmation uses bounded robust consensus, reports spatial uncertainty and fresh evidence.
p=ROOT/'official_adapter/beamng_wildos_adapter/goal_standoff.py'
edit(p,'import rclpy','import json\nimport math\nimport rclpy')
edit(p,'        self.active_query = None',
'''        self.active_query = None
        self.query_started_ns = 0
        self.observations = []
        self.confirmation_pub = self.create_publisher(String, '/spot1/object_confirmation', 10)
        self.create_subscription(String, '/spot1/object_localization', self.on_localization, 10)''')
edit(p,'            PoseStamped, str(self.get_parameter("input_topic").value), self.on_target, 10)',
       '            PoseStamped, str(self.get_parameter("input_topic").value), lambda m: None, 10)')
edit(p,'        self.targets.clear()\n        self.last_goal = None',
'''        self.targets.clear()
        self.observations.clear()
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.last_goal = None''')
edit(p,'    def on_target(self, target):',
'''    def on_localization(self, message):
        try:
            data = json.loads(message.data)
            now_ns = self.get_clock().now().nanoseconds
            stamp = int(data['observation_stamp_ns'])
            values = data['position']
            if (data['query'] != self.active_query or data['frame'] != 'spot1/odom'
                    or stamp < self.query_started_ns or now_ns - stamp > 5000000000
                    or stamp > now_ns or len(values) != 3
                    or not all(map(math.isfinite, values))):
                return
            if any(item['observation_stamp_ns'] == stamp for item in self.observations):
                return
        except (ValueError, KeyError, TypeError):
            return
        self.observations.append(data)
        self.observations = [item for item in self.observations[-10:]
                             if stamp - item['observation_stamp_ns'] <= 30000000000]
        frames = int(self.get_parameter('confirmation_frames').value)
        radius = float(self.get_parameter('confirmation_radius').value)
        if len(self.observations) < frames:
            return
        values = np.asarray([item['position'] for item in self.observations])
        clusters = []
        for seed in values:
            inliers = np.linalg.norm(values[:, :2] - seed[:2], axis=1) <= 2 * radius
            centre = np.median(values[inliers], axis=0)
            inliers = np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= radius
            if np.count_nonzero(inliers):
                centre = np.median(values[inliers], axis=0)
                inliers &= np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= radius
                clusters.append((int(np.count_nonzero(inliers)), centre, inliers))
        if not clusters:
            return
        count, centre, inliers = max(clusters, key=lambda x: x[0])
        if count < max(3, math.ceil(.8 * frames)) or count / len(values) < .7 or not inliers[-1]:
            return
        self.targets = [tuple(centre)] * frames
        target = PoseStamped()
        target.header.frame_id = 'spot1/odom'
        target.header.stamp.sec, target.header.stamp.nanosec = divmod(stamp, 1000000000)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, centre)
        self.on_target(target)
        spread = float(np.max(np.linalg.norm(values[inliers, :2] - centre[:2], axis=1)))
        confirmation = {
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': stamp, 'object_position': centre.tolist(),
            'observer_position': data['observer_position'], 'radius': spread,
            'inliers': count, 'observations': len(values), 'source': data['source'],
            'search_confirmed': True, 'standoff': list(self.last_goal) if self.last_goal else None}
        self.confirmation_pub.publish(String(data=json.dumps(confirmation)))

    def on_target(self, target):''')
# Store actual object region in addition to alternative approach views.
p=BASE/'visual_navigation/visual_navigation/wildos/nav.py'
edit(p,'''            self.object_search_waypoint_sub = self.create_subscription(
                PoseStamped,
                str(config.get("object_search_waypoint_topic", "/spot1/imgnav_waypoint")),
                self.object_search_waypoint_callback,
                10,
            )''','''            self.object_search_waypoint_sub = self.create_subscription(
                String, '/spot1/object_confirmation', self.object_confirmation_callback, 10)''')
edit(p,'    def object_search_waypoint_callback(self, message):',
'''    def object_confirmation_callback(self, message):
        try:
            data = json.loads(message.data)
            query = self.text_queries[0] if self.text_queries else None
            if data['query'] != query or data['frame'] != self.global_frame:
                return
            observer = self.last_positive_mask_odom
            if observer is None:
                return
            stamp_ns = observer.header.stamp.sec * 1000000000 + observer.header.stamp.nanosec
            if abs(stamp_ns - data['observation_stamp_ns']) > 5000000000:
                return
            waypoint = PoseStamped()
            waypoint.header.frame_id = data['frame']
            self.object_search_waypoint_callback(waypoint)
            views = self.object_goal_memory.views.get(self.object_goal_memory.key(query), [])
            if views:
                latest = max(views, key=lambda v: v['observed_at'])
                latest.update(object_position=data['object_position'], object_radius=data['radius'])
        except (ValueError, KeyError, TypeError):
            return

    def object_search_waypoint_callback(self, message):''')
edit(p,'''            self.model_viz_pub.publish(self.br.cv2_to_imgmsg(
                self.viz.visualize_model_det(nav_data, all_cam_data), encoding="rgb8"))''',
'''            model_image = self.br.cv2_to_imgmsg(
                self.viz.visualize_model_det(nav_data, all_cam_data), encoding="rgb8")
            model_image.header = odom_msg.header
            self.model_viz_pub.publish(model_image)''')
print('patched route correlation and object-region confirmation')

