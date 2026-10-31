"""Convert a semantic object position to a road-level vehicle standoff goal."""
from __future__ import annotations

import json
import math
from copy import deepcopy
from collections import deque
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String
from object_search_msgs.msg import ObjectMaskWithTf

from .geometry import standoff_xy
from .geometry import quaternion_matrix
from .mapping import reachable_ring_candidate, safe_travel_distance
from .target_selection import (required_cluster_support, select_target_cluster,
                               supported_position_clusters)
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT
from wildos_nav2_adapter.sensor_geometry import (SBR_CAMERA_OFFSET, observation_clearance,
                                               geometry_restart_reason)
from wildos_nav2_adapter.arrival_criteria import STANDOFF_ARRIVAL_TOLERANCE_M
from rcl_interfaces.msg import SetParametersResult
from .visible_approach import visible_direction, observation_step


class GoalStandoff(Node):
    def __init__(self):
        super().__init__("beamng_vehicle_goal_standoff")
        self.declare_parameter("output_topic", "/spot1/imgnav_waypoint")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("query_topic", "/spot1/object_search/query")
        self.declare_parameter("carrier_model", "ackermann")
        self.declare_parameter("standoff", 10.0)
        self.declare_parameter("traffic_light_standoff", 4.0)
        self.declare_parameter("vehicle_front_extent", -1.0)
        self.declare_parameter("target_front_clearance", 0.6)
        self.declare_parameter("camera_offset", list(SBR_CAMERA_OFFSET))
        self.declare_parameter("minimum_camera_target_range", 1.0)
        observation_clearance(self.get_parameter('camera_offset').value,
            float(self.get_parameter('minimum_camera_target_range').value),
            STANDOFF_ARRIVAL_TOLERANCE_M)
        self.declare_parameter("min_update_distance", 1.0)
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("confirmation_radius", 1.5)
        self.declare_parameter("obstacle_margin", 2.0)
        radius = float(self.get_parameter("confirmation_radius").value)
        if (not 3 <= int(self.get_parameter("confirmation_frames").value) <= 10
                or not math.isfinite(radius) or not 0 < radius <= 3.):
            raise ValueError("confirmation requires 3–10 observations and a radius in (0, 3] metres")
        self.odom = None
        self.occupancy = None
        self.odom_history = deque(maxlen=200)
        self.occupancy_odom = None
        self.last_goal = None
        self.last_standoff_clamped = False
        self.last_standoff_object_distance = None
        self.active_query = None
        self.query_started_ns = 0
        self.last_localization_ns = 0
        self.query_origin = None
        self.observations = []
        self.confirmed_instance_anchor = None
        self.pending_confirmation = None
        self.pending_status = None
        self.active_approach = None
        self.visible_target = None
        self.visible_target_count = 0
        self.visible_step_published_ns = 0
        self.visible_approach_pub = self.create_publisher(String, '/spot1/visible_target_approach', 10)
        self.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', self.on_visible_target, 10)
        self.confirmation_pub = self.create_publisher(String, '/spot1/object_confirmation', 10)
        self.create_subscription(String, '/spot1/object_localization', self.on_localization, 10)
        self.instance_ack_pub = self.create_publisher(String, '/spot1/object_search/instance_release_ack', 10)
        self.create_subscription(String, '/spot1/object_search/instance_release', self.on_instance_release, 10)
        self.released_requests = deque(maxlen=16)
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.create_subscription(OccupancyGrid, "/spot1/local_map", self.on_occupancy, 10)
        self.create_subscription(
            String, str(self.get_parameter("query_topic").value), self.on_query, 10)
        self.create_timer(.2, self.on_timer)
        self.add_on_set_parameters_callback(self.on_parameters)

    def on_parameters(self, parameters):
        reason = geometry_restart_reason(parameters, self.get_parameter)
        return SetParametersResult(successful=reason is None, reason=reason or '')

    def on_timer(self):
        self.retry_confirmation()
        self.refresh_active_approach()
        self.publish_visible_step()

    def on_visible_target(self, message):
        stamp = self.stamp_ns(message)
        now = self.get_clock().now().nanoseconds
        if (message.query != self.active_query or stamp < self.query_started_ns
                or not 0 <= now-stamp <= 2_000_000_000):
            return
        previous = self.visible_target
        if previous is not None and stamp <= previous['stamp']:
            return
        try:
            bearing = visible_direction(message)
        except (ValueError, TypeError, AttributeError, np.linalg.LinAlgError):
            return
        if bearing is None:
            return
        consistent = (previous is not None and stamp-previous['stamp'] <= 2_000_000_000
            and abs(math.atan2(math.sin(bearing-previous['bearing']),
                               math.cos(bearing-previous['bearing']))) <= .35)
        self.visible_target_count = self.visible_target_count+1 if consistent else 1
        self.visible_target = {'stamp': stamp, 'bearing': bearing}
        self.publish_visible_step()

    def publish_visible_step(self):
        evidence = self.visible_target
        now = self.get_clock().now().nanoseconds
        if (evidence is None or self.visible_target_count < 3
                or not 0 <= now-evidence['stamp'] <= 2_000_000_000
                or self.pending_confirmation is not None or self.active_approach is not None
                or now-self.visible_step_published_ns < 400_000_000
                or self.odom is None or not self.fresh(self.odom, 1.)
                or self.occupancy is None or self.occupancy_odom is None
                or not self.fresh(self.occupancy, 1.)):
            return
        robot = self.odom.pose.pose.position
        map_pose = self.occupancy_odom.pose.pose
        if math.hypot(robot.x-map_pose.position.x, robot.y-map_pose.position.y) > 1.:
            return
        q = map_pose.orientation
        map_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        info = self.occupancy.info
        grid = np.asarray(self.occupancy.data, dtype=np.int16).reshape(info.height, info.width)
        point = observation_step(grid, float(info.resolution),
            (info.origin.position.x, info.origin.position.y),
            (map_pose.position.x, map_pose.position.y), map_yaw, evidence['bearing'])
        if point is None or math.hypot(point[0]-robot.x, point[1]-robot.y) < 2.:
            return
        self.visible_step_published_ns = now
        self.visible_approach_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': evidence['stamp'], 'point': list(point),
            'bearing': evidence['bearing'], 'evidence': 'bearing_only',
            'position_confirmed': False, 'search_confirmed': False})))

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

    @staticmethod
    def valid_pose(pose):
        p, q = pose.position, pose.orientation
        return (all(map(math.isfinite, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .1)

    def fresh(self, message, seconds):
        age = self.get_clock().now().nanoseconds - self.stamp_ns(message)
        return self.stamp_ns(message) > 0 and -50_000_000 <= age <= seconds * 1e9

    def on_odom(self, message):
        if (message.header.frame_id != 'spot1/odom' or not self.valid_pose(message.pose.pose)
                or not self.fresh(message, 1.) or (self.odom is not None
                    and self.stamp_ns(message) <= self.stamp_ns(self.odom))):
            return
        self.odom = message
        self.odom_history.append(message)
        if self.active_query is not None and self.query_origin is None:
            position = message.pose.pose.position
            self.query_origin = (float(position.x), float(position.y))
        self.pair_occupancy_pose()
        self.retry_confirmation()

    def pair_occupancy_pose(self):
        if self.occupancy is None or not self.odom_history:
            return
        stamp = self.stamp_ns(self.occupancy)
        paired = min(self.odom_history, key=lambda o: abs(self.stamp_ns(o) - stamp))
        self.occupancy_odom = paired if abs(self.stamp_ns(paired) - stamp) <= .2 * 1e9 else None

    def on_occupancy(self, message):
        i = message.info
        if (message.header.frame_id != 'spot1/base_link' or not self.fresh(message, 3.)
                or i.width <= 0 or i.height <= 0 or not math.isfinite(i.resolution)
                or i.resolution <= 0 or len(message.data) != i.width * i.height
                or not self.valid_pose(i.origin) or not all(-1 <= v <= 100 for v in message.data)
                or any(abs(v) > 1e-6 for v in (i.origin.orientation.x,
                        i.origin.orientation.y, i.origin.orientation.z))
                or (self.occupancy is not None and self.stamp_ns(message) <= self.stamp_ns(self.occupancy))):
            return
        self.occupancy = message
        self.pair_occupancy_pose()

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if not query or query == self.active_query:
            return
        self.active_query = query
        self.visible_target = None
        self.visible_target_count = 0
        self.visible_step_published_ns = 0
        self.observations.clear()
        self.confirmed_instance_anchor = None
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.last_localization_ns = 0
        self.query_origin = None
        if self.odom is not None and self.fresh(self.odom, 1.):
            position = self.odom.pose.pose.position
            self.query_origin = (float(position.x), float(position.y))
        self.last_goal = None
        self.pending_confirmation = None
        self.pending_status = None
        self.active_approach = None

    def on_instance_release(self, message):
        """Release only the matching query/instance, and ACK retries idempotently."""
        try:
            data = json.loads(message.data)
            stamp = int(data['request_stamp_ns'])
            fence = int(data['observation_fence_ns'])
            request_id = data['request_id']
            instance_id = data['instance_id']
            now = self.get_clock().now().nanoseconds
            if (data['query'] != self.active_query or fence < self.query_started_ns
                    or fence > stamp
                    or not isinstance(request_id, str) or not request_id
                    or not isinstance(instance_id, str)
                    or not -50_000_000 <= now-stamp <= 30_000_000_000):
                return
            identity = (data['query'], request_id, instance_id)
            if identity not in self.released_requests:
                anchor = self.confirmed_instance_anchor
                expected_id = anchor.get('instance_id', '') if anchor else ''
                # A consumer may never receive the confirmation (DDS loss or
                # candidate comparison). Its fixed abandonment fence still
                # identifies producer state from before that transition.
                clear_current = (instance_id == expected_id or
                    (not instance_id and anchor is not None
                     and int(anchor['observation_stamp_ns']) <= fence))
                if clear_current:
                    self.observations.clear()
                    self.confirmed_instance_anchor = None
                    self.pending_confirmation = self.pending_status = self.active_approach = None
                    self.last_goal = None
                    self.visible_target = None
                    self.visible_target_count = 0
                    self.query_started_ns = max(self.query_started_ns, fence)
                    self.last_localization_ns = max(self.last_localization_ns, fence)
                # A different/newer current instance means the requested old
                # lock is already absent. ACK without mutating that instance;
                # retain a tombstone so duplicate requests cannot clear it.
                self.released_requests.append(identity)
            self.instance_ack_pub.publish(String(data=json.dumps(dict(data, released=True))))
        except (KeyError, TypeError, ValueError):
            return

    def reject_localization_evidence(self, stamp, reason):
        """Revoke evidence, retaining route ownership until bounded recovery."""
        self.last_localization_ns = stamp
        self.observations.clear()
        self.pending_confirmation = self.pending_status = self.active_approach = None
        anchor = self.confirmed_instance_anchor or {}
        self.confirmation_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': stamp, 'evidence_rejected': True,
            'localization_valid': False, 'search_confirmed': False,
            'instance_id': anchor.get('instance_id', ''), 'reason': reason})))

    def on_localization(self, message):
        try:
            data = json.loads(message.data)
            now_ns = self.get_clock().now().nanoseconds
            stamp = int(data['observation_stamp_ns'])
            if (data['query'] != self.active_query or data['frame'] != 'spot1/odom'
                    or stamp < self.query_started_ns or stamp <= self.last_localization_ns
                    or not -50_000_000 <= now_ns-stamp <= 5_000_000_000
                    or data.get('source') not in ('lidar', 'multiview')):
                return
            if (data.get('localization_valid') is False
                    and data.get('reason') == 'ambiguous_depth_surfaces'):
                self.reject_localization_evidence(stamp, data['reason'])
                return
            if data.get('localization_valid') is not True:
                return
            values = data['position']
            observer = data['observer_position']
            orientation = data['observer_orientation']
            if (data['query'] != self.active_query or data['frame'] != 'spot1/odom'
                    or stamp < self.query_started_ns or now_ns - stamp > 5000000000
                    or stamp > now_ns + 50_000_000 or len(values) != 3
                    or not all(map(math.isfinite, values)) or len(observer) != 3
                    or not all(map(math.isfinite, observer)) or len(orientation) != 4
                    or not all(map(math.isfinite, orientation))
                    or abs(sum(v*v for v in orientation) - 1) > .1):
                return
            if self.observations and stamp <= self.observations[-1]['observation_stamp_ns']:
                return
            quality = data.get('quality')
            if (not isinstance(quality, dict)
                    or quality.get('association_validated') is not True
                    or not math.isfinite(quality.get('confidence', 0.))
                    or quality.get('confidence', 0.) < .5):
                return
            if data.get('source') not in ('lidar', 'multiview'):
                return
            if any(item['observation_stamp_ns'] == stamp for item in self.observations):
                return
        except (ValueError, KeyError, TypeError):
            return
        if self.confirmed_instance_anchor is not None:
            anchor = self.confirmed_instance_anchor
            anchor_position = anchor.get('object_position', ())
            if len(anchor_position) >= 2:
                gap = math.hypot(float(values[0]) - float(anchor_position[0]),
                                 float(values[1]) - float(anchor_position[1]))
                association_limit = (3.0 + float(anchor.get('radius', 0.0))
                                     + float(data.get('radius', 0.0)))
                if not math.isfinite(gap) or gap > association_limit:
                    self.reject_localization_evidence(stamp, 'instance_mismatch')
                    return
        self.last_localization_ns = stamp
        self.observations.append(data)
        self.observations = [item for item in self.observations[-10:]
                             if stamp - item['observation_stamp_ns'] <= 30000000000]
        frames = int(self.get_parameter('confirmation_frames').value)
        radius = float(self.get_parameter('confirmation_radius').value)
        minimum_support = required_cluster_support(frames)
        if len(self.observations) < minimum_support:
            return
        values = np.asarray([item['position'] for item in self.observations])
        clusters = supported_position_clusters(values, radius, minimum_support)
        if not clusters:
            return
        nearest_signal = self.active_query in (
            'traffic light', 'traffic lights', 'traffic signal')
        selected = select_target_cluster(
            clusters, self.query_origin, prefer_nearest=nearest_signal)
        count, centre, inliers = selected
        latest_index = int(np.flatnonzero(inliers)[-1])
        if latest_index != len(self.observations)-1:
            # A new observation of another surface must not refresh the
            # timestamp/quality of a larger, historical localization cluster.
            self.reject_localization_evidence(stamp, 'current_observation_outside_cluster')
            return
        observation = self.observations[latest_index]
        selected_stamp = int(observation['observation_stamp_ns'])
        if now_ns - selected_stamp > 5_000_000_000:
            return
        target = PoseStamped()
        target.header.frame_id = 'spot1/odom'
        target.header.stamp.sec, target.header.stamp.nanosec = divmod(selected_stamp, 1000000000)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, centre)
        spread = float(np.max(np.linalg.norm(values[inliers, :2] - centre[:2], axis=1)))
        confirmation = {
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': selected_stamp, 'object_position': centre.tolist(),
            'observer_position': observation['observer_position'],
            'observer_orientation': observation['observer_orientation'], 'radius': spread,
            'inliers': count, 'observations': len(values), 'source': observation['source'],
            'position_confirmed': True,
            'localization_valid': True,
            'quality': observation.get('quality'),
            'observation_id': f'{self.active_query}:{selected_stamp}'}
        self.pending_confirmation = (target, confirmation)
        self.retry_confirmation(force=True)

    def retry_confirmation(self, force=False):
        if self.pending_confirmation is None:
            return
        target, confirmation = self.pending_confirmation
        stamp = confirmation['observation_stamp_ns']
        if (confirmation['query'] != self.active_query or stamp < self.query_started_ns
                or self.get_clock().now().nanoseconds - stamp > 5_000_000_000):
            self.pending_confirmation = None
            return
        reason = self.publish_standoff(target)
        ready = reason is None and self.last_goal is not None
        confirmation['instance_id'] = (self.confirmed_instance_anchor.get('instance_id',
                f'{self.active_query}:{self.confirmed_instance_anchor["observation_stamp_ns"]}')
            if self.confirmed_instance_anchor else f'{self.active_query}:{stamp}')
        status = (ready, reason, self.last_goal)
        if force or status != self.pending_status:
            self.confirmation_pub.publish(String(data=json.dumps({**confirmation,
                'search_confirmed': ready, 'approach_goal_ready': ready,
                'standoff': list(self.last_goal) if ready else None,
                'standoff_object_distance_m': (self.last_standoff_object_distance
                                               if ready else None),
                'standoff_clamped': (self.last_standoff_clamped if ready else None),
                'approach_failure_reason': reason})))
            self.pending_status = status
        if ready:
            # Keep the geometrically confirmed object for this query. A
            # map-clamped standoff can be only an intermediate scan boundary;
            # the next safe waypoint must be recomputed as the vehicle and
            # rolling local map move forward.
            self.active_approach = (deepcopy(target), dict(confirmation))
            if self.confirmed_instance_anchor is None:
                self.confirmed_instance_anchor = dict(confirmation)
            self.pending_confirmation = None

    def refresh_active_approach(self):
        if self.active_approach is None or self.active_query is None:
            return
        target, confirmation = self.active_approach
        if confirmation.get('query') != self.active_query:
            self.active_approach = None
            return
        if (self.odom is None or self.occupancy is None or self.occupancy_odom is None
                or not self.fresh(self.odom, 1.) or not self.fresh(self.occupancy, 3.)
                or not self.last_goal or not self.last_standoff_clamped):
            return
        robot = self.odom.pose.pose.position
        object_position = confirmation.get('object_position', ())
        if len(object_position) < 2:
            self.active_approach = None
            return
        object_gap = math.hypot(float(object_position[0]) - robot.x,
                                float(object_position[1]) - robot.y)
        standoff, _ = self.approach_distance()
        # Once inside the final approach ring, stop advancing on carried
        # geometry and wait for fresh target evidence before declaring found.
        if object_gap <= standoff + 1.0:
            return
        update_distance = max(.75, float(self.get_parameter('min_update_distance').value))
        if math.hypot(robot.x - self.last_goal[0], robot.y - self.last_goal[1]) > update_distance:
            return

        previous_goal = self.last_goal
        reason = self.publish_standoff(target)
        if (reason is not None or self.last_goal is None
                or math.hypot(self.last_goal[0] - previous_goal[0],
                              self.last_goal[1] - previous_goal[1]) < update_distance):
            return
        now_ns = self.get_clock().now().nanoseconds
        refresh = dict(confirmation,
            approach_refresh=True,
            base_observation_stamp_ns=int(confirmation['observation_stamp_ns']),
            approach_update_stamp_ns=now_ns,
            search_confirmed=True,
            approach_goal_ready=True,
            standoff=list(self.last_goal),
            standoff_object_distance_m=self.last_standoff_object_distance,
            standoff_clamped=self.last_standoff_clamped,
            approach_failure_reason=None)
        self.confirmation_pub.publish(String(data=json.dumps(refresh)))

    def approach_distance(self):
        """Keep the configured target distance beyond the carrier's front extent."""
        name = ('traffic_light_standoff' if self.active_query in
                ('traffic light', 'traffic lights', 'traffic signal') else 'standoff')
        configured = float(self.get_parameter(name).value)
        model = str(self.get_parameter('carrier_model').value).strip().lower()
        configured_front = float(self.get_parameter('vehicle_front_extent').value)
        front = (configured_front if configured_front >= 0 else
                 (0.0 if model == 'ideal_point' else SBR_FRONT_EXTENT))
        clearance = float(self.get_parameter('target_front_clearance').value)
        if (model not in ('ideal_point', 'ackermann')
                or not all(map(math.isfinite, (configured, front, clearance)))
                or configured <= 0 or front < 0 or clearance < 0):
            return math.nan, False
        try:
            sensor_clearance = observation_clearance(
                self.get_parameter('camera_offset').value,
                float(self.get_parameter('minimum_camera_target_range').value),
                STANDOFF_ARRIVAL_TOLERANCE_M)
        except (ValueError, TypeError):
            return math.nan, False
        safe = max(configured, front + clearance, sensor_clearance)
        return safe, safe > configured

    def publish_standoff(self, target):
        if self.odom is None:
            self.last_goal = None
            return 'missing_vehicle_pose'
        if not self.fresh(self.odom, 1.):
            self.last_goal = None
            return 'stale_vehicle_pose'
        stable = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
        robot = self.odom.pose.pose.position
        standoff, clearance_adjusted = self.approach_distance()
        if not math.isfinite(standoff) or standoff <= 0:
            self.last_goal = None
            return 'invalid_standoff_distance'
        x, y = standoff_xy((robot.x, robot.y), (stable[0], stable[1]), standoff,
                          maintain_distance=True)
        clamped = clearance_adjusted
        if (self.occupancy is not None and self.occupancy_odom is not None
                and self.fresh(self.occupancy, 3.)):
            map_robot = self.occupancy_odom.pose.pose.position
            target_world = np.array([stable[0] - map_robot.x, stable[1] - map_robot.y, 0.0])
            q = self.occupancy_odom.pose.pose.orientation
            rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
            target_local = target_world @ rotation
            info = self.occupancy.info
            grid = np.asarray(self.occupancy.data, dtype=np.int16).reshape(
                int(info.height), int(info.width))
            origin = (info.origin.position.x, info.origin.position.y)
            # Unknown ground is a mapping gap, not evidence of an obstacle.
            # Approach the visible object to within one map cell of unknown
            # space, then let the updated scan determine the next step. Keep
            # the wider configured margin for explicitly occupied cells.
            unknown_margin = min(
                float(self.get_parameter("obstacle_margin").value),
                max(float(info.resolution), 0.5))
            candidate_local = reachable_ring_candidate(
                grid, float(info.resolution), origin, target_local[:2], standoff,
                float(self.get_parameter("obstacle_margin").value), unknown_margin,
                maintain_distance=True)
            if candidate_local is not None:
                candidate_world = rotation @ np.array(
                    [candidate_local[0], candidate_local[1], 0.0])
                x = map_robot.x + candidate_world[0]
                y = map_robot.y + candidate_world[1]
                clamped = clearance_adjusted or not np.allclose(candidate_local, target_local[:2] *
                    max(0.0, 1.0 - standoff / max(np.linalg.norm(target_local[:2]), 1e-6)),
                    atol=0.2)
            else:
                target_distance = float(np.linalg.norm(target_world[:2]))
                desired_travel = max(0.0, target_distance - standoff)
                safe_travel = safe_travel_distance(
                    grid, float(info.resolution), origin,
                    target_local[:2], desired_travel,
                    float(self.get_parameter("obstacle_margin").value),
                    unknown_margin)
                direction = target_world[:2] / max(target_distance, 1e-6)
                x = map_robot.x + direction[0] * safe_travel
                y = map_robot.y + direction[1] * safe_travel
                clamped = True
        candidate = (x, y)
        if math.hypot(stable[0]-x, stable[1]-y) < standoff - .05:
            self.last_goal = None
            return 'no_observable_standoff'
        self.last_standoff_clamped = clamped
        if (math.hypot(stable[0] - robot.x, stable[1] - robot.y) > standoff + .2
                and math.hypot(x - robot.x, y - robot.y) < .2):
            self.last_goal = None
            return 'no_reachable_standoff'
        minimum = float(self.get_parameter("min_update_distance").value)
        previous_clearance_ok = (self.last_goal is not None and
            (not clearance_adjusted or math.hypot(stable[0] - self.last_goal[0],
                stable[1] - self.last_goal[1]) >= standoff))
        if previous_clearance_ok and ((candidate[0] - self.last_goal[0]) ** 2
                + (candidate[1] - self.last_goal[1]) ** 2) ** 0.5 < minimum:
            self.last_standoff_object_distance = math.hypot(
                float(stable[0]) - self.last_goal[0],
                float(stable[1]) - self.last_goal[1])
            return
        output = PoseStamped()
        output.header = target.header
        output.header.frame_id = "spot1/odom"
        output.header.stamp = self.get_clock().now().to_msg()
        output.pose.position.x = x
        output.pose.position.y = y
        output.pose.position.z = robot.z
        heading = math.atan2(stable[1] - y, stable[0] - x)
        output.pose.orientation.z = math.sin(heading / 2)
        output.pose.orientation.w = math.cos(heading / 2)
        self.publisher.publish(output)
        self.last_goal = candidate
        self.last_standoff_object_distance = math.hypot(
            float(stable[0]) - self.last_goal[0],
            float(stable[1]) - self.last_goal[1])
        self.get_logger().info(
            f"stable object ({stable[0]:.2f}, {stable[1]:.2f}, {stable[2]:.2f}) -> "
            f"road standoff ({x:.2f}, {y:.2f}, {robot.z:.2f})"
            f" object_gap={self.last_standoff_object_distance:.2f}m"
            f" obstacle_clamped={clamped}")


def main(args=None):
    rclpy.init(args=args); node = GoalStandoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
