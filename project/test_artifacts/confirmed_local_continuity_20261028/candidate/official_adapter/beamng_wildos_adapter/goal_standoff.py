"""Convert a semantic object position to a road-level vehicle standoff goal."""
from __future__ import annotations

import json
import math
import time
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
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH
from wildos_nav2_adapter.graph_route import footprint_blockers,cell_cost
from wildos_nav2_adapter.sensor_geometry import (SBR_CAMERA_OFFSET, observation_clearance,
                                               geometry_restart_reason)
from wildos_nav2_adapter.arrival_criteria import (
    STANDOFF_ARRIVAL_TOLERANCE_M, vehicle_body_distance, within_body_arrival)
from rcl_interfaces.msg import SetParametersResult
from .visible_approach import visible_direction, observation_step, vehicle_observation_step
from wildos_nav2_adapter.observation_pose import (
    connected_free_cells, grid_cell, observation_pose_clear)


class GoalStandoff(Node):
    def __init__(self):
        super().__init__("beamng_vehicle_goal_standoff")
        self.declare_parameter("output_topic", "/spot1/imgnav_waypoint")
        self.declare_parameter("odom_topic", "/spot1/odom")
        self.declare_parameter("query_topic", "/spot1/object_search/query")
        self.declare_parameter("carrier_model", "ackermann")
        self.declare_parameter("standoff", 10.0)
        self.declare_parameter("graph_destination_approach", False)
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
        self.declare_parameter("confirmation_window_sec", 30.0)
        self.declare_parameter("visible_approach_masks", 3)
        self.declare_parameter("visible_approach_continuity_sec", 2.0)
        self.declare_parameter("visible_observation_max_m", 8.0)
        self.declare_parameter("observation_minimum_turning_radius", 0.0)
        self.observation_minimum_turning_radius=float(
            self.get_parameter('observation_minimum_turning_radius').value)
        if (not math.isfinite(self.observation_minimum_turning_radius)
                or self.observation_minimum_turning_radius<0):
            raise ValueError('observation_minimum_turning_radius must be finite and nonnegative')
        self.declare_parameter("obstacle_margin", 2.0)
        radius = float(self.get_parameter("confirmation_radius").value)
        window = float(self.get_parameter("confirmation_window_sec").value)
        masks = int(self.get_parameter("visible_approach_masks").value)
        continuity = float(self.get_parameter("visible_approach_continuity_sec").value)
        observation = float(self.get_parameter("visible_observation_max_m").value)
        if (not 3 <= int(self.get_parameter("confirmation_frames").value) <= 10
                or not math.isfinite(radius) or not 0 < radius <= 3.
                or not math.isfinite(window) or not 5. <= window <= 120.
                or not 1 <= masks <= 5 or not math.isfinite(continuity)
                or not .5 <= continuity <= 20. or not math.isfinite(observation)
                or not 2. <= observation <= 10.):
            raise ValueError("confirmation requires 3–10 observations, a radius in (0, 3] metres "
                             "and a 5–120 s window; visible approach requires 1–5 masks, "
                             "0.5–20 s continuity and 2–10 m observation steps")
        self.confirmation_window_sec = window
        self.visible_approach_masks = masks
        self.visible_approach_continuity_sec = continuity
        self.visible_observation_max_m = observation
        self.odom = None
        self.occupancy = None
        self.odom_history = deque(maxlen=200)
        self.occupancy_odom = None
        self.vehicle_grid = None
        self.local_vehicle_grid = None
        self.vehicle_connectivity = None
        self.rejected_observation_poses = deque(maxlen=16)
        self.last_visible_goal = None
        self.accepted_visible_goal = None
        self.accepted_visible_stamp_ns = 0
        self.last_navigation_failure_ns = 0
        self.last_goal = None
        self.last_standoff_clamped = False
        self.last_standoff_navigation_heading = None
        self.last_standoff_object_distance = None
        self.active_query = None
        self.query_started_ns = 0
        self.last_localization_ns = 0
        self.query_origin = None
        self.observations = []
        self.replacement_observations = []
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
        self.navigation_region_backoff = deque(maxlen=16)
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.create_subscription(OccupancyGrid, "/spot1/local_map", self.on_occupancy, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_vehicle_grid, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
            lambda message: self.on_vehicle_grid(message,local=True), 10)
        self.create_subscription(String, "/wildos/plan_status", self.on_navigation_plan_status, 10)
        self.create_subscription(
            String, str(self.get_parameter("query_topic").value), self.on_query, 10)
        self.create_timer(.2, self.on_timer)
        self.add_on_set_parameters_callback(self.on_parameters)

    def on_parameters(self, parameters):
        reason = geometry_restart_reason(parameters, self.get_parameter)
        if any(p.name=='observation_minimum_turning_radius'
               and p.value!=self.observation_minimum_turning_radius for p in parameters):
            reason='observation_minimum_turning_radius requires a coordinated planner restart'
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
        # Establish a new candidate with three fresh, closely spaced masks.
        # An established bearing may be reacquired across a short occlusion;
        # the new mask itself must still pass the unchanged 2-second gate.
        continuity_ns = (15_000_000_000
                         if self.visible_target_count >= self.visible_approach_masks
                         else int(self.visible_approach_continuity_sec*1e9))
        consistent = (previous is not None and stamp-previous['stamp'] <= continuity_ns
            and abs(math.atan2(math.sin(bearing-previous['bearing']),
                               math.cos(bearing-previous['bearing']))) <= .35)
        self.visible_target_count = (min(self.visible_approach_masks,
                                         self.visible_target_count+1)
                                     if consistent else 1)
        self.visible_target = {'stamp': stamp, 'bearing': bearing}
        self.publish_visible_step()

    def vehicle_observation_pose(self, bearing, target=None, minimum_target_distance=0.):
        """Shared measured-free intermediate pose for a bearing or fixed object."""
        if (self.observation_minimum_turning_radius<=0 or self.odom is None
                or self.occupancy is None or self.occupancy_odom is None
                or self.vehicle_grid is None or not self.fresh(self.odom,1.)
                or not self.fresh(self.occupancy,1.) or not self.fresh(self.vehicle_grid,3.)):
            return None
        robot=self.odom.pose.pose;map_pose=self.occupancy_odom.pose.pose
        if math.hypot(robot.position.x-map_pose.position.x,robot.position.y-map_pose.position.y)>1.:
            return None
        def heading(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        info=self.occupancy.info
        grid=np.asarray(self.occupancy.data,dtype=np.int16).reshape(info.height,info.width)
        accept=self.observation_pose_filter(bearing=bearing,target=target)
        start=(robot.position.x,robot.position.y)
        def endpoint(point,yaw):
            if target is not None:
                gap=math.dist(point,target)
                if not minimum_target_distance-.05<=gap<math.dist(start,target)-.6:
                    return False
            return accept(point,yaw)
        return vehicle_observation_step(grid,float(info.resolution),
            (info.origin.position.x,info.origin.position.y),
            (map_pose.position.x,map_pose.position.y),heading(map_pose.orientation),bearing,
            start,heading(robot.orientation),self.observation_minimum_turning_radius,
            maximum=self.visible_observation_max_m,accept=endpoint,
            sweep_clear=self.observation_pose_filter(bearing=bearing,include_rejected=False))

    def publish_visible_step(self):
        evidence = self.visible_target
        now = self.get_clock().now().nanoseconds
        if (evidence is None or self.visible_target_count < self.visible_approach_masks
                or not 0 <= now-evidence['stamp'] <= 2_000_000_000
                or self.active_approach is not None
                or now-self.visible_step_published_ns < 400_000_000
                or self.odom is None or not self.fresh(self.odom, 1.)
                or self.occupancy is None or self.occupancy_odom is None
                or not self.fresh(self.occupancy, 1.)):
            return
        robot = self.odom.pose.pose.position
        map_pose = self.occupancy_odom.pose.pose
        if math.hypot(robot.x-map_pose.position.x, robot.y-map_pose.position.y) > 1.:
            return
        if self.pending_confirmation is not None:
            target, _ = self.pending_confirmation
            direction = math.atan2(target.pose.position.y-map_pose.position.y,
                                   target.pose.position.x-map_pose.position.x)
            if abs(math.remainder(evidence['bearing']-direction, 2*math.pi)) > .35:
                return  # Another visible instance cannot replace the confirmed one.
        q = map_pose.orientation
        map_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        info = self.occupancy.info
        grid = np.asarray(self.occupancy.data, dtype=np.int16).reshape(info.height, info.width)
        navigation={}
        bearing=evidence['bearing']
        candidate=None
        # A current supported range is useful for approaching to observe, even
        # before temporal consensus can confirm the object's identity. Reuse
        # only this exact mask's accepted measurement; never carry a range to
        # a new mask/instance or turn it into arrival evidence.
        if self.pending_confirmation is None and self.observations:
            latest=self.observations[-1]
            if latest['observation_stamp_ns']==evidence['stamp']:
                point=latest['position']
                measured_bearing=math.atan2(point[1]-robot.y,point[0]-robot.x)
                if abs(math.remainder(measured_bearing-bearing,2*math.pi))<=.35:
                    candidate=latest
                    bearing=measured_bearing
        if (str(self.get_parameter('carrier_model').value).lower()=='ackermann'
                and self.observation_minimum_turning_radius>0):
            if candidate is None:
                selected=self.vehicle_observation_pose(bearing)
            else:
                clearance=observation_clearance(self.get_parameter('camera_offset').value,
                    float(self.get_parameter('minimum_camera_target_range').value),
                    STANDOFF_ARRIVAL_TOLERANCE_M)
                selected=self.vehicle_observation_pose(bearing,tuple(candidate['position'][:2]),clearance)
            if selected is None:return
            point,heading=selected
            navigation={'navigation_heading':heading}
        else:
            candidate=None
            bearing=evidence['bearing']
            point = observation_step(grid, float(info.resolution),
                (info.origin.position.x, info.origin.position.y),
                (map_pose.position.x, map_pose.position.y), map_yaw, evidence['bearing'],
                maximum=self.visible_observation_max_m,
                accept=self.observation_pose_filter(bearing=evidence['bearing']))
        if point is None or math.hypot(point[0]-robot.x, point[1]-robot.y) < 2.:
            return
        self.visible_step_published_ns = now
        self.last_visible_goal = tuple(point)
        self.visible_approach_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': evidence['stamp'], 'point': list(point),
            'bearing': bearing, 'evidence': 'range_candidate' if candidate else 'bearing_only',
            'position_confirmed': False, 'search_confirmed': False,**navigation,
            **(dict(target_range_known=True,object_position=candidate['position'],
                    localization_source=candidate['source']) if candidate else {})})))

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

    def on_vehicle_grid(self, message, *, local=False):
        info=message.info
        previous=self.local_vehicle_grid if local else self.vehicle_grid
        if (message.header.frame_id!='spot1/odom' or not self.fresh(message,3.)
                or not math.isfinite(info.resolution) or info.resolution<=0
                or info.width<=0 or info.height<=0
                or len(message.data)!=info.width*info.height
                or not self.valid_pose(info.origin)
                or any(not -1<=v<=100 for v in message.data)
                or (previous is not None
                    and self.stamp_ns(message)<=self.stamp_ns(previous))):
            return
        if local:
            self.local_vehicle_grid=message
            return
        self.vehicle_grid=message
        self.vehicle_connectivity=None

    def observation_pose_filter(self, *, target=None, bearing=None, include_rejected=True):
        """Filter proposals; the bridge still requires a checked Smac path."""
        now=time.monotonic()
        self.rejected_observation_poses=deque(
            (entry for entry in self.rejected_observation_poses if entry[2]>now),maxlen=16)
        grid=self.vehicle_grid
        local=self.local_vehicle_grid
        ackermann=str(self.get_parameter('carrier_model').value).lower()!='ideal_point'
        connected=None
        if ackermann and grid is not None:
            if self.odom is None or not self.fresh(grid,3.):
                return lambda point, navigation_heading=None: False
            current=self.odom.pose.pose.position
            key=(id(grid),grid_cell(grid,(current.x,current.y)))
            if self.vehicle_connectivity is None or self.vehicle_connectivity[0]!=key:
                self.vehicle_connectivity=(key,connected_free_cells(grid,(current.x,current.y)))
            connected=self.vehicle_connectivity[1]

        def accept(point, navigation_heading=None):
            if include_rejected and any(math.dist(point,p)<radius for p,radius,_ in self.rejected_observation_poses):
                return False
            heading=navigation_heading if navigation_heading is not None else (math.atan2(target[1]-point[1],target[0]-point[0])
                     if target is not None else bearing)
            if connected is not None and not observation_pose_clear(grid,connected,point,heading):
                return False
            if ackermann and local is not None and self.fresh(local,1.5) and heading is not None:
                # The controller's rolling map is authoritative where it
                # covers this pose. Distant destinations still use the global
                # map; a local-map edge is not an obstacle in the world.
                c,s=math.cos(heading),math.sin(heading)
                corners=[(point[0]+c*x-s*y,point[1]+s*x+c*y)
                    for x in (SBR_FRONT_EXTENT+.01,-SBR_REAR_EXTENT-.01)
                    for y in (SBR_HALF_WIDTH+.01,-SBR_HALF_WIDTH-.01)]
                if all(grid_cell(local,p) is not None for p in corners):
                    return (0<=cell_cost(local,*point)<99
                            and not footprint_blockers(local,point,heading,limit=1))
            return True
        return accept

    def on_navigation_plan_status(self, message):
        try:
            data=json.loads(message.data)
            stamp=int(data['stamp_ns']);now=self.get_clock().now().nanoseconds
            state=data.get('status')
            if (data.get('query')!=self.active_query
                    or stamp<self.query_started_ns
                    or not 0<=now-stamp<=2_000_000_000
                    or state not in ('visible_target_approach',
                                     'visible_target_no_vehicle_path','confirmed_approach_failed')
                    or (state=='confirmed_approach_failed'
                        and data.get('reason')!='no_vehicle_path')):
                return
            failed=tuple(map(float,data['goal']))
            if len(failed)!=2 or not all(map(math.isfinite,failed)):
                return
            if state=='visible_target_approach':
                # Proposals continue while the bridge follows its accepted
                # step. Remember the actual accepted goal independently.
                if stamp>max(self.accepted_visible_stamp_ns,self.last_navigation_failure_ns):
                    self.accepted_visible_goal=failed
                    self.accepted_visible_stamp_ns=stamp
                return
            visible=state=='visible_target_no_vehicle_path'
            previous=self.accepted_visible_goal if visible else self.last_goal
            instance_id = data.get('instance_id') if not visible else None
            anchor = self.confirmed_instance_anchor or {}
            matched_instance = (isinstance(instance_id, str) and bool(instance_id)
                                and instance_id == anchor.get('instance_id'))
            if instance_id and not matched_instance:
                return
            # last_goal is the latest PROPOSAL. The bridge can still be
            # executing an older accepted waypoint for this same instance.
            # Its correlated failure must blacklist that accepted point and
            # recompute from the current pose/map, rather than being dropped.
            if (stamp<=self.last_navigation_failure_ns
                    or (visible and stamp<self.accepted_visible_stamp_ns)
                    or (not matched_instance and (previous is None
                        or math.dist(previous,failed)>STANDOFF_ARRIVAL_TOLERANCE_M))):
                return
        except (ValueError,TypeError,KeyError,OverflowError):
            return
        self.last_navigation_failure_ns=stamp
        self.rejected_observation_poses.append(
            (failed,2. if visible else 2*STANDOFF_ARRIVAL_TOLERANCE_M,time.monotonic()+15.))
        if visible:
            self.accepted_visible_goal=None
            self.visible_step_published_ns=0
            self.publish_visible_step()
        elif self.active_approach is not None:
            target,confirmation=self.active_approach
            if (confirmation.get('query')==self.active_query
                    and self.publish_standoff(target, publish_pose=False) is None and self.last_goal is not None):
                self.publish_approach_refresh(confirmation)

    def on_query(self, message):
        query = " ".join(message.data.casefold().split())
        if not query or query == self.active_query:
            return
        self.active_query = query
        self.rejected_observation_poses.clear()
        self.last_navigation_failure_ns=0
        self.last_visible_goal=None
        self.last_standoff_navigation_heading=None
        self.accepted_visible_goal=None
        self.accepted_visible_stamp_ns=0
        self.navigation_region_backoff.clear()
        self.visible_target = None
        self.visible_target_count = 0
        self.visible_step_published_ns = 0
        self.observations.clear()
        self.replacement_observations.clear()
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
                    if (anchor is not None and (data.get('navigation_failed') is True
                            or data.get('reason') == 'instance_superseded')):
                        # Route failure is not semantic absence. Temporarily
                        # leave this instance available for later revisits,
                        # while frontier exploration gets a chance to advance.
                        self.navigation_region_backoff.append(dict(
                            position=tuple(anchor['object_position'][:2]),
                            radius=float(anchor.get('radius', 0.)),
                            until=time.monotonic()+60.))
                    self.observations.clear()
                    self.replacement_observations.clear()
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

    def reject_localization_evidence(self, stamp, reason, replacement=None):
        """Revoke evidence, retaining route ownership until bounded recovery."""
        self.last_localization_ns = stamp
        self.observations.clear()
        if reason != 'instance_mismatch':
            self.replacement_observations.clear()
        self.pending_confirmation = self.pending_status = self.active_approach = None
        anchor = self.confirmed_instance_anchor or {}
        self.confirmation_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': 'spot1/odom',
            'observation_stamp_ns': stamp, 'evidence_rejected': True,
            'localization_valid': False, 'search_confirmed': False,
            'instance_id': anchor.get('instance_id', ''), 'reason': reason,
            **({'replacement_candidate': replacement} if replacement is not None else {})})))

    def supported_replacement(self, data):
        """Keep disjoint evidence separate; it can request release, never arrival."""
        stamp = data['observation_stamp_ns']
        self.replacement_observations = [item for item in
            (self.replacement_observations + [deepcopy(data)])[-10:]
            if 0 <= stamp-item['observation_stamp_ns'] <= self.confirmation_window_sec*1e9]
        values = np.asarray([item['position'] for item in self.replacement_observations])
        clusters = supported_position_clusters(values,
            float(self.get_parameter('confirmation_radius').value),
            required_cluster_support(int(self.get_parameter('confirmation_frames').value)))
        # A larger old cluster must not borrow the latest disjoint frame's age.
        clusters = [cluster for cluster in clusters if cluster[2][-1]]
        selected = select_target_cluster(clusters)
        if selected is None:
            return None
        count, centre, inliers = selected
        return dict(position=centre.tolist(), inliers=count,
            observations=len(values), observation_stamp_ns=stamp,
            radius=float(np.max(np.linalg.norm(values[inliers, :2]-centre[:2], axis=1))))

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
        self.navigation_region_backoff = deque(
            (p for p in self.navigation_region_backoff if time.monotonic() < p['until']), maxlen=16)
        if any(math.hypot(values[0]-p['position'][0], values[1]-p['position'][1])
               <= 3. + p['radius']
               for p in self.navigation_region_backoff):
            self.last_localization_ns = stamp
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
                    self.reject_localization_evidence(stamp, 'instance_mismatch',
                                                     self.supported_replacement(data))
                    return
        self.replacement_observations.clear()
        self.last_localization_ns = stamp
        self.observations.append(data)
        self.observations = [item for item in self.observations[-10:]
                             if stamp - item['observation_stamp_ns']
                             <= int(self.confirmation_window_sec * 1e9)]
        frames = int(self.get_parameter('confirmation_frames').value)
        radius = float(self.get_parameter('confirmation_radius').value)
        minimum_support = required_cluster_support(frames)
        if len(self.observations) < minimum_support:
            self.publish_visible_step()
            return
        values = np.asarray([item['position'] for item in self.observations])
        clusters = supported_position_clusters(values, radius, minimum_support)
        if not clusters:
            self.publish_visible_step()
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
        within_arrival = False
        if self.odom is not None and self.fresh(self.odom, 1.):
            pose = self.odom.pose.pose
            q = pose.orientation
            heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            within_arrival = within_body_arrival(vehicle_body_distance(
                (pose.position.x, pose.position.y), heading,
                confirmation['object_position'],
                str(self.get_parameter('carrier_model').value).strip().lower()))
        # Completion needs no new waypoint, especially one behind the vehicle
        # produced by the nominal standoff ring after it is already close.
        reason = None if within_arrival else self.publish_standoff(target, publish_pose=False)
        ready = not within_arrival and reason is None and self.last_goal is not None
        confirmation['instance_id'] = (self.confirmed_instance_anchor.get('instance_id',
                f'{self.active_query}:{self.confirmed_instance_anchor["observation_stamp_ns"]}')
            if self.confirmed_instance_anchor else f'{self.active_query}:{stamp}')
        status = (ready, reason, self.last_goal, within_arrival)
        if force or status != self.pending_status:
            self.confirmation_pub.publish(String(data=json.dumps({**confirmation,
                # Semantic/geometric consensus is independent of route generation.
                # The bridge measures current body clearance before completion.
                'search_confirmed': True, 'approach_goal_ready': ready,
                'standoff': list(self.last_goal) if ready else None,
                'standoff_object_distance_m': (self.last_standoff_object_distance
                                               if ready else None),
                'standoff_clamped': (self.last_standoff_clamped if ready else None),
                'standoff_intermediate': (self.last_standoff_object_distance
                    > self.approach_distance()[0] + .2 if ready else None),
                'standoff_navigation_heading': (self.last_standoff_navigation_heading if ready else None),
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
        reason = self.publish_standoff(target, publish_pose=False)
        if (reason is not None or self.last_goal is None
                or math.hypot(self.last_goal[0] - previous_goal[0],
                              self.last_goal[1] - previous_goal[1]) < update_distance):
            return
        self.publish_approach_refresh(confirmation)

    def publish_approach_refresh(self, confirmation):
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
            standoff_intermediate=(self.last_standoff_object_distance > self.approach_distance()[0]+.2),
            standoff_navigation_heading=self.last_standoff_navigation_heading,
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

    def publish_standoff(self, target, *, publish_pose=True):
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
        if (self.get_parameter('graph_destination_approach').value
                and math.hypot(stable[0]-robot.x,stable[1]-robot.y)>self.visible_observation_max_m+standoff):
            # A confirmed remote destination belongs to GraphNav. Its normal
            # incomplete-route branch selects frontiers toward this goal;
            # Nav2 still admits only executable paths in the current maps.
            candidate=(x,y)
            if self.last_goal is not None and math.dist(candidate,self.last_goal)<float(self.get_parameter('min_update_distance').value):
                return
            # retry_confirmation publishes this destination with query and
            # instance identity. Do not race it through the legacy bare pose.
            self.last_goal=candidate;self.last_standoff_clamped=False
            self.last_standoff_navigation_heading=None
            self.last_standoff_object_distance=math.hypot(stable[0]-x,stable[1]-y)
            return
        accept=self.observation_pose_filter(target=stable[:2])
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
                maintain_distance=True,
                accept=lambda p: accept(tuple(np.asarray((map_robot.x,map_robot.y))
                    +(rotation@np.array([p[0],p[1],0.]))[:2])))
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
        navigation_heading=None
        if (str(self.get_parameter('carrier_model').value).lower()=='ackermann'
                and self.observation_minimum_turning_radius>0
                and (not accept(candidate) or (clamped
                    and math.hypot(stable[0]-x,stable[1]-y)>standoff+.2))):
            # A scan-clamped point is an intermediate navigation seed. It
            # does not need to face the object before the car can get there.
            # Preserve the final viewpoint rule and its body/camera clearance.
            bearing=math.atan2(stable[1]-robot.y,stable[0]-robot.x)
            selected=self.vehicle_observation_pose(bearing,tuple(stable[:2]),standoff)
            if selected is not None:
                candidate,navigation_heading=selected
                x,y=candidate;clamped=True
        # A vehicle stage is checked at its arrival yaw. Visual bearing is
        # not an additional terminal-heading constraint on that same point.
        accepted=accept(candidate,navigation_heading)
        if not accepted:
            # A blocked direct ray does not rule out staged observation.
            # Reuse the bounded, measured-free observation selector; GraphNav
            # and Smac still decide whether this vehicle can execute the pose.
            candidate = None
            if (self.occupancy is not None and self.occupancy_odom is not None
                    and self.fresh(self.occupancy, 1.)):
                map_pose = self.occupancy_odom.pose.pose
                if math.hypot(robot.x-map_pose.position.x, robot.y-map_pose.position.y) <= 1.:
                    q = map_pose.orientation
                    heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    info = self.occupancy.info
                    grid = np.asarray(self.occupancy.data, dtype=np.int16).reshape(info.height, info.width)
                    bearing = math.atan2(stable[1]-map_pose.position.y, stable[0]-map_pose.position.x)
                    candidate = observation_step(grid, float(info.resolution),
                        (info.origin.position.x, info.origin.position.y),
                        (map_pose.position.x, map_pose.position.y), heading, bearing,
                        maximum=self.visible_observation_max_m, accept=accept)
            if candidate is None:
                self.last_goal=None
                return 'no_reachable_standoff'
            x, y = candidate
            clamped = True
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
            # Retaining a point also retains the heading belonging to it.
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
        heading = (navigation_heading if navigation_heading is not None
                   else math.atan2(stable[1] - y, stable[0] - x))
        output.pose.orientation.z = math.sin(heading / 2)
        output.pose.orientation.w = math.cos(heading / 2)
        # Confirmed approaches enter through the query/instance payload.
        # A simultaneous unlabelled pose would reset that route's ownership.
        if publish_pose:
            self.publisher.publish(output)
        self.last_goal = candidate
        self.last_standoff_navigation_heading=navigation_heading
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
