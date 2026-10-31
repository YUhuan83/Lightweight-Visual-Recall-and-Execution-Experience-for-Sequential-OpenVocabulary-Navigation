import rclpy
import sqlite3
import json
from copy import deepcopy
from collections import deque
from time import time as wall_time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image as ImageMsg, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from graphnav_msgs.msg import NavigationGraph, KeyValue, HeadingCandidate, TraversalFeedback
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from object_search_msgs.msg import ObjectMaskWithTf

from pathlib import Path
from omegaconf import OmegaConf
import cv2
import numpy as np
import torch
from torchvision import transforms

from visual_navigation.utils.tf_lookup_sub import TFEdge, TFLookupSubscriber
from visual_navigation.wildos.goalagnostic_scoring import GoalAgnosticScoring
from visual_navigation.wildos.visual_memory import VisualMemory
from visual_navigation.wildos.visual_memory_map import VisualMemoryMap
from visual_navigation.wildos.frontier_history import RecentFrontierHistory
from visual_navigation.wildos.feedback_matching import select_feedback_sample
from visual_navigation.wildos.remembered_goal import observed_viewpoint_pose
from visual_navigation.wildos.viewpoint_memory import ViewpointMemory
from visual_navigation.geofrontier_nav.geofrontier_to_image import GeoFrontierToImage
from visual_navigation.wildos.viz import VisualizeGoalAgnosticGeoFrontierScoring
from explorfm import ExploRFMInference
from visual_navigation.utils.local_models import model_paths
from visual_navigation.utils.object_search_utils import localize_query, get_objectmask_msg

HOME_DIR = Path.home()
CAMERA_MAPPING = {
    0: "front",
    1: "left",
    2: "right"
}    

class WildOS_Nav(TFLookupSubscriber):
    default_config = {
        # Model Params
        "frontier_ckpt": "frontier_head.ckpt",
        "traversability_ckpt": "trav_head.ckpt",
        "model_version": "c-radio_v3-b",
        "adaptor_version": None,
        "use_naclip": True,
        "use_summary_for_spatial": True,
        "radio_dim": 768,
        "static_scale_factor": 0.75,
        "model_precision": "FP16",

        # Nav Params
        "num_cameras": 3,
        "cams_inverted": True,
        "num_angular_bins": 16,
        "reach_in_2D": True,

        # Nav graph params
        "frontiers_range": 12.0,
        "traversability_class": "default",
        "heading_sim_thresh": 0.866,  # cosine similarity threshold between camera and frontier heading
        "default_max_score": 0.5,
        "std_for_default_scores": 30.0,  # degrees
        "std_for_frontier_heading": 30.0,  # degrees
        "min_frontier_separation": 0.5,  # meters
        "removed_frontier_ttl": 300.0,
        "removed_frontier_max_entries": 512,
        "frontier_rescore_interval": 5.0,

        # Pixel Scoring Params
        "frontier_threshold": 0.6,
        "frontier_opening_kernel_size": 0,
        "traversability_threshold": 0.8,
        "frontier_w": 2.0,
        "goal_w": 3.0,
        "reachability_w": 2.0,
        "scoring_method": "ADDITIVE",
        "reach_scale": 0.25,
        "compute_paths": True,

        # Feedback-backed visual memory stores pooled RADIO descriptors and
        # only biases heading scores when a similar observation recurs.
        "visual_memory_enabled": True,
        "visual_memory_path": ":memory:",
        "visual_memory_similarity_threshold": 0.82,
        "visual_memory_score_weight": 0.3,
        "visual_memory_max_records": 4096,
        "visual_memory_pending_ttl": 60.0,
        "traversal_feedback_topic": "/spot1/traversal_feedback",
        "visual_memory_map_enabled": True,
        "visual_memory_map_path": ":memory:",
        "visual_memory_map_id": "default",
        "visual_memory_map_save_images": False,
        "visual_memory_map_image_root": "~/.ros/wildos_memory_images",
        "object_search_query_topic": "/spot1/object_search/query",
        "object_search_waypoint_topic": "/spot1/imgnav_waypoint",
        "remembered_object_goal_topic": "/spot1/remembered_object_goal",
        "remembered_object_candidates_topic": "/spot1/remembered_object_candidates",
        "publish_legacy_remembered_goal": False,
        "object_search_memory_capture_ttl": 5.0,
        "object_search_feature_memory_ttl": 1.0,
        "object_search_feature_memory_max_translation": 0.2,
        "object_search_feature_memory_max_rotation_deg": 8.0,
        "keyframe_min_translation": 3.0,
        "keyframe_min_rotation_deg": 25.0,
        "keyframe_min_embedding_distance": 0.18,
        "node_observation_min_embedding_distance": 0.18,
        "frontier_uncertainty_beta": 0.35,
        "heading_turn_cost_weight": 0.15,
        "heading_dynamic_risk_weight": 0.5,
        "heading_low_confidence_weight": 0.5,
        "object_memory_heading_weight": 0.2,
        "search_coverage_weight": 0.2,
        "negative_search_interval": 5.0,
        "object_confirmation_observations": 2,
        "object_independent_heading_bins": 2,
        "object_prototype_novelty_threshold": 0.95,

        # ROS2 frames and topics
        "parent_frame": "spot1/odom",
        "cam_frame": "spot1/realsense/{}_color_optical_frame",
        "camera_img_topic": "/spot1/realsense/{}/color/image_raw/compressed",
        "camera_info_topic": "/spot1/realsense/{}/color/camera_info",
        "odometry_topic": "/spot1/odom",
        "navigation_graph_topic": "/spot1/nav_graph",
        "sync_navgraph_with_sensors": False,
        "navigation_graph_max_age_sec": 5.0,

        # ROS2 Publisher topics
        "scored_navgraph_topic": "/spot1/scored_nav_graph",
        "model_viz_topic": "model_visualization",
        "valid_geofrontiers_topic": "within_range_geofrontiers",
        "score_ring_topic": "/spot1/score_rings",
        "graph_viz_topic": "/spot1/navgraph_viz",

        # ROS2 subscriber params
        "qos_history_depth": 1,
        "syncsub_queue_size": 1,
        "syncsub_slop": 0.2,

        # Object Search Params
        "object_search_config": {
            # "text_queries": ["NASA logo"],
            "text_queries": ["orange flag"],
            # "text_queries": ["golf cart"],
            # "text_queries": ["garbage container"],
            "pixel_level_seg": False,
            "mask_threshold": 0.09,#0.08,
            "obj_frontier_score": 0.9,
            "obj_trav_score": 0.9
        },

        # TFLookup Config
        "tf_lookup_config": {
            "buffer_size": 1,       # number of messages
            "cache_time": 10,       # seconds
            "timer_duration": 0.5,  # seconds
            "lookup_timeout": 0,     # seconds
            "qos_history_depth": 1,  # depth for QoS profile
            "wait_for_oldest": True,  # whether to wait when buffer is full
            "clear_buffer_on_process": True,  # whether to clear buffer after processing
            "spin_thread": False,     # whether to spin tf listener in a separate thread
        }
    }

    def __init__(self, config: OmegaConf=OmegaConf.create(), do_object_search=False):
        config = OmegaConf.merge(OmegaConf.create(self.default_config), config)

        # init model before to prevent tf listener from spinning too early
        np.random.seed(42)
        self.init_model(config, do_object_search)

        super().__init__(
            node_name='wildos',
            config=config.tf_lookup_config
        )
        self.get_logger().info('Finished initializing models!')
        if self.object_search_mode:
            self.get_logger().info(f'Computed Text Feats for {self.text_queries}!')

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.visual_memory = None
        self.visual_memory_map = None
        self.last_node_observation = None
        self.pending_memory_samples = deque(maxlen=64)
        self.visual_memory_score_weight = float(config.visual_memory_score_weight)
        self.visual_memory_pending_ttl = float(config.visual_memory_pending_ttl)
        self.frontier_uncertainty_beta = float(config.frontier_uncertainty_beta)
        self.heading_turn_cost_weight = float(config.heading_turn_cost_weight)
        self.heading_dynamic_risk_weight = float(config.heading_dynamic_risk_weight)
        self.heading_low_confidence_weight = float(config.heading_low_confidence_weight)
        self.object_memory_heading_weight = float(config.object_memory_heading_weight)
        self.search_coverage_weight = float(config.search_coverage_weight)
        self.negative_search_interval = max(0.0, float(config.negative_search_interval))
        self.last_negative_search_at = float("-inf")
        if config.visual_memory_enabled:
            try:
                self.visual_memory = VisualMemory(
                    database_path=config.visual_memory_path,
                    encoder_id=f"{config.model_version}:{config.adaptor_version or 'default'}",
                    map_id=str(config.visual_memory_map_id),
                    max_records=config.visual_memory_max_records,
                    similarity_threshold=config.visual_memory_similarity_threshold,
                )
                scope = "in-process" if config.visual_memory_path == ":memory:" else "on-disk"
                self.get_logger().info(f"Feedback-backed visual memory enabled ({scope}).")
            except (OSError, ValueError, sqlite3.Error) as exc:
                self.get_logger().warning(f"Visual memory disabled: {exc}")
        self.memory_map_id = str(config.visual_memory_map_id)
        self.memory_map_save_images = bool(config.visual_memory_map_save_images)
        self.memory_map_image_root = Path(config.visual_memory_map_image_root).expanduser()
        # Kept under the old config name for deployment compatibility; it now
        # controls repeated observations of the same graph node/heading bin.
        self.node_observation_min_embedding_distance = float(
            config.get("node_observation_min_embedding_distance",
                       config.keyframe_min_embedding_distance))
        if config.visual_memory_map_enabled:
            try:
                self.visual_memory_map = VisualMemoryMap(
                    database_path=config.visual_memory_map_path,
                    encoder_id=f"{config.model_version}:{config.adaptor_version or 'default'}",
                    object_confirmation_observations=config.object_confirmation_observations,
                    object_independent_heading_bins=config.object_independent_heading_bins,
                    object_heading_bin_count=config.num_angular_bins,
                    object_prototype_novelty_threshold=config.object_prototype_novelty_threshold,
                )
                self.visual_memory_map.register_map(self.memory_map_id, self.memory_map_id)
                self.get_logger().info(
                    f"Visual memory map enabled: map_id={self.memory_map_id}")
            except (OSError, ValueError, sqlite3.Error) as exc:
                self.get_logger().warning(f"Visual memory map disabled: {exc}")

        # Nav parameters and initializations
        self.num_cameras = config.num_cameras
        self.cam_inverted = config.cams_inverted
        assert self.num_cameras in [1, 3], "Only 1 or 3 cameras are supported."
        self.num_angular_bins = config.num_angular_bins
        self.reach_in_2D = config.reach_in_2D
        
        # Store current frontier nodes and their scores
        self.frontier_uuid_to_scores = {}
        self.frontier_score_stats = {}
        self.frontier_uuid_to_scoring_distance = {}
        self.frontier_last_scored_at = {}
        self.frontier_rescore_interval = max(0.0, float(config.frontier_rescore_interval))
        self.recent_frontiers = RecentFrontierHistory(
            config.removed_frontier_ttl, config.removed_frontier_max_entries)

        # Navgraph frontiers to img
        self.geofrontier_to_image = GeoFrontierToImage(
            camera_mapping=CAMERA_MAPPING,
            frontiers_range=config.frontiers_range,
            traversability_class=config.traversability_class,
            cams_inverted=self.cam_inverted,
            heading_sim_thresh=config.heading_sim_thresh,
            reach_in_2D=False
        )
        self.traversability_class = config.traversability_class
        self.default_max_score = config.default_max_score
        self.std_for_default_scores = config.std_for_default_scores
        self.std_for_frontier_heading = config.std_for_frontier_heading
        self.min_frontier_separation = config.min_frontier_separation

        # Initialize pixel scoring params
        frontier_threshold = config.frontier_threshold
        traversability_threshold = config.traversability_threshold
        pixel_scoring_params = {
            "frontier": config.frontier_w,
            "goal": config.goal_w,
            "reachability": config.reachability_w,
            "method": config.scoring_method,
        }
        reach_scale = config.reach_scale
        self.scorer = GoalAgnosticScoring(
            num_angular_bins=config.num_angular_bins,
            pixel_scoring_params=pixel_scoring_params,
            frontier_threshold=frontier_threshold,
            frontier_opening_kernel_size=config.frontier_opening_kernel_size,
            traversability_threshold=traversability_threshold,
            reach_in_2D=self.reach_in_2D,
            cam_inverted=self.cam_inverted,
            reach_scale=reach_scale
        )
        self.compute_paths = config.compute_paths

        # Visualization
        self.geofrontier_viz_colors = np.array([
            [0.528, 0.471, 0.701],
            [0.772, 0.432, 0.102],
            [0.572, 0.586, 0.0],
        ])
        self.viz = VisualizeGoalAgnosticGeoFrontierScoring(
            angular_bins=self.scorer.angles,
            camera_mapping=CAMERA_MAPPING,
            num_cameras=self.num_cameras
        )

        self.clbk_cntr = 0

        # Frames and Topic Names
        self.global_frame = config.parent_frame
        self.cam_tf_frame = config.cam_frame
        self.using_compressed_imgs = "compressed" in config.camera_img_topic
        self.object_search_feature_memory_ttl = max(
            0.0, float(config.object_search_feature_memory_ttl))
        self.object_search_feature_memory_max_translation = max(
            0.0, float(config.object_search_feature_memory_max_translation))
        self.object_search_feature_memory_max_rotation = np.deg2rad(max(
            0.0, float(config.object_search_feature_memory_max_rotation_deg)))
        self.last_scene_features = None
        self.latest_odom_msg = None
        self.object_goal_memory = ViewpointMemory(bin_count=self.num_angular_bins)
        self.object_search_memory_capture_ttl = max(
            0.0, float(config.object_search_memory_capture_ttl))
        self.last_positive_mask_query = None
        self.last_positive_mask_at = 0.0
        self.last_positive_mask_odom = None

        # TFLookup
        self.required_transforms = {
            f"world_from_cam{idx}": TFEdge(
                source_frame=self.cam_tf_frame.format(CAMERA_MAPPING[idx]),
                target_frame=self.global_frame
            )
            for idx in range(self.num_cameras)
        }

        # Subscribers and Publishers
        self.init_publishers(config)
        self.init_subscribers(config)
        self.start_timer()

    def init_model(self, config, do_object_search):
        # vlm initializations
        self.device = "cuda"

        if do_object_search and config.adaptor_version is None:
            config.adaptor_version = "siglip2"

        ckpt_root, model_version, adaptor_path = model_paths(config)
        # radio model
        self.model = ExploRFMInference(
            frontier_ckpt=ckpt_root / config.frontier_ckpt,
            traversability_ckpt=ckpt_root / config.traversability_ckpt,
            model_version=model_version,
            adaptor_version=config.adaptor_version,
            adaptor_ckpt_path=adaptor_path,
            use_naclip=config.use_naclip,
            use_summary_for_spatial=config.use_summary_for_spatial,
            radio_dim=config.radio_dim,
            static_scale_factor=config.static_scale_factor,
            model_precision=config.model_precision,
        )
        self.transforms = transforms.Compose([
            transforms.ToTensor(),
        ])
        self.object_search_mode = False
        self.search_target_cache = {}

        if do_object_search:
            self.object_search_mode = True
            self.text_queries = [str(query).strip() for query in config.object_search_config.text_queries
                                 if str(query).strip()]
            self.pixel_level_seg = config.object_search_config.pixel_level_seg
            self.mask_threshold = config.object_search_config.mask_threshold
            self.obj_frontier_score = config.object_search_config.obj_frontier_score
            self.obj_trav_score = config.object_search_config.obj_trav_score

            assert self.text_queries, "At least one object-search target is required."
            text_features = self.model.forward_on_text(self.text_queries)
            self.search_target_cache = {
                query: text_features[index:index + 1]
                for index, query in enumerate(self.text_queries)
            }
            self.text_queries = [self.text_queries[0]]
            self.text_feats = self.search_target_cache[self.text_queries[0]]


    def init_publishers(self, config: OmegaConf):
        self.scored_navgraph_pub = self.create_publisher(
            NavigationGraph,
            config.scored_navgraph_topic,
            10
        )
        self.model_viz_pub = self.create_publisher(
            ImageMsg,
            config.model_viz_topic,
            10
        )
        self.withinrange_geofront_pub = self.create_publisher(
            MarkerArray,
            config.valid_geofrontiers_topic,
            10
        )
        self.score_rings_pub = self.create_publisher(
            MarkerArray,
            config.score_ring_topic,
            10
        )
        self.navgraph_vis_pub = self.create_publisher(
            MarkerArray,
            config.graph_viz_topic,
            10
        )
        if self.object_search_mode:
            self.object_mask_publisher = self.create_publisher(
                ObjectMaskWithTf,
                config.get("object_mask_topic", "/spot1/object_mask"),
                10
            )
            self.remembered_object_goal_publisher = self.create_publisher(
                PoseStamped,
                str(config.get("remembered_object_goal_topic", "/spot1/remembered_object_goal")),
                10,
            )
            self.remembered_candidates_publisher = self.create_publisher(
                String, str(config.remembered_object_candidates_topic), 10)
            self.publish_legacy_remembered_goal = bool(config.publish_legacy_remembered_goal)
            self.create_subscription(String, '/spot1/remembered_object_feedback',
                                     self.remembered_candidate_feedback, 10)

    def init_subscribers(self, config):

        cameraimg_topic_str = config.camera_img_topic
        camerainfo_topic_str = config.camera_info_topic
        img_msg_type = CompressedImage if self.using_compressed_imgs else ImageMsg

        self.camera_subs = {}
        for i in range(self.num_cameras):
            self.camera_subs[i] = {
                "image": Subscriber(self, img_msg_type, cameraimg_topic_str.format(CAMERA_MAPPING[i]), qos_profile=config.qos_history_depth),
                "info": Subscriber(self, CameraInfo, camerainfo_topic_str.format(CAMERA_MAPPING[i]), qos_profile=config.qos_history_depth)
            }
        self.odom_sub = Subscriber(self, Odometry, config.odometry_topic, qos_profile=config.qos_history_depth)
        self.latest_navgraph = None
        self.latest_navgraph_at = 0.0
        self.navigation_graph_max_age_sec = float(config.navigation_graph_max_age_sec)
        ts_subs = [self.odom_sub]
        if config.sync_navgraph_with_sensors:
            self.navgraph_sub = Subscriber(self, NavigationGraph, config.navigation_graph_topic,
                                          qos_profile=config.qos_history_depth)
            ts_subs.append(self.navgraph_sub)
        else:
            # A navigation graph is an accumulated world snapshot. Its mapping
            # work can finish later than a synchronized camera/odom sample.
            self.navgraph_sub = self.create_subscription(NavigationGraph,
                config.navigation_graph_topic, self.on_navgraph_snapshot, 1)
        for i in range(self.num_cameras):
            ts_subs.append(self.camera_subs[i]["image"])
            ts_subs.append(self.camera_subs[i]["info"])

        self.ts = ApproximateTimeSynchronizer(
            ts_subs, queue_size=config.syncsub_queue_size, slop=config.syncsub_slop
        )
        self.ts.registerCallback(self.listener_callback if config.sync_navgraph_with_sensors
                                 else self.sensor_sync_callback)
        if self.visual_memory is not None:
            self.feedback_sub = self.create_subscription(
                TraversalFeedback,
                config.traversal_feedback_topic,
                self.traversal_feedback_callback,
                10,
            )
        if self.object_search_mode:
            self.search_query_sub = self.create_subscription(
                String,
                str(config.get("object_search_query_topic", "/spot1/object_search/query")),
                self.search_query_callback,
                10,
            )
            self.object_search_waypoint_sub = self.create_subscription(
                PoseStamped,
                str(config.get("object_search_waypoint_topic", "/spot1/imgnav_waypoint")),
                self.object_search_waypoint_callback,
                10,
            )

    def on_navgraph_snapshot(self, message):
        if message.header.frame_id == self.global_frame:
            self.latest_navgraph = message
            self.latest_navgraph_at = wall_time()

    def sensor_sync_callback(self, odom_msg, *camera_messages):
        if (self.latest_navgraph is None or
                wall_time() - self.latest_navgraph_at > self.navigation_graph_max_age_sec):
            return
        self.listener_callback(odom_msg, deepcopy(self.latest_navgraph), *camera_messages)

    def search_query_callback(self, message):
        query = " ".join(str(message.data).split())
        if not query:
            self.get_logger().warning("Ignoring an empty object-search query.")
            return
        if query == self.text_queries[0]:
            # A repeated command should still replay an approach remembered by
            # this process, for example after a navigation retry.
            if self._query_memory_key(query) in self.object_goal_memory:
                self._reuse_remembered_object_goal(query)
            return

        text_features = self.search_target_cache.get(query)
        if text_features is None:
            try:
                text_features = self.model.forward_on_text([query])
            except Exception as exc:
                self.get_logger().error(f"Could not encode object-search query {query!r}: {exc}")
                return
            self.search_target_cache[query] = text_features
            cache_status = "encoded"
        else:
            cache_status = "cached"

        previous = self.text_queries[0]
        self.text_queries = [query]
        self.text_feats = text_features
        self.last_negative_search_at = float("-inf")
        self.frontier_uuid_to_scores.clear()
        self.frontier_score_stats.clear()
        self.frontier_uuid_to_scoring_distance.clear()
        self.frontier_last_scored_at.clear()
        self.last_positive_mask_query = None
        self.last_positive_mask_at = 0.0
        self.last_positive_mask_odom = None
        self.get_logger().info(
            f"Object-search target changed: {previous!r} -> {query!r} ({cache_status} text features).")
        self._reuse_recent_scene_features(query)
        self._reuse_remembered_object_goal(query)

    @staticmethod
    def _query_memory_key(query):
        return " ".join(str(query).casefold().split())

    def object_search_waypoint_callback(self, message):
        query = self.text_queries[0] if self.text_queries else None
        if (query is None or self.last_positive_mask_query != query or
                wall_time() - self.last_positive_mask_at > self.object_search_memory_capture_ttl):
            return
        observer = self.last_positive_mask_odom
        observed_pose = observed_viewpoint_pose(observer, message.header.frame_id)
        if observed_pose is None:
            self.get_logger().warning(
                "Cannot remember object viewpoint: invalid observation pose or frame.")
            return
        # The triangulator publishes the object's occupied position with a
        # neutral yaw. Remember the vehicle pose that actually saw the object:
        # it was traversed, has a meaningful viewing heading, and can be
        # replanned to from a later location without retracing the old route.
        viewpoint = PoseStamped()
        viewpoint.header.frame_id = observer.header.frame_id
        viewpoint.header.stamp = observer.header.stamp
        viewpoint.pose = observed_pose
        q = viewpoint.pose.orientation
        heading = float(np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
        p = viewpoint.pose.position
        self.object_goal_memory.observe(query, viewpoint.header.frame_id,
                                        (p.x, p.y, p.z), heading, wall_time())
        self.get_logger().info(
            f"Remembered in-process observed viewpoint for {query!r} at "
            f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) in {message.header.frame_id}.")

    def _reuse_remembered_object_goal(self, query):
        views = self.object_goal_memory.candidates(query, wall_time())
        # Publish an empty set too: the bridge can release an old memory goal.
        self.remembered_candidates_publisher.publish(String(data=json.dumps(
            {'query': query, 'views': views})))
        if not views:
            self.get_logger().info(
                f"No in-process approach is remembered for {query!r}; "
                "continuing with the normal object-search pipeline.")
            return
        if self.publish_legacy_remembered_goal:
            view = max(views, key=lambda v: v['observed_at'])
            replay = PoseStamped()
            replay.header.frame_id = view['frame']
            replay.header.stamp = self.get_clock().now().to_msg()
            replay.pose.position.x, replay.pose.position.y, replay.pose.position.z = view['position']
            replay.pose.orientation.z = float(np.sin(view['heading'] / 2))
            replay.pose.orientation.w = float(np.cos(view['heading'] / 2))
            self.remembered_object_goal_publisher.publish(replay)
        self.get_logger().info(
            f"Published {len(views)} in-process viewpoints for {query!r}; "
            "the vehicle planner will select a reachable approach.")

    def remembered_candidate_feedback(self, message):
        try:
            feedback = json.loads(message.data)
            if feedback.get('outcome') == 'unreachable':
                self.object_goal_memory.reject(feedback['query'], feedback['id'], wall_time())
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Ignoring invalid remembered-viewpoint feedback')

    @staticmethod
    def _odom_xy_yaw(odom_msg):
        pose = odom_msg.pose.pose
        q = pose.orientation
        yaw = float(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return float(pose.position.x), float(pose.position.y), yaw

    def _reuse_recent_scene_features(self, query):
        """Evaluate a new target against a recent same-pose scene in this process."""
        cached = self.last_scene_features
        current_odom = self.latest_odom_msg
        if cached is None or current_odom is None or self.object_search_feature_memory_ttl <= 0:
            return
        age = wall_time() - cached["created_at"]
        if age < 0.0 or age > self.object_search_feature_memory_ttl:
            return

        old_x, old_y, old_yaw = self._odom_xy_yaw(cached["odom"])
        new_x, new_y, new_yaw = self._odom_xy_yaw(current_odom)
        translation = float(np.hypot(new_x - old_x, new_y - old_y))
        rotation = abs(float(np.arctan2(
            np.sin(new_yaw - old_yaw), np.cos(new_yaw - old_yaw))))
        if (translation > self.object_search_feature_memory_max_translation or
                rotation > self.object_search_feature_memory_max_rotation):
            return

        try:
            _, binary_mask = localize_query(
                text_feats=self.text_feats,
                spatial_feats=cached["spatial_feats"],
                orig_img_shape=cached["image_shape"],
                pixel_level_seg=self.pixel_level_seg,
                mask_threshold=self.mask_threshold)
        except Exception as exc:
            self.get_logger().warning(
                f"Could not reuse recent visual features for {query!r}: {exc}")
            return

        pixels = int(np.count_nonzero(binary_mask))
        if pixels:
            self.last_positive_mask_query = query
            self.last_positive_mask_at = wall_time()
            self.last_positive_mask_odom = deepcopy(cached["odom"])
            self.object_mask_publisher.publish(get_objectmask_msg(
                binary_mask, self.cam_inverted, cached["odom"],
                cached["tf_list"], cached["cam_info_msgs"], query=query))
            self.get_logger().info(
                f"Reused in-process scene features for {query!r}: "
                f"mask_pixels={pixels}, age={age:.3f}s, "
                f"pose_delta={translation:.3f}m/{np.rad2deg(rotation):.1f}deg.")
        else:
            self.get_logger().info(
                f"Reused in-process scene features for {query!r}: no mask "
                f"(age={age:.3f}s, pose_delta={translation:.3f}m/"
                f"{np.rad2deg(rotation):.1f}deg).")
    
    def listener_callback(self, odom_msg, navgraph_msg, *msgs):
        self.clbk_cntr += 1
        self.latest_odom_msg = odom_msg
        self.get_logger().info(f"Received callback {self.clbk_cntr}")

        assert odom_msg.header.frame_id == self.global_frame, \
            f"Odom frame {odom_msg.header.frame_id} does not match global frame {self.global_frame}"
        # assert navgraph_msg.header.frame_id == self.global_frame, \
        #     f"Navgraph frame {navgraph_msg.header.frame_id} does not match global frame {self.global_frame}"

        self.msg_buffer.add_msg(
            msg={
                "odom": odom_msg,
                "navgraph": navgraph_msg,
                "cam_msgs": msgs
            },
            stamp=odom_msg.header.stamp,
        )

    def do_processing(self, msg, tf_data):
        

        # Extract messages
        odom_msg = msg["odom"]
        navgraph_msg = msg["navgraph"]
        msgs = msg["cam_msgs"]

        # Extract camera images and info
        rgb_imgs, cam_info_msgs = [], []
        for i in range(self.num_cameras):
            if self.using_compressed_imgs:
                convert_func = self.br.compressed_imgmsg_to_cv2
            else:
                convert_func = self.br.imgmsg_to_cv2

            if self.cam_inverted:
                rgb_imgs.append(
                    np.rot90(convert_func(msgs[i * 2], desired_encoding='rgb8'), k=2)
                )
            else:
                rgb_imgs.append(
                    convert_func(msgs[i * 2], desired_encoding='rgb8')
                )
            cam_info_msgs.append(msgs[i * 2 + 1])

        # Get geofrontiers from navgraph_msg
        all_cam_data = []
        for i, cam_info_msg in enumerate(cam_info_msgs):
            cam_data = self.fetch_cam_intrinsics_extrinsics(cam_info_msg, tf_data[f"world_from_cam{i}"])
            all_cam_data.append(cam_data)

        try:
            geofrontiers = self.geofrontier_to_image.extract_geofrontiers(
                current_odom_msg=odom_msg,
                navgraph=navgraph_msg,
                all_cam_data=all_cam_data
            )
        except Exception as e:
            self.get_logger().error(f"Error in extracting geofrontiers: {e}")
            return

        
        # Model Forward pass
        rgb_tensors = [self.transforms(img.copy()) for img in rgb_imgs]
        batch_tensor = torch.stack(rgb_tensors)
        batch_img_traversability, batch_img_frontiers, spatial_feats = self.model.forward(batch_tensor)

        memory_embedding = self.pool_memory_embedding(spatial_feats)
        memory_prior, memory_similarity = (None, 0.0)
        if self.visual_memory is not None and memory_embedding is not None:
            memory_prior, memory_similarity = self.visual_memory.retrieve(
                memory_embedding, self.num_angular_bins
            )

        batch_img_frontiers = batch_img_frontiers.cpu().numpy().astype(np.float32)
        batch_img_traversability = batch_img_traversability.cpu().numpy().astype(np.float32)
        if self.object_search_mode:
            if self.model.model_precision.is_fp16():
                spatial_feats = spatial_feats.half()

            self.last_scene_features = {
                "spatial_feats": spatial_feats.detach(),
                "image_shape": rgb_imgs[0].shape[:2],
                "odom": odom_msg,
                "tf_list": [tf_data[f"world_from_cam{i}"] for i in range(self.num_cameras)],
                "cam_info_msgs": cam_info_msgs,
                "created_at": wall_time(),
            }

            text_sim_spatial, binary_mask = localize_query(
                text_feats=self.text_feats,
                spatial_feats=spatial_feats,
                orig_img_shape=rgb_imgs[0].shape[:2],
                pixel_level_seg=self.pixel_level_seg,
                mask_threshold=self.mask_threshold
            )

            if np.sum(binary_mask) > 0:
                self.last_positive_mask_query = self.text_queries[0]
                self.last_positive_mask_at = wall_time()
                self.last_positive_mask_odom = deepcopy(odom_msg)
                tf_list = [tf_data[f"world_from_cam{i}"] for i in range(self.num_cameras)]
                self.object_mask_publisher.publish(
                    get_objectmask_msg(binary_mask, self.cam_inverted, odom_msg, tf_list, cam_info_msgs,
                                       query=self.text_queries[0])
                )
                batch_img_frontiers = np.maximum(batch_img_frontiers, self.obj_frontier_score*binary_mask)
                batch_img_traversability = np.maximum(batch_img_traversability, self.obj_trav_score*binary_mask)
            else:
                self.last_positive_mask_query = None
                self.last_positive_mask_at = 0.0
                self.last_positive_mask_odom = None
                self.get_logger().warn("No object detected in the scene, skipping object mask publish.")
                self._record_negative_search(navgraph_msg, geofrontiers,
                                             text_sim_spatial, odom_msg)

        # Scoring Geometric Frontiers
        nav_data = []
        raw_score_rows = []
        for i in range(self.num_cameras):
            cam_data = all_cam_data[i]
            if not geofrontiers[i]:
                # no geometric frontiers for this camera
                nav_data.append({
                    "image": rgb_imgs[i],
                    "traversability": batch_img_traversability[i][0],
                    "img_frontiers": batch_img_frontiers[i][0],
                    "object_mask": binary_mask[i][0] if self.object_search_mode else None,
                })
                continue

            scores, paths, score_maps = self.scorer.score_geofrontiers(
                geometric_frontiers=geofrontiers[i]["frontier_pixel_coords"],
                img_frontiers=batch_img_frontiers[i][0],
                traversability=batch_img_traversability[i][0],
                cam_data=cam_data,
                compute_paths=self.compute_paths
            )
            scores = np.asarray(scores, dtype=np.float32)
            if scores.size:
                raw_score_rows.append(scores.copy())
            if memory_prior is not None:
                threshold = self.visual_memory.similarity_threshold
                confidence = np.clip(
                    (memory_similarity - threshold) / max(1.0 - threshold, 1e-6),
                    0.0,
                    1.0,
                )
                memory_weight = float(np.clip(self.visual_memory_score_weight, 0.0, 1.0)) * confidence
                # Memory can down-rank a direction, but cannot make a visually
                # unsupported frontier appear safe or promising.
                scores = scores * ((1.0 - memory_weight) + memory_weight * memory_prior[None, :])
            nav_data.append({
                "image": rgb_imgs[i],
                "traversability": batch_img_traversability[i][0],
                "img_frontiers": batch_img_frontiers[i][0],
                "object_mask": binary_mask[i][0] if self.object_search_mode else None,
                "score_map": score_maps,
                "geo_frontiers": geofrontiers[i]["frontier_pixel_coords"],
                "scores": scores,
                "paths": paths,
            })

        if self.visual_memory is not None and memory_embedding is not None:
            if (raw_score_rows and
                    0 <= navgraph_msg.current_node_idx < len(navgraph_msg.nodes)):
                profile = np.max(np.concatenate(raw_score_rows, axis=0), axis=0)
                self.pending_memory_samples.append({
                    "created_at": wall_time(),
                    "ros_stamp": odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9,
                    "embedding": memory_embedding,
                    "heading_scores": profile,
                    "node_uuid": self.uuid_to_str(
                        navgraph_msg.nodes[navgraph_msg.current_node_idx].uuid),
                })

        # Publish scored navgraph
        robot_pos = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ], dtype=np.float32).reshape(1, 3)
        robot_quat = odom_msg.pose.pose.orientation
        robot_yaw = float(np.arctan2(
            2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
            1.0 - 2.0 * (robot_quat.y * robot_quat.y + robot_quat.z * robot_quat.z),
        ))
        updated_navgraph, removed_uuids, updated_uuids = self.update_navgraph_with_scores(
            navgraph_msg, geofrontiers, nav_data, robot_pos, robot_yaw
        )
        self._observe_node_heading_memory(
            odom_msg, updated_navgraph, memory_embedding, rgb_imgs,
            batch_img_traversability)
        self.scored_navgraph_pub.publish(updated_navgraph)
        if self.withinrange_geofront_pub.get_subscription_count():
            self.viz.delete_markers(self.withinrange_geofront_pub)
            self.withinrange_geofront_pub.publish(
                self.viz.viz_valid_geofrontiers(
                    geofrontiers, all_cam_data, odom_msg.header, self.geofrontier_viz_colors))
        if self.model_viz_pub.get_subscription_count():
            self.model_viz_pub.publish(self.br.cv2_to_imgmsg(
                self.viz.visualize_model_det(nav_data, all_cam_data), encoding="rgb8"))
        if self.navgraph_vis_pub.get_subscription_count():
            self.navgraph_vis_pub.publish(self.viz.visualize_navgraph(
                navgraph_msg, self.global_frame, self.get_clock().now().to_msg()))
        if self.score_rings_pub.get_subscription_count():
            self.score_rings_pub.publish(self.viz.visualize_all_heading_scores(
                self.frontier_uuid_to_scores, removed_uuids, updated_uuids,
                self.global_frame, self.get_clock().now().to_msg()))

    def remove_old_frontiers(self, navgraph_msg):
        # Remove frontiers that are no longer in the navgraph
        now = wall_time()
        self.recent_frontiers.prune(now)
        trav_class_idx = navgraph_msg.trav_classes.index(self.traversability_class)
        current_uuids = set()
        for node in navgraph_msg.nodes:
            if node.trav_properties[trav_class_idx].is_frontier:
                current_uuids.add(self.uuid_to_str(node.uuid))

        old_uuids = set(self.frontier_uuid_to_scores.keys())
        removed_uuids = []
        for old_uuid in old_uuids:
            if old_uuid not in current_uuids:
                # Store removed frontier position
                del_node = self.frontier_uuid_to_scores[old_uuid][1]
                pos = (
                    del_node.pose.position.x,
                    del_node.pose.position.y,
                    del_node.pose.position.z
                )
                self.recent_frontiers.add(pos, now)

                del self.frontier_uuid_to_scores[old_uuid]
                self.frontier_score_stats.pop(old_uuid, None)
                self.frontier_uuid_to_scoring_distance.pop(old_uuid, None)
                self.frontier_last_scored_at.pop(old_uuid, None)
                removed_uuids.append(old_uuid)

        return removed_uuids

    def update_navgraph_with_scores(self, navgraph_msg, geofrontiers, nav_data, robot_pos, robot_yaw=0.0):
        removed_uuids = self.remove_old_frontiers(navgraph_msg)
        now = wall_time()
        updated_uuids = set()
        semantic_by_uuid = {}
        trav_class_idx = navgraph_msg.trav_classes.index(self.traversability_class)
        scored_navgraph = navgraph_msg
        
        for i in range(self.num_cameras):
            if not geofrontiers[i]:
                continue

            trav_map = np.asarray(nav_data[i]["traversability"], dtype=np.float32)
            image_height, image_width = nav_data[i]["image"].shape[:2]
            for frontier_node, heading, scores, pixel in zip(
                geofrontiers[i]["frontier_nodes"],
                geofrontiers[i]["frontier_headings"],
                nav_data[i]["scores"],
                geofrontiers[i]["frontier_pixel_coords"],
            ):
                uuid = self.uuid_to_str(frontier_node.uuid)
                if trav_map.size and image_height > 0 and image_width > 0:
                    py = int(np.clip(pixel[0] * trav_map.shape[0] / image_height,
                                     0, trav_map.shape[0] - 1))
                    px = int(np.clip(pixel[1] * trav_map.shape[1] / image_width,
                                     0, trav_map.shape[1] - 1))
                    patch = trav_map[max(0, py - 2):py + 3, max(0, px - 2):px + 3]
                    semantic = float(np.clip(np.mean(patch), 0.0, 1.0))
                    if np.isfinite(semantic):
                        semantic_by_uuid[uuid] = min(
                            semantic_by_uuid.get(uuid, 1.0), semantic)
                frontier_pos = np.array([
                    frontier_node.pose.position.x,
                    frontier_node.pose.position.y,
                    frontier_node.pose.position.z
                ], dtype=np.float32).reshape(1, 3)

                # Check if this frontier is too close to any removed frontier
                # set scores to zero if too close to removed frontier
                if self.recent_frontiers.nearby(frontier_pos[0],
                                                self.min_frontier_separation, now):
                    scores *= 0.0

                # only update scores if the robot is closer to the frontier than before
                node_dist = np.linalg.norm(robot_pos - frontier_pos)
                if uuid in self.frontier_uuid_to_scoring_distance:
                    prev_dist = self.frontier_uuid_to_scoring_distance[uuid]
                    if (node_dist >= prev_dist and
                            now - self.frontier_last_scored_at.get(uuid, 0.0)
                            < self.frontier_rescore_interval):
                        continue
                    self.frontier_uuid_to_scoring_distance[uuid] = min(node_dist, prev_dist)
                else:
                    self.frontier_uuid_to_scoring_distance[uuid] = node_dist
                self.frontier_last_scored_at[uuid] = now

                # Modulate scores based on heading alignment
                if self.std_for_frontier_heading is not None:
                    scores *= self.scorer.get_gauss_scores(
                        np.rad2deg(np.arctan2(heading[1], heading[0])),
                        std=self.std_for_frontier_heading,
                        max_score=1.0
                    )

                # Update scores
                updated_uuids.add(uuid)
                self._update_frontier_score(uuid, scores, frontier_node)

        scored_navgraph.header.stamp = self.get_clock().now().to_msg()
        for node in scored_navgraph.nodes:
            uuid = self.uuid_to_str(node.uuid)
            node.heading_candidates.clear()
            if uuid in self.frontier_uuid_to_scores:
                scores = self.frontier_uuid_to_scores[uuid][0].astype(np.float32)
                stats = self.frontier_score_stats.get(uuid)
                mean = stats["mean"] if stats is not None else scores
                variance = stats["m2"] / max(stats["count"] - 1, 1) if stats is not None else np.zeros_like(scores)
                std = np.sqrt(np.maximum(variance, 0.0))
                node.properties.append(KeyValue(key="frontier_scores", value=list(scores)))
                node.properties.append(KeyValue(key="frontier_score_raw", value=list(scores)))
                node.properties.append(KeyValue(key="frontier_score_mean", value=list(mean)))
                node.properties.append(KeyValue(key="frontier_score_std", value=list(std)))
                node.properties.append(KeyValue(key="frontier_score_observations", value=[float(stats["count"]) if stats else 1.0]))
                node.properties.append(KeyValue(key="is_default_scored", value=[0.0]))
                self._append_heading_candidates(
                    node, scores, robot_yaw, semantic_by_uuid.get(uuid, 0.0))

            elif node.trav_properties[trav_class_idx].is_frontier:
                # if frontier is not scored, set default scores
                scores = self.scorer.get_default_scores(
                    node, trav_class_idx, std=self.std_for_default_scores, def_max_score=self.default_max_score
                ).astype(np.float32)

                node_pos = np.array([
                    node.pose.position.x,
                    node.pose.position.y,
                    node.pose.position.z
                ], dtype=np.float32).reshape(1, 3)

                # Check if this frontier is too close to any removed frontier
                # set scores to zero if too close to removed frontier
                if self.recent_frontiers.nearby(node_pos[0],
                                                self.min_frontier_separation, now):
                    scores *= 0.0

                updated_uuids.add(uuid)
                self._update_frontier_score(uuid, scores, node, default=True)
                stats = self.frontier_score_stats[uuid]
                safe_scores = self.frontier_uuid_to_scores[uuid][0]
                std = np.sqrt(np.maximum(stats["m2"] / max(stats["count"] - 1, 1), 0.0))
                node.properties.append(KeyValue(key="frontier_scores", value=list(safe_scores)))
                node.properties.append(KeyValue(key="frontier_score_raw", value=list(scores)))
                node.properties.append(KeyValue(key="frontier_score_mean", value=list(stats["mean"])))
                node.properties.append(KeyValue(key="frontier_score_std", value=list(std)))
                node.properties.append(KeyValue(key="frontier_score_observations", value=[float(stats["count"])]))
                node.properties.append(KeyValue(key="is_default_scored", value=[1.0]))
                self._append_heading_candidates(
                    node, self.frontier_uuid_to_scores[uuid][0], robot_yaw,
                    semantic_by_uuid.get(uuid, 0.0), default=True)

        return scored_navgraph, removed_uuids, updated_uuids

    def _update_frontier_score(self, uuid, scores, node, default=False):
        """Keep raw/mean/std scores and expose a conservative planning score."""
        scores = np.asarray(scores, dtype=np.float32)
        stats = self.frontier_score_stats.get(uuid)
        if stats is None or stats["mean"].shape != scores.shape:
            stats = {"mean": scores.copy(), "m2": np.zeros_like(scores), "count": 1}
            self.frontier_score_stats[uuid] = stats
        else:
            stats["count"] += 1
            delta = scores - stats["mean"]
            stats["mean"] += delta / stats["count"]
            stats["m2"] += delta * (scores - stats["mean"])
        variance = stats["m2"] / max(stats["count"] - 1, 1)
        std = np.sqrt(np.maximum(variance, 0.0))
        safe_scores = np.clip(
            stats["mean"] - self.frontier_uncertainty_beta * std,
            1e-4,
            1.0,
        ).astype(np.float32)
        self.frontier_uuid_to_scores[uuid] = (safe_scores, node)

    def _append_heading_candidates(self, node, scores, robot_yaw,
                                   semantic_traversability=0.0, default=False):
        """Persist the existing visual bins as direction-conditioned candidates."""
        assessment = next(
            (item for item in node.robot_traversability if item.robot_profile_id == "default"),
            None,
        )
        physical = float(assessment.physical_traversability) if assessment else 0.0
        clearance = float(assessment.minimum_clearance) if assessment else 0.0
        dynamic = float(assessment.dynamic_risk) if assessment else 0.0
        confidence = float(assessment.confidence) if assessment else 0.0
        scores = np.asarray(scores, dtype=np.float32).reshape(-1)
        object_bonus = np.zeros(scores.size, dtype=np.float32)
        if (self.visual_memory_map is not None and self.object_search_mode and
                self.text_queries and scores.size):
            object_bonus = self.visual_memory_map.confirmed_heading_bonus(
                self.memory_map_id, self.text_queries[0], self.uuid_to_str(node.uuid),
                scores.size)
        coverage = None
        if self.visual_memory_map is not None and self.object_search_mode and self.text_queries:
            coverage = self.visual_memory_map.get_search_coverage(
                self.memory_map_id, self.text_queries[0], self.uuid_to_str(node.uuid))
        for index, score in enumerate(scores):
            heading = (2.0 * np.pi * index) / max(len(scores), 1)
            turn_cost = abs(np.arctan2(np.sin(heading - robot_yaw), np.cos(heading - robot_yaw)))
            utility = float(score) - self.heading_turn_cost_weight * turn_cost
            utility -= self.heading_dynamic_risk_weight * dynamic
            utility -= self.heading_low_confidence_weight * (1.0 - confidence)
            utility += self.object_memory_heading_weight * float(object_bonus[index])
            if coverage is not None and int(coverage["heading_mask"]) & (1 << index):
                utility -= self.search_coverage_weight * float(coverage["negative_evidence"])
            candidate = HeadingCandidate()
            candidate.heading_frame = self.global_frame
            candidate.heading = float(heading)
            candidate.frontier_score = float(score)
            candidate.semantic_traversability = float(semantic_traversability)
            candidate.physical_traversability = physical
            candidate.minimum_clearance = clearance
            candidate.dynamic_risk = dynamic
            candidate.motion_cost = float(turn_cost)
            candidate.predicted_utility = float(np.clip(utility, 0.0, 1.0))
            candidate.traversable = bool(
                assessment is not None and assessment.feasible and
                physical > 0.0 and dynamic < 1.0)
            candidate.selected = False
            candidate.has_supervision = False
            candidate.target_utility = 0.0
            node.heading_candidates.append(candidate)
        feasible = [i for i, candidate in enumerate(node.heading_candidates)
                    if candidate.traversable]
        if feasible:
            best = max(feasible,
                       key=lambda i: node.heading_candidates[i].predicted_utility)
            node.heading_candidates[best].selected = True
    
    @staticmethod
    def pool_memory_embedding(spatial_feats):
        """Pool spatial RADIO features into one normalized CPU descriptor."""
        if spatial_feats is None:
            return None
        features = spatial_feats.detach().float()
        if features.ndim < 2 or features.shape[0] == 0:
            return None
        # Reduce on the model device; transfer only the small pooled descriptor.
        descriptor = features.mean(dim=(0, *range(2, features.ndim))).cpu().numpy()
        if not np.all(np.isfinite(descriptor)):
            return None
        norm = float(np.linalg.norm(descriptor))
        if not np.isfinite(norm) or norm <= 1e-8:
            return None
        return descriptor / norm

    def traversal_feedback_callback(self, feedback):
        """Turn the latest attempted heading into a supervised memory sample."""
        self._record_memory_map_traversal(feedback)
        if self.visual_memory is None or feedback.outcome == TraversalFeedback.OUTCOME_UNKNOWN:
            return
        now = wall_time()
        while self.pending_memory_samples:
            if now - self.pending_memory_samples[0]["created_at"] <= self.visual_memory_pending_ttl:
                break
            self.pending_memory_samples.popleft()
        if not self.pending_memory_samples:
            self.get_logger().warning("Traversal feedback arrived without a recent visual-memory sample.")
            return

        feedback_stamp = feedback.header.stamp.sec + feedback.header.stamp.nanosec * 1e-9
        action_start_stamp = feedback_stamp - max(0.0, float(feedback.duration))
        samples = list(self.pending_memory_samples)
        matched_idx = select_feedback_sample(
            samples, action_start_stamp, self.uuid_to_str(feedback.from_node),
            self.visual_memory_pending_ttl)
        if matched_idx is None:
            self.get_logger().warning("No matching visual-memory sample for traversal start node and time.")
            return
        sample = samples.pop(matched_idx)
        self.pending_memory_samples = deque(samples, maxlen=64)

        # A temporary moving obstacle is not evidence that a scene heading is bad.
        if feedback.outcome in (TraversalFeedback.OUTCOME_BLOCKED_DYNAMIC,
                                TraversalFeedback.OUTCOME_ABORTED):
            return
        commanded = max(0.0, float(feedback.commanded_distance))
        observed = max(0.0, float(feedback.observed_distance))
        slip = float(np.clip(feedback.slip_ratio, 0.0, 1.0))
        progress = float(np.clip(observed / max(commanded, 1e-3), 0.0, 1.0))
        if feedback.outcome == TraversalFeedback.OUTCOME_SUCCEEDED:
            reward = 1.0
        elif feedback.outcome in (TraversalFeedback.OUTCOME_BLOCKED_GEOMETRY,
                                  TraversalFeedback.OUTCOME_COLLISION):
            reward = 0.0
        else:
            reward = progress * (1.0 - slip)
        stored = self.visual_memory.record_feedback(
            embedding=sample["embedding"],
            heading_scores=sample["heading_scores"],
            commanded_heading=feedback.commanded_heading,
            reward=reward,
            outcome=feedback.outcome,
        )
        if stored:
            self.get_logger().info(
                f"Stored visual-memory feedback: outcome={feedback.outcome}, "
                f"heading={feedback.commanded_heading:.2f} rad."
            )
    def _record_memory_map_traversal(self, feedback):
        if self.visual_memory_map is None or feedback.outcome == TraversalFeedback.OUTCOME_UNKNOWN:
            return
        bin_size = 2.0 * np.pi / max(self.num_angular_bins, 1)
        start_bin = int(np.round((feedback.commanded_heading % (2.0 * np.pi)) / bin_size)) % self.num_angular_bins
        end_bin = int(np.round((feedback.observed_heading % (2.0 * np.pi)) / bin_size)) % self.num_angular_bins
        self.visual_memory_map.record_traversal(
            map_id=self.memory_map_id,
            from_node=self.uuid_to_str(feedback.from_node),
            to_node=self.uuid_to_str(feedback.to_node),
            robot_profile_id=feedback.robot_profile_id or "default",
            outcome=feedback.outcome,
            start_heading_bin=start_bin,
            end_heading_bin=end_bin,
            duration=feedback.duration,
            commanded_distance=feedback.commanded_distance,
            observed_distance=feedback.observed_distance,
            slip_ratio=feedback.slip_ratio,
            energy_cost=feedback.energy_cost,
            timestamp=feedback.header.stamp.sec + feedback.header.stamp.nanosec * 1e-9,
        )

    def _observe_node_heading_memory(self, odom_msg, navgraph_msg, embedding, images,
                                     traversability):
        if self.visual_memory_map is None or embedding is None:
            return
        if not (0 <= navgraph_msg.current_node_idx < len(navgraph_msg.nodes)):
            return
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        yaw = float(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        position = (float(p.x), float(p.y), float(p.z))
        node = navgraph_msg.nodes[navgraph_msg.current_node_idx]
        node_uuid = self.uuid_to_str(node.uuid)
        heading_bin = VisualMemoryMap.heading_to_bin(yaw, self.num_angular_bins)
        previous = self.last_node_observation
        if previous is not None and not VisualMemoryMap.should_update_node_observation(
                previous["node_uuid"], previous["heading_bin"], previous["embedding"],
                node_uuid, heading_bin, embedding,
                self.node_observation_min_embedding_distance):
            return
        image_path = None
        if self.memory_map_save_images and images:
            self.memory_map_image_root.mkdir(parents=True, exist_ok=True)
            stamp = odom_msg.header.stamp.sec * 1000000000 + odom_msg.header.stamp.nanosec
            path = self.memory_map_image_root / f"{self.memory_map_id}_{stamp}.jpg"
            cv2.imwrite(str(path), cv2.cvtColor(images[0], cv2.COLOR_RGB2BGR))
            image_path = str(path)
        candidate = node.heading_candidates[heading_bin] if heading_bin < len(node.heading_candidates) else None
        assessment = next(
            (item for item in node.robot_traversability if item.robot_profile_id == "default"),
            None)
        physical = candidate.physical_traversability if candidate else (
            assessment.physical_traversability if assessment else 0.0)
        dynamic = candidate.dynamic_risk if candidate else (
            assessment.dynamic_risk if assessment else 0.0)
        stamp = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        self.visual_memory_map.observe_node_heading(
            self.memory_map_id, node_uuid, heading_bin, yaw, position, embedding,
            image_path=image_path,
            frontier_score=(candidate.frontier_score if candidate else None),
            semantic_traversability=(candidate.semantic_traversability if candidate else float(np.mean(traversability))),
            physical_traversability=float(physical),
            dynamic_risk=float(dynamic),
            uncertainty=float(np.std(traversability)),
            predicted_utility=(candidate.predicted_utility if candidate else None),
            timestamp=stamp)
        self.last_node_observation = {
            "node_uuid": node_uuid, "heading_bin": heading_bin,
            "embedding": np.asarray(embedding, dtype=np.float32).copy(),
        }

    def _record_negative_search(self, navgraph_msg, geofrontiers,
                                text_similarity, odom_msg):
        if self.visual_memory_map is None or not self.object_search_mode:
            return
        now = wall_time()
        if now - self.last_negative_search_at < self.negative_search_interval:
            return
        if not (0 <= navgraph_msg.current_node_idx < len(navgraph_msg.nodes)):
            return
        node_uuid = self.uuid_to_str(navgraph_msg.nodes[navgraph_msg.current_node_idx].uuid)
        if torch.is_tensor(text_similarity):
            finite = torch.nan_to_num(text_similarity.detach().float(),
                                      nan=0.0, posinf=0.0, neginf=0.0)
            maximum = float(finite.max().item()) if finite.numel() else 0.0
        else:
            values = np.asarray(text_similarity)
            finite = values[np.isfinite(values)]
            maximum = float(np.max(finite)) if finite.size else 0.0
        negative = float(np.clip(1.0 - maximum, 0.0, 1.0))
        q = odom_msg.pose.pose.orientation
        yaw = float(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        heading_bin = VisualMemoryMap.heading_to_bin(yaw, self.num_angular_bins)
        self.visual_memory_map.record_search_coverage(
            self.memory_map_id, self.text_queries[0], node_uuid,
            [heading_bin], negative)
        seen = {node_uuid}
        for camera in range(self.num_cameras):
            visible = geofrontiers[camera]
            if not visible:
                continue
            for node, heading in zip(visible["frontier_nodes"],
                                     visible["frontier_headings"]):
                frontier_uuid = self.uuid_to_str(node.uuid)
                if frontier_uuid in seen:
                    continue
                seen.add(frontier_uuid)
                direction = float(np.arctan2(heading[1], heading[0]))
                direction_bin = VisualMemoryMap.heading_to_bin(
                    direction, self.num_angular_bins)
                self.visual_memory_map.record_search_coverage(
                    self.memory_map_id, self.text_queries[0], frontier_uuid,
                    [direction_bin], negative)
                if len(seen) >= 9:
                    break
            if len(seen) >= 9:
                break
        self.last_negative_search_at = now

    def close_visual_memory(self):
        if self.visual_memory is not None:
            self.visual_memory.close()
        if self.visual_memory_map is not None:
            self.visual_memory_map.close()

    @staticmethod
    def uuid_to_str(uuid):
        return ''.join([f"{x:03}" for x in uuid.id])
                

def main(args=None):
    rclpy.init(args=args)
    from ament_index_python.packages import get_package_share_directory
    import argparse

    # Separate ROS args from your custom args
    custom_args = rclpy.utilities.remove_ros_args(args)
    
    # Now parse the remaining (non-ROS) args with argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Configuration file name (YAML) for the node.")
    def str2bool(v):
        return v.lower() in ('true')
    parser.add_argument("--do_object_search", type=str2bool, default=False, help="Enable object search.")
    parser.add_argument("--memory_in_process_only", action="store_true",
                        help="Keep visual memories in this WildOS process only.")
    custom_args = parser.parse_args(custom_args[1:])

    conf_name = f"{custom_args.config}"
    if conf_name.endswith(".yaml") is False:
        conf_name += ".yaml"

    package_share_directory = Path(get_package_share_directory('visual_navigation'))
    conf = package_share_directory / "configs" / conf_name

    config = OmegaConf.load(conf)
    if custom_args.memory_in_process_only:
        config.visual_memory_path = ":memory:"
        config.visual_memory_map_path = ":memory:"
        config.visual_memory_map_save_images = False
    ros2_node = WildOS_Nav(config, do_object_search=custom_args.do_object_search)
    rclpy.spin(ros2_node)

    ros2_node.close_visual_memory()
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
