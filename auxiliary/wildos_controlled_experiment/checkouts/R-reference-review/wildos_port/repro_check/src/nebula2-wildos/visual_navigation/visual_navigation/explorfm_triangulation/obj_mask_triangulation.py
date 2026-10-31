import json
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visual_navigation.utils.buffer import MessageBuffer
from visual_navigation.utils.tf_lookup_sub import TFLookupSubscriber

from sensor_msgs_py import point_cloud2
from object_search_msgs.msg import ObjectMaskWithTf
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as ImageMsg, CameraInfo, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from pathlib import Path
from omegaconf import OmegaConf
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


from triangulation3d.camera_data import Camera
from triangulation3d.particle_generator import ParticleGenerator
from triangulation3d.bbox_generator import BoundingBoxGenerator
from triangulation3d.triangulator import Triangulator
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth, triangulate_bearings, ground_like_surface

CAMERA_MAPPING = {
    0: "front",
    1: "left",
    2: "right"
}    

class ObjectMaskTriangulator(Node):
    default_config = {
        # Robot Parameters
        "num_cameras": 3,  # Number of cameras to use
        "parent_frame": "spot1/odom",
        "cam_frame": "spot1/realsense/{}_color_optical_frame",
        "lidar_frame": "spot1/ouster/front/os_lidar",
        "object_mask_topic": "/spot1/object_mask",  # Topic for object mask input
        "lidar_topic": "/spot1/ouster/front/points_filtered",  # Topic for lidar input

        # Publisher topics
        "triangulated_object_topic": "/spot1/triangulated_object",
        "navigation_goal_topic": "/spot1/imgnav_waypoint",
        "particle_viz_topic": "/spot1/object_hypotheses",

        # ROS2 subscriber params
        "qos_history_depth": 10,
        "syncsub_queue_size": 50,  # Retain LiDAR while the visual model runs.
        "syncsub_slop": 0.1,
        "camera_mask_max_skew_sec": 0.05,

        # Triangulation config
        "max_views": 12,  # Sliding window of independent camera observations.
        "min_lidar_points": 20,
        "sparse_lidar_points": 8,
        "observation_timeout_sec": 5.0,  # Maximum age of the original measurement.
        "min_view_distance": 1.0,  # Minimum distance between camera views to consider them different
        "particle_generator_config": {
            "num_particles": 1000, # Number of particles to generate
            "depth_range": [1.0, 100.0],  # Depth range in meters
            "add_odom_drift": False, # Whether to add odometry drift to the camera pose
        },
        "use_mask_for_projection": True,  # Whether to use the object mask for projection


        # TFLookup Config
        "buffer_size": 10,       # number of messages
        "timer_duration": 0.2,   # seconds
    }

    def __init__(self, config: OmegaConf=OmegaConf.create()):
        config = OmegaConf.merge(OmegaConf.create(self.default_config), config)
        super().__init__('obj_mask_triangulator')

        np.random.seed(42)

        # Robot Parameters
        self.num_cameras = config.num_cameras
        assert self.num_cameras in [1, 3], "Only 1 or 3 cameras are supported."

        # Triangulation parameters and initializations
        self.views = []
        self.max_views = int(config.max_views)
        self.min_lidar_points = int(config.min_lidar_points)
        self.sparse_lidar_points = int(config.sparse_lidar_points)
        self.observation_timeout = float(config.observation_timeout_sec)
        sync_slop = float(config.syncsub_slop)
        camera_mask_skew = float(config.camera_mask_max_skew_sec)
        if (not 3 <= self.max_views <= 64 or not np.isfinite(self.observation_timeout)
                or self.observation_timeout <= 0):
            raise ValueError("Triangulation requires 3–64 views and a finite positive timeout")
        if (not np.isfinite(sync_slop) or not 0 < sync_slop <= 0.2
                or not np.isfinite(camera_mask_skew)
                or not 0 < camera_mask_skew <= 0.1):
            raise ValueError("Invalid sensor timestamp synchronization tolerances")
        self.sync_slop_ns = int(round(sync_slop * 1e9))
        self.camera_mask_max_skew_ns = int(round(camera_mask_skew * 1e9))
        if self.min_lidar_points < self.sparse_lidar_points or self.sparse_lidar_points < 6:
            raise ValueError("LiDAR confirmation requires at least 6 coherent returns")
        self.bearing_views = []
        self.localization_quality = None
        self.query_started_ns = 0
        self.last_observation_ns = 0
        self.min_view_distance = config.min_view_distance
        self.use_mask_for_projection = config.use_mask_for_projection

        self.particle_generator = ParticleGenerator(config.particle_generator_config)
        self.triangulator = Triangulator()
        self.triangulated_position = None
        self.prev_view_pos = None
        self.found_lidar_in_mask = False
        self.active_query = None

        # Subscribers and Publishers
        self.init_publishers(config)
        self.init_subscribers(config)

        # Frames and Topic Names
        self.global_frame = config.parent_frame
        self.cam_tf_frame = config.cam_frame
        self.lidar_tf_frame = config.lidar_frame

        # TFLookup
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.msg_buffer = MessageBuffer(
            max_size=config.buffer_size, wait_for_oldest=True
        )
        self.clbk_cntr = 0

        self.timer = self.create_timer(config.timer_duration, self.do_processing)

    def init_publishers(self, config: OmegaConf):
        self.triangulated_obj_publisher = self.create_publisher(
            Marker, config.triangulated_object_topic, 10
        )
        self.localization_pub = self.create_publisher(String, '/spot1/object_localization', 10)
        self.timing_pub = self.create_publisher(String, '/spot1/measurement_timing', 10)
        self.nav_goal_publisher = self.create_publisher(
            PoseStamped, config.navigation_goal_topic, 10
        )
        self.particle_viz_publisher = self.create_publisher(
            PointCloud2, config.particle_viz_topic, 10
        )

    def init_subscribers(self, config):
        self.create_subscription(String, '/spot1/object_search/query', self.on_query, 10)
        self.object_mask_sub = Subscriber(
            self, ObjectMaskWithTf, config.object_mask_topic, qos_profile=config.qos_history_depth
        )
        self.lidar_sub = Subscriber(
            self, PointCloud2, config.lidar_topic, qos_profile=config.qos_history_depth
        )

        ts_subs = [self.object_mask_sub, self.lidar_sub]
        self.ts = ApproximateTimeSynchronizer(
            ts_subs, queue_size=config.syncsub_queue_size, slop=config.syncsub_slop
        )
        self.ts.registerCallback(self.listener_callback)

    def listener_callback(self, object_mask_msg, lidar_msg):
        query = ' '.join(object_mask_msg.query.casefold().split())
        stamp_ns = self.stamp_ns(object_mask_msg.header.stamp)
        lidar_ns = self.stamp_ns(lidar_msg.header.stamp)
        camera_infos = list(getattr(object_mask_msg, 'cam_infos', []))
        camera_info_stamps_ns = [self.stamp_ns(info.header.stamp)
                                 for info in camera_infos]
        if not query:
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'empty_query')
            return  # Unlabelled masks cannot be associated with a search target.
        if self.active_query is None:
            self.on_query(String(data=query))
        if query != self.active_query:
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'stale_query')
            return  # Late mask from the preceding target.
        self.clbk_cntr += 1

        self.get_logger().info(f"Received callback {self.clbk_cntr}")
        if lidar_msg.header.frame_id != self.lidar_tf_frame:
            self.get_logger().warning("Ignoring LiDAR with an unexpected frame")
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'unexpected_lidar_frame')
            return
        if (len(camera_info_stamps_ns) != self.num_cameras
                or any(abs(camera_stamp - stamp_ns) > self.camera_mask_max_skew_ns
                       for camera_stamp in camera_info_stamps_ns)):
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'camera_mask_timestamp_mismatch')
            return
        now_ns = self.get_clock().now().nanoseconds
        if (stamp_ns < self.query_started_ns or stamp_ns <= self.last_observation_ns
                or stamp_ns > now_ns or now_ns - stamp_ns > self.observation_timeout * 1e9):
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'stale_or_nonmonotonic_mask')
            return
        if abs(lidar_ns - stamp_ns) > self.sync_slop_ns:
            self.record_sync_timing('dropped', stamp_ns, lidar_ns,
                                    camera_info_stamps_ns, 'mask_lidar_timestamp_mismatch')
            return

        self.msg_buffer.add_msg(
            msg={
                "obj_mask": object_mask_msg,
                "lidar": lidar_msg
            },
            stamp=object_mask_msg.header.stamp,
        )
        self.record_sync_timing('paired', stamp_ns, lidar_ns, camera_info_stamps_ns)

    def on_query(self, message):
        query = ' '.join(message.data.casefold().split())
        if not query or query == self.active_query:
            return
        self.active_query = query
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.last_observation_ns = 0
        self.bearing_views.clear()
        self.views.clear()
        self.prev_view_pos = None
        self.triangulated_position = None
        self.found_lidar_in_mask = False
        if hasattr(self, 'msg_buffer'):
            self.msg_buffer.clear()
        self.get_logger().info(f'Triangulation target changed to {query!r}; old hypotheses cleared')

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def record_sync_timing(self, status, mask_stamp_ns, lidar_stamp_ns,
                           camera_info_stamps_ns, reason=None):
        now_ns = self.get_clock().now().nanoseconds
        payload = {
            'schema_version': 1, 'sensor': 'mask_lidar_sync', 'status': status,
            'clock': 'ros', 'received_ros_ns': now_ns,
            'mask_stamp_ns': mask_stamp_ns, 'lidar_stamp_ns': lidar_stamp_ns,
            'camera_info_stamps_ns': camera_info_stamps_ns,
            'camera_mask_skew_ns': [stamp - mask_stamp_ns
                                    for stamp in camera_info_stamps_ns],
            'lidar_mask_skew_ns': lidar_stamp_ns - mask_stamp_ns,
        }
        if reason is not None:
            payload['reason'] = reason
        self.timing_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def do_processing(self):
        try:
            self.process_observation()
        except (ValueError, TypeError, AssertionError, TransformException) as exc:
            # Malformed or unmatched sensor data must not terminate perception.
            self.get_logger().warning("Rejected triangulation observation: " + str(exc)[:160])

    def process_observation(self):
        if not self.msg_buffer.buffer:
            self.get_logger().warn("Message buffer is empty, waiting for messages...")
            return

        # Extract messages and data
        oldest_msg, _, _ = self.msg_buffer.pop_oldest_msg()

        obj_mask_msg = oldest_msg["obj_mask"]
        if ' '.join(obj_mask_msg.query.casefold().split()) != self.active_query:
            return
        stamp_ns = self.stamp_ns(obj_mask_msg.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        if (stamp_ns < self.query_started_ns or stamp_ns <= self.last_observation_ns
                or stamp_ns > now_ns or now_ns - stamp_ns > self.observation_timeout * 1e9):
            return
        self.last_observation_ns = stamp_ns
        self.current_observation = obj_mask_msg
        self.localization_quality = None
        lidar_msg = oldest_msg["lidar"]
        all_cam_data = self.extract_data_from_obj_mask_msg(obj_mask_msg)
        cur_pos = np.array([
            obj_mask_msg.odom.pose.pose.position.x,
            obj_mask_msg.odom.pose.pose.position.y,
            obj_mask_msg.odom.pose.pose.position.z
        ])

        # Check if LiDAR points fall within the object mask
        self.found_lidar_in_mask = False
        self.triangulated_position = None
        self.check_lidar_in_mask(all_cam_data, lidar_msg)

        # Triangulate using multiple views if LiDAR triangulation was not successful
        if not self.found_lidar_in_mask:
            if self.prev_view_pos is not None:
                
                dist = np.linalg.norm(cur_pos - self.prev_view_pos)
                if dist < self.min_view_distance:
                    self.get_logger().info(f"Current view is too close to previous view (distance: {dist:.2f}m), skipping...")
                    return

            self.add_views(all_cam_data)
            self.prev_view_pos = cur_pos
            if len(self.views) >= 2:
                quality = triangulate_bearings(self.bearing_views, self.min_view_distance)
                if quality is not None:
                    self.localization_quality = quality
                    self.triangulated_position = quality['position']
                    self.get_logger().info(f"Validated multi-view position: {self.triangulated_position}")
            else:
                self.get_logger().info("Not enough views for triangulation.")
        else:
            self.get_logger().info("Object LOCK using LiDAR!")
        
        # Publish triangulated position as a navigation goal
        self.publish_navigation_goal_and_marker()
        self.publish_goal_hypotheses()

    def extract_data_from_obj_mask_msg(self, obj_mask_msg):
        """
        Extract relevant data from ObjectMaskWithTf message.
        """
        obj_mask = self.get_objmask_from_multiarray(obj_mask_msg.object_mask)
        assert obj_mask.shape[0] == self.num_cameras, f"Expected {self.num_cameras} object masks, but got {obj_mask.shape[0]}"

        camera_infos = obj_mask_msg.cam_infos
        assert len(camera_infos) == self.num_cameras, f"Expected {self.num_cameras} camera infos, but got {len(camera_infos)}"

        camera_tfs = obj_mask_msg.cam_transforms.transforms
        assert len(camera_tfs) == self.num_cameras, f"Expected {self.num_cameras} camera transforms, but got {len(camera_tfs)}"

        all_cam_data = []
        for cam_id in range(self.num_cameras):
            cam_data = {}
            cam_frame = self.cam_tf_frame.format(CAMERA_MAPPING[cam_id])

            assert camera_infos[cam_id].header.frame_id.lstrip('/') == cam_frame, \
                f"CameraInfo frame_id {camera_infos[cam_id].header.frame_id} does not match expected {cam_frame}"
            
            assert camera_tfs[cam_id].child_frame_id == cam_frame, \
                f"Camera TF child_frame_id {camera_tfs[cam_id].child_frame_id} does not match expected {cam_frame}"

            cam_data["object_mask"] = obj_mask[cam_id, 0, :, :]  # HxW
            cam_data["K"] = np.array(camera_infos[cam_id].k).reshape(3, 3)
            cam_data["R_wc"] = R.from_quat([
                camera_tfs[cam_id].transform.rotation.x,
                camera_tfs[cam_id].transform.rotation.y,
                camera_tfs[cam_id].transform.rotation.z,
                camera_tfs[cam_id].transform.rotation.w
            ]).as_matrix()
            cam_data["t_wc"] = np.array([
                camera_tfs[cam_id].transform.translation.x,
                camera_tfs[cam_id].transform.translation.y,
                camera_tfs[cam_id].transform.translation.z
            ]).reshape(3, 1)
            cam_data["width"] = camera_infos[cam_id].width
            cam_data["height"] = camera_infos[cam_id].height
            cam_data["frame_id"] = cam_frame
            cam_data["camera_info"] = camera_infos[cam_id]
            cam_data["camera_tf"] = camera_tfs[cam_id]
        
            if (not np.all(np.isfinite(cam_data['K'])) or cam_data['K'][0, 0] <= 0
                    or cam_data['K'][1, 1] <= 0 or not np.all(np.isfinite(cam_data['t_wc']))
                    or cam_data['object_mask'].shape != (cam_data['height'], cam_data['width'])
                    or camera_tfs[cam_id].header.frame_id.lstrip('/') != self.global_frame):
                raise ValueError('invalid camera calibration or acquisition transform')
            mask_ns = self.stamp_ns(obj_mask_msg.header.stamp)
            tf_ns = self.stamp_ns(camera_tfs[cam_id].header.stamp)
            info_ns = self.stamp_ns(camera_infos[cam_id].header.stamp)
            if abs(tf_ns - mask_ns) > self.camera_mask_max_skew_ns:
                raise ValueError('camera transform does not match mask measurement time')
            if abs(info_ns - mask_ns) > self.camera_mask_max_skew_ns:
                raise ValueError('camera image time does not match mask measurement time')
            all_cam_data.append(cam_data)

        return all_cam_data

    def check_lidar_in_mask(self, all_cam_data, lidar_msg):
        points = np.asarray([list(pt) for pt in point_cloud2.read_points(
            lidar_msg, field_names=("x", "y", "z"), skip_nans=True)], dtype=float).reshape(-1, 3)
        if not len(points):
            return
        # Use the LiDAR acquisition pose, then the camera acquisition pose.
        # A latest cam<-lidar transform would ignore motion between the scans.
        world_from_lidar = self.tf_buffer.lookup_transform(
            self.global_frame, self.lidar_tf_frame,
            rclpy.time.Time.from_msg(lidar_msg.header.stamp), timeout=Duration(seconds=0.0))
        t = world_from_lidar.transform
        rotation = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
        world = points @ rotation.T + [t.translation.x, t.translation.y, t.translation.z]
        candidates = []
        for cam in all_cam_data:
            camera = (world - cam['t_wc'].reshape(1, 3)) @ cam['R_wc']
            quality = foreground_depth(camera, cam['K'], cam['object_mask'],
                                       self.min_lidar_points, self.sparse_lidar_points)
            if quality is None:
                continue
            if (self.active_query in ('traffic light', 'traffic lights', 'traffic signal')
                    and ground_like_surface(quality, cam['R_wc'])):
                continue
            quality['plane_normal'] = cam['R_wc'] @ quality['plane_normal']
            quality['confidence_kind'] = 'geometric_support'
            quality['position'] = cam['R_wc'] @ quality['position'] + cam['t_wc'].ravel()
            quality['covariance'] = cam['R_wc'] @ quality['covariance'] @ cam['R_wc'].T
            quality['horizontal_fov'] = float(2 * np.arctan(cam['width'] / (2 * cam['K'][0, 0])))
            candidates.append(quality)
        if candidates:
            quality = max(candidates, key=lambda v: v['confidence'])
            self.localization_quality = quality
            self.triangulated_position = quality['position']
            self.found_lidar_in_mask = True

    def add_views(self, all_cam_data):
        """
        Add camera views for triangulation.
        """
        # Keep recent independent views: an old incorrect mask must not freeze
        # all later triangulation by filling the original max_views buffer.

        for cam_id in range(self.num_cameras):
            cam_name = CAMERA_MAPPING[cam_id]
            cam_data = all_cam_data[cam_id]

            if np.sum(cam_data["object_mask"]) == 0:
                self.get_logger().info(f"No object detected in camera {cam_name}, skipping...")
                continue
            
            bbox = cv2.boundingRect(cam_data["object_mask"].astype(np.uint8))
            x,y,w,h = bbox
            bbox = np.array([x,y,x+w,y+h])
            view = Camera(
                camera_info=cam_data["camera_info"],
                camera_tf=cam_data["camera_tf"],
                bounding_box=bbox,
                object_mask=cam_data["object_mask"],
                image=None
            )
            view = BoundingBoxGenerator.generate_ray_from_bbox(view)

            view = self.particle_generator.generate_particles(
                view,
                use_mask=self.use_mask_for_projection,
                pcl_frame_id=self.global_frame
            )

            self.views.append(view)
            pixels = np.argwhere(cam_data['object_mask'] > 0)
            v, u = np.median(pixels, axis=0)
            ray = np.linalg.solve(cam_data['K'], [u, v, 1.])
            ray = cam_data['R_wc'] @ ray
            self.bearing_views.append(dict(origin=cam_data['t_wc'].ravel(), ray=ray,
                                           angular_tolerance=.04))
        self.views = self.views[-self.max_views:]
        self.bearing_views = self.bearing_views[-self.max_views:]

    def get_objmask_from_multiarray(self, multiarray_msg):
        """
        Convert MultiArray message to binary object mask.
        """
        offset = multiarray_msg.layout.data_offset
        obj_mask = np.array(multiarray_msg.data[offset:], dtype=np.uint8)
        dims = multiarray_msg.layout.dim
        assert len(dims) == 4, "Expected 4D MultiArray for object mask (Bx1xHxW)"
        obj_mask = obj_mask.reshape((multiarray_msg.layout.dim[0].size,
                                     multiarray_msg.layout.dim[1].size,
                                     multiarray_msg.layout.dim[2].size,
                                     multiarray_msg.layout.dim[3].size))
        return obj_mask
            
    def publish_navigation_goal_and_marker(self):
        if (self.triangulated_position is None or self.localization_quality is None
                or not np.all(np.isfinite(self.triangulated_position))):
            self.get_logger().info("No triangulated position available to publish.")
            return

        # Publish as a navigation goal
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = self.global_frame
        nav_goal.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.position.x = float(self.triangulated_position[0])
        nav_goal.pose.position.y = float(self.triangulated_position[1])
        nav_goal.pose.position.z = float(self.triangulated_position[2])
        nav_goal.pose.orientation.w = 1.0  # Neutral orientation
        self.nav_goal_publisher.publish(nav_goal)
        obs = self.current_observation
        stamp_ns = obs.header.stamp.sec * 1000000000 + obs.header.stamp.nanosec
        pose = obs.odom.pose.pose.position
        orientation = obs.odom.pose.pose.orientation
        self.localization_pub.publish(String(data=json.dumps({
            'query': self.active_query, 'frame': self.global_frame,
            'observation_stamp_ns': stamp_ns,
            'position': self.triangulated_position.tolist(),
            'observer_position': [pose.x, pose.y, pose.z],
            'observer_orientation': [orientation.x, orientation.y, orientation.z, orientation.w],
            'source': 'lidar' if self.found_lidar_in_mask else 'multiview',
            'views': len(self.bearing_views),
            'quality': {key: (value.tolist() if isinstance(value, np.ndarray) else value)
                        for key, value in self.localization_quality.items() if key != 'position'}})))
        self.get_logger().info(f"Published navigation goal at {self.triangulated_position}")

        # Publish as a visualization marker
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.ns = "triangulated_object"
        marker.id = 0
        marker.pose.position.x = float(self.triangulated_position[0])
        marker.pose.position.y = float(self.triangulated_position[1])
        marker.pose.position.z = float(self.triangulated_position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.triangulated_obj_publisher.publish(marker)

    def publish_goal_hypotheses(self):
        # Combine the points from all the cameras into a single PointCloud message
        combined_pcl_msg = self.triangulator.combine_points(self.views, pcl_frame_id=self.global_frame)
        self.particle_viz_publisher.publish(combined_pcl_msg)

def main(args=None):
    rclpy.init(args=args)
    from ament_index_python.packages import get_package_share_directory
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="triangulation3d_objsearch_conf.yaml")
    custom_args = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])

    package_share_directory = Path(get_package_share_directory('visual_navigation'))
    conf = package_share_directory / "configs" / custom_args.config

    mask_triang_node = ObjectMaskTriangulator(OmegaConf.load(conf))
    rclpy.spin(mask_triang_node)

    mask_triang_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
