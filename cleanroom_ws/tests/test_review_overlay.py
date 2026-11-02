"""Display-only geometry and real camera/confirmation topic flow."""

import json
import time
import unittest

import numpy as np

from scripts.review_overlay import confirmed_target, project_target


class ReviewGeometryTests(unittest.TestCase):
    def test_confirmed_target_matches_query_frame_and_finite_xyz(self):
        data = dict(
            query="green house",
            frame="spot1/odom",
            search_confirmed=True,
            object_position=[10, 2, 58],
        )
        np.testing.assert_equal(confirmed_target(data, "green house"), [10, 2, 58])
        self.assertIsNone(confirmed_target(data, "bus stop"))
        self.assertIsNone(
            confirmed_target(dict(data, search_confirmed=False), "green house")
        )
        self.assertIsNone(confirmed_target(dict(data, frame="camera"), "green house"))
        self.assertIsNone(
            confirmed_target(dict(data, object_position=[np.nan, 0, 0]), "green house")
        )

    def test_camera_projection_uses_current_pose_and_bounds(self):
        k = np.array([[100, 0, 100], [0, 100, 100], [0, 0, 1]])
        self.assertEqual(
            project_target([1, 2, 5], [1, 2, 0], np.eye(3), k, 200, 200), (100, 100)
        )
        optical_to_world = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self.assertEqual(
            project_target([5, 0, 0], [0, 0, 0], optical_to_world, k, 200, 200),
            (100, 100),
        )
        self.assertIsNone(project_target([0, 0, -1], [0, 0, 0], np.eye(3), k, 200, 200))
        self.assertIsNone(project_target([10, 0, 1], [0, 0, 0], np.eye(3), k, 200, 200))
        self.assertEqual(
            project_target([0.999, 0, 1], [0, 0, 0], np.eye(3), k, 200, 200), (199, 100)
        )


class ReviewFlowTests(unittest.TestCase):
    def test_camera_keeps_updating_without_tf_then_projects_confirmed_target(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from cv_bridge import CvBridge
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from visualization_msgs.msg import MarkerArray
        from scripts.review_overlay import ReviewOverlay

        rclpy.init()
        io, overlay = Node("review_display_test"), ReviewOverlay()
        executor = SingleThreadedExecutor()
        executor.add_node(io)
        executor.add_node(overlay)
        bridge = CvBridge()
        images, markers = [], []
        io.create_subscription(
            Image, "/wildos/review/camera", images.append, qos_profile_sensor_data
        )
        io.create_subscription(MarkerArray, "/wildos/review/target", markers.append, 10)
        query = io.create_publisher(String, "/spot1/object_search/query", 10)
        confirmation = io.create_publisher(String, "/spot1/object_confirmation", 10)
        camera = io.create_publisher(
            Image, "/spot1/realsense/front/color/image_raw", qos_profile_sensor_data
        )
        info = io.create_publisher(
            CameraInfo,
            "/spot1/realsense/front/color/camera_info",
            qos_profile_sensor_data,
        )
        static = StaticTransformBroadcaster(io)

        def wait(predicate):
            end = time.monotonic() + 5
            while not predicate() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(predicate())

        def publish_image():
            message = bridge.cv2_to_imgmsg(
                np.zeros((200, 200, 3), dtype=np.uint8), encoding="bgr8"
            )
            message.header.frame_id = "review_test_optical"
            message.header.stamp = io.get_clock().now().to_msg()
            camera.publish(message)
            return message

        try:
            wait(
                lambda: (
                    query.get_subscription_count() == 1
                    and camera.get_subscription_count() == 1
                )
            )
            query.publish(String(data="green house"))
            wait(lambda: bool(markers))
            stamp = io.get_clock().now().nanoseconds
            confirmation.publish(
                String(
                    data=json.dumps(
                        dict(
                            query="green house",
                            frame="spot1/odom",
                            search_confirmed=True,
                            object_position=[2, 1, 5],
                            observation_stamp_ns=stamp,
                        )
                    )
                )
            )
            wait(lambda: any(len(m.markers) == 3 for m in markers))
            first = publish_image()
            wait(lambda: bool(images))
            self.assertEqual(images[-1].header, first.header)
            np.testing.assert_equal(
                bridge.imgmsg_to_cv2(images[-1], "bgr8")[120, 140], [0, 0, 0]
            )
            calibration = CameraInfo(width=200, height=200)
            calibration.k = [100.0, 0.0, 100.0, 0.0, 100.0, 100.0, 0.0, 0.0, 1.0]
            info.publish(calibration)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = "spot1/odom", "review_test_optical"
            tf.header.stamp = io.get_clock().now().to_msg()
            tf.transform.rotation.w = 1.0
            static.sendTransform(tf)
            # Continue publishing camera frames; no new confirmation or mask.
            end = time.monotonic() + 5
            while time.monotonic() < end:
                publish_image()
                executor.spin_once(timeout_sec=0.02)
                if (
                    images
                    and bridge.imgmsg_to_cv2(images[-1], "bgr8")[120, 140, 1] > 200
                ):
                    break
            self.assertGreater(
                bridge.imgmsg_to_cv2(images[-1], "bgr8")[120, 140, 1], 200
            )
            self.assertGreater(len(images), 1)
            query.publish(String(data="bus stop"))
            count = len(markers)
            wait(lambda: len(markers) > count)
            self.assertEqual(markers[-1].markers[0].action, 3)  # DELETEALL
        finally:
            executor.shutdown()
            overlay.destroy_node()
            io.destroy_node()
            rclpy.shutdown()
