"""Read-only RViz recording labels; never publishes navigation/control messages."""

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from wildos_nav2_adapter.math3d import matrix_from_quaternion


def confirmed_target(data, query):
    if (
        data.get("search_confirmed") is not True
        or data.get("query") != query
        or data.get("frame") != "spot1/odom"
    ):
        return None
    point = np.asarray(data.get("object_position", ()), dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    return point


def project_target(point, translation, rotation, intrinsic, width, height):
    camera = np.asarray(rotation).T @ (np.asarray(point) - np.asarray(translation))
    if camera[2] <= 0:
        return None
    pixel = np.asarray(intrinsic) @ camera
    x, y = pixel[:2] / pixel[2]
    if not (0 <= x < width and 0 <= y < height):
        return None
    return int(x), int(y)


class ReviewOverlay(Node):
    def __init__(self):
        super().__init__("rviz_review_overlay")
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.query = ""
        self.target = None
        self.target_stamp = 0
        self.info = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.image_pub = self.create_publisher(
            Image, "/wildos/review/camera", qos_profile_sensor_data
        )
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/wildos/review/target", marker_qos
        )
        self.create_subscription(
            String, "/spot1/object_search/query", self.on_query, 10
        )
        self.create_subscription(
            String, "/spot1/object_confirmation", self.on_confirmation, 10
        )
        self.create_subscription(
            CameraInfo,
            "/spot1/realsense/front/color/camera_info",
            self.on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/spot1/realsense/front/color/image_raw",
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_query(self, message):
        self.query = " ".join(message.data.casefold().split())
        self.target = None
        self.target_stamp = 0
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[clear]))

    def on_info(self, message):
        self.info = message

    def on_confirmation(self, message):
        try:
            data = json.loads(message.data)
            point = confirmed_target(data, self.query)
            stamp = int(data["observation_stamp_ns"])
        except (ValueError, TypeError, KeyError):
            return
        if point is None or stamp <= self.target_stamp:
            return
        self.target, self.target_stamp = point, stamp
        dot = Marker()
        dot.header.frame_id = "spot1/odom"
        dot.header.stamp = self.get_clock().now().to_msg()
        dot.ns, dot.id, dot.type, dot.action = (
            "confirmed_target",
            0,
            Marker.SPHERE,
            Marker.ADD,
        )
        # Display-only XY projection. Camera projection retains measured XYZ.
        dot.pose.position = Point(x=float(point[0]), y=float(point[1]), z=0.3)
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.8
        dot.color.g, dot.color.a = 1.0, 1.0
        label = Marker()
        label.header = dot.header
        label.ns, label.id, label.type = "confirmed_target", 1, Marker.TEXT_VIEW_FACING
        label.pose.position = Point(x=float(point[0]), y=float(point[1]), z=1.4)
        label.pose.orientation.w = 1.0
        label.scale.z = 1.4
        label.color.g = label.color.a = 1.0
        label.text = self.query + " [confirmed]"
        ring = Marker()
        ring.header = dot.header
        ring.ns, ring.id, ring.type = "confirmed_target", 2, Marker.LINE_STRIP
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.12
        ring.color.g, ring.color.a = 1.0, 0.7
        ring.points = [
            Point(
                x=float(point[0] + 3 * math.cos(a)),
                y=float(point[1] + 3 * math.sin(a)),
                z=0.2,
            )
            for a in np.linspace(0, 2 * math.pi, 65)
        ]
        self.marker_pub.publish(MarkerArray(markers=[dot, label, ring]))

    def on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8").copy()
        text = f"Searching: {self.query}" if self.query else "Waiting for query"
        if self.target is not None:
            stamp = (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )
            age = (stamp - self.target_stamp) / 1e9
            if age >= 0:
                text = f"{self.query} | " + (
                    "CONFIRMED" if age <= 5 else f"last confirmed {age:.1f}s ago"
                )
                pixel = None
                if self.info is not None and (self.info.width, self.info.height) == (
                    message.width,
                    message.height,
                ):
                    try:
                        transform = self.tf.lookup_transform(
                            "spot1/odom",
                            message.header.frame_id,
                            rclpy.time.Time.from_msg(message.header.stamp),
                        )
                        t, q = (
                            transform.transform.translation,
                            transform.transform.rotation,
                        )
                        pixel = project_target(
                            self.target,
                            (t.x, t.y, t.z),
                            matrix_from_quaternion((q.x, q.y, q.z, q.w)),
                            np.asarray(self.info.k).reshape(3, 3),
                            message.width,
                            message.height,
                        )
                    except TransformException:
                        pass
                if pixel is not None:
                    color = (0, 255, 80) if age <= 5 else (0, 190, 255)
                    cv2.drawMarker(image, pixel, color, cv2.MARKER_CROSS, 30, 3)
                    cv2.circle(image, pixel, 18, color, 2)
                else:
                    text += " | projection outside view / TF unavailable"
        cv2.putText(image, text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
        cv2.putText(
            image, text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 255, 120), 1
        )
        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = message.header
        self.image_pub.publish(output)


def main():
    rclpy.init()
    node = ReviewOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
