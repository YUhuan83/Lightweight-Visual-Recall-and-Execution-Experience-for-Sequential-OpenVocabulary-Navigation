"""Read-only ROS2 Camera/LiDAR/State/TF capture adapter.

This node never publishes vehicle controls and never loads scenarios. It saves
bounded, inspectable samples for the WildOS adapter and calibration work.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from beamng_msgs.msg import StateSensor
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from .camera import nominal_camera_info
from .pointcloud import decode_xyz, voxel_downsample


class SensorAdapter(Node):
    def __init__(self) -> None:
        super().__init__("wildos_sensor_adapter")
        self.declare_parameter("output_dir", str(Path.home() / "wildos-ros2-captures"))
        self.declare_parameter("image_topic", "/vehicles/ego/sensors/front_cam/colour")
        self.declare_parameter("lidar_topic", "/vehicles/ego/sensors/front_lidar")
        self.declare_parameter("state_topic", "/vehicles/ego/sensors/state")
        self.declare_parameter("camera_width", 960)
        self.declare_parameter("camera_height", 640)
        self.declare_parameter("camera_fov_y", 60.0)
        self.declare_parameter("voxel", 0.15)
        self.declare_parameter("max_points", 50000)
        self.declare_parameter("sync_slop_sec", 0.15)
        self.output = Path(self.get_parameter("output_dir").value).expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.slop = float(self.get_parameter("sync_slop_sec").value)
        self.latest = {}
        self.sequence = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        self.create_subscription(StateSensor, self.state_topic, self.state_callback, 10)
        self.get_logger().info("Read-only WildOS sensor adapter started")

    @staticmethod
    def stamp(message) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def image_callback(self, message: Image) -> None:
        self.latest["image"] = message
        self.try_capture()

    def lidar_callback(self, message: PointCloud2) -> None:
        self.latest["lidar"] = message
        self.try_capture()

    def state_callback(self, message: StateSensor) -> None:
        self.latest["state"] = message
        self.try_capture()

    def try_capture(self) -> None:
        if set(self.latest) != {"image", "lidar", "state"}:
            return
        messages = self.latest
        stamps = {name: self.stamp(message) for name, message in messages.items()}
        if max(stamps.values()) - min(stamps.values()) > self.slop:
            return
        self.latest = {}
        try:
            self.save_capture(messages, stamps)
        except Exception as exc:
            self.get_logger().error(f"capture failed: {exc}")

    def transform_dict(self, transform: TransformStamped) -> dict:
        t = transform.transform.translation
        q = transform.transform.rotation
        return {
            "parent": transform.header.frame_id,
            "child": transform.child_frame_id,
            "translation": [t.x, t.y, t.z],
            "rotation_xyzw": [q.x, q.y, q.z, q.w],
            "stamp": self.stamp(transform),
        }

    def lookup_latest_transform(self, child: str) -> dict:
        """Wait briefly for static/dynamic TF before recording a sample."""
        target = "map"
        source = child
        timeout = rclpy.duration.Duration(seconds=0.5)
        for _ in range(5):
            try:
                if self.tf_buffer.can_transform(target, source, rclpy.time.Time(), timeout=timeout):
                    transform = self.tf_buffer.lookup_transform(
                        target, source, rclpy.time.Time(), timeout=timeout
                    )
                    return self.transform_dict(transform)
            except Exception:
                pass
            time.sleep(0.05)
        return {"error": f"TF unavailable: {target} <- {source}"}

    def save_capture(self, messages: dict, stamps: dict) -> None:
        image = messages["image"]
        lidar = messages["lidar"]
        state = messages["state"]
        points = decode_xyz(lidar)
        points = voxel_downsample(
            points,
            float(self.get_parameter("voxel").value),
            int(self.get_parameter("max_points").value),
        )
        stamp = max(stamps.values())
        capture_dir = self.output / f"sample_{self.sequence:06d}"
        temp_dir = self.output / f".sample_{self.sequence:06d}.tmp"
        self.sequence += 1
        temp_dir.mkdir(parents=True, exist_ok=False)
        if image.encoding != "rgb8" or int(image.step) != int(image.width) * 3:
            raise ValueError(f"unsupported image encoding/step: {image.encoding}/{image.step}")
        image_array = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(
            int(image.height), int(image.width), 3
        )
        from PIL import Image as PILImage
        PILImage.fromarray(image_array, mode="RGB").save(temp_dir / "image.png")
        np.save(temp_dir / "cloud_map.npy", points, allow_pickle=False)
        state_json = {
            "stamp": self.stamp(state),
            "frame_id": state.header.frame_id,
            "position": [state.position.x, state.position.y, state.position.z],
            "velocity": [state.velocity.x, state.velocity.y, state.velocity.z],
            "front": [state.front.x, state.front.y, state.front.z],
            "up": [state.up.x, state.up.y, state.up.z],
            "dir": [state.dir.x, state.dir.y, state.dir.z],
        }
        (temp_dir / "state.json").write_text(json.dumps(state_json, indent=2), encoding="utf-8")
        camera_info = nominal_camera_info(
            int(self.get_parameter("camera_width").value),
            int(self.get_parameter("camera_height").value),
            float(self.get_parameter("camera_fov_y").value),
            "ego_front_cam",
        )
        (temp_dir / "camera_info.json").write_text(json.dumps(camera_info, indent=2), encoding="utf-8")
        transforms = {}
        for child in ("ego", "ego_front_cam", "ego_front_lidar"):
            transforms[child] = self.lookup_latest_transform(child)
        (temp_dir / "tf.json").write_text(json.dumps(transforms, indent=2), encoding="utf-8")
        metadata = {
            "schema": "wildos-ros2-capture/1",
            "stamp": stamp,
            "message_stamps": stamps,
            "image_topic": self.image_topic,
            "lidar_topic": self.lidar_topic,
            "state_topic": self.state_topic,
            "image_frame_id": image.header.frame_id,
            "lidar_frame_id": lidar.header.frame_id,
            "points_raw": int(lidar.width * lidar.height),
            "points_saved": int(len(points)),
            "camera_calibration": "NOMINAL_UNVERIFIED",
            "read_only": True,
        }
        (temp_dir / "capture.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        temp_dir.rename(capture_dir)
        self.get_logger().info(
            f"saved {capture_dir} raw_points={metadata['points_raw']} saved_points={metadata['points_saved']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
