"""Read-only real-time WildOS perception node.

Subscribes to BeamNG ROS2 Camera/LiDAR/State topics, runs the persistent CUDA
vision worker in a separate interpreter, performs LiDAR-in-mask localization,
"""
from __future__ import annotations

import json
import math
import select
import subprocess
import threading
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from beamng_msgs.msg import StateSensor
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .camera import nominal_camera_info, optical_rotation_from_body
from .exploration import exploration_bearing, smooth_bearing
from .pointcloud import decode_xyz, voxel_downsample
from .node import SensorAdapter
from .tracking import update_track


class VisionClient:
    def __init__(self, python, script, checkpoint_root, source_root, timeout=180.0):
        self.timeout = float(timeout)
        self.proc = subprocess.Popen(
            [python, "-s", "-B", str(script), "--checkpoint-root", str(checkpoint_root),
             "--source-root", str(source_root)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, encoding="utf-8",
        )
        ready = self._read()
        if ready.get("status") != "ready":
            raise RuntimeError(f"vision worker not ready: {ready}")
        self.lock = threading.Lock()

    def _read(self):
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"vision worker did not respond within {self.timeout:.0f}s")
            ready, _, _ = select.select([self.proc.stdout], [], [], remaining)
            if not ready:
                raise TimeoutError(f"vision worker did not respond within {self.timeout:.0f}s")
            line = self.proc.stdout.readline()
            if not line:
                break
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)
        raise RuntimeError("vision worker closed")

    def infer(self, image, query, output):
        with self.lock:
            self.proc.stdin.write(json.dumps({"image": str(image), "query": query, "out": str(output)}) + "\n")
            self.proc.stdin.flush()
            return self._read()

    def close(self):
        try:
            self.proc.stdin.write("quit\n")
            self.proc.stdin.flush()
        except Exception:
            pass
        self.proc.terminate()


def quaternion_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1-yy-zz, xy-wz, xz+wy], [xy+wz, 1-xx-zz, yz-wx], [xz-wy, yz+wx, 1-xx-yy]])


def localize(cloud, mask, K, R_wc, t_wc, minimum=20):
    if mask is None or not np.asarray(mask).any():
        return None, 0
    R_cw = np.asarray(R_wc).T
    t_cw = -R_cw @ np.asarray(t_wc)
    points = np.asarray(cloud) @ R_cw.T + t_cw
    points = points[points[:, 2] > 0.1]
    pixels = (K @ points.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    h, w = np.asarray(mask).shape
    ui, vi = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    if not inside.any():
        return None, 0
    selected = np.asarray(mask)[vi[inside], ui[inside]].astype(bool)
    points = points[inside][selected]
    if len(points) < minimum:
        return None, int(len(points))
    distances = np.linalg.norm(points, axis=1)
    point = points[np.argsort(distances)[len(distances) // 2]]
    return np.asarray(R_wc) @ point + np.asarray(t_wc), int(len(points))


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("wildos_perception")
        self.declare_parameter("query", "house")
        self.declare_parameter("worker_python", "/home/ly/vision-cu128/bin/python")
        self.declare_parameter("worker_script", "/home/ly/wildos-opt/vocabulary_query/vision_worker.py")
        self.declare_parameter("checkpoint_root", "/home/ly/ckpts")
        self.declare_parameter("source_root", "/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos")
        self.declare_parameter("output_dir", "/home/ly/wildos-perception-runtime")
        self.declare_parameter("max_points", 50000)
        self.declare_parameter("voxel", 0.15)
        self.declare_parameter("explore_distance", 8.0)
        self.declare_parameter("process_period", 1.0)
        self.declare_parameter("sync_slop", 0.2)
        self.declare_parameter("target_confirmations", 3)
        self.declare_parameter("target_max_jump", 8.0)
        self.declare_parameter("target_miss_tolerance", 2)
        self.declare_parameter("confirmed_miss_tolerance", 10)
        self.query = str(self.get_parameter("query").value)
        self.output = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.latest = {}
        self.busy = False
        self.sequence = 0
        self.last_process_time = 0.0
        self.target_track = {"observations": [], "count": 0, "misses": 0,
                             "confirmed": None}
        self.previous_explore_bearing = None
        self.result_pub = self.create_publisher(String, "/wildos/perception/result", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = nominal_camera_info(960, 640, 60.0, "ego_front_cam")
        self.K = np.array(self.camera_info["k"], dtype=float).reshape(3, 3)
        self.vision = VisionClient(
            str(self.get_parameter("worker_python").value),
            str(self.get_parameter("worker_script").value),
            str(self.get_parameter("checkpoint_root").value),
            str(self.get_parameter("source_root").value),
        )
        self.create_subscription(Image, "/vehicles/ego/sensors/front_cam/colour", self.image_cb, 10)
        self.create_subscription(PointCloud2, "/vehicles/ego/sensors/front_lidar", self.lidar_cb, 10)
        self.create_subscription(StateSensor, "/vehicles/ego/sensors/state", self.state_cb, 10)
        self.get_logger().info(f"WildOS read-only perception started query={self.query}")

    def image_cb(self, msg):
        self.latest["image"] = msg
        self.try_process()

    def lidar_cb(self, msg):
        self.latest["lidar"] = msg
        self.try_process()

    def state_cb(self, msg):
        self.latest["state"] = msg
        self.try_process()

    def try_process(self):
        if self.busy or set(self.latest) != {"image", "lidar", "state"}:
            return
        now = time.monotonic()
        if now - self.last_process_time < float(self.get_parameter("process_period").value):
            return
        data = self.latest
        stamps = {
            key: float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            for key, msg in data.items()
        }
        if max(stamps.values()) - min(stamps.values()) > float(
                self.get_parameter("sync_slop").value):
            self.latest.pop(min(stamps, key=stamps.get), None)
            return
        self.latest = {}
        self.busy = True
        self.last_process_time = now
        try:
            self.process(data)
        except Exception as exc:
            self.get_logger().error(f"perception failed: {exc}")
        finally:
            self.busy = False

    def process(self, data):
        image, lidar, state = data["image"], data["lidar"], data["state"]
        if image.encoding != "rgb8" or int(image.step) != int(image.width) * 3:
            raise ValueError(f"unsupported Camera image: {image.encoding}, step={image.step}")
        if (int(image.width), int(image.height)) != (960, 640):
            raise ValueError(f"unexpected Camera resolution: {image.width}x{image.height}")
        array = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.width, 3)
        image_path = self.output / f"frame_{self.sequence:06d}.png"
        from PIL import Image as PILImage
        PILImage.fromarray(array, mode="RGB").save(image_path)
        cloud = voxel_downsample(decode_xyz(lidar), float(self.get_parameter("voxel").value), int(self.get_parameter("max_points").value))
        run_dir = self.output / f"vision_{self.sequence:06d}"
        result = self.vision.infer(image_path, self.query, run_dir)
        mask_path = run_dir / "region_mask.png"
        from PIL import Image as PILImage
        mask = np.asarray(PILImage.open(mask_path).convert("L")) > 127
        tf = self.tf_buffer.lookup_transform("map", "ego_front_cam", rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        body_rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
        camera_rotation = optical_rotation_from_body(body_rotation)
        target, points = localize(
            cloud, mask, self.K, camera_rotation, np.array([t.x, t.y, t.z]))
        semantic_target = update_track(
            self.target_track, target,
            float(self.get_parameter("target_max_jump").value),
            int(self.get_parameter("target_confirmations").value),
            int(self.get_parameter("target_miss_tolerance").value),
            int(self.get_parameter("confirmed_miss_tolerance").value))
        mode = "target" if semantic_target is not None else "idle"
        target = None if semantic_target is None else np.asarray(semantic_target, dtype=float)
        if semantic_target is None:
            frontier_path = run_dir / "frontier.npy"
            trav_path = run_dir / "traversability.npy"
            frontier = np.load(frontier_path) if frontier_path.is_file() else None
            trav = np.load(trav_path) if trav_path.is_file() else None
            if frontier is not None:
                requested_bearing = exploration_bearing(frontier, trav)
                bearing = smooth_bearing(self.previous_explore_bearing,
                                         requested_bearing)
                self.previous_explore_bearing = bearing
                heading = np.array([state.dir.x, state.dir.y], dtype=float)
                heading /= max(float(np.linalg.norm(heading)), 1e-6)
                c, s = math.cos(bearing), math.sin(bearing)
                direction = np.array([
                    heading[0] * c - heading[1] * s,
                    heading[0] * s + heading[1] * c,
                ])
                distance = float(self.get_parameter("explore_distance").value)
                target = np.array([
                    state.position.x + direction[0] * distance,
                    state.position.y + direction[1] * distance,
                    state.position.z,
                ])
                mode = "explore"
        output = {
            "seq": self.sequence,
            "query": self.query,
            "mode": mode,
            "target_confirmations": self.target_track["count"],
            "target_misses": self.target_track["misses"],
            "mask_pixels": int(mask.sum()),
            "mask_lidar_points": points,
            "regions": result.get("regions", []),
            "semantic_target_world_xyz": None if semantic_target is None else [float(v) for v in semantic_target],
            "target_world_xyz": None if target is None else [float(v) for v in target],
            "read_only": True,
        }
        self.result_pub.publish(String(data=json.dumps(output)))
        self.get_logger().info(json.dumps(output))
        self.sequence += 1


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vision.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
