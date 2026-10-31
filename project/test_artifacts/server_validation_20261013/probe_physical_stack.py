"""Sample the live physical input/health chain without issuing motion commands."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from beamng_msgs.msg import StateSensor
    from grid_map_msgs.msg import GridMap
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("physical_stack_interface_probe")
    topics = {
        "/vehicles/ego/sensors/state": StateSensor,
        "/spot1/odom": Odometry,
        "/spot1/realsense/front/color/image_raw": Image,
        "/spot1/ouster/front/points_filtered": PointCloud2,
        "/spot1/local_map": OccupancyGrid,
        "/spot1/local_terrain": GridMap,
        "/global_costmap/costmap": OccupancyGrid,
        "/local_costmap/costmap": OccupancyGrid,
        "/wildos/native_control_status": String,
    }
    rows = {topic: {"count": 0, "first_rx_mono": None, "last_rx_mono": None,
                    "max_rx_gap_s": 0.0, "first_stamp_ns": None, "last_stamp_ns": None,
                    "frame_id": None} for topic in topics}
    latest_native = {}

    def make_callback(topic):
        def callback(message):
            now = time.monotonic()
            row = rows[topic]
            if row["last_rx_mono"] is not None:
                row["max_rx_gap_s"] = max(row["max_rx_gap_s"], now - row["last_rx_mono"])
            row["first_rx_mono"] = row["first_rx_mono"] or now
            row["last_rx_mono"] = now
            row["count"] += 1
            header = getattr(message, "header", None)
            if header is not None:
                stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
                row["first_stamp_ns"] = row["first_stamp_ns"] or stamp
                row["last_stamp_ns"] = stamp
                row["frame_id"] = header.frame_id
            if topic == "/wildos/native_control_status":
                try:
                    latest_native.clear()
                    latest_native.update(json.loads(message.data))
                except (ValueError, TypeError):
                    latest_native.clear()
                    latest_native["parse_error"] = True
            elif topic == "/spot1/realsense/front/color/image_raw":
                row.update(width=message.width, height=message.height, encoding=message.encoding)
            elif topic == "/spot1/ouster/front/points_filtered":
                row.update(width=message.width, point_step=message.point_step,
                           fields=[field.name for field in message.fields])
            elif isinstance(message, OccupancyGrid):
                row.update(width=message.info.width, height=message.info.height,
                           resolution=message.info.resolution,
                           nonzero_cells=sum(value != 0 for value in message.data))
            elif isinstance(message, GridMap):
                row.update(layers=list(message.layers), length_x=message.info.length_x,
                           length_y=message.info.length_y)
        return callback

    subscriptions = [node.create_subscription(
        msg_type, topic, make_callback(topic), qos_profile_sensor_data)
                     for topic, msg_type in topics.items()]
    start = time.monotonic()
    deadline = start + max(1.0, min(args.duration, 20.0))
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    active = latest_native.get("actual") or {}
    sent = latest_native.get("sent") or {}
    telemetry_age = latest_native.get("telemetry_age_sec")
    report = {
        "duration_s": time.monotonic() - start,
        "topics": rows,
        "native_control_status": latest_native,
        "native_health": {
            "actuator_ok": latest_native.get("actuator_ok"),
            "control_acknowledged": latest_native.get("control_acknowledged"),
            "telemetry_ok": latest_native.get("telemetry_ok"),
            "telemetry_age_sec": telemetry_age,
            "speed_m_s": latest_native.get("speed"),
            "sent_brake": sent.get("brake"),
            "actual_brake": active.get("brake"),
            "actual_parkingbrake": active.get("parkingbrake"),
        },
        "control_commands_sent": 0,
    }
    required = ("/vehicles/ego/sensors/state", "/spot1/odom",
                "/spot1/realsense/front/color/image_raw",
                "/spot1/ouster/front/points_filtered", "/spot1/local_map",
                "/wildos/native_control_status")
    errors = [f"{topic}: no live samples" for topic in required if rows[topic]["count"] == 0]
    if rows["/spot1/local_terrain"]["count"] == 0:
        errors.append("/spot1/local_terrain: no live samples")
    if latest_native.get("actuator_ok") is not True:
        errors.append("native actuator is not healthy")
    if latest_native.get("control_acknowledged") is not True:
        errors.append("native actuator has no control acknowledgement")
    if latest_native.get("telemetry_ok") is not True:
        errors.append("native telemetry is not healthy")
    if (isinstance(telemetry_age, bool) or not isinstance(telemetry_age, (int, float))
            or not math.isfinite(telemetry_age) or telemetry_age < 0 or telemetry_age > 0.5):
        errors.append("native telemetry sample is missing, invalid, or stale")
    if rows["/spot1/realsense/front/color/image_raw"].get("encoding") != "rgb8":
        errors.append("camera output is not rgb8")
    if rows["/spot1/ouster/front/points_filtered"].get("point_step") != 12:
        errors.append("filtered LiDAR output is not an XYZ float cloud")
    report["errors"] = errors
    report["passed"] = not errors
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "errors": errors, "topics": rows,
                      "native_health": report["native_health"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
