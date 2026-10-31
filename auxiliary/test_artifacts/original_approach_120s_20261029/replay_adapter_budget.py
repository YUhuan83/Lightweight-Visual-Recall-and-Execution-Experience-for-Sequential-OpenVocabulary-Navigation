"""Test-only real ROS Adapter processing/DDS replay; never starts BeamNG."""
import argparse
import importlib.util
import json
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap
from beamng_msgs.msg import StateSensor
from beamng_wildos_adapter.node import Adapter
import beamng_wildos_adapter.mapping as mapping
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET


def percentile(values, q):
    return float(np.quantile(values, q)) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--module')
    parser.add_argument('--label', required=True)
    parser.add_argument('--root', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--frames', type=int, default=48)
    args = parser.parse_args()
    if args.module:
        spec = importlib.util.spec_from_file_location('review_cloud', args.module)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        mapping.split_costmap_points = module.split_costmap_points

    root = Path(args.root)
    frames = []
    for number in (40, 46):
        data = np.load(root / 'request_audit' /
                       f'request_{number}__spot1_ouster_front_points_filtered.npz')
        body, world = data['body'], data['world']
        transform = np.linalg.lstsq(np.column_stack((body, np.ones(len(body)))), world,
                                    rcond=None)[0]
        rotation, base = transform[:3].T, transform[3]
        assert np.max(np.abs(body @ rotation.T + base - world)) < 1e-4
        native = base - rotation @ np.asarray(SBR_BASE_OFFSET)
        frames.append((world, rotation, native))

    rclpy.init()
    adapter = Adapter()
    feeder = Node('adapter_budget_feeder')
    sink = Node('adapter_budget_sink')
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (adapter, feeder, sink):
        executor.add_node(node)
    counts = dict(cloud=0, occupancy=0, terrain=0)
    subscriptions = []
    for kind, topic, key in (
        (PointCloud2, '/spot1/ouster/front/points_filtered', 'cloud'),
        (OccupancyGrid, '/spot1/local_map', 'occupancy'),
        (GridMap, '/spot1/local_terrain', 'terrain'),
    ):
        def count(message, key=key):
            counts[key] += 1
        subscriptions.append(sink.create_subscription(kind, topic, count, 64))
    states, sent = {}, {}
    completed, callback_s, frame_lag_s, pulse_lag_s, pulse_times = [], [], [], [], []
    failures = []

    def cloud_callback(message):
        stamp = message.header.stamp
        ns = stamp.sec * 10**9 + stamp.nanosec
        started = time.monotonic()
        try:
            adapter.process_lidar(message, states[ns], stamp)
        except Exception as error:
            failures.append(repr(error))
        callback_s.append(time.monotonic() - started)
        frame_lag_s.append(time.monotonic() - sent[ns])
        completed.append(ns)

    def pulse_callback(message):
        now = time.monotonic()
        pulse_lag_s.append(now - float(message.data))
        pulse_times.append(now)

    subscriptions.append(adapter.create_subscription(PointCloud2, '/review/raw', cloud_callback, 64))
    subscriptions.append(adapter.create_subscription(String, '/review/pulse', pulse_callback, 256))
    publisher = feeder.create_publisher(PointCloud2, '/review/raw', 64)
    pulse = feeder.create_publisher(String, '/review/pulse', 256)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(.75)
    start = time.monotonic()
    next_cloud = start
    next_pulse = start
    sequence = 0
    while sequence < args.frames:
        now = time.monotonic()
        if now >= next_pulse:
            pulse.publish(String(data=str(now)))
            next_pulse += .05
        if now >= next_cloud:
            world, rotation, native = frames[sequence % len(frames)]
            stamp = feeder.get_clock().now().to_msg()
            ns = stamp.sec * 10**9 + stamp.nanosec
            state = StateSensor()
            state.header.stamp = stamp
            for name, vector in (('position', native), ('dir', rotation[:, 0]),
                                 ('up', rotation[:, 2])):
                value = getattr(state, name)
                value.x, value.y, value.z = map(float, vector)
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = 'world'
            message.height, message.width = 1, len(world)
            message.point_step, message.row_step = 12, 12 * len(world)
            message.is_dense = True
            message.fields = [PointField(name=name, offset=index * 4,
                                        datatype=PointField.FLOAT32, count=1)
                              for index, name in enumerate(('x', 'y', 'z'))]
            message.data = np.asarray(world, dtype='<f4').tobytes()
            states[ns] = state
            sent[ns] = time.monotonic()
            publisher.publish(message)
            sequence += 1
            next_cloud = start + sequence / 6.
        time.sleep(.001)
    feed_end = time.monotonic()
    deadline = feed_end + 6.
    while len(completed) < args.frames and time.monotonic() < deadline:
        time.sleep(.02)
    time.sleep(.25)
    report = dict(
        label=args.label, adapter_source=__import__('inspect').getfile(Adapter),
        classifier_source=args.module or __import__('inspect').getfile(mapping.split_costmap_points),
        scope='Actual Adapter.process_lidar + input/output DDS, paired recorded geometry, synthetic matched timestamps. No simulator, state association, actuator or full navigation.',
        input_hz=6., sent=sequence, completed=len(completed), outputs=counts,
        failures=failures, feed_s=feed_end-start,
        drain_s=time.monotonic()-feed_end,
        process_median_s=percentile(callback_s, .5),
        process_p95_s=percentile(callback_s, .95),
        completion_lag_median_s=percentile(frame_lag_s, .5),
        completion_lag_p95_s=percentile(frame_lag_s, .95),
        completion_lag_max_s=max(frame_lag_s, default=0.),
        pulse_lag_p95_s=percentile(pulse_lag_s, .95),
        pulse_lag_max_s=max(pulse_lag_s, default=0.),
        pulse_callback_gap_max_s=max(np.diff(pulse_times), default=0.),
        all_callback_s=callback_s, all_completion_lag_s=frame_lag_s,
    )
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if not k.startswith('all_')}, indent=2))
    executor.shutdown(timeout_sec=3.)
    spin.join(timeout=3.)
    for node in (adapter, feeder, sink):
        node.destroy_node()
    rclpy.shutdown()
    assert len(completed) == args.frames and not failures, report


if __name__ == '__main__':
    main()
