"""Labeled display-only DDS probe, with no vehicle or navigation commands."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--output-topic', default='/wildos/viz/rgb_target')
    parser.add_argument('--bridge-node', default='wildos_visualization_bridge')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '79':
        raise RuntimeError('This probe requires the existing isolated display domain 79')
    if args.output.exists():
        raise FileExistsError(args.output)
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav2_adapter'))
    from wildos_nav2_adapter.model_visualization import rgb_target_image

    rclpy.init()
    node = Node('display_only_image_delivery_probe')
    report = {'scope': 'Synthetic labeled images only; no simulator or navigation commands',
              'passed': False, 'received': [], 'mismatch': [], 'raw_received': 0}
    output_topic = args.output_topic
    raw_topic = '/spot1/realsense/front/color/image_raw'
    expected = {}
    try:
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        for topic in (raw_topic, '/wildos/cmd_vel', '/wildos/native_control',
                      '/wildos/native_control_command', '/spot1/object_mask'):
            owners = node.get_publishers_info_by_topic(topic)
            if owners:
                raise RuntimeError(f'Active producer on {topic}; stop the owned trial before this probe')
        owners = node.get_publishers_info_by_topic(output_topic)
        if len(owners) != 1 or owners[0].node_name != args.bridge_node:
            raise RuntimeError('Expected exactly the existing visualization bridge')
        args.output.mkdir(parents=True)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = node.create_publisher(Image, raw_topic, 10)

        def receive_raw(_):
            report['raw_received'] += 1

        def receive_output(message):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            known = expected.get(stamp)
            if known is None:
                report['mismatch'].append({'stamp': stamp, 'reason': 'Unknown source frame'})
                return
            seq, pixels = known
            match = bytes(message.data) == pixels
            report['received'].append({'sequence': seq, 'stamp_ns': stamp, 'matched': match,
                'age_sec': (node.get_clock().now().nanoseconds-stamp)/1e9})
            if not match:
                report['mismatch'].append({'sequence': seq, 'reason': 'Preview pixels differ'})

        node.create_subscription(Image, raw_topic, receive_raw, qos)
        node.create_subscription(Image, output_topic, receive_output, qos)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        for seq in range(48):
            pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
            pixels[:] = (30, 40 + seq*4, 160)
            cv2.putText(pixels, 'DISPLAY PROBE - SYNTHETIC', (70, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255, 255, 255), 3)
            cv2.putText(pixels, f'FRAME {seq:02d}', (70, 310),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 4)
            message = Image()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'display_probe_synthetic'
            message.height, message.width, message.step = 720, 1280, 3840
            message.encoding = 'bgr8'; message.data = pixels.tobytes()
            preview = rgb_target_image(message, None)
            stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
            expected[stamp] = (seq, bytes(preview.data))
            publisher.publish(message)
            end = time.monotonic() + .125
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        completed = subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
            '-f', 'x11grab', '-video_size', '1280x1024', '-i', os.environ.get('DISPLAY', ':138'),
            '-frames:v', '1', str(args.output/'rviz_synthetic.png')],
            stdin=subprocess.DEVNULL, capture_output=True, timeout=10)
        report['screenshot_returncode'] = completed.returncode
        report['screenshot_error'] = completed.stderr.decode(errors='replace')
        report['sent'] = len(expected)
        report['passed'] = bool(len(report['received']) >= 40 and not report['mismatch']
                                and completed.returncode == 0)
    finally:
        if args.output.exists():
            (args.output/'result.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: v for k, v in report.items() if k not in ('received', 'mismatch')}))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
