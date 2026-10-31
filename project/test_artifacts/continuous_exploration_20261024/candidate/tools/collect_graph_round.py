#!/usr/bin/env python3
import json
import os
import sqlite3
import time

import rclpy
from graphnav_msgs.msg import NavigationGraph


class Collector:
    def __init__(self, output):
        self.output = output
        self.msg = None

    def callback(self, msg):
        self.msg = msg


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/spot1/scored_nav_graph')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('wildos_round_collector')
    state = Collector(args.output)
    node.create_subscription(NavigationGraph, args.topic, state.callback, 10)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and state.msg is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    result = {'topic': args.topic, 'received': state.msg is not None}
    if state.msg is not None:
        msg = state.msg
        physical = []
        dynamic = []
        score_nodes = 0
        frontier_nodes = 0
        metadata_keys = {}
        for node_msg in msg.nodes:
            if node_msg.properties:
                score_nodes += any(k.key == 'frontier_scores' for k in node_msg.properties)
                for item in node_msg.properties:
                    metadata_keys[item.key] = metadata_keys.get(item.key, 0) + 1
            if msg.trav_classes:
                idx = min(len(node_msg.robot_traversability), 0)
            for assessment in node_msg.robot_traversability:
                physical.append({
                    'profile': assessment.robot_profile_id,
                    'feasible': assessment.feasible,
                    'physical_traversability': assessment.physical_traversability,
                    'minimum_clearance': assessment.minimum_clearance,
                    'dynamic_risk': assessment.dynamic_risk,
                    'confidence': assessment.confidence,
                })
                dynamic.append(assessment.dynamic_risk)
            if msg.trav_classes:
                try:
                    idx = msg.trav_classes.index('default')
                    if idx < len(node_msg.trav_properties) and node_msg.trav_properties[idx].is_frontier:
                        frontier_nodes += 1
                except ValueError:
                    pass
        result.update({
            'nodes': len(msg.nodes),
            'edges': len(msg.edges),
            'trav_classes': list(msg.trav_classes),
            'score_nodes': int(score_nodes),
            'frontier_nodes': frontier_nodes,
            'metadata_keys': metadata_keys,
            'robot_assessments': physical[:20],
            'dynamic_risk_max': max(dynamic) if dynamic else None,
        })
    memory_path = os.path.expanduser('~/.ros/wildos_visual_memory.sqlite3')
    result['memory_path'] = memory_path
    result['memory_rows'] = None
    if os.path.exists(memory_path):
        db = sqlite3.connect(memory_path)
        result['memory_rows'] = db.execute('select count(*) from experiences').fetchone()[0]
        db.close()
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['received'] else 1)


if __name__ == '__main__':
    main()
