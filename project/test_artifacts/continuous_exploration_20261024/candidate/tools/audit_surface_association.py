"""Real ROS/DDS replay in an isolated domain; no BeamNG sockets or driving.

Uses recorded trial14 projection/camera geometry and actual production
triangulator, consensus, rejection fence and instance release/ACK callbacks.
Action servers, costmaps, motion and semantic model inference are not tested.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
VIS = ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'official_adapter'), str(VIS)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '40'):
        raise ValueError('Use an explicitly isolated ROS domain, never the live domain 40')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from nav_msgs.msg import Odometry
    from object_search_msgs.msg import ObjectMaskWithTf
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String, Header, MultiArrayDimension
    from tf2_msgs.msg import TFMessage
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from omegaconf import OmegaConf
    from beamng_wildos_adapter.goal_standoff import GoalStandoff
    from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
    from visual_navigation.explorfm_triangulation.obj_mask_triangulation import ObjectMaskTriangulator
    import visual_navigation.explorfm_triangulation.localization_geometry as geometry

    record = json.loads((ROOT/'test_artifacts/server_validation_20261009/'
                         'surface_association_fix/recorded_case.json').read_text(encoding='utf-8-sig'))
    row, arrival = record['row'], record['arrival']
    fixture = VIS/'test/fixtures/trial14_ambiguous_surfaces.npz'
    with np.load(fixture) as data:
        points, mask, k = data['points_cam'], data['mask'], data['k']
    camera_rotation, camera_position = np.array(row['camera_rotation']), np.array(row['camera_position'])
    world = points @ camera_rotation.T + camera_position
    far_world = world[points[:, 2] > 10.]
    query = 'green residential house'
    rclpy.init()
    tri = ObjectMaskTriangulator(OmegaConf.create(dict(num_cameras=1,
        parent_frame='spot1/odom', cam_frame='spot1/realsense/{}_color_optical_frame',
        lidar_frame='surface_audit/lidar', object_mask_topic='/spot1/object_mask',
        lidar_topic='/surface_audit/points', navigation_goal_topic='/surface_audit/raw_waypoint')))
    goal, bridge = GoalStandoff(), GraphPathBridge()
    helper = rclpy.create_node('surface_association_audit')
    executor = SingleThreadedExecutor()
    nodes = (tri, goal, bridge, helper)
    for node in nodes:
        executor.add_node(node)
    events, confirmations, raw_goals, statuses = [], [], [], []
    helper.create_subscription(String, '/spot1/object_localization', lambda m: events.append(json.loads(m.data)), 10)
    helper.create_subscription(String, '/spot1/object_confirmation', lambda m: confirmations.append(json.loads(m.data)), 10)
    helper.create_subscription(PoseStamped, '/surface_audit/raw_waypoint', lambda m: raw_goals.append(m), 10)
    query_pub = helper.create_publisher(String, '/spot1/object_search/query', 10)
    odom_pub = helper.create_publisher(Odometry, '/spot1/odom', 10)
    mask_pub = helper.create_publisher(ObjectMaskWithTf, '/spot1/object_mask', 10)
    cloud_pub = helper.create_publisher(PointCloud2, '/surface_audit/points', 10)
    confirmation_pub = helper.create_publisher(String, '/spot1/object_confirmation', 10)
    bearing_pub = helper.create_publisher(String, '/spot1/visible_target_approach', 10)
    broadcaster = StaticTransformBroadcaster(helper)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'spot1/odom', 'surface_audit/lidar'
    transform.transform.rotation.w = 1.
    broadcaster.sendTransform(transform)
    report = dict(scope='Recorded sensor projection through real DDS; synthetic state setup, '
                        'no model inference, Nav2 action server, motion or simulator',
                  domain=os.environ['ROS_DOMAIN_ID'], checks={},
                  loaded_geometry=str(Path(geometry.__file__).resolve()),
                  geometry_sha256=hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest())

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)

    def wait_for(predicate, description, timeout=4.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        if not predicate():
            raise AssertionError(description)

    def publish_sample(cloud):
        stamp = helper.get_clock().now().to_msg()
        stamp_ns = stamp.sec*10**9+stamp.nanosec
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='spot1/odom')
        odom.pose.pose.position.x, odom.pose.pose.position.y = arrival['vehicle_position']
        odom.pose.pose.position.z = arrival['observer_position'][2]
        orientation = arrival['confirmation']['observer_orientation']
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = orientation
        odom_pub.publish(odom)
        camera = CameraInfo()
        camera.header = Header(stamp=stamp, frame_id='spot1/realsense/front_color_optical_frame')
        camera.height, camera.width = mask.shape
        camera.k = k.ravel().tolist()
        cam_tf = TransformStamped()
        cam_tf.header = Header(stamp=stamp, frame_id='spot1/odom')
        cam_tf.child_frame_id = camera.header.frame_id
        t, q = cam_tf.transform.translation, cam_tf.transform.rotation
        t.x, t.y, t.z = camera_position.tolist()
        q.x, q.y, q.z, q.w = Rotation.from_matrix(camera_rotation).as_quat().tolist()
        message = ObjectMaskWithTf()
        message.header, message.query, message.odom = odom.header, query, odom
        message.cam_infos = [camera]
        message.cam_transforms = TFMessage(transforms=[cam_tf])
        message.object_mask.layout.dim = [MultiArrayDimension(label='', size=int(n), stride=0)
                                          for n in (1, 1, *mask.shape)]
        message.object_mask.data = mask.astype(np.uint8).ravel().tolist()
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,
                            frame_id='surface_audit/lidar'), cloud.astype(np.float32)))
        mask_pub.publish(message)
        return stamp_ns

    try:
        spin(1.3)
        query_pub.publish(String(data=query))
        wait_for(lambda: tri.active_query == goal.active_query == bridge.query == query, 'query delivery')
        # Reproduce an existing wrong lock. Ownership is retained until the
        # same release/ACK transaction used by production recovery completes.
        wrong_id = 'recorded-near-wall-instance'
        base = helper.get_clock().now().nanoseconds
        goal.confirmed_instance_anchor = dict(query=query, instance_id=wrong_id,
            observation_stamp_ns=base, object_position=arrival['object_position'], radius=.6)
        bridge.active_object_confirmation = dict(query=query, instance_id=wrong_id,
            observation_stamp_ns=base, object_position=arrival['object_position'][:2])
        bridge.upstream_instance_id = wrong_id
        ambiguous_stamp = publish_sample(world)
        wait_for(lambda: any(m.get('localization_valid') is False for m in events), 'ambiguous localization DDS')
        wait_for(lambda: bridge.confirmation_evidence_not_before_ns > ambiguous_stamp, 'rejection fence DDS')
        report['checks']['ambiguous_surface_no_raw_goal'] = len(raw_goals) == 0
        report['checks']['two_candidates_retained'] = len(events[-1].get('depth_candidates', [])) == 2
        report['checks']['wrong_anchor_not_silently_replaced'] = goal.confirmed_instance_anchor['instance_id'] == wrong_id
        old = dict(arrival['confirmation'], observation_stamp_ns=ambiguous_stamp-1,
            instance_id=wrong_id, localization_valid=True,
            quality=dict(arrival['confirmation']['quality'], association_validated=True))
        confirmation_pub.publish(String(data=json.dumps(old)))
        hint = dict(query=query, frame='spot1/odom', observation_stamp_ns=ambiguous_stamp,
            point=[arrival['vehicle_position'][0]+5., arrival['vehicle_position'][1]],
            bearing=0., evidence='bearing_only', position_confirmed=False, search_confirmed=False)
        bearing_pub.publish(String(data=json.dumps(hint)))
        spin(.4)
        report['checks']['late_wrong_arrival_blocked'] = not bridge.object_found
        report['checks']['same_frame_bearing_preserved'] = bridge.visible_hint == hint
        bridge.request_instance_release()
        bridge.retry_instance_release()
        wait_for(lambda: bridge.instance_release_pending is None and goal.confirmed_instance_anchor is None,
                 'instance release ACK DDS')
        report['checks']['wrong_lock_released_with_ack'] = True
        for _ in range(5):
            stamp = publish_sample(far_world)
            wait_for(lambda: any(m.get('observation_stamp_ns') == stamp and m.get('localization_valid') is True
                                for m in events), 'fresh single-surface localization DDS')
            spin(.15)
        wait_for(lambda: goal.confirmed_instance_anchor is not None, 'fresh supported cluster confirmation')
        centre = np.array(goal.confirmed_instance_anchor['object_position'])
        expected = np.median(far_world, axis=0)
        report['checks']['fresh_far_target_confirms_after_release'] = bool(np.linalg.norm(centre[:2]-expected[:2]) < .05)
        report['checks']['no_false_arrival_at_blue_wall'] = not bridge.object_found
        report['new_anchor'] = goal.confirmed_instance_anchor
        report['localization_events'] = events
        report['confirmations'] = confirmations
        report['raw_goals'] = len(raw_goals)
        report['passed'] = all(report['checks'].values())
    except Exception as exc:
        report['passed'] = False
        report['error'] = repr(exc)
        raise
    finally:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
            print(json.dumps({k: v for k, v in report.items() if k not in ('localization_events', 'confirmations')}, indent=2))
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
