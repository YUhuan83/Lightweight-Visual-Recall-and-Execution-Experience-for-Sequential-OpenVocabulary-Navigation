"""Isolated real DDS/action test with synthetic inputs; no BeamNG actuation.

The deliberately slow action server rejects cancellation and returns a late
success. The bridge must still retire that action and retain its destination.
This checks ROS contracts, not motion, obstacle sensing or visual arrival.
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated localhost domain 86')
    import rclpy
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
    from server_test_preflight import process_rows
    competitors = [r for r in process_rows() if r['domain']=='86']
    if competitors: raise RuntimeError('Domain already used: '+str(competitors))
    rclpy.init(args=['--ros-args','-p','carrier_model:=ackermann',
                     '-p','minimum_turning_radius:=8.0','-p','follow_path:=true'])
    report = dict(passed=False, errors=[], phases=[], statuses=[], cancellations=[],
                  accepted=[], returned=[], no_beamng_connection=True,
                  no_physical_execution=True, no_visual_arrival=True, no_video=True)
    origin = time.monotonic(); phase = 'baseline'; position = 0.; healthy = True
    release = threading.Event(); stopping = threading.Event()
    bridge = service = producer = single = parallel = worker = action = None
    try:
        bridge = GraphPathBridge()
        service = rclpy.create_node('execution_transport_test_action_server')
        def execute(handle):
            index = len(report['accepted']); report['accepted'].append(index)
            deadline = time.monotonic()+12.
            while not release.wait(.02):
                if stopping.is_set() or time.monotonic()>deadline:
                    handle.abort(); return FollowPath.Result()
            handle.succeed(); report['returned'].append(index); return FollowPath.Result()
        def cancel(request):
            report['cancellations'].append(dict(phase=phase, elapsed=time.monotonic()-origin))
            return CancelResponse.REJECT  # Forces the late-success ownership case.
        action = ActionServer(service, FollowPath, '/follow_path', execute_callback=execute,
                              cancel_callback=cancel, callback_group=ReentrantCallbackGroup())
        producer = rclpy.create_node('execution_transport_test_inputs')
        native = producer.create_publisher(String, '/wildos/native_control_status', 10)
        odom = producer.create_publisher(Odometry, '/spot1/odom', 10)
        grids = [producer.create_publisher(OccupancyGrid, '/'+name+'_costmap/costmap', 10)
                 for name in ('global','local')]
        producer.create_subscription(String, '/wildos/plan_status', lambda msg:
            report['statuses'].append(dict(phase=phase, data=json.loads(msg.data))), 10)
        def publish():
            stamp = producer.get_clock().now().to_msg()
            native.publish(String(data=json.dumps(dict(wall_time=time.time(), speed=0.,
                actuator_ok=healthy, control_acknowledged=healthy, telemetry_ok=True,
                telemetry_age_sec=0., actual={'brake':1.}, sent={'brake':1.}))))
            msg = Odometry(); msg.header.frame_id='spot1/odom'; msg.header.stamp=stamp
            msg.pose.pose.orientation.w=1.; msg.pose.pose.position.x=position; odom.publish(msg)
            grid = OccupancyGrid(); grid.header=deepcopy(msg.header)
            grid.info.width=grid.info.height=80; grid.info.resolution=1.
            grid.info.origin.position.x=grid.info.origin.position.y=-40.
            grid.info.origin.orientation.w=1.; grid.data=[0]*6400
            for publisher in grids: publisher.publish(grid)
        producer.create_timer(.04,publish)
        single=SingleThreadedExecutor()
        for node in (bridge,producer): single.add_node(node)
        parallel=MultiThreadedExecutor(num_threads=3); parallel.add_node(service)
        worker=threading.Thread(target=parallel.spin,daemon=True); worker.start()
        def spin_until(predicate, timeout=4.):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                single.spin_once(timeout_sec=.02)
                if predicate():return
            raise RuntimeError('DDS phase deadline: '+phase)
        spin_until(lambda: bridge.execution_transport_issue() is None and bridge.odom is not None
                   and bridge.grid is not None and bridge.local_grid is not None)
        assert bridge.follow_client.wait_for_server(timeout_sec=3.)
        route=Route(); route.header=deepcopy(bridge.odom.header)
        for x in (0.,10.):
            p=PoseStamped(); p.header=deepcopy(route.header); p.pose.orientation.w=1.
            p.pose.position.x=x; route.poses.append(p)
        bridge.route=bridge.latest_route=deepcopy(route); bridge.route_endpoint=(10.,0.)
        bridge.motion_queue=[deepcopy(route)]; bridge.send_motion_segment()
        spin_until(lambda: bridge.follow_handle is not None and len(report['accepted'])==1)
        report['phases'].append('healthy_action_accepted')
        old_epoch=bridge.epoch; phase='actuator_failed'; healthy=False
        spin_until(lambda: bridge.last_status.get('status')=='execution_unavailable'
                   and bool(report['cancellations']))
        assert bridge.epoch>old_epoch and not bridge.following and not bridge.motion_queue
        assert bridge.route_endpoint==(10.,0.) and bridge.route is not None
        report['phases'].append('fault_cancels_and_preserves_destination')
        phase='late_success'; release.set()
        spin_until(lambda: bool(report['returned']))
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:single.spin_once(timeout_sec=.02)
        assert bridge.last_status.get('status')=='execution_unavailable'
        assert not any(r['data']['status'] in ('route_done','segment_done','maneuver_done')
                       for r in report['statuses'])
        report['phases'].append('late_success_not_promoted')
        phase='recovered'; healthy=True
        spin_until(lambda: bridge.execution_transport_issue() is None)
        assert bridge.route_endpoint==(10.,0.) and bridge.route is not None
        report['phases'].append('new_fresh_health_can_resume_same_destination')
        report['passed']=True
    except BaseException as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        stopping.set(); release.set()
        if parallel: parallel.shutdown(timeout_sec=4.)
        if worker: worker.join(timeout=4.)
        if single: single.shutdown(timeout_sec=4.)
        if action: action.destroy()
        for node in (bridge,producer,service):
            if node: node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['elapsed']=time.monotonic()-origin
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('statuses','accepted','returned')}))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
