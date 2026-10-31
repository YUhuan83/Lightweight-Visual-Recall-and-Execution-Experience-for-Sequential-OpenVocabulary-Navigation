"""One genuine-physics search with interface logs and matched arrival images.

Run the server preflight before constructing a command publisher. Map loading,
vehicle resets and component restarts belong to the round preparation step.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--session-dir',type=Path,required=True)
    parser.add_argument('--arm',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--query',default='green residential house')
    parser.add_argument('--timeout',type=float,default=420.)
    parser.add_argument('--display',default=':138')
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or not 1<=args.timeout<=420:
        parser.error('Search timeout must be between 1 and 420 seconds')
    args.output.mkdir(parents=True,exist_ok=False)
    out=args.output
    report=dict(passed=False,automatic_arrival=False,visual_review_pending=False,
        carrier_model='ackermann',query=args.query,queries_sent=[],
        scenario='Town gas-station spawn; genuine SBR single target',
        map_reload_count=0,vehicle_teleports_during_sequence=0,
        station_preseeded=False,speed_tuning_performed=False,
        arm=str(args.arm.resolve()),domain=os.environ.get('ROS_DOMAIN_ID','0'),
        failure_reason=None,video_recording_enabled=False)
    node=None; beamng=None; bag=None; ros_started=False
    shutdown_errors=[]; started=None; distance=0.
    try:
        vision_pid=int((args.session_dir/'vision.pid').read_text())
        tokens=(Path('/proc')/str(vision_pid)/'cmdline').read_bytes().split(b'\0')
        if b'visual_navigation.wildos.nav' not in tokens:
            raise RuntimeError('Registered vision PID is not the actual model process')
        report['vision_pid']=vision_pid
        preflight=subprocess.run([sys.executable,str(args.arm/'tools/server_test_preflight.py'),
            '--session-dir',str(args.session_dir),'--arm',str(args.arm),
            '--output',str(out/'server_preflight.json'),'--carrier-model','ackermann'],
            text=True,capture_output=True,timeout=150)
        (out/'preflight.log').write_text(preflight.stdout+preflight.stderr)
        if preflight.returncode:
            raise RuntimeError('Server preflight failed; controls were not armed')

        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        from wildos_nav2_adapter.graph_path_bridge import xy
        from wildos_nav2_adapter.test_progress import ManeuverProgress
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from town_trial_harness import Harness
        install_fail_closed_reads(PrefixedLengthSocket)
        beamng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        before=read_physics_snapshot(beamng)
        report['physics_before']=before
        errors=physical_snapshot_errors(before)
        if errors: raise RuntimeError('Physical vehicle contract: '+str(errors))
        if 'east_coast_usa' not in before.get('mission',''):
            raise RuntimeError('Wrong map for the fixed Town experiment')
        spawn_gap=math.dist(before['position'][:2],(734.294,-11.946))
        report['spawn_distance_m']=spawn_gap
        if spawn_gap>1.25 or before.get('speed',99)>.05:
            raise RuntimeError('Vehicle is not stationary at the requested Town spawn')

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO); ros_started=True
        origin=time.monotonic(); report['capture_started_wall_ns']=time.time_ns()
        node=Harness(out,vision_pid,carrier_model='ackermann',record_video=False)
        node.stop()
        if not node.spin(8,lambda:bool(node.frames) and 'odom' in node.state):
            raise RuntimeError('Missing live camera or odometry')
        node.watch()

        # Record the interfaces for diagnosis, without storing RGB/large point clouds twice.
        topics=['/spot1/object_search/query','/spot1/object_mask','/spot1/object_localization',
            '/spot1/object_confirmation','/spot1/visible_target_approach',
            '/spot1/remembered_object_candidates','/spot1/remembered_object_feedback',
            '/wildos/memory_selection','/wildos/plan_status','/wildos/planned_path',
            '/wildos/native_control_status','/wildos/control_status',
            '/wildos/cmd_vel','/wildos/native_control','/wildos/execution_feedback',
            '/vehicles/ego/sensors/state','/tf',
            '/global_costmap/costmap','/local_costmap/costmap','/received_global_plan','/plan',
            '/spot1/odom','/spot1/measurement_timing',
            '/spot1/imgnav_waypoint','/spot1/graphnav_planner/route_status',
            '/spot1/graphnav_planner/rejected_route_endpoint','/spot1/graphnav_planner/path']
        log=(out/'bag.log').open('x')
        try:
            bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'interfaces'),*topics],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        finally: log.close()
        if not node.spin(8,lambda:any(i.node_name.startswith('rosbag2_recorder') for i in
                node.get_subscriptions_info_by_topic('/spot1/object_search/query'))):
            raise RuntimeError('Interface recorder did not subscribe to query')
        if bag.poll() is not None: raise RuntimeError('Interface recorder exited')
        node.phase='single_target_search'
        node.change_query(args.query)
        report['queries_sent']=[args.query]
        if not node.spin(8,lambda:bool(node.last_status)
                and node.last_status.get('query')==args.query
                and node.last_status.get('schema_version')==2):
            raise RuntimeError('Bridge did not acknowledge the physical query/profile')
        node.watch()
        node.set_bool('/wildos_cmd_vel_to_control','enable',True)
        node.set_bool('/wildos_graph_path_bridge','follow_path',True)
        started=time.monotonic(); report['search_started_elapsed']=started-origin
        progress=ManeuverProgress(); last_report=0.; distance=0.; previous=None
        report['search_timeout_s']=args.timeout
        while time.monotonic()-started<args.timeout:
            node.spin(.1); node.watch()
            if bag.poll() is not None: raise RuntimeError('Interface recorder exited during search')
            if 'odom' in node.state:
                point=xy(node.state['odom'].pose.pose)
                if previous is not None: distance+=math.dist(point,previous)
                previous=point
                progress.observe(point,node.last_status)
            if time.monotonic()-last_report>10:
                node.event('search_progress',position=node.state.get('native',{}).get('position'),
                    speed=node.state.get('native',{}).get('speed'),status=node.last_status,
                    mask_count=node.masks.get(args.query,{}).get('count',0),travelled_m=distance)
                last_report=time.monotonic()
            arrival=node.arrival_evidence(args.query,minimum_mask_count=3)
            if arrival:
                images=node.save_target_images(args.query)
                report.update(automatic_arrival=True,visual_review_pending=True,
                    arrival=arrival,arrival_images=images,
                    arrival_vehicle_position=node.state['native'].get('position'))
                node.event('automatic_arrival_pending_visual_review',evidence=arrival,images=images)
                break
        report.update(search_elapsed_s=time.monotonic()-started,travelled_m=distance,
                      final_status=node.last_status)
        if not report['automatic_arrival']:
            report['failure_reason']='search_timeout_without_strict_arrival'
    except BaseException as exc:
        report['failure_reason']=f'{type(exc).__name__}: {exc}'
        report['visual_review_pending']=False
        if node:
            try: node.event('test_error',error=report['failure_reason'])
            except Exception: pass
    finally:
        if started is not None:
            report.setdefault('search_elapsed_s',time.monotonic()-started)
            report.setdefault('travelled_m',distance)
        if node:
            report.setdefault('final_status',node.last_status)
            if report['failure_reason']:
                try: report['failure_image']=node.save_failure_image()
                except Exception as exc: report['failure_image_error']=repr(exc)
        if node:
            try: node.stop(); report['stationary_after_stop']=True
            except Exception as exc: shutdown_errors.append('stop: '+repr(exc))
        if beamng:
            try:
                from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
                after=read_physics_snapshot(beamng); report['physics_after']=after
                settle_started=time.monotonic()
                while after.get('speed',99)>.05 and time.monotonic()-settle_started<5:
                    node.spin(.1) if node else time.sleep(.1)
                    after=read_physics_snapshot(beamng)
                report['physics_after']=after
                report['settle_elapsed_s']=time.monotonic()-settle_started
                errors=physical_snapshot_errors(after)
                if errors or after.get('speed',99)>.05:
                    shutdown_errors.append('final vehicle check: '+str(errors)+' speed='+str(after.get('speed')))
                beamng.control.pause(); report['simulation_paused']=True
            except Exception as exc: shutdown_errors.append('pause/physics: '+repr(exc))
            try: beamng.disconnect()
            except Exception as exc: shutdown_errors.append('disconnect: '+repr(exc))
        if bag:
            try:
                if bag.poll() is None: bag.send_signal(signal.SIGINT)
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag.terminate()
                try: bag.wait(timeout=5)
                except Exception as exc: shutdown_errors.append('bag shutdown: '+repr(exc))
            except Exception as exc: shutdown_errors.append('bag shutdown: '+repr(exc))
        if shutdown_errors:
            report['shutdown_errors']=shutdown_errors
            report['visual_review_pending']=False
            report['failure_reason']=report['failure_reason'] or 'shutdown_or_vehicle_contract_failed'
        report['recordings']={}
        if node:
            report['events']=node.events
            report['queries_sent']=[e['query'] for e in node.events if e['kind']=='query_sent']
            for handle in (node.log,node.native_log,node.sensor_timing_log): handle.close()
            node.destroy_node()
        if ros_started:
            import rclpy
            if rclpy.ok(): rclpy.shutdown()
        (out/'full_test.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','recordings')},indent=2),flush=True)
    # Pending human review is not a passed trial.
    return 0 if report['visual_review_pending'] else 1


if __name__=='__main__':
    raise SystemExit(main())
