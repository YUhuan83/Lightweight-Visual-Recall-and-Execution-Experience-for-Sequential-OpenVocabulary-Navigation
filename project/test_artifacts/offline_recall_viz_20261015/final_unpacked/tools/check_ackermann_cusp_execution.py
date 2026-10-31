"""Bounded real-vehicle GraphPathBridge handoff probe; not a target trial.

Run with vision/triangulation/GraphNav planner stopped, sensors and physical
stack active. A read-only planner sweep selects one short reversing path before
arming. The production bridge owns planning, FollowPath and its handoff checks.
No synthetic arrival, map clearing, teleport, speed or collision changes.
"""
import argparse
import copy
from collections import Counter
import json
import math
import os
from pathlib import Path
import time


def candidate_check(status, points, headings, start, heading, global_grid, local_grid):
    """Check the real path before opening either control gate."""
    from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix
    from wildos_nav2_adapter.motion_path import (
        motion_sections,initial_maneuver_already_reached,diagnose_maneuver_join,
        ACKERMANN_CUSP_POSITION_TOLERANCE_M)
    from wildos_nav2_adapter.tracking_preview import preview_tracking
    sections=motion_sections(points)
    lengths=[sum(math.dist(points[i],points[i+1]) for i in range(a,b)) for a,b in sections]
    if status!=4: return 'planner_not_success'
    if len(sections)<2: return 'no_cusp'
    # Same minimum as the continuous-start planner: at least two checker
    # radii. The checker remains .2 m; a larger arbitrary probe minimum must
    # not make it impossible to exercise a valid production connector.
    if sum(lengths)>24 or min(lengths)<2*ACKERMANN_CUSP_POSITION_TOLERANCE_M:
        return 'outside_component_length'
    if initial_maneuver_already_reached(points,headings,start,heading): return 'initial_section_already_reached'
    reason=validate_vehicle_path(points,headings,[start,points[-1]],global_grid,points[-1],
        start=start,minimum_turning_radius=8.,heading_quantization_bins=72)
    if reason: return reason
    current=(*start,heading)
    for a,b in sections:
        preview=preview_tracking(points[a:b+1],headings[a:b+1],current[:2],current[2],8.)
        if not preview.reached:return preview.reason
        current=preview.trajectory[-1]
    a,b=sections[0];p,h=points[a:b+1],headings[a:b+1]
    reason=validate_local_prefix(p,h,local_grid,start,heading)
    if reason: return reason
    join,reason=diagnose_maneuver_join(p,h,start,heading,8.,endpoint_position_tolerance=.2)
    return None if join is not None else reason


def execution_issue(events, recovery_budget):
    """One rejected join can recover; repeated recovery must remain bounded."""
    statuses=[e for e in events if e.get('kind')=='plan_status']
    bad={e.get('status') for e in statuses}&{'follow_failed','path_changed_before_execution',
        'start_footprint_blocked','no_vehicle_path','no_free_graph_candidate'}
    if bad:return 'execution_failure:'+','.join(sorted(bad))
    recoveries=[e for e in statuses if e.get('status')=='replanning_after_maneuver']
    if any(e.get('reason')!='actual_cusp_pose_differs' for e in recoveries):
        return 'unexpected_handoff_recovery'
    if len(recoveries)>recovery_budget:return 'excessive_handoff_recovery'
    return None


def observed_motion_directions(samples):
    directions=set()
    for row in samples:
        speed=row.get('speed');gear=row.get('actual',{}).get('gear_index')
        if (row.get('telemetry_ok') is True and row.get('control_acknowledged') is True
                and type(speed) in (int,float) and math.isfinite(speed) and speed>.1
                and type(gear) in (int,float) and math.isfinite(gear) and gear!=0):
            directions.add(1 if gear>0 else -1)
    return sorted(directions)


def publisher_snapshot(node, topics):
    """Record discovery identities before enforcing command ownership."""
    return {topic:[{'node_name':p.node_name,'node_namespace':p.node_namespace,
        'topic_type':p.topic_type,'endpoint_gid':list(p.endpoint_gid)}
        for p in node.get_publishers_info_by_topic(topic)] for topic in topics}


def wait_for_publishers(node, expected, spin, *, clock=time.monotonic,
                        timeout=5., stable=.5):
    """Wait for asynchronous discovery; reject conflicts instead of arming."""
    began=clock();ready_since=None;history=[];snapshot={}
    while clock()-began<=timeout:
        snapshot=publisher_snapshot(node,expected)
        history.append({'elapsed':clock()-began,'publishers':snapshot})
        complete=True
        for topic,owners in expected.items():
            found=Counter((p['node_name'],p['node_namespace'],p['topic_type'])
                          for p in snapshot[topic]);required=Counter(owners)
            if any(count>required[owner] for owner,count in found.items()):
                return snapshot,history,'unexpected_or_duplicate_publisher:'+topic
            complete=complete and found==required
        if complete:
            if ready_since is None:ready_since=clock()
            if clock()-ready_since>=stable:return snapshot,history,None
        else:ready_since=None
        spin(.1)
    return snapshot,history,'publisher_discovery_timeout'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--arm',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(exist_ok=False,parents=True)
    out=args.output;report={'scope':'Physical bridge reversing handoff component; no target/20-round acceptance',
        'passed':False,'errors':[],'plans':[],'map_reloads':0,'teleports':0,
        'video_recording_enabled':False,'visual_arrival_validated':False}
    node=None;beamng=None;initialized=False
    try:
        from server_test_preflight import process_rows
        rows=process_rows();domain=os.environ.get('ROS_DOMAIN_ID','0')
        for marker in ('planner_node','visual_navigation.wildos.nav','obj_mask_triangulation.py'):
            assert not any(r['domain']==domain and any(Path(t).name==marker or t==marker
                for t in r['tokens']) for r in rows),'Competing route/vision owner: '+marker
        from beamngpy import BeamNGpy
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
        from wildos_nav2_adapter.motion_path import motion_sections,initial_maneuver_already_reached
        from wildos_nav2_adapter.graph_path_bridge import xy,yaw
        import yaml
        import rclpy
        from rclpy.action import ActionClient
        from rcl_interfaces.srv import GetParameters
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import Path as Route,OccupancyGrid
        from std_msgs.msg import String
        from town_trial_harness import Harness
        rclpy.init();initialized=True
        node=Harness(out,os.getpid(),carrier_model='ackermann',record_video=False)
        node.phase='cusp_component';node.query=''
        node.create_subscription(OccupancyGrid,'/local_costmap/costmap',
            lambda m:node.state.update(local=m,local_at=time.monotonic()),10)
        def wait(f,seconds=8):
            rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
            if not f.done() or f.exception():raise RuntimeError('Component service/action timeout')
            return f.result()
        config=yaml.safe_load((args.arm/'nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
        required=navigation_runtime_contract(config,carrier_model='ackermann')
        required['/wildos_cmd_vel_to_control']['enable']=False
        required['/wildos_graph_path_bridge']['follow_path']=False
        actual={}
        for owner,fields in required.items():
            client=node.create_client(GetParameters,owner+'/get_parameters')
            assert client.wait_for_service(timeout_sec=4),owner
            result=wait(client.call_async(GetParameters.Request(names=list(fields))))
            assert len(result.values)==len(fields),owner
            actual[owner]={}
            for (key,expected),value in zip(fields.items(),result.values):
                got={1:value.bool_value,2:value.integer_value,3:value.double_value,
                     4:value.string_value,9:list(value.string_array_value)}.get(value.type)
                actual[owner][key]=got
                assert got==expected,(owner,key,got,expected)
            node.destroy_client(client)
        report['actual_parameters']=actual
        expected_publishers={
            '/wildos/native_control':[('wildos_cmd_vel_to_control','/','beamng_msgs/msg/VehicleControl')],
            '/wildos/native_control_status':[('wildos_native_vehicle_control','/','std_msgs/msg/String')],
            '/wildos/cmd_vel':[(node.get_name(),'/','geometry_msgs/msg/Twist'),
                               ('controller_server','/','geometry_msgs/msg/Twist')]}
        publishers,history,issue=wait_for_publishers(node,expected_publishers,node.spin)
        report['publishers_before_arm']=publishers;report['publisher_discovery']=history
        assert issue is None,(issue,publishers)
        assert not node.get_publishers_info_by_topic('/spot1/graphnav_planner/path'),'Unexpected graph route publisher'
        assert node.spin(5,lambda:all(k in node.state for k in ('odom','grid','local','native')) and bool(node.frames))
        beamng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        report['physics_before']=read_physics_snapshot(beamng)
        assert not physical_snapshot_errors(report['physics_before'])
        assert report['physics_before']['speed']<.05,'Vehicle moving before component'
        node.stop();pose=copy.deepcopy(node.state['odom'].pose.pose);start=xy(pose);heading=yaw(pose)
        report['initial']=[*start,heading]
        client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        assert client.wait_for_server(timeout_sec=4)
        chosen=None
        for dx,dy,turn in ((3.,1.,.6),(4.,2.,.8),(-4.,2.,.6),(-3.,-1.,-.6),(6.,-2.,-.8)):
            goal=ComputePathToPose.Goal();goal.planner_id='CarLike'
            goal.goal.header.frame_id=node.state['odom'].header.frame_id
            goal.goal.header.stamp=node.get_clock().now().to_msg();goal.goal.pose=copy.deepcopy(pose)
            p=goal.goal.pose.position;p.x=start[0]+dx*math.cos(heading)-dy*math.sin(heading)
            p.y=start[1]+dx*math.sin(heading)+dy*math.cos(heading)
            goal.goal.pose.orientation.z=math.sin((heading+turn)/2);goal.goal.pose.orientation.w=math.cos((heading+turn)/2)
            handle=wait(client.send_goal_async(goal));assert handle.accepted
            result=wait(handle.get_result_async());points=[xy(p.pose) for p in result.result.path.poses]
            headings=[yaw(p.pose) for p in result.result.path.poses];sections=motion_sections(points)
            lengths=[sum(math.dist(points[i],points[i+1]) for i in range(a,b)) for a,b in sections]
            item={'relative_goal':[dx,dy,turn],'status':result.status,'sections':sections,'lengths':lengths,
                  'points':points,'headings':headings}
            item['rejection']=candidate_check(result.status,points,headings,start,heading,
                                              node.state['grid'],node.state['local'])
            report['plans'].append(item)
            if item['rejection'] is None:
                chosen=goal;report['selected']=item;break
        assert chosen is not None,'No short feasible cusp plan; controls stay closed'
        routes=node.create_publisher(Route,'/spot1/graphnav_planner/path',10)
        metadata=node.create_publisher(String,'/spot1/graphnav_planner/route_status',10)
        assert node.spin(4,lambda:routes.get_subscription_count()>0 and metadata.get_subscription_count()>0)
        node.set_bool('/wildos_cmd_vel_to_control','enable',True)
        node.set_bool('/wildos_graph_path_bridge','follow_path',True)
        route=Route();route.header=copy.deepcopy(chosen.goal.header);route.header.stamp=node.get_clock().now().to_msg()
        begin=copy.deepcopy(chosen.goal);begin.pose=copy.deepcopy(pose);begin.header=route.header
        end=copy.deepcopy(chosen.goal);end.header=route.header;route.poses=[begin,end]
        stamp=route.header.stamp.sec*10**9+route.header.stamp.nanosec
        metadata.publish(String(data=json.dumps({'stamp_ns':stamp,'goal':list(xy(end.pose)),
            'complete':True,'mode':'component','observation_heading':yaw(end.pose)})))
        routes.publish(route);report['armed_at_ns']=time.time_ns();deadline=time.monotonic()+100
        terminal=None
        while time.monotonic()<deadline:
            node.spin(.1)
            native=node.state.get('native',{})
            assert native.get('telemetry_ok') and native.get('actuator_ok'),'Native health failure'
            assert time.monotonic()-node.state.get('native_at',0)<1.,'Stale native telemetry'
            assert not node.state.get('runtime_profile_error'),'Changed runtime profile'
            states=[e.get('status') for e in node.events if e.get('kind')=='plan_status']
            issue=execution_issue(node.events,len(report['selected']['sections'])-1)
            assert issue is None,issue
            if node.last_status and node.last_status.get('status')=='route_done':terminal=node.last_status;break
        assert terminal is not None,'Component did not complete in 100 seconds'
        assert 'maneuver_done' in states,'Actual bridge did not exercise a cusp'
        report['handoff_recoveries']=[e for e in node.events if e.get('status')=='replanning_after_maneuver']
        final=node.state['odom'].pose.pose;report['final']=[*xy(final),yaw(final)]
        report['endpoint_distance_m']=math.dist(xy(final),xy(end.pose))
        assert report['endpoint_distance_m']<=1.
        report['terminal']=terminal;report['passed']=True
    except BaseException as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        if node:
            try:node.stop()
            except Exception as exc:report['errors'].append('Stop: '+repr(exc))
            if report['errors']:
                try:report['failure_image']=node.save_failure_image()
                except Exception as exc:report['failure_image_error']=repr(exc)
        if beamng:
            try:
                from check_continuous_start_execution import settle
                after,samples=settle(lambda:read_physics_snapshot(beamng),node.spin)
                report['physics_after']=after;report['stop_samples']=samples
                assert not physical_snapshot_errors(after),'Final physics changed'
                beamng.control.pause()
            except Exception as exc:report['errors'].append('Physics: '+repr(exc))
            beamng.disconnect()
        report['passed']=report['passed'] and not report['errors']
        if node:
            report['events']=node.events
            for log in (node.log,node.native_log,node.sensor_timing_log):log.close()
            samples=[json.loads(line) for line in (out/'native_samples.jsonl').read_text().splitlines()]
            directions=observed_motion_directions(samples)
            report['observed_motion_directions']=directions
            if report['passed'] and directions!=[-1,1]:
                report['passed']=False;report['errors'].append('Both actual forward/reverse motion were not observed')
            node.destroy_node()
        if initialized:rclpy.shutdown()
        (out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('plans','events','actual_parameters','recordings')},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
