"""One admitted recorded FollowPath, original RPP/physics, with native contacts."""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as S
from uuid import uuid4


def action_uuid_values(identifier):
    values=[int(v) for v in identifier.uuid]
    if len(values)!=16 or any(not 0<=v<=255 for v in values):
        raise ValueError('Invalid action UUID')
    return values


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle-port',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();assert not args.output.exists()
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters,SetParameters
    from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as Route
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from rosidl_runtime_py.set_message import set_message_fields
    from rosidl_runtime_py.convert import message_to_ordereddict
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
    from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix
    from wildos_nav2_adapter.continuous_path import trim_moving_path
    from wildos_nav2_adapter.motion_path import diagnose_maneuver_join
    from wildos_nav2_adapter.graph_path_bridge import xy,yaw
    from wildos_nav2_adapter.execution_feedback import ExecutionFeedback
    from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
    import yaml
    import contact_chunks
    import recorded_pose
    install_fail_closed_reads(PrefixedLengthSocket)
    rclpy.init();node=Node('recorded_follow_contact_audit')
    fixture=json.loads(Path(__file__).with_name('recorded_section_fixture.json').read_text())
    path=Route();set_message_fields(path,fixture['path'])
    proxy=S(carrier_model='ackermann',follow_enabled=True,following=False,pending=False,
        require_source_control=True,odom=None,control_status=None,control_status_at=None,
        native_status=None,native_status_at=None,stall_anchor=None,motion_queue=[path],
        execution_attempt=None,get_clock=node.get_clock,
        get_parameter=lambda name:S(value={'stall_timeout_sec':5.,'stall_progress_m':.2,'stall_progress_rad':.2}[name]))
    proxy.failure=None
    proxy.execution_transport_unavailable=lambda reason:setattr(proxy,'failure',reason)
    state={};controls=deque(maxlen=512);native=deque(maxlen=512);contact_seq=0
    timeouts={}
    log=args.output.with_suffix('.events.jsonl').open('x')
    report=dict(scope=__doc__,protocol_closed=False,execution_passed=False,errors=[],
        outcome='initializing',goal_sent=False,motion_goals=0,contacts=[],video=False,
        instance_id='audit_'+uuid4().hex,bridge_follow_path=False,
        selection_and_visual_pipeline_tested=False,fixture_candidate=fixture['frozen_candidate'])
    def event(kind,**data):
        log.write(json.dumps(dict(kind=kind,wall_ns=time.time_ns(),mono_ns=time.monotonic_ns(),**data))+'\n');log.flush()
    def on_native(message):
        ExecutionFeedback.on_native_status(proxy,message)
        value=json.loads(message.data);native.append(value)
    def on_control(message):
        ExecutionFeedback.on_control_status(proxy,message);controls.append(json.loads(message.data))
    def on_odom(message):
        if message.header.frame_id=='spot1/odom':proxy.odom=message;state['odom_at']=time.monotonic()
    node.create_subscription(String,'/wildos/native_control_status',on_native,10)
    node.create_subscription(String,'/wildos/control_status',on_control,10)
    node.create_subscription(Odometry,'/spot1/odom',on_odom,10)
    node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:state.update(grid=m),10)
    node.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:state.update(local=m),10)
    node.create_subscription(GoalStatusArray,'/follow_path/_action/status',lambda m:state.update(actions=m),10)
    node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',lambda m:state.update(image=m),1)
    diagnostics=node.create_publisher(String,'/wildos/diagnostic/contact_snapshot',10)
    bng=vehicle=handle=result_future=accept_future=None
    initialized_observer=False
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def wait(future,seconds=4.):
        rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
        if not future.done() or future.exception():raise RuntimeError('Service/action response unavailable')
        return future.result()
    def get_fields(owner,names):
        client=node.create_client(GetParameters,owner+'/get_parameters')
        try:
            assert client.wait_for_service(timeout_sec=4.),owner
            response=wait(client.call_async(GetParameters.Request(names=names)))
            return [parameter_value_to_python(v) for v in response.values]
        finally:node.destroy_client(client)
    def set_enabled(value):
        client=node.create_client(SetParameters,'/wildos_cmd_vel_to_control/set_parameters')
        try:
            assert client.wait_for_service(timeout_sec=4.)
            request=SetParameters.Request(parameters=[Parameter(name='enable',value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,bool_value=value))])
            results=wait(client.call_async(request)).results
            assert len(results)==1 and results[0].successful,'Speed gate update rejected'
            event('speed_gate',enabled=value)
        finally:node.destroy_client(client)
    def contacts(phase):
        nonlocal contact_seq
        start=time.time_ns()
        value=json.loads(vehicle.queue_lua_command(contact_chunks.snapshot(report['instance_id']),response=True))
        contact_seq+=1;assert value['window_sequence']==contact_seq
        assert value['instance_id']==report['instance_id'] and value['protocol']==contact_chunks.PROTOCOL
        sample=dict(phase=phase,request_wall_ns=start,receipt_wall_ns=time.time_ns(),contacts=value,
                    vehicle_id=report['physics_before']['vehicleID'])
        report['contacts'].append(sample);diagnostics.publish(String(data=json.dumps(sample)))
        reference=value.get('reference')
        assert reference is not None and value.get('reference_error') is None,'Snapshot reference unavailable'
        assert abs(reference['engine_sim_time']-value['end_sim_time'])<=1e-6
        assert all(n.get('peak_reference') is not None and n.get('peak_reference_error') is None
                   for n in (value['nodes'] or [])),'Collision-node reference unavailable'
        return value
    def freshness():
        if proxy.odom is None:return 'odom_missing'
        now=node.get_clock().now().nanoseconds
        for name,message,limit in [('odom',proxy.odom,timeouts['odom']),('global_map',state.get('grid'),timeouts['global']),('local_map',state.get('local'),timeouts['local'])]:
            if message is None:return name+'_missing'
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            if not 0<=(now-stamp)*1e-9<=limit:return name+'_stale'
        return ExecutionFeedback.execution_transport_issue(proxy)
    try:
        deadline=time.monotonic()+45
        while (proxy.odom is None or proxy.native_status is None or proxy.control_status is None
               or 'grid' not in state or 'local' not in state):
            if time.monotonic()>deadline:raise TimeoutError('Readiness')
            rclpy.spin_once(node,timeout_sec=.1)
        config_path=Path('/home/ly/smoothing_boundary_repair_20261018/candidate/nav2_adapter/config/nav2_ackermann_control.yaml')
        required=navigation_runtime_contract(yaml.safe_load(config_path.read_text()),'ackermann')
        required['/wildos_cmd_vel_to_control']['enable']=False
        required['/wildos_graph_path_bridge']['follow_path']=False
        actual={}
        for owner,fields in required.items():
            got=get_fields(owner,list(fields));assert len(got)==len(fields)
            actual[owner]=dict(zip(fields,got))
            for (name,want),value in zip(fields.items(),got):assert value==want,(owner,name,value,want)
        report['actual_parameters']=actual
        values=get_fields('/wildos_graph_path_bridge',['odom_timeout_sec','costmap_timeout_sec','local_costmap_timeout_sec'])
        assert len(values)==3 and all(type(v) in (int,float) and v>0 for v in values)
        timeouts.update(zip(('odom','global','local'),values));report['input_timeouts_sec']=timeouts
        endpoints={topic:[dict(name=i.node_name,namespace=i.node_namespace,type=i.topic_type)
                   for i in node.get_publishers_info_by_topic(topic)] for topic in
                   ('/wildos/native_control_command','/wildos/native_control_status','/wildos/cmd_vel_stamped')}
        for topic,owner in [('/wildos/native_control_command','wildos_cmd_vel_to_control'),
                            ('/wildos/native_control_status','wildos_native_vehicle_control'),
                            ('/wildos/cmd_vel_stamped','controller_server')]:
            assert len(endpoints[topic])==1 and endpoints[topic][0]['name']==owner,topic
        report['endpoints']=endpoints
        assert not any(s.status in (1,2,3) for s in state.get('actions',GoalStatusArray()).status_list),'Competing FollowPath'
        bng=BeamNGpy('127.0.0.1',25252,quit_on_close=False,socket_timeout=3).open(launch=False)
        report['physics_before']=read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['physics_before']) and report['physics_before']['speed']<.05
        vehicle=bng.vehicles.get_current()['ego'];vehicle.connection=Connection('127.0.0.1',args.vehicle_port,timeout=3);vehicle.connect(bng)
        report['install']=json.loads(vehicle.queue_lua_command(contact_chunks.install(report['instance_id']),response=True))
        initialized_observer=True;bng.control.resume()
        for _ in range(4):spin(.2);contacts('before_motion')
        point=xy(proxy.odom.pose.pose);heading=yaw(proxy.odom.pose.pose)
        expected=recorded_pose.spawn_from_measurement(fixture['nearby_measured_native_state'])
        report['initial_pose']=[*point,heading]
        assert math.dist(point,expected['rear_axle'][:2])<=.35 and abs(math.remainder(heading-expected['yaw'],2*math.pi))<=.1
        points=[xy(p.pose) for p in path.poses];headings=[yaw(p.pose) for p in path.poses]
        reason=freshness()
        reason=reason or validate_vehicle_path(points,headings,points,state['grid'],points[-1],start=point,
                                               minimum_turning_radius=8.,heading_quantization_bins=72)
        reason=reason or validate_local_prefix(points,headings,state['local'],point,heading)
        join,join_reason=diagnose_maneuver_join(points,headings,point,heading,8.,endpoint_position_tolerance=.2)
        reason=reason or (join_reason if join is None else None)
        if any(s['contacts']['class_events']['body'] for s in report['contacts']):reason=reason or 'body_contact_at_spawn'
        report['admission_reason']=reason
        report['admission_pose']=message_to_ordereddict(proxy.odom)
        if reason is not None:
            report['outcome']='admission_rejected';event('admission_rejected',reason=reason)
        else:
            client=ActionClient(node,FollowPath,'/follow_path');assert client.wait_for_server(timeout_sec=4.)
            set_enabled(True)
            deadline=time.monotonic()+3
            while freshness() is not None:
                if time.monotonic()>deadline:raise TimeoutError('Source ACK after enable: '+str(freshness()))
                spin(.05)
            path.header.stamp=node.get_clock().now().to_msg()
            for pose in path.poses:pose.header.stamp=path.header.stamp
            action=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id=fixture['original_planned_status']['goal_checker_id'])
            accept_future=client.send_goal_async(action)
            report['goal_sent']=True;report['motion_goals']=1
            handle=wait(accept_future);assert handle.accepted,'FollowPath rejected'
            report['goal_uuid']=action_uuid_values(handle.goal_id);result_future=handle.get_result_async()
            proxy.following=True;deadline=time.monotonic()+20.;sample_at=0.
            event('follow_accepted',goal_uuid=report['goal_uuid'])
            while not result_future.done():
                spin(.02)
                issue=freshness()
                if proxy.failure or issue:
                    report['outcome']='transport_or_input_unavailable';report['execution_issue']=proxy.failure or issue;break
                if time.monotonic()>=sample_at:contacts('moving');sample_at=time.monotonic()+.2
                stalled=ExecutionFeedback.execution_stall(proxy)
                if stalled:
                    report['outcome']='execution_stalled';report['stall']=stalled;event('execution_stalled',**stalled);break
                if time.monotonic()>=deadline:report['outcome']='execution_deadline';break
            if result_future.done():
                response=result_future.result();report['action_status']=response.status
                final=proxy.odom.pose.pose
                distance=math.dist(xy(final),points[-1]);error=abs(math.remainder(yaw(final)-headings[-1],2*math.pi))
                report['final_pose']=[*xy(final),yaw(final)];report['endpoint_error_m']=distance
                report['execution_passed']=response.status==4 and distance<=.2 and error<=.35
                report['outcome']='section_reached' if report['execution_passed'] else 'action_not_reached'
            event('execution_end',outcome=report['outcome'])
        report['protocol_closed']=True
    except BaseException as exc:report['errors'].append(repr(exc));event('probe_error',error=repr(exc))
    finally:
        try:set_enabled(False)
        except Exception as exc:report['errors'].append('close speed gate: '+repr(exc))
        if handle is not None and handle.accepted and result_future is not None and not result_future.done():
            try:
                wait(handle.cancel_goal_async());terminal=wait(result_future,6.)
                assert terminal.status in (4,5,6)
                report['stopped_action_status']=terminal.status
            except Exception as exc:report['errors'].append('action terminal: '+repr(exc))
        elif accept_future is not None and not accept_future.done():report['errors'].append('Unresolved action acceptance')
        if vehicle and initialized_observer:
            try:
                from check_continuous_start_execution import settle
                stopped,samples=settle(lambda:read_physics_snapshot(bng),spin,timeout=8.)
                assert not physical_snapshot_errors(stopped)
                report['stop']=dict(physics=stopped,samples=samples)
                contacts('after_stop')
                report['unload']=json.loads(vehicle.queue_lua_command(contact_chunks.uninstall(report['instance_id']),response=True))
                assert report['unload']['unloaded'] is True
            except Exception as exc:report['errors'].append('stop/unload: '+repr(exc))
        if bng:
            try:bng.control.pause()
            except Exception as exc:report['errors'].append('pause: '+repr(exc))
        if vehicle:vehicle.disconnect()
        if bng:bng.disconnect()
        report['event_totals']={kind:sum(s['contacts']['class_events'][kind] for s in report['contacts']) for kind in ('body','wheel','unknown')}
        report['protocol_closed']=report['protocol_closed'] and not report['errors']
        report['execution_passed']=report['execution_passed'] and report['protocol_closed']
        report['complete_navigation_passed']=False
        report['maximum_native_speed']=max((v['speed'] for v in native),default=None)
        if 'image' in state:
            try:
                import numpy as np
                from PIL import Image as PillowImage
                message=state['image'];raw=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
                if message.encoding=='bgr8':raw=raw[:,:,::-1]
                else:assert message.encoding=='rgb8'
                image=args.output.with_suffix('.last_rgb.png');PillowImage.fromarray(raw).save(image)
                report['image']=str(image)
                report['image_source']=dict(stamp=message_to_ordereddict(message.header.stamp),
                                           frame=message.header.frame_id,is_arrival_evidence=False)
            except Exception as exc:report['image_error']=repr(exc)
        log.close();node.destroy_node();rclpy.shutdown()
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('contacts','actual_parameters','physics_before','stop','admission_pose')},indent=2))
    return 0 if report['protocol_closed'] else 1


if __name__=='__main__':raise SystemExit(main())
