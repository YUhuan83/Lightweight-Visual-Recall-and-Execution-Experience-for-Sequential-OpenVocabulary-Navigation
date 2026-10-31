"""Real bridge component using automatic Smac entry; no target/benchmark/video."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time


def settle(read, spin, clock=time.monotonic, timeout=5., stable=.5):
    """Require fresh SDK reads below .05 continuously, without pausing physics."""
    began = clock(); quiet = None; samples = []
    while clock()-began <= timeout:
        snapshot = read(); now = clock(); speed = snapshot.get('speed', float('inf'))
        if not math.isfinite(speed) or speed < 0:
            raise RuntimeError('Invalid SDK speed')
        samples.append({'elapsed': now-began, 'speed': speed})
        quiet = (now if quiet is None else quiet) if speed < .05 else None
        if quiet is not None and now-quiet >= stable:
            return snapshot, samples
        spin(.1)
    raise RuntimeError('No sustained SDK stop below .05 within deadline')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--arm',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    out=args.output;node=None;beamng=None;initialized=False
    report={'scope':'Automatic planner + real GraphPathBridge short execution only',
        'passed':False,'errors':[],'plans':[],'video_recording_enabled':False,
        'map_reloads':0,'teleports':0,'visual_arrival_validated':False}
    try:
        import rclpy
        import yaml
        from rclpy.action import ActionClient
        from rcl_interfaces.srv import GetParameters
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import Path as Route,OccupancyGrid
        from std_msgs.msg import String
        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
        from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
        from wildos_nav2_adapter.graph_path_bridge import xy,yaw
        from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix
        from wildos_nav2_adapter.motion_path import motion_sections,diagnose_maneuver_join
        from town_trial_harness import Harness
        rclpy.init();initialized=True
        node=Harness(out,os.getpid(),carrier_model='ackermann',record_video=False)
        node.phase='automatic_entry_component';node.query=''
        node.create_subscription(OccupancyGrid,'/local_costmap/costmap',
            lambda m:node.state.update(local=m,local_at=time.monotonic()),10)
        def wait(f,seconds=6):
            rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
            if not f.done() or f.exception(): raise RuntimeError('Component service/action timeout')
            return f.result()
        config=yaml.safe_load((args.arm/'nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
        required=navigation_runtime_contract(config,'ackermann')
        required['/wildos_cmd_vel_to_control']['enable']=False
        required['/wildos_graph_path_bridge']['follow_path']=False
        actual={}
        for owner,fields in required.items():
            c=node.create_client(GetParameters,owner+'/get_parameters')
            assert c.wait_for_service(timeout_sec=4),owner
            response=wait(c.call_async(GetParameters.Request(names=list(fields))))
            assert len(response.values)==len(fields),owner
            actual[owner]={}
            for (key,expected),v in zip(fields.items(),response.values):
                got={1:v.bool_value,2:v.integer_value,3:v.double_value,
                     4:v.string_value,9:list(v.string_array_value)}.get(v.type)
                actual[owner][key]=got;assert got==expected,(owner,key,got,expected)
            node.destroy_client(c)
        report['actual_parameters']=actual
        assert not node.get_publishers_info_by_topic('/spot1/graphnav_planner/path'),'Competing route owner'
        for topic,count in [('/wildos/native_control_command',1),('/wildos/native_control_status',1),('/wildos/cmd_vel',2)]:
            assert len(node.get_publishers_info_by_topic(topic))==count,topic
        assert node.spin(5,lambda:all(k in node.state for k in ('odom','grid','local','native')) and bool(node.frames))
        install_fail_closed_reads(PrefixedLengthSocket)
        beamng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
        report['physics_before']=read_physics_snapshot(beamng)
        assert not physical_snapshot_errors(report['physics_before'])
        assert report['physics_before']['speed']<.05
        node.stop();pose=copy.deepcopy(node.state['odom'].pose.pose);start=xy(pose);heading=yaw(pose)
        report['initial']=[*start,heading]
        client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        assert client.wait_for_server(timeout_sec=4)
        chosen=None
        for dx,dy,turn in ((-3.,0.,0.),(3.,0.,0.),(-4.,1.,.25),(4.,1.,.25)):
            goal=ComputePathToPose.Goal();goal.planner_id='CarLike';goal.use_start=False
            goal.goal.header=copy.deepcopy(node.state['odom'].header)
            goal.goal.header.stamp=node.get_clock().now().to_msg();goal.goal.pose=copy.deepcopy(pose)
            goal.goal.pose.position.x=start[0]+dx*math.cos(heading)-dy*math.sin(heading)
            goal.goal.pose.position.y=start[1]+dx*math.sin(heading)+dy*math.cos(heading)
            goal.goal.pose.orientation.z=math.sin((heading+turn)/2);goal.goal.pose.orientation.w=math.cos((heading+turn)/2)
            began=time.monotonic();handle=wait(client.send_goal_async(goal));assert handle.accepted
            result=wait(handle.get_result_async());points=[xy(p.pose) for p in result.result.path.poses]
            headings=[yaw(p.pose) for p in result.result.path.poses];sections=motion_sections(points)
            item={'relative_goal':[dx,dy,turn],'status':result.status,'elapsed_s':time.monotonic()-began,
                  'sections':sections,'points':points,'headings':headings}
            report['plans'].append(item)
            if result.status!=4 or not sections: continue
            length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            reason=validate_vehicle_path(points,headings,[start,points[-1]],node.state['grid'],points[-1],
                start=start,minimum_turning_radius=8.,heading_quantization_bins=72)
            a,b=sections[0];q=points[a:b+1];h=headings[a:b+1]
            local=validate_local_prefix(q,h,node.state['local'],start,heading)
            join,join_reason=diagnose_maneuver_join(q,h,start,heading,8.)
            item.update(length_m=length,global_reason=reason,local_reason=local,join_reason=join_reason)
            if reason is None and local is None and join is not None and .8<=length<=12.:
                chosen=goal;report['selected']=item;break
        assert chosen is not None,'No safe bounded short path; controls remain closed'
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
        routes.publish(route);deadline=time.monotonic()+75
        while time.monotonic()<deadline:
            node.spin(.1);native=node.state.get('native',{})
            assert native.get('telemetry_ok') and native.get('actuator_ok'),'Native health failure'
            assert time.monotonic()-node.state.get('native_at',0)<1.,'Native receipt stale'
            assert not node.state.get('runtime_profile_error'),'Runtime profile changed'
            states=[e.get('status') for e in node.events if e.get('kind')=='plan_status']
            bad=set(states)&{'follow_failed','replanning_after_maneuver','no_vehicle_path','stale_input','stale_local_costmap'}
            assert not bad,sorted(bad)
            if node.last_status and node.last_status.get('status')=='route_done':break
        assert node.last_status and node.last_status.get('status')=='route_done','Execution deadline'
        final=node.state['odom'].pose.pose;report['final']=[*xy(final),yaw(final)]
        report['displacement_m']=math.dist(start,xy(final))
        report['endpoint_distance_m']=math.dist(xy(final),xy(end.pose))
        assert report['displacement_m']>.5 and report['endpoint_distance_m']<=1.
        report['terminal']=node.last_status;report['passed']=True
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
                after,samples=settle(lambda:read_physics_snapshot(beamng),node.spin)
                report['physics_after']=after;report['stop_samples']=samples
                assert not physical_snapshot_errors(after),'Physics identity/collision changed'
            except Exception as exc:report['errors'].append('Final stop/physics: '+repr(exc))
            try:beamng.control.pause();report['simulation_paused']=True
            except Exception as exc:report['errors'].append('Pause: '+repr(exc))
            beamng.disconnect()
        if node:
            report['events']=node.events
            for log in (node.log,node.native_log,node.sensor_timing_log):log.close()
            node.destroy_node()
        if initialized:rclpy.shutdown()
        report['passed']=report['passed'] and not report['errors']
        (out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('plans','events','actual_parameters')},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
