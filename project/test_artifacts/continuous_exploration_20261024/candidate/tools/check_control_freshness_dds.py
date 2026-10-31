"""Actual speed-loop/DDS input-age diagnostics in domain 85; no BeamNG."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='85' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated localhost domain 85')
    if args.output.exists():raise FileExistsError(args.output)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from beamng_msgs.msg import StateSensor
    from std_msgs.msg import String
    from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
    rclpy.init();relay=CmdVelToControl();probe=rclpy.create_node('control_freshness_diagnostic')
    executor=SingleThreadedExecutor();executor.add_node(relay);executor.add_node(probe)
    report={'scope':'Synthetic input messages and actual speed loop/DDS; no simulator or motion',
            'passed':False,'phases':{},'errors':[]}
    received=[]
    try:
        state=probe.create_publisher(StateSensor,'/vehicles/ego/sensors/state',10)
        command=probe.create_publisher(Twist,'/wildos/cmd_vel',10)
        native=probe.create_publisher(String,'/wildos/native_control_status',10)
        probe.create_subscription(String,'/wildos/control_status',
                                  lambda m:received.append(json.loads(m.data)),10)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        assert not probe.get_subscriptions_info_by_topic('/control'),'Unexpected actuator subscriber'
        forbidden={'wildos_native_vehicle_control','controller_server','wildos_kinematic_vehicle_control'}
        assert not forbidden & {name for name,_ in probe.get_node_names_and_namespaces()}
        result=relay.set_parameters([Parameter('enable',value=True)])
        assert all(r.successful for r in result)

        def phase(name,send_state,duration,native_mode='fresh'):
            received.clear();deadline=time.monotonic()+duration;last=0.
            duplicate_wall = time.time() - .1
            while time.monotonic()<deadline:
                if time.monotonic()-last>.025:
                    msg=Twist();msg.linear.x=1.2;command.publish(msg)
                    native.publish(String(data=json.dumps(dict(
                        wall_time=(duplicate_wall if native_mode=='duplicate' else
                                   time.time()-(.3 if native_mode=='aged' else 0.)),
                        speed=0.,actuator_ok=True,telemetry_ok=True,
                        control_acknowledged=native_mode!='unacknowledged',
                        telemetry_age_sec=(.3 if native_mode=='aged' else .25 if native_mode=='duplicate' else 0.),
                        control_error=None,actual=dict(wheelspeed=0.)))))
                    if send_state:
                        msg=StateSensor();msg.header.stamp=probe.get_clock().now().to_msg()
                        msg.header.frame_id='diagnostic_state_frame';msg.dir.x=1.;msg.up.z=1.
                        state.publish(msg)
                    last=time.monotonic()
                executor.spin_once(timeout_sec=.005)
            assert received,'No real DDS control status received'
            report['phases'][name]=list(received)
            return received[-1]

        healthy=phase('healthy',True,.7)
        assert healthy['reason']=='tracking' and healthy['brake']==0.
        assert healthy['state_source_frame']=='diagnostic_state_frame'
        stale=phase('state_missing',False,.85)
        assert stale['reason']=='stale_state' and stale['brake']==1. and not stale['actuating']
        assert stale['state_receipt_age_steady_sec']>.5
        assert stale['command_receipt_age_steady_sec']<.5 and stale['native_receipt_age_steady_sec']<.5
        recovered=phase('state_recovered',True,.5)
        assert recovered['reason']=='tracking' and recovered['brake']==0.
        aged=phase('native_combined_age',True,.7,'aged')
        assert aged['reason']=='native_control_unavailable' and aged['brake']==1.
        assert aged['state_receipt_age_steady_sec']<.5 and aged['native_receipt_age_steady_sec']<.5
        duplicate=phase('native_duplicate',True,.7,'duplicate')
        assert any(r['reason']=='tracking' for r in received),'No initial healthy duplicate sample'
        assert duplicate['reason']=='native_control_unavailable' and duplicate['brake']==1.
        unack=phase('native_unacknowledged',True,.5,'unacknowledged')
        assert unack['reason']=='native_control_unavailable' and unack['brake']==1.
        final=phase('native_recovered',True,.6)
        assert final['reason']=='tracking' and final['brake']==0. and final['native_health_ok']
        report['passed']=True
    except BaseException as exc:report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        relay.enabled=False
        relay.tick()
        executor.remove_node(relay);executor.remove_node(probe);executor.shutdown()
        relay.destroy_node();probe.destroy_node();rclpy.shutdown()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({'passed':report['passed'],'errors':report['errors'],
                      'statuses':{k:len(v) for k,v in report['phases'].items()}}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
