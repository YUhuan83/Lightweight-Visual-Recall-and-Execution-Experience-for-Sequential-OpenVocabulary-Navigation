"""Real Humble FollowPath/RPP -> DDS -> speed loop -> source receiver.

Synthetic stationary odometry and an empty map; ACK represents a synthetic
actuator only. No BeamNG, physics, perception or navigation success is claimed.
"""
import json, os, signal, subprocess, time, traceback
from pathlib import Path
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from beamng_msgs.msg import StateSensor
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
from wildos_nav2_adapter.source_control import SourceControlReceiver, PROTOCOL, COMMAND_TOPIC

ROOT = Path(__file__).parent
assert os.environ['ROS_DOMAIN_ID'] == '181'
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
OUT = ROOT / ('dds_' + time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir(exist_ok=False)
config = yaml.safe_load((ROOT/'candidate/nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
params = config['controller_server']['ros__parameters']
params['progress_checker']['movement_time_allowance'] = 100.
local = config['local_costmap']['local_costmap']['ros__parameters']
local.update(plugins=['inflation_layer'], width=30, height=30, global_frame='spot1/odom',
             track_unknown_space=False, update_frequency=20., publish_frequency=5.)
(OUT/'controller.yaml').write_text(yaml.safe_dump({
    'controller_server':config['controller_server'], 'local_costmap':config['local_costmap']}))
report = {'passed':False, 'scope':__doc__, 'phases':[], 'errors':[]}
log = (OUT/'events.jsonl').open('x')
child = None
executor = None
probe = None
speed = None

def event(kind, data):
    log.write(json.dumps(dict(wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(),
                              kind=kind, data=data), separators=(',', ':'))+'\n')
    log.flush()

try:
    rclpy.init(args=['--ros-args', '-p', 'require_stamped_command:=true',
        '-p', 'cmd_vel_topic:=/wildos/cmd_vel_stamped', '-p', 'control_protocol:='+PROTOCOL,
        '-p', 'control_topic:='+COMMAND_TOPIC])
    speed = CmdVelToControl()
    probe = rclpy.create_node('controller_source_chain_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(speed); executor.add_node(probe)
    receiver = SourceControlReceiver()
    statuses = []
    frames = []
    raw = []
    zeros = []
    queue = []
    mode = 'forward'
    delay = 0.
    replay = None
    native = probe.create_publisher(String, '/wildos/native_control_status', 10)
    state = probe.create_publisher(StateSensor, '/vehicles/ego/sensors/state', 10)
    odom = probe.create_publisher(Odometry, '/spot1/odom', 10)
    forwarded = probe.create_publisher(TwistStamped, '/wildos/cmd_vel_stamped', 1)
    tf = StaticTransformBroadcaster(probe)
    transform = TransformStamped()
    transform.header.stamp = probe.get_clock().now().to_msg()
    transform.header.frame_id = 'spot1/odom'; transform.child_frame_id = 'spot1/nav_base_link'
    transform.transform.rotation.w = 1.
    tf.sendTransform(transform)

    def control(message):
        frame = json.loads(message.data)
        error = receiver.accept(message.data, time.time_ns(), time.monotonic_ns(), stopped=True)
        frames.append((time.monotonic(), frame, error)); event('source_control', dict(frame=frame,error=error))

    def status(message):
        value = json.loads(message.data); statuses.append((time.monotonic(), value))
        event('control_status', value)
        if value['reason'] == 'tracking' and value['throttle'] > 0:
            assert value['source_control']['expires_wall_ns'] <= value['controller_stamp_ros_ns']+500_000_000

    def source(message):
        raw.append(message)
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        event('rpp_source', dict(stamp_ns=stamp, frame=message.header.frame_id,
                                  v=message.twist.linear.x, w=message.twist.angular.z))
        if mode == 'forward': queue.append((time.monotonic()+delay, message))

    def zero(message):
        if message == Twist():
            zeros.append(time.monotonic()); event('controller_terminal_zero', {})

    probe.create_subscription(String, COMMAND_TOPIC, control, 10)
    probe.create_subscription(String, '/wildos/control_status', status, 10)
    probe.create_subscription(TwistStamped, '/controller_probe/raw', source, 1)
    probe.create_subscription(Twist, '/wildos/cmd_vel', zero, 10)
    last = 0.

    def spin(duration):
        global last
        end = time.monotonic()+duration
        while time.monotonic() < end:
            now = time.monotonic()
            if now-last >= .025:
                stamp = probe.get_clock().now().to_msg()
                msg = StateSensor(); msg.header.stamp=stamp
                msg.header.frame_id='synthetic_state'; msg.dir.x=1.; msg.up.z=1.; state.publish(msg)
                msg = Odometry(); msg.header.stamp=stamp; msg.header.frame_id='spot1/odom'
                msg.child_frame_id='spot1/nav_base_link'; msg.pose.pose.orientation.w=1.; odom.publish(msg)
                current = receiver.current(time.time_ns(), time.monotonic_ns())
                receipt = receiver.receipt(current, current['control'], True,
                    time.time_ns(), time.monotonic_ns()) if current else None
                native.publish(String(data=json.dumps(dict(wall_time=time.time(),speed=0.,
                    actual=dict(wheelspeed=0.), actuator_ok=True, telemetry_ok=True,
                    control_acknowledged=True, telemetry_age_sec=0.,control_error=None,
                    control_protocol=PROTOCOL,source_receipt=receipt))))
                if mode == 'replay' and replay is not None: forwarded.publish(replay)
                last = now
            while queue and queue[0][0] <= now:
                _,msg = queue.pop(0); forwarded.publish(msg)
            executor.spin_once(timeout_sec=.002)

    def wait(predicate, seconds=8.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():return
            if child and child.poll() is not None:raise RuntimeError('controller_server exited')
            spin(.01)
        raise TimeoutError('DDS phase wait expired')

    assert not probe.get_subscriptions_info_by_topic('/control')
    forbidden={'wildos_native_vehicle_control','wildos_kinematic_vehicle_control','controller_server'}
    spin(.5)
    assert not forbidden & {n for n,_ in probe.get_node_names_and_namespaces()}
    childlog=(OUT/'controller.log').open('x')
    child=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args',
        '--params-file',str(OUT/'controller.yaml'),'-r','cmd_vel:=/wildos/cmd_vel',
        '-r','/wildos/cmd_vel_stamped:=/controller_probe/raw'],
        stdout=childlog,stderr=subprocess.STDOUT,start_new_session=True)
    lifecycle=probe.create_client(ChangeState,'/controller_server/change_state')
    wait(lambda:lifecycle.service_is_ready())
    for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
        request=ChangeState.Request();request.transition.id=transition
        future=lifecycle.call_async(request);wait(future.done)
        assert future.result().success
    spin(.5)
    assert all(r.successful for r in speed.set_parameters([Parameter('enable',value=True)]))
    client=ActionClient(probe,FollowPath,'/follow_path')
    wait(client.server_is_ready)

    def start_goal():
        request=FollowPath.Goal();request.controller_id='FollowPath';request.goal_checker_id='cusp_goal_checker'
        request.path.header.frame_id='spot1/odom';request.path.header.stamp=probe.get_clock().now().to_msg()
        for i in range(41):
            point=PoseStamped();point.header=request.path.header
            point.pose.position.x=i*.2;point.pose.orientation.w=1.;request.path.poses.append(point)
        future=client.send_goal_async(request);wait(future.done)
        handle=future.result();assert handle.accepted
        return handle

    handle=start_goal()
    wait(lambda:statuses and statuses[-1][1]['reason']=='tracking' and statuses[-1][1]['throttle']>0)
    assert raw[-1].header.frame_id=='spot1/nav_base_link'
    report['phases'].append('real_follow_path_and_stamped_rpp_drive')
    delay=.35;spin(.9)
    delayed=[s for _,s in statuses[-12:] if s['reason']=='tracking']
    assert delayed and any(s['controller_source_age_ros_sec']>.3 for s in delayed)
    report['phases'].append('real_rpp_350ms_delay_consumes_original_lease')
    mode='drop';queue.clear();spin(.1);replay=raw[-1]
    # Feed one real generated demand, then repeat exactly the same timestamp.
    forwarded.publish(replay);spin(.05);mode='replay';spin(.7)
    assert statuses[-1][1]['reason']=='stale_command' and statuses[-1][1]['brake']==1.
    report['phases'].append('duplicate_dds_packets_do_not_renew_source')
    mode='forward';delay=0.;spin(.4)
    wait(lambda:statuses[-1][1]['reason']=='tracking' and statuses[-1][1]['throttle']>0)
    report['phases'].append('fresh_generation_recovers_tracking')
    previous=raw[-1];mode='drop';queue.clear();began=time.monotonic();nzero=len(zeros)
    future=handle.cancel_goal_async();wait(future.done)
    wait(lambda:len(zeros)>nzero)
    wait(lambda:statuses[-1][1]['brake']==1. and statuses[-1][1]['throttle']==0.)
    report['cancel_brake_sec']=time.monotonic()-began
    assert report['cancel_brake_sec'] < .5
    forwarded.publish(previous);spin(.15)
    assert speed.target_speed==0. and statuses[-1][1]['brake']==1.
    report['phases'].append('real_controller_cancel_zero_brakes_and_fences_old_torque')
    mode='forward';handle=start_goal()
    wait(lambda:statuses[-1][1]['reason']=='tracking' and statuses[-1][1]['throttle']>0)
    report['phases'].append('new_follow_path_after_cancel_recovers')
    mode='drop';queue.clear();previous=raw[-1]
    assert all(r.successful for r in speed.set_parameters([Parameter('enable',value=False)]))
    spin(.05)
    assert all(r.successful for r in speed.set_parameters([Parameter('enable',value=True)]))
    forwarded.publish(previous);spin(.15)
    assert speed.target_speed==0. and statuses[-1][1]['brake']==1.
    mode='forward';wait(lambda:statuses[-1][1]['reason']=='tracking' and statuses[-1][1]['throttle']>0)
    report['phases'].append('enable_boundary_fences_buffered_demand')
    future=handle.cancel_goal_async();wait(future.done);spin(.3)
    report.update(passed=True, source_messages=len(raw), control_frames=len(frames),
                  control_statuses=len(statuses), rejected_source_packets=sum(e is not None for _,_,e in frames))
except BaseException:
    report['errors'].append(traceback.format_exc())
finally:
    if speed:
        speed.set_parameters([Parameter('enable',value=False)])
        if executor:
            try:spin(.15)
            except BaseException:pass
    if child and child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    if executor:executor.shutdown()
    if speed:speed.destroy_node()
    if probe:probe.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    log.close()
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),**report),indent=2))
raise SystemExit(not report['passed'])
