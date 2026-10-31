"""Actual DDS/Native source identity probe, brake held and no velocity publisher."""
import json,sys,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue
from std_msgs.msg import String
from wildos_nav2_adapter.source_control import COMMAND_TOPIC,PROTOCOL,receipt_issue

output=Path(sys.argv[1]);assert not output.exists()
rclpy.init();node=rclpy.create_node('source_control_lifecycle_probe')
report=dict(passed=False,scope='Real DDS and installed speed-loop/Native; brakes only, no velocity publisher',
    phases=[],errors=[],rows=[],started_wall=time.time())
latest={};clients=[];phase='bootstrap'
def receive(kind,message):
    value=json.loads(message.data)
    latest[kind]=dict(data=value,at=time.monotonic())
    report['rows'].append(dict(phase=phase,kind=kind,received_wall_ns=time.time_ns(),
        received_monotonic_ns=time.monotonic_ns(),data=value))
    if kind=='source':assert value['control']['throttle']==0. and value['control']['brake']>=.9,value
    if kind=='native':assert value['sent']['throttle']==0. and value['sent']['brake']>=.9 and value['speed']<.05,value
for kind,topic in (('source',COMMAND_TOPIC),('native','/wildos/native_control_status'),('speed','/wildos/control_status')):
    node.create_subscription(String,topic,lambda m,k=kind:receive(k,m),10)
def wait(done,timeout=6.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if done():return
    raise TimeoutError('source lifecycle: '+phase)
def request(service,kind,names,values=None):
    client=node.create_client(kind,service);clients.append(client)
    assert client.wait_for_service(timeout_sec=3),service
    if kind is GetParameters:msg=kind.Request(names=names)
    else:msg=kind.Request(parameters=[Parameter(name=n,value=ParameterValue(type=1,bool_value=v)) for n,v in zip(names,values)])
    future=client.call_async(msg);wait(future.done,4.)
    result=future.result()
    if kind is SetParameters:assert all(v.successful for v in result.results),result
    return result
def ready(enabled,minimum_generation):
    if set(latest)!=set(('source','native','speed')) or any(time.monotonic()-v['at']>.5 for v in latest.values()):return False
    source,native,speed=(latest[k]['data'] for k in ('source','native','speed'))
    receipt=native.get('source_receipt')
    if receipt_issue(receipt,time.time_ns()):return False
    return (source['version']==native.get('control_protocol')==speed.get('control_protocol')==PROTOCOL
        and source['sender_id']==receipt['sender_id']==speed['source_control']['sender_id']
        and source['generation']==receipt['generation']==speed['source_control']['generation']
        and source['generation']>=minimum_generation
        and speed['enabled'] is enabled and speed['source_ack_ready'] is True
        and native.get('control_acknowledged') is True and native.get('telemetry_ok') is True
        and native.get('actuator_ok') is True)
try:
    state=request('/wildos_graph_path_bridge/get_parameters',GetParameters,['follow_path','require_source_control'])
    assert [v.bool_value for v in state.values]==[False,True],state
    wait(lambda:ready(False,0))
    origin=latest['source']['data']['sender_id'];generation=latest['source']['data']['generation']
    for label,enabled in (('enable_brake',True),('disable_brake',False)):
        phase=label
        request('/wildos_cmd_vel_to_control/set_parameters',SetParameters,['enable'],[enabled])
        wait(lambda:ready(enabled,generation+1))
        source=latest['source']['data']
        assert source['sender_id']==origin
        generation=source['generation']
        began=time.monotonic()
        wait(lambda:time.monotonic()-began>=.6 and ready(enabled,generation))
        report['phases'].append(dict(name=phase,enabled=enabled,generation=generation,
            source_sender=origin,native_command_age=latest['native']['data']['command_age'],
            source_acknowledged=True))
    report['passed']=True
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    try:request('/wildos_cmd_vel_to_control/set_parameters',SetParameters,['enable'],[False])
    except Exception as exc:report['errors'].append('final_disable: '+repr(exc))
    report['passed']=report['passed'] and not report['errors']
    report['elapsed_s']=time.time()-report['started_wall']
    output.write_text(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
raise SystemExit(not report['passed'])
