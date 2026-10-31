"""Retire the identified previous component, then its original owned engine."""
import json,os,signal,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1')
session=BASE/'physical_component_session2'
def state(pid):
    try:
        raw=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()
        return {'pid':pid,'state':raw[0],'starttime':raw[19],'pgid':os.getpgid(pid),
            'command':(Path('/proc')/str(pid)/'cmdline').read_bytes().decode().replace('\0',' ').strip()}
    except (FileNotFoundError,ProcessLookupError):return None
def alive(old):
    now=state(old['pid']);return now and now['starttime']==old['starttime'] and now['state'] not in ('Z','X')
def stop(rows,sig,grace):
    for row in rows:
        if alive(row):os.kill(row['pid'],sig)
    until=time.monotonic()+grace
    while any(alive(r) for r in rows) and time.monotonic()<until:time.sleep(.1)
stack_pid=int((session/'stack.pid').read_text());sensor_pid=int((session/'sensor.pid').read_text())
stack=state(stack_pid);sensor=state(sensor_pid)
assert stack and 'physical_vehicle_control.launch.py' in stack['command'] and stack['pgid']==stack_pid
assert sensor and 'attach_current_vehicle_sensors.py' in sensor['command'] and str(session) in sensor['command']
owned=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:
        row=state(int(entry.name))
        if row and row['pgid']==stack_pid:owned.append(row)
    except PermissionError:continue
assert all(r['pid']==stack_pid or str(BASE/'install') in r['command'] or '/opt/ros/humble/lib/nav2_' in r['command'] or '/home/ly/town_memory_validation_20261014/tracking_contract_candidate/install' in r['command'] for r in owned)
engine=state(1709748)
assert engine and engine['pgid']==engine['pid'] and '/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64' in engine['command']
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
rclpy.init();node=rclpy.create_node('retire_previous_console_preparation')
try:
    for owner,name in [('/wildos_cmd_vel_to_control','enable'),('/wildos_graph_path_bridge','follow_path')]:
        client=node.create_client(SetParameters,owner+'/set_parameters')
        assert client.wait_for_service(timeout_sec=5),owner
        future=client.call_async(SetParameters.Request(parameters=[Parameter(name=name,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=False))]))
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        assert future.done() and not future.exception() and len(future.result().results)==1 and future.result().results[0].successful,owner
        node.destroy_client(client)
finally:node.destroy_node();rclpy.shutdown()
from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
try:
    snapshot=read_physics_snapshot(b)
    assert not physical_snapshot_errors(snapshot)
    assert snapshot['speed']<.05,'Vehicle not stationary; refuse cleanup'
    b.control.pause()
finally:b.disconnect()
stop(owned,signal.SIGINT,8);stop(owned,signal.SIGTERM,3);stop(owned,signal.SIGKILL,1)
stop([sensor],signal.SIGTERM,6);stop([sensor],signal.SIGKILL,1)
assert not any(alive(r) for r in owned+[sensor])
assert alive(engine),'engine identity changed before shutdown'
stop([engine],signal.SIGTERM,10);stop([engine],signal.SIGKILL,2)
report={'scope':'Previous verified component stack, sensors and its engine cleanup','passed':not any(alive(r) for r in owned+[sensor,engine]),'owned':owned+[sensor,engine],'physics_before':snapshot,'gates_closed':True,'pause_ack':True}
(session/'retired_stack.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
assert report['passed']
