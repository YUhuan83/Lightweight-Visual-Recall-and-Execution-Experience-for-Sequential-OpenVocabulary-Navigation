"""Restore a sustained stop at the current pose; no map or physics edits."""
import json,time,sys
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
sys.path.insert(0,str(BASE/'bundle/tools'))
from beamngpy import BeamNGpy
from beamngpy.connection import Connection
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
from check_continuous_start_execution import settle
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False);vehicle=None
report={'passed':False,'scope':'Actual current-pose brake recovery; no search success',
    'teleports':0,'map_reloads':0,'gear_changes_requested':False}
try:
    report['before']=read_physics_snapshot(b)
    assert not physical_snapshot_errors(report['before'])
    vehicle=b.vehicles.get_current()['ego']
    port=json.loads((BASE/'green_house_session1/sensor_attachment.json').read_text())['vehicle_port']
    vehicle.connection=Connection('127.0.0.1',int(port),timeout=5);vehicle.connect(b,tries=1)
    vehicle.control(throttle=0.,brake=1.,parkingbrake=1.,steering=0.)
    b.control.resume()
    def spin(seconds):
        vehicle.control(throttle=0.,brake=1.,parkingbrake=1.,steering=0.)
        time.sleep(seconds)
    after,samples=settle(lambda:read_physics_snapshot(b),spin,timeout=8.,stable=.5)
    report['after']=after;report['stop_samples']=samples
    assert not physical_snapshot_errors(after)
    b.control.pause();report['passed']=True
except BaseException as exc:
    report['error']=repr(exc)
    try:b.control.pause()
    except Exception as error:report['pause_error']=repr(error)
finally:
    if vehicle:vehicle.disconnect()
    b.disconnect();(BASE/'brake_recovery1.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('before','after')},indent=2))
assert report['passed']
