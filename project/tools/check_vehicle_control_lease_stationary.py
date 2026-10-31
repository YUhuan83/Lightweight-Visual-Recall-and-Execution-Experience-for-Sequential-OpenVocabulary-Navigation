"""Real stationary SBR lease expiry; only full-brake inputs, no navigation/video."""
import argparse
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--vehicle-port',type=int,required=True)
    args=p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection
    from beamngpy.sensors import Electrics
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
    from wildos_nav2_adapter.vehicle_control_lease import VehicleControlLease
    from wildos_nav2_adapter.direct_vehicle_control import install_direct_steering_control
    from server_test_preflight import process_rows
    competitors=[r for r in process_rows() if any(Path(t).name in
        ('native_vehicle_control','cmd_vel_to_control','graph_path_bridge','planner_server','controller_server')
        for t in r['tokens'])]
    if competitors: raise RuntimeError('Existing motion clients: '+str(competitors))
    report=dict(passed=False,errors=[],samples=[],map_reload_count=0,vehicle_teleports=0,
                moving_test=False,video_recording_enabled=False)
    bng=BeamNGpy('127.0.0.1',25252,socket_timeout=3).open(launch=False)
    vehicle=lease=sdk_control=None
    def sample(phase):
        began=time.monotonic(); vehicle.sensors.poll()
        e=dict(vehicle.sensors['lease_electrics'])
        s=dict(vehicle.state)
        row=dict(phase=phase,elapsed=time.monotonic()-origin,poll_ms=(time.monotonic()-began)*1000,
            speed=sum(v*v for v in s['vel'])**.5,
            actual={k:e.get(k) for k in ('throttle','brake','parkingbrake','steering_input','wildosControlLeaseTripped')})
        report['samples'].append(row)
        assert row['speed']<.05, 'Stationary test unexpectedly moved'
        return row
    origin=time.monotonic()
    try:
        report['before']=read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['before'],require_sensors=False)
        assert report['before']['speed']<.05
        vehicle=bng.vehicles.get_current()['ego']
        vehicle.connection=Connection('127.0.0.1',args.vehicle_port,timeout=3)
        vehicle.connect(bng); vehicle.sensors.attach('lease_electrics',Electrics())
        sdk_control=vehicle.control
        lease=VehicleControlLease(bng,vehicle,.8)
        report['source_sha256']=lease.source_sha256
        install_direct_steering_control(vehicle,lease.owner)
        vehicle.ai.set_mode('disabled'); vehicle.set_shift_mode('realistic_automatic')
        bng.control.resume()
        for i in range(20):
            vehicle.control(throttle=0.,brake=1.,parkingbrake=1.,steering=0.,clutch=0.,gear=0)
            row=sample('healthy'); time.sleep(.04)
        assert row['actual']['brake']>.95 and row['actual']['throttle']==0
        report['last_feed_elapsed']=time.monotonic()-origin
        # Do not call feed/release/status here: only the installed callback can expire it.
        deadline=time.monotonic()+1.2
        while time.monotonic()<deadline:
            row=sample('command_gap'); time.sleep(.04)
        report['expiry_actual_observed_before_status']=row['actual']
        assert row['actual']['wildosControlLeaseTripped']==1, 'Vehicle callbacks did not expire lease'
        assert row['actual']['brake']>.95 and row['actual']['throttle']==0
        report['expired_status']=json.loads(vehicle.queue_lua_command(
            'return jsonEncode(extensions.wildosControlLease.status())',response=True))
        assert report['expired_status']['tripped']
        try:
            vehicle.control(throttle=0.,brake=1.,gear=0)
        except Exception as exc:
            report['late_command_rejection']=repr(exc)
        else: raise AssertionError('Expired owner was rearmed')
        report['release']=lease.release(vehicle)
        lease=VehicleControlLease(bng,vehicle,.8)
        install_direct_steering_control(vehicle,lease.owner)
        vehicle.control(throttle=0.,brake=1.,parkingbrake=1.,gear=0)
        sample('fresh_owner'); report['fresh_release']=lease.release(vehicle)
        lease=None
        report['after']=read_physics_snapshot(bng)
        assert report['after']['vehicleID']==report['before']['vehicleID']
        assert not physical_snapshot_errors(report['after'],require_sensors=False)
        report['passed']=True
    except Exception as exc:
        report['errors'].append(repr(exc))
    finally:
        # Independent SDK brake ignores the candidate lease; recovery isn't counted as pass.
        try:
            if vehicle:
                if sdk_control:
                    sdk_control(throttle=0.,brake=1.,parkingbrake=1.,steering=0.,clutch=0.,gear=0)
                if lease: lease.release(vehicle)
                row=sample('cleanup'); assert row['speed']<.05
            bng.control.pause()
        except Exception as exc:
            report['passed']=False; report['errors'].append('cleanup: '+repr(exc))
        if vehicle: vehicle.disconnect()
        bng.disconnect()
        args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
