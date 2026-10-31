"""Load stock Town/SBR at a measured section start, then verify actual settled pose."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home',required=True);parser.add_argument('--user',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();assert not args.output.exists()
    import numpy as np
    from beamngpy import BeamNGpy,Scenario,Vehicle
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
    from check_continuous_start_execution import settle
    import recorded_pose
    install_fail_closed_reads(PrefixedLengthSocket)
    fixture=json.loads(Path(__file__).with_name('recorded_section_fixture.json').read_text())
    spawn=recorded_pose.spawn_from_measurement(fixture['nearby_measured_native_state'])
    bng=BeamNGpy('127.0.0.1',25252,home=args.home,user=args.user,quit_on_close=False,socket_timeout=20).open(launch=False)
    vehicle=None;report=dict(passed=False,map_loads=0,teleports=0,collision_override=False,
                             scope=__doc__,requested_spawn=spawn,fresh_vehicle=True)
    try:
        scenario=Scenario('east_coast_usa','wildos_static_geometry_survey')
        vehicle=Vehicle('ego','sbr',license='WILDOS',color='red',part_config='vehicles/sbr/TT_AWD_S2_DCT.pc')
        scenario.add_vehicle(vehicle,pos=tuple(spawn['position']),rot_quat=tuple(spawn['rot_quat']),safe_spawn=False)
        scenario.make(bng);bng.scenario.load(scenario,connect_player_vehicle=False,connect_existing_vehicles=False)
        report['map_loads']=1;bng.scenario.start();bng.env.set_tod(0.)
        vehicle.connect(bng);vehicle.set_shift_mode('realistic_automatic')
        vehicle.control(throttle=0.,brake=1.,parkingbrake=0.,steering=0.)
        after,samples=settle(lambda:read_physics_snapshot(bng),time.sleep,timeout=12.)
        assert not physical_snapshot_errors(after,require_sensors=False)
        vehicle.sensors.poll();state=dict(vehicle.state)
        actual=recorded_pose.spawn_from_measurement(dict(position=list(state['pos']),direction=list(state['dir']),up=list(state['up'])))
        position_error=math.dist(actual['rear_axle'][:2],spawn['rear_axle'][:2])
        yaw_error=abs(math.remainder(actual['yaw']-spawn['yaw'],2*math.pi))
        report.update(physics=after,stop_samples=samples,actual_pose=actual,
                      rear_axle_error_m=position_error,yaw_error_rad=yaw_error)
        assert position_error<=.35 and yaw_error<=.1,'Settled pose differs; refuse motion'
        report['passed']=True
    except BaseException as exc:report['error']=repr(exc)
    finally:
        try:bng.control.pause()
        finally:
            if vehicle and vehicle.is_connected():vehicle.disconnect()
            bng.disconnect();args.output.write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
