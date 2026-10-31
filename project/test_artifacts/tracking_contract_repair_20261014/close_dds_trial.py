"""Verify stationary physics and stop only this round's registered owners."""
import json
import os
from pathlib import Path
import signal
import time

ROOT=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
SESSION=ROOT/'green_house_session1'


def alive(pid):
    try: return (Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()[0] not in ('Z','X')
    except FileNotFoundError: return False


def main():
    from beamngpy import BeamNGpy
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
    install_fail_closed_reads(PrefixedLengthSocket)
    report={'passed':False,'stopped':{},'errors':[]}
    full=json.loads((ROOT/'green_house_trial1/full_test.json').read_text())
    if full.get('stationary_after_stop') is not True or full.get('simulation_paused') is not True:
        raise RuntimeError('Recorded safe stop/pause is missing; refuse unattended teardown')
    bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
    try:
        report['before']=read_physics_snapshot(bng)
        if report['before']['speed']>.05: raise RuntimeError('Vehicle not stationary')
        for role,marker,sig in (('stack','physical_vehicle_control.launch.py',signal.SIGINT),
                ('vision','visual_navigation.wildos.nav',signal.SIGINT),
                ('triangulation','obj_mask_triangulation.py',signal.SIGINT),
                ('sensor','attach_current_vehicle_sensors.py',signal.SIGTERM)):
            pid=int((SESSION/f'{role}.pid').read_text())
            if alive(pid):
                tokens=[x for x in (Path('/proc')/str(pid)/'cmdline').read_bytes().decode().split('\0') if x]
                if not any(x==marker or Path(x).name==marker for x in tokens):
                    report['errors'].append('PID ownership mismatch: '+role); continue
                os.kill(pid,sig)
                end=time.monotonic()+25
                while alive(pid) and time.monotonic()<end: time.sleep(.1)
            report['stopped'][role]={'pid':pid,'stopped':not alive(pid)}
            if alive(pid): report['errors'].append('Owner did not stop: '+role)
        report['after']=read_physics_snapshot(bng)
        report['errors'].extend(physical_snapshot_errors(report['after'],require_sensors=False))
        if report['after']['speed']>.05 or report['after'].get('cameras') or report['after'].get('lidars'):
            report['errors'].append('Final stationary/sensor cleanup check failed')
        bng.control.pause()
        report['passed']=not report['errors']
    finally:
        bng.disconnect()
        (ROOT/'round_shutdown.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','after')},indent=2),flush=True)


if __name__=='__main__': main()
