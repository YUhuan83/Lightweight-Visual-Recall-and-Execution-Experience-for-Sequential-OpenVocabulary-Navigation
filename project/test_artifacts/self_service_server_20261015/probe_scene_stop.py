"""Preserve SDK telemetry for the unmodified scene setup, then compare known brake initialization."""
import importlib.util,json,os,socket,subprocess,sys,time
from pathlib import Path
realistic='--realistic' in sys.argv
root=Path('/home/ly/wildos-console');out=root/('scene_probe_'+('realistic_' if realistic else '')+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
home=Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
assert not any(b'BeamNG.tech.x64' in p.read_bytes().split(b'\0')[0] for p in Path('/proc').glob('[0-9]*/cmdline') if p.exists())
with (out/'engine.log').open('x') as log:
    engine=subprocess.Popen([str(home/'BinLinux/BeamNG.tech.x64'),'-headless','-nosfx','-userpath',
        '/home/ly/beamng-east-coast-user/current','-tcom','-tport','25252','-tcom-listen-ip','127.0.0.1','-nosteam'],
        cwd=home,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
report={}
try:
    for _ in range(300):
        assert engine.poll() is None
        try:
            with socket.create_connection(('127.0.0.1',25252),timeout=.3):break
        except OSError:time.sleep(.2)
    from wildos_nav2_adapter import physical_vehicle
    read=physical_vehicle.read_physics_snapshot
    start=time.monotonic();samples=[]
    def logged(bng):
        sample=read(bng);samples.append(dict(elapsed=time.monotonic()-start,**sample))
        with (out/'original_samples.jsonl').open('a') as handle:handle.write(json.dumps(samples[-1])+'\n')
        return sample
    physical_vehicle.read_physics_snapshot=logged
    if realistic:
        import beamngpy
        original_vehicle=beamngpy.Vehicle
        def configured_vehicle(*args,**kwargs):
            vehicle=original_vehicle(*args,**kwargs)
            connect=vehicle.connect
            def connected(*a,**kw):
                result=connect(*a,**kw)
                vehicle.set_shift_mode('realistic_automatic')
                return result
            vehicle.connect=connected
            return vehicle
        beamngpy.Vehicle=configured_vehicle
    spec=importlib.util.spec_from_file_location('console_scene_probe',root/'bundle/tools/wildos_console_scene.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    sys.argv=['scene','--home',str(home),'--user','/home/ly/beamng-east-coast-user/current','--output',str(out/'original_scene.json')]
    report['original_returncode']=module.main()
    report['realistic_automatic_only']=realistic
    physical_vehicle.read_physics_snapshot=read
    if report['original_returncode']:
        command=[sys.executable,'/home/ly/town_memory_validation_20261014/tracking_contract_candidate/component_tools/brake_and_hold_spawn.py',
                 '--output',str(out/'known_brake.json')]
        result=subprocess.run(command,capture_output=True,timeout=35)
        (out/'known_brake.log').write_bytes(result.stdout+result.stderr)
        report['known_brake_returncode']=result.returncode
    report['samples']=len(samples)
except BaseException as exc:report['error']=repr(exc)
finally:
    if engine.poll() is None:
        engine.terminate()
        try:engine.wait(timeout=10)
        except subprocess.TimeoutExpired:engine.kill();engine.wait(timeout=3)
    (out/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(output=str(out),**report)))
