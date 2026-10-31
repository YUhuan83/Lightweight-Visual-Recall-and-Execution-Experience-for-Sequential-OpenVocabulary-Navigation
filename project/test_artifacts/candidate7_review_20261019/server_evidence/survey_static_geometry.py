"""Bounded static-world survey at a historical contact; no navigation/motion goals."""
from pathlib import Path
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

BASE = Path('/home/ly/smoothing_boundary_repair_20261018/candidate')
sys.path[:0] = [str(BASE / 'tools'), str(BASE / 'nav2_adapter')]
from wildos_console import OwnedProcesses

ROOT = Path(__file__).resolve().parent
OUT = ROOT / ('static_' + time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir(exist_ok=False)
owned = OwnedProcesses(OUT, ROOT)
halt = threading.Event()
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, lambda *_: halt.set())
report = dict(scope=__doc__, completed=False, errors=[], motion_goals=0,
              navigation_stack_started=False, video=False, map_loads=0,
              contact_world=[740.51798379933, -79.251197721227, 52.416893189598],
              historical_asset_identity_proven=False)
bng = None


def wait(predicate, timeout):
    end = time.monotonic() + timeout
    while not predicate():
        if halt.is_set():
            raise InterruptedError('Interrupted')
        owned.healthy()
        if time.monotonic() >= end:
            raise TimeoutError('Readiness deadline')
        time.sleep(.1)


try:
    prestart = subprocess.run(
        [sys.executable, '/home/ly/planning_phase_physical_20261018/prestart.py'],
        capture_output=True, text=True, timeout=20, check=True)
    (OUT / 'prestart.log').write_text(prestart.stdout + prestart.stderr)
    pre = json.loads(prestart.stdout)
    assert pre['processes'] == [] and pre['display148_lock'] is False
    env = dict(os.environ, DISPLAY=':148', CUDA_VISIBLE_DEVICES='0')
    owned.spawn('display', ['Xvfb', ':148', '-screen', '0', '1920x1080x24', '-nolisten', 'tcp'], env=env)
    wait(lambda: Path('/tmp/.X11-unix/X148').exists(), 5)
    home = Path('/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0')
    user = '/home/ly/beamng-east-coast-user/current'
    report['primary_source_hashes'] = {}
    for relative in ('lua/ge/extensions/core/terrain.lua',
                     'lua/ge/extensions/core/cameraModes/steadycam.lua'):
        source = home / relative
        data = source.read_bytes()
        (OUT / source.name).write_bytes(data)
        report['primary_source_hashes'][str(source)] = hashlib.sha256(data).hexdigest()
    owned.spawn('engine', [str(home / 'BinLinux/BeamNG.tech.x64'), '-headless', '-nosfx',
                          '-userpath', user, '-tcom', '-tport', '25252',
                          '-tcom-listen-ip', '127.0.0.1', '-nosteam'], cwd=home, env=env)

    def ready():
        try:
            with socket.create_connection(('127.0.0.1', 25252), timeout=.3):
                return True
        except OSError:
            return False

    wait(ready, 90)
    # Preserve the trusted prior stationary scene helper; only give the new
    # generated scenario a unique name. No sensor or actuation stack is loaded.
    old = Path('/home/ly/contact_frame_audit_20261019/scene_recorded.py')
    scene = old.read_text()
    assert scene.count("'wildos_recorded_section_audit'") == 1
    scene = scene.replace("'wildos_recorded_section_audit'", "'wildos_static_geometry_survey'")
    scene_path = OUT / 'scene_survey.py'
    scene_path.write_text(scene)
    (OUT / 'recorded_section_fixture.json').write_bytes(old.with_name('recorded_section_fixture.json').read_bytes())
    env['PYTHONPATH'] = os.pathsep.join([str(old.parent), str(BASE / 'tools'),
                                        str(BASE / 'official_adapter'), str(BASE / 'nav2_adapter')])
    subprocess.run([sys.executable, str(scene_path), '--home', str(home), '--user', user,
                    '--output', str(OUT / 'scene.json')], env=env,
                   stdout=(OUT / 'scene.log').open('w'), stderr=subprocess.STDOUT,
                   timeout=150, check=True)
    scene_result = json.loads((OUT / 'scene.json').read_text())
    assert scene_result['passed']
    report['map_loads'] = scene_result['map_loads']
    from beamngpy import BeamNGpy
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    install_fail_closed_reads(PrefixedLengthSocket)
    bng = BeamNGpy('127.0.0.1', 25252, quit_on_close=False, socket_timeout=15).open(launch=False)
    bng.control.pause()
    report['before'] = read_physics_snapshot(bng)
    assert not physical_snapshot_errors(report['before'], require_sensors=False)
    assert report['before']['speed'] < .05
    x0, y0, z0 = report['contact_world']
    points = [(x0 + ix * .1, y0 + iy * .1)
              for ix in range(-30, 31) for iy in range(-30, 31)]
    records = []
    for begin in range(0, len(points), 96):
        if halt.is_set():
            raise InterruptedError('Interrupted')
        owned.healthy()
        values = ','.join('{%.12f,%.12f}' % xy for xy in points[begin:begin + 96])
        lua = '''
assert(type(castRayStatic)=='function', 'static ray API unavailable')
assert(core_terrain and core_terrain.getTerrainHeight, 'terrain API unavailable')
local points={%s}
local out={}
local fromZ=%.12f
for _,p in ipairs(points) do
  local origin=vec3(p[1],p[2],fromZ)
  local reach=12
  local hit=castRayStatic(origin,vec3(0,0,-1),reach)
  local ground=core_terrain.getTerrainHeight(vec3(p[1],p[2],0))
  local item={x=p[1],y=p[2],rayDistance=hit,hasStatic=hit<reach}
  if hit<reach then item.staticZ=fromZ-hit end
  if ground~=nil then item.terrainZ=ground end
  out[#out+1]=item
end
return jsonEncode(out)
''' % (values, z0 + 4.)
        rows = json.loads(bng.control.queue_lua_command(lua, response=True))
        assert len(rows) == min(96, len(points) - begin)
        for row in rows:
            for name in ('x', 'y', 'rayDistance'):
                assert isinstance(row[name], (int, float)) and math.isfinite(row[name])
            assert 0 <= row['rayDistance'] <= 12.
            for name in ('staticZ', 'terrainZ'):
                if name in row:
                    assert isinstance(row[name], (int, float)) and math.isfinite(row[name])
        records.extend(rows)
    (OUT / 'surface_samples.json').write_text(json.dumps(records, indent=2) + '\n')
    report['after'] = read_physics_snapshot(bng)
    assert not physical_snapshot_errors(report['after'], require_sensors=False)
    assert math.dist(report['before']['position'], report['after']['position']) < .001
    paired = [r['staticZ'] - r['terrainZ'] for r in records
              if 'staticZ' in r and 'terrainZ' in r]
    report.update(samples=len(records), static_hits=sum(r['hasStatic'] for r in records),
                  terrain_pairs=len(paired),
                  static_minus_terrain_min=min(paired) if paired else None,
                  static_minus_terrain_max=max(paired) if paired else None,
                  centre=records[len(records) // 2], completed=True)
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    if bng:
        try:
            bng.control.pause()
            bng.disconnect()
        except Exception as exc:
            report['errors'].append('Disconnect: ' + repr(exc))
    report['errors'].extend(owned.close())
    report['completed'] = report['completed'] and not report['errors']
    (OUT / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(output=str(OUT), **report), indent=2))
raise SystemExit(0 if report['completed'] else 1)
