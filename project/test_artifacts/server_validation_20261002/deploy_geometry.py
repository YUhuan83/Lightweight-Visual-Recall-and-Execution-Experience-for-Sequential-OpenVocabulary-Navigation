"""Deploy calibrated geometry before a new trial; preserve the loaded map."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import zipfile
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env

env=runtime_env()

def spec(pid):
    p=Path('/proc',str(pid))
    return {'pid':pid,'args':p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1],
            'cwd':str(p.joinpath('cwd').resolve())}

def stop(s):
    p=Path('/proc',str(s['pid']))
    os.kill(s['pid'],signal.SIGINT)
    deadline=time.monotonic()+15
    while p.exists() and time.monotonic()<deadline:
        if p.joinpath('stat').read_text().split()[2]=='Z':break
        time.sleep(.1)
    if p.exists() and p.joinpath('stat').read_text().split()[2]!='Z':raise RuntimeError('Process did not stop '+str(s['pid']))

def parameter(args,key,value):
    match=next((i for i,s in enumerate(args) if s.startswith(key+':=')),None)
    if match is not None:args[match]=key+':='+str(value)
    else:args.extend(['-p',key+':='+str(value)])

def start(name,s):
    with ROOT.joinpath(name+'_geometry.log').open('ab') as log:
        p=subprocess.Popen(s['args'],cwd=s['cwd'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    ROOT.joinpath(name+'.pid').write_text(str(p.pid))
    time.sleep(.5)
    if p.poll() is not None:raise RuntimeError(name+' failed at startup')
    print(name,p.pid,flush=True)

if sys.argv[1]=='verify':
    for package,pattern in [('beamng_wildos_adapter','test_node_geometry_ros.py'),('wildos_nav2_adapter','test_graph_path_bridge_routing.py'),('wildos_nav2_adapter','test_cmd_vel_to_control_ros.py'),('wildos_nav2_adapter','test_visualization_bridge_ros.py')]:
        test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(WS/'src'/package/'test'),'-p',pattern,'-v'],env=dict(env,ROS_DOMAIN_ID='49'),cwd=WS,capture_output=True,text=True,timeout=60)
        ROOT.joinpath('geometry_final_'+pattern+'.log').write_text(test.stdout+test.stderr)
        print(pattern,test.returncode,(test.stdout+test.stderr)[-160:],flush=True)
        if test.returncode:raise RuntimeError('Final ROS regression failed')
elif sys.argv[1]=='numeric_visual':
    result=subprocess.run([str(WS/'.venv/bin/python'),'/home/ly/session_tools_20261002.py','inspect','before_numeric_visual'],env=env,capture_output=True,text=True,timeout=25)
    if result.returncode:raise RuntimeError('Cannot stop navigation')
    wanted={'nav2_adapter/wildos_nav2_adapter/graph_route.py','nav2_adapter/test/test_graph_route.py',
            'nav2_adapter/test/fixtures/town_analytic_join.json','nav2_adapter/config/beamng_navigation.rviz',
            'nav2_adapter/wildos_nav2_adapter/visualization_bridge.py'}
    manifest=json.loads(ROOT.joinpath('geometry_manifest.json').read_text())
    with zipfile.ZipFile(ROOT/'geometry_stage.zip') as archive:
        for name in wanted:
            _,rest=name.split('/',1);target=WS/'src/wildos_nav2_adapter'/rest
            backup=ROOT/'before_numeric_visual'/name;backup.parent.mkdir(parents=True,exist_ok=True)
            if target.exists() and not backup.exists():shutil.copy2(target,backup)
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(archive.read(name))
            manifest[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(WS/'src/wildos_nav2_adapter/test'),'-p','test_graph_route.py','-v'],env=dict(env,ROS_DOMAIN_ID='49'),capture_output=True,text=True,timeout=60)
    ROOT.joinpath('geometry_numeric_tests.log').write_text(test.stdout+test.stderr)
    print((test.stdout+test.stderr)[-180:],flush=True)
    if test.returncode:raise RuntimeError('Numerical geometry test failed')
    for name in ('graph_bridge','visualization'):
        s=spec(int(ROOT.joinpath(name+'.pid').read_text()));stop(s);start(name,s)
    ROOT.joinpath('geometry_manifest.json').write_text(json.dumps(manifest,indent=2))
elif sys.argv[1]=='planar_frame':
    result=subprocess.run([str(WS/'.venv/bin/python'),'/home/ly/session_tools_20261002.py','inspect','before_planar_frame'],env=env,capture_output=True,text=True,timeout=25)
    if result.returncode:raise RuntimeError('Cannot stop navigation before planar frame change')
    names=['official_adapter','lifecycle','planner','controller']
    specs={n:spec(int(ROOT.joinpath(n+'.pid').read_text())) for n in names}
    wanted={'official_adapter/beamng_wildos_adapter/node.py','official_adapter/test/test_node_geometry_ros.py','nav2_adapter/config/nav2_official_control.yaml'}
    manifest=json.loads(ROOT.joinpath('geometry_manifest.json').read_text())
    with zipfile.ZipFile(ROOT/'geometry_stage.zip') as archive:
        for name in wanted:
            first,rest=name.split('/',1)
            package={'official_adapter':'beamng_wildos_adapter','nav2_adapter':'wildos_nav2_adapter'}[first]
            target=WS/'src'/package/rest
            backup=ROOT/'before_planar_frame'/name;backup.parent.mkdir(parents=True,exist_ok=True)
            if not backup.exists():shutil.copy2(target,backup)
            target.write_bytes(archive.read(name))
            manifest[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(WS/'src/beamng_wildos_adapter/test'),'-p','test_node_geometry_ros.py','-v'],env=dict(env,ROS_DOMAIN_ID='49'),capture_output=True,text=True,timeout=60)
    ROOT.joinpath('geometry_planar_frame_tests.log').write_text(test.stdout+test.stderr)
    print(test.stdout+test.stderr,flush=True)
    if test.returncode:raise RuntimeError('Planar frame test failed')
    for name in ('lifecycle','controller','planner','official_adapter'):stop(specs[name])
    for name in ('official_adapter','planner','controller','lifecycle'):start(name,specs[name])
    ROOT.joinpath('geometry_manifest.json').write_text(json.dumps(manifest,indent=2))
elif sys.argv[1]=='deploy':
    # Explicit stop services and native brake verification precede all edits.
    result=subprocess.run([str(WS/'.venv/bin/python'),'/home/ly/session_tools_20261002.py','inspect','before_geometry'],env=env,capture_output=True,text=True,timeout=25)
    ROOT.joinpath('before_geometry_check.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError('Cannot confirm safe stopped state')
    final=json.loads(ROOT.joinpath('before_geometry_state.json').read_text())
    if not final['paused'] or final['native']['sent']['throttle']!=0:raise RuntimeError('Vehicle not stopped')
    names=['official_adapter','speed_control','graph_bridge','planner','controller','lifecycle']
    specs={n:spec(int(ROOT.joinpath(n+'.pid').read_text())) for n in names}
    # Existing mapper/filter must clear their old-origin caches before testing.
    for name,pid in [('cloud_filter',2197083),('mapper',2261526)]:
        specs[name]=spec(pid)
    backup=ROOT/'before_rear_axle_geometry'
    backup.mkdir(exist_ok=True)
    hashes={}
    with zipfile.ZipFile(ROOT/'geometry_stage.zip') as archive:
        for name in archive.namelist():
            first,rest=name.split('/',1)
            package={'nav2_adapter':'wildos_nav2_adapter','official_adapter':'beamng_wildos_adapter'}[first]
            destination=WS/'src'/package/rest
            if not destination.resolve().is_relative_to((WS/'src'/package).resolve()):raise RuntimeError('Unexpected archive path')
            if destination.exists():
                copy=backup/first/rest; copy.parent.mkdir(parents=True,exist_ok=True)
                if not copy.exists():shutil.copy2(destination,copy)
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(archive.read(name))
            hashes[name]=hashlib.sha256(destination.read_bytes()).hexdigest()
    for name in ('lifecycle','controller','planner','graph_bridge','speed_control','official_adapter','cloud_filter','mapper'):stop(specs[name])
    build=subprocess.run(['colcon','build','--packages-select','wildos_nav2_adapter','beamng_wildos_adapter','--symlink-install','--event-handlers','console_direct+'],cwd=WS,env=env,capture_output=True,text=True,timeout=180)
    ROOT.joinpath('geometry_build.log').write_text(build.stdout+build.stderr)
    if build.returncode:raise RuntimeError('Geometry build failed: '+(build.stdout+build.stderr)[-1000:])
    for package,pattern in [('beamng_wildos_adapter','test_official_adapter.py'),('beamng_wildos_adapter','test_node_geometry_ros.py'),('wildos_nav2_adapter','test_graph_route.py'),('wildos_nav2_adapter','test_graph_path_bridge_routing.py'),('wildos_nav2_adapter','test_cmd_vel_to_control_ros.py')]:
        test=subprocess.run([str(WS/'.venv/bin/python'),'-m','unittest','discover','-s',str(WS/'src'/package/'test'),'-p',pattern,'-v'],env=dict(env,ROS_DOMAIN_ID='49'),cwd=WS,capture_output=True,text=True,timeout=60)
        ROOT.joinpath('geometry_'+pattern+'.log').write_text(test.stdout+test.stderr)
        print(pattern,test.returncode,(test.stdout+test.stderr)[-140:],flush=True)
        if test.returncode:raise RuntimeError('Geometry test failed')
    parameter(specs['speed_control']['args'],'wheelbase',2.501)
    parameter(specs['speed_control']['args'],'max_steer_angle',.54)
    parameter(specs['speed_control']['args'],'enable','false')
    parameter(specs['graph_bridge']['args'],'follow_path','false')
    for key,value in [('footprint_front_extent',3.4),('footprint_rear_extent',1.05),('footprint_half_width',1.05)]:parameter(specs['cloud_filter']['args'],key,value)
    for name in ('official_adapter','cloud_filter','mapper','speed_control','graph_bridge','planner','controller','lifecycle'):start(name,specs[name])
    ROOT.joinpath('geometry_manifest.json').write_text(json.dumps(hashes,indent=2))
elif sys.argv[1]=='display':
    uid=os.getuid(); candidates=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or p.stat().st_uid!=uid:continue
        try:
            comm=p.joinpath('comm').read_text().strip()
            if comm not in ('gnome-shell','Xvfb','nxnode.bin') and p.name!='909616':continue
            values=dict(e.split('=',1) for e in p.joinpath('environ').read_bytes().decode().split('\0') if '=' in e)
            entry={'pid':int(p.name),'comm':comm,**{k:values.get(k) for k in ('DISPLAY','XAUTHORITY')}}
            if comm=='Xvfb':
                args=p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
                entry['DISPLAY']=next((s for s in args if s.startswith(':')),None)
                if '-auth' in args:entry['XAUTHORITY']=args[args.index('-auth')+1]
            candidates.append(entry)
        except (OSError,ValueError):pass
    print(json.dumps(candidates,indent=2),flush=True)
    ROOT.joinpath('display_sessions.json').write_text(json.dumps(candidates,indent=2))
