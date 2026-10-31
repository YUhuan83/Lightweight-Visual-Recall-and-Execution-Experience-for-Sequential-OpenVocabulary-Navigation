"""Read-only process/resource inventory, no killing unrelated processes."""
from pathlib import Path
import json,os,subprocess,time
root=Path('/home/ly/controller_source_field_20261018')
relevant=[]
executables={'BeamNG.tech.x64','BeamNG.drive.x64','planner_node','mapper_node',
    'planner_server','controller_server','graph_path_bridge','cmd_vel_to_control',
    'native_vehicle_control','kinematic_vehicle_control','rviz2'}
scripts={'wildos_console.py','sensor_console.py','dds_chain.py','drive_sensor_full.py'}
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:args=(p/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
    except (PermissionError,ProcessLookupError,FileNotFoundError):continue
    if args and (Path(args[0]).name in executables or any(Path(a).name in scripts for a in args[1:3])):
        relevant.append(dict(pid=int(p.name),args=args))
report=dict(wall=time.time(),processes=relevant,
    display148_lock=Path('/tmp/.X148-lock').exists(),
    disk=subprocess.run(['df','-h','/home/ly'],capture_output=True,text=True).stdout,
    gpu=subprocess.run(['nvidia-smi','--query-gpu=index,memory.used,utilization.gpu','--format=csv'],capture_output=True,text=True).stdout)
(root/('prestart_'+time.strftime('%Y%m%d_%H%M%S')+'.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
assert not relevant and not report['display148_lock'],'Existing processes/display must be investigated before test'
