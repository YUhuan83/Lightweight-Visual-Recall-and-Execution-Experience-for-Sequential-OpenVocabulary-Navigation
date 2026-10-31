"""Read-only pre-test host/process/version inventory; no simulator connection."""
import hashlib,json,os,shutil,subprocess,time
from pathlib import Path

def command(args):
    p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=12)
    return dict(exit=p.returncode,output=p.stdout)
rows=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if not argv or 'server_status.py' in argv:continue
        if any(s in argv for s in ('BeamNG','beamng','wildos','graphnav','planner_server','controller_server','rviz','ros2 bag','Xvfb')):
            rows.append(dict(pid=int(proc.name),command=argv))
    except (PermissionError,FileNotFoundError,ProcessLookupError):pass
paths={}
for folder in ('wildos-console','frontier_motion_handoff_20261016','frontier_resolution_repair_20261016'):
    p=Path('/home/ly')/folder
    paths[str(p)]=dict(exists=p.is_dir(),manifest=[])
    for name in ('manifest.json','official_wildos_beamng_bundle.manifest.json'):
        if (p/name).is_file():
            v=json.loads((p/name).read_text());paths[str(p)]['manifest'].append(dict(file=name,sha256=v.get('sha256'),members=len(v.get('members',{}))))
print(json.dumps(dict(host=os.uname().nodename,wall=time.time(),uptime=Path('/proc/uptime').read_text(),
    disk=shutil.disk_usage('/home/ly')._asdict(),processes=rows,versions=paths,
    gpu=command(['nvidia-smi','--query-gpu=index,name,memory.used,memory.free,utilization.gpu','--format=csv,noheader']),
    memory=command(['free','-m'])),indent=2))
