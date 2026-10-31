"""Read-only DDS socket/scheduling snapshot; never dump unrelated environment."""
import json,os,time
from pathlib import Path
root=Path('/home/ly/planning_phase_physical_20261018')
out=Path('/home/ly/execution_wait_audit_20261018/startup_receipt')
rows=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=(proc/'cmdline').read_bytes().decode().strip('\0').split('\0')
        if not args or not args[0].startswith(str(root/'install')) or Path(args[0]).name not in ('native_vehicle_control','cmd_vel_to_control'):continue
        env=dict(x.split('=',1) for x in (proc/'environ').read_bytes().decode().split('\0') if '=' in x)
        sockets={}
        for fd in (proc/'fd').iterdir():
            try:
                link=os.readlink(fd)
                if link.startswith('socket:['):sockets[link[8:-1]]=fd.name
            except OSError:pass
        udp=[]
        for line in Path('/proc/net/udp').read_text().splitlines()[1:]:
            fields=line.split()
            if fields[9] in sockets:udp.append(dict(local=fields[1],remote=fields[2],queues=fields[4],inode=fields[9],drops=fields[-1]))
        threads=[]
        for task in (proc/'task').iterdir():
            try:threads.append(dict(tid=int(task.name),name=(task/'comm').read_text().strip(),schedstat=(task/'schedstat').read_text().strip()))
            except OSError:pass
        rows.append(dict(pid=int(proc.name),argv=args,environment={k:env.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE')},udp=udp,threads=threads))
    except (OSError,ValueError,UnicodeError):continue
assert {Path(r['argv'][0]).name for r in rows}=={'native_vehicle_control','cmd_vel_to_control'},rows
report=dict(wall=time.time(),scope=__doc__,processes=rows)
path=out/('transport_snapshot_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
path.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
