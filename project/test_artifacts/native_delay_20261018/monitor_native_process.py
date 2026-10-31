"""Read-only kernel queue/drop and scheduling samples for one Native incarnation."""
import json
from pathlib import Path
import sys,time
out=Path(sys.argv[1]);assert not out.exists()
end=time.monotonic()+float(sys.argv[2]);target=None
with out.open('x',buffering=1) as log:
    while time.monotonic()<end:
        if target is None:
            for p in Path('/proc').iterdir():
                if not p.name.isdigit():continue
                try:args=(p/'cmdline').read_bytes().decode().split('\0')
                except (OSError,UnicodeError):continue
                if any(Path(t).name=='native_vehicle_control' for t in args[:2]):
                    target=p;break
        if target is None:time.sleep(.1);continue
        try:
            sockets=set()
            for fd in (target/'fd').iterdir():
                try:link=fd.readlink().as_posix()
                except OSError:continue
                if link.startswith('socket:['):sockets.add(link[8:-1])
            udp=[]
            for line in (target/'net/udp').read_text().splitlines()[1:]:
                fields=line.split()
                if fields[9] in sockets:
                    udp.append(dict(inode=fields[9],local=fields[1],tx_rx_queue=fields[4],drops=int(fields[-1])))
            threads=[]
            for task in (target/'task').iterdir():
                try:
                    sched=(task/'schedstat').read_text().split()
                    threads.append(dict(tid=task.name,wchan=(task/'wchan').read_text(),runtime_ns=int(sched[0]),runqueue_wait_ns=int(sched[1])))
                except OSError:continue
            log.write(json.dumps(dict(wall_ns=time.time_ns(),mono_ns=time.monotonic_ns(),pid=int(target.name),udp=udp,threads=threads))+'\n')
        except OSError:break
        time.sleep(.05)
