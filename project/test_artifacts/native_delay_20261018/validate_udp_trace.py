"""Local UDP tracer API/errno/payload preservation; no ROS or vehicle."""
import json,os
from pathlib import Path
import socket
import subprocess
import sys,time
root=Path(__file__).parent
if len(sys.argv)>1:
    packet=b'RTPS'+b'\0'*16+json.dumps(dict(issued_wall_ns=time.time_ns(),sequence=27)).encode()
    receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);receiver.bind(('127.0.0.1',0));receiver.settimeout(1)
    receiver.setsockopt(socket.SOL_SOCKET,35,1)  # Linux SO_TIMESTAMPNS before arrival.
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sender.sendto(packet,receiver.getsockname())
    data,_=receiver.recvfrom(2048);assert data==packet
    sender.close();receiver.close();raise SystemExit()
target=root/'native_vehicle_control'
assert not target.exists();target.write_text(Path(__file__).read_text())
out=root/'udp_tracer_validation_v2.jsonl';assert not out.exists()
env=dict(os.environ,LD_PRELOAD=str(root/'libnative_rx_trace.so'),WILDOS_NATIVE_RX_TRACE_FILE=str(out))
result=subprocess.run([sys.executable,str(target),'--child'],env=env,check=True)
target.unlink()
rows=[json.loads(l) for l in out.read_text().splitlines()]
assert len(rows)==1 and rows[0]['sequence']==27
assert rows[0]['recv_wall_ns']>=rows[0]['kernel_wall_ns']>=rows[0]['issued_wall_ns']>0
print(json.dumps(dict(passed=True,kernel_timestamp_valid=True,payload_preserved=True,rows=len(rows))))
