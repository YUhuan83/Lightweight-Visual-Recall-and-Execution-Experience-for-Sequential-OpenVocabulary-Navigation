"""Read /proc identities of the owned field stack; no source/config mutation."""
import hashlib,json,os
from pathlib import Path
import sys
session=Path(sys.argv[1])
owned=json.loads((session/'owned_processes.json').read_text())
stack=next(r['pid'] for r in owned if r['role']=='stack')
expected={'libwildos_ackermann_smac.so':'fc149c802333ebbea9504e175ae92050821a86a2debb31aaed276fbf1d711a33',
          'libwildos_ackermann_rpp.so':'d2fd9cee5ecabb49e8be06a24079813eb46a9e77872ef6982d1ed547bb04d846',
          'libgraphnav_planner_component.so':'db52f00e06d7c598ecba3e8c42d1b34019693cd879f03537a5a36e9f6e1c6b8f'}
found={};native=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        pid=int(proc.name)
        if os.getpgid(pid)!=stack:continue
        tokens=(proc/'cmdline').read_bytes().decode().split('\0')
        if any(Path(a).name=='native_vehicle_control' for a in tokens[:2]):
            env=dict(item.split(b'=',1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
            assert env.get(b'WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT')!=b'1'
            native.append(dict(pid=pid,ros_domain=env.get(b'ROS_DOMAIN_ID',b'').decode(),
                               native_qos_unchanged=True,udp_diagnostic=env.get(b'LD_PRELOAD',b'').decode()))
        paths={Path(l.split()[-1]) for l in (proc/'maps').read_text().splitlines() if '.so' in l and l.split()[-1].startswith('/')}
        for path in paths:
            if path.name not in expected:continue
            digest=hashlib.sha256(path.read_bytes()).hexdigest();assert digest==expected[path.name],str(path)
            found[path.name]=dict(pid=pid,path=str(path),sha256=digest)
    except (FileNotFoundError,ProcessLookupError):continue
assert set(found)==set(expected),found
assert len(native)==1,native
report=dict(passed=True,session=str(session),stack_pgid=stack,libraries=found,native=native)
(session/'running_identity.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
