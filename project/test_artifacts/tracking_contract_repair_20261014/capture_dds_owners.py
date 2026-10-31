"""Whitelist process policy evidence; never save arbitrary environment fields."""
import json,hashlib,os,sys
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
sys.path.insert(0,str(BASE/'bundle/tools'))
from server_test_preflight import process_rows,dds_process_errors
session=Path(sys.argv[1]);rows=process_rows()
root=int((session/'stack.pid').read_text());sensor=int((session/'sensor.pid').read_text())
models={int((session/(role+'.pid')).read_text()) for role in ('vision','triangulation')
        if (session/(role+'.pid')).exists()}
owned=[r for r in rows if r['domain']=='88' and (os.getpgid(r['pid'])==root or r['pid']==sensor or r['pid'] in models)]
groups={str(r['pid']):[r] for r in owned if r['pid']!=sensor}
groups.update({'model/'+str(pid):[row for row in owned if row['pid']==pid] for pid in models})
sensor_rows=[r for r in owned if r['pid']==sensor]
profile=(BASE/'bundle/nav2_adapter/config/fastdds_local_udp.xml').read_bytes()
errors=dds_process_errors(groups,sensor_rows,profile)
assert len(owned)>=12,'Owned physical stack missing processes'
for row in owned:
    channels=[]
    for path in (Path('/proc')/str(row['pid'])/'fd').iterdir():
        try:
            target=os.readlink(path)
            if 'fastrtps_port' in target:channels.append(target)
        except FileNotFoundError:continue
    row['dds_shm_port_handles']=channels
    if channels:errors.append('Unexpected SHM port handles: '+str(row['pid']))
report={'passed':not errors,'scope':'Policy/source/FD inspection, not target arrival',
    'errors':errors,'owners':owned,'profile_sha256':hashlib.sha256(profile).hexdigest()}
(session/'dds_owners.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'passed':report['passed'],'owners':len(owned),'errors':errors}))
assert report['passed']
