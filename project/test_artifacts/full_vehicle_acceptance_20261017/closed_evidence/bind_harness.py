from pathlib import Path
import hashlib,json
root=Path('/home/ly/full_vehicle_acceptance_20261017')
config=json.loads((root/'field/sensor_full_console.json').read_text())
lib=root/'install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so'
config['library_sha256']['libwildos_ackermann_smac.so']=hashlib.sha256(lib.read_bytes()).hexdigest()
(root/'field/sensor_full_console.json').write_text(json.dumps(config,indent=2)+'\n')
h=json.loads((root/'field/harness_manifest.json').read_text())
h['condition']['scope']='Combined execution-progress, complete-route selection and retry-admission field acceptance'
h['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'field').iterdir() if p.suffix in ('.py','.sh','.json','.yaml') and p.name!='harness_manifest.json'}
(root/'field/harness_manifest.json').write_text(json.dumps(h,indent=2)+'\n')
print(json.dumps(dict(candidate='932ac9bf8cbe6444076f2ad702c560e0ab2eed781fdc749d7687998e75b38e44',libraries=config['library_sha256'],harness=h),indent=2))
