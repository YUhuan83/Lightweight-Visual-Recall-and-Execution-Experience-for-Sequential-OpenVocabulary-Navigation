from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/wildos-console');p=Path('/home/ly/exploration_owner_field_20261016');candidate=Path('/home/ly/exploration_owner_repair_20261016')
m=json.loads((candidate/'official_wildos_beamng_bundle.manifest.json').read_text());assert m['sha256']=='11af6c17b0335766c50ce892bf128af5ab88f9a7a82571e0799f3e555e06d9cd'
z=candidate/'official_wildos_beamng_bundle.zip';assert hashlib.sha256(z.read_bytes()).hexdigest()==m['sha256']
old=json.loads((r/'source_identity.json').read_text());assert old['sha256']=='9bcaf8700786cbaf2ab40a22405121f20288fae5e8f51d6d777fbf5eb3d87997'
for n,h in old['members'].items():assert hashlib.sha256((r/'bundle'/n).read_bytes()).hexdigest()==h,n
allowed=json.loads((p/'delta.json').read_text())['changed'];changed={n for n,h in m['members'].items() if old['members'].get(n)!=h};assert changed==set(allowed)
for n,h in m['members'].items():
 if n!='docs/SELF_SERVICE_CONSOLE.md':assert hashlib.sha256((candidate/'frozen_da6aeea0'/n).read_bytes()).hexdigest()==h,n
lib=candidate/'final_install/lib/libgraphnav_planner_component.so';assert hashlib.sha256(lib.read_bytes()).hexdigest()=='cb188e47921472dd6e598e20fac3be8874575cdde1ecb848a7b1449d15c15dc7'
dest=r/'bundle_candidate_11af6c17';dest.mkdir(exist_ok=False)
with zipfile.ZipFile(z) as archive:
 assert set(archive.namelist())==set(m['members'])
 for n,h in m['members'].items():
  target=dest/n;assert target.resolve().is_relative_to(dest.resolve());b=archive.read(n);assert hashlib.sha256(b).hexdigest()==h
  target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(b)
(r/'bundle').rename(r/'bundle_9bcaf870');dest.rename(r/'bundle')
(r/'source_identity.json').rename(r/'source_identity_9bcaf870.json');(r/'source_identity.json').write_text(json.dumps(m,indent=2))
cfg=json.loads((r/'console.json').read_text());(p/'console_before.json').write_text(json.dumps(cfg,indent=2))
cfg['setups'].append(str(candidate/'final_install/setup.bash'));cfg['library_sha256']['libgraphnav_planner_component.so']=hashlib.sha256(lib.read_bytes()).hexdigest()
(r/'console.json').write_text(json.dumps(cfg,indent=2))
cfg.update(output_root=str(r/'audit_results'),display=':148',rviz=True)
(p/'sensor_full_console.json').write_text(json.dumps(cfg,indent=2))
(p/'deployment.json').write_text(json.dumps(dict(source=m['sha256'],changed=sorted(changed),planner_library=str(lib),planner_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),old_bundle=str(r/'bundle_9bcaf870')),indent=2))
print('Paired frozen deployment ready; startup verification still required')
