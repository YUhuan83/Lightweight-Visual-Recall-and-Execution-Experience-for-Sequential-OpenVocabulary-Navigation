from pathlib import Path
import hashlib,json,shutil
root=Path('/home/ly/execution_continuity_review_20261022');old=root/'v1';new=root/'v2'
full=new/'full';assert not full.exists();full.mkdir()
for p in (old/'full').glob('*.py'):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
changed=[]
for n,h in manifest['members'].items():
    actual=hashlib.sha256((new/'candidate'/n).read_bytes()).hexdigest()
    if actual!=h:changed.append(n);manifest['members'][n]=actual
assert set(changed)=={'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src/mapper.cpp',
                      'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/test/test_mapper.cpp'},changed
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((old/'full/config.json').read_text());config['setups'].append(str(new/'install/local_setup.bash'))
config['python_paths'].insert(0,str(new/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['output_root']=str(full/'sessions')
lib=new/'install/graphnav_mapper/lib/libgraphnav_mapper_component.so'
config['library_sha256'][lib.name]=hashlib.sha256(lib.read_bytes()).hexdigest()
(full/'config.json').write_text(json.dumps(config,indent=2))
shell=(old/'run_full.sh').read_text().replace('export PYTHONPATH=', 'source '+str(new/'install/local_setup.bash')+'\nexport PYTHONPATH=',1)
shell=shell.replace(str(old/'full'),str(full)).replace('cd '+str(old),'cd '+str(new))
(new/'run_full.sh').write_text(shell)
(new/'deployment_receipt.json').write_text(json.dumps({'base':str(old),'changed':changed,'mapper_sha256':config['library_sha256'][lib.name]},indent=2))
print(json.dumps({'changed':changed,'mapper':str(lib),'sha256':config['library_sha256'][lib.name]}))
