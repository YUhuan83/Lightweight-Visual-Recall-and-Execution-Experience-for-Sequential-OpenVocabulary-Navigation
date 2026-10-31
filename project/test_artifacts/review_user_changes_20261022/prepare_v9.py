from pathlib import Path
import hashlib,json,shutil,zipfile
root=Path('/home/ly/execution_continuity_review_20261022');old,new=root/'v8',root/'v9'
assert not new.exists()
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((old/'candidate'/n).read_bytes()).hexdigest()==s for n,s in manifest['members'].items())
shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
with zipfile.ZipFile(root/'incoming/bounded_handoff_delta.zip') as archive:
    assert archive.testzip() is None
    for name in archive.namelist():
        target=new/'candidate'/name;assert target.resolve().is_relative_to((new/'candidate').resolve())
        data=archive.read(name);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        manifest['members'][name]=hashlib.sha256(data).hexdigest()
full=new/'full';full.mkdir()
for source in (old/'full').glob('*.py'):shutil.copy2(source,full/source.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((old/'full/config.json').read_text())
config['setups'].append(str(new/'install/local_setup.bash'))
config['python_paths'].insert(0,str(new/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['output_root']=str(full/'sessions');config['goal_timeout_s']=180
(full/'config.json').write_text(json.dumps(config,indent=2))
review=(old/'real_ros_review.py').read_text()
(new/'real_ros_review.py').write_text(review)
shell=(old/'run_full.sh').read_text().replace(str(old),str(new))
shell=shell.replace('source '+str(root/'v7/install/local_setup.bash')+'\n',
    'source '+str(root/'v7/install/local_setup.bash')+'\nsource '+str(old/'install/local_setup.bash')+'\n')
(new/'run_full.sh').write_text(shell)
build=(old/'build.sh').read_text().replace(str(root/'v7/run_full.sh'),str(old/'run_full.sh')).replace(str(old),str(new))
build=build.replace(str(new/'run_full.sh'),str(old/'run_full.sh'))
(new/'build.sh').write_text(build)
(new/'launch_trial.py').write_text((old/'launch_trial.py').read_text().replace(str(old),str(new)))
print(json.dumps({'destination':str(new),'frozen_members':len(manifest['members'])}))
