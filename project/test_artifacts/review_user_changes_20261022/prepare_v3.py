from pathlib import Path
import hashlib,json,shutil
root=Path('/home/ly/execution_continuity_review_20261022');old=root/'v2';new=root/'v3'
if new.exists():
    assert new.resolve().is_relative_to(root.resolve()) and new.name=='v3'
    shutil.rmtree(new)
new.mkdir();shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
smac=root/'incoming/smac_v3'
rels=['include/wildos_ackermann_smac/connector.hpp','src/connector.cpp','src/planner.cpp','test/test_connector.cpp','test/test_planner.cpp']
for rel in rels: shutil.copy2(smac/rel,new/'candidate/ackermann_smac'/rel)
full=new/'full';full.mkdir()
for p in (old/'full').glob('*.py'):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
for rel in rels:
    key='ackermann_smac/'+rel
    manifest['members'][key]=hashlib.sha256((new/'candidate'/key).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((old/'full/config.json').read_text());config['setups'].append(str(new/'install/local_setup.bash'))
config['python_paths'].insert(0,str(new/'candidate/wildos_port/repro_check/src/nebula2-wildos'));config['output_root']=str(full/'sessions')
(full/'config.json').write_text(json.dumps(config,indent=2));shutil.copy2(root/'v1/real_ros_review.py',new/'real_ros_review.py')
(new/'build.sh').write_text('''#!/usr/bin/env bash\nset -eo pipefail\nsource <(sed '/^exec /,' /home/ly/execution_continuity_review_20261022/v2/run_full.sh)\nsource /home/ly/execution_continuity_review_20261022/v3/install/local_setup.bash\nexport PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH\ncd /home/ly/execution_continuity_review_20261022/v3\ncolcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_smac --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_smac --build-base build --install-base install --parallel-workers 2 >build.log 2>&1\nsource install/local_setup.bash\ncolcon test --build-base build --install-base install --packages-select wildos_ackermann_smac --event-handlers console_direct+ >smac_tests.log 2>&1\ncolcon test-result --test-result-base build/wildos_ackermann_smac --verbose >>smac_tests.log 2>&1\nexport ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1\nexport PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH\npython real_ros_review.py >real_ros_review.log 2>&1\n''')
shell=(old/'run_full.sh').read_text().replace(str(old),str(new))
shell=shell.replace('source '+str(root/'v1/install/local_setup.bash')+'\n','source '+str(root/'v1/install/local_setup.bash')+'\nsource '+str(root/'v2/install/local_setup.bash')+'\n')
(new/'run_full.sh').write_text(shell)
(new/'deployment_receipt.json').write_text(json.dumps({'base':str(old),'smac_changed':rels,'source_members':len(manifest['members'])},indent=2))
print(json.dumps({'destination':str(new),'changed_smac':len(rels),'source_members':len(manifest['members'])}))


