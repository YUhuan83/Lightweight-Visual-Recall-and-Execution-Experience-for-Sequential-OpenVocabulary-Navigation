from pathlib import Path
import hashlib,json,shutil
root=Path('/home/ly/execution_continuity_review_20261022');old=root/'v3';new=root/'v4'
assert not new.exists();new.mkdir();shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
files={
'nav2_adapter/wildos_nav2_adapter/execution_feedback.py':'native_status_v4_execution_feedback.py',
'nav2_adapter/wildos_nav2_adapter/navigation_transactions.py':'native_status_v4_navigation_transactions.py',
'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py':'native_status_v4_graph_path_bridge.py',
'nav2_adapter/test/test_native_status_executor_ros.py':'test_native_status_executor_ros.py',
'nav2_adapter/test/test_source_control_ros.py':'test_source_control_ros.py'}
for rel,src in files.items():shutil.copy2(root/'incoming'/src,new/'candidate'/rel)
full=new/'full';full.mkdir()
for p in (old/'full').glob('*.py'):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
driver=full/'driver_full.py'
driver_text=driver.read_text()
driver_text=driver_text.replace(
    "ROOT=Path(__file__).resolve().parent\nOUT=ROOT/('trial_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()",
    "ROOT=Path(__file__).resolve().parent\nROUND_TIMEOUT_SEC=180.0\nROUND_STARTED_AT=time.monotonic()\nROUND_DEADLINE=ROUND_STARTED_AT+ROUND_TIMEOUT_SEC\nOUT=ROOT/('trial_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()")
driver_text=driver_text.replace(
    "report=dict(scope=__doc__,passed=False,errors=[],complete_navigation_passed=False,video=False)",
    "report=dict(scope=__doc__,passed=False,errors=[],complete_navigation_passed=False,video=False,round_timeout_sec=ROUND_TIMEOUT_SEC)")
driver_text=driver_text.replace(
    "until=time.monotonic()+seconds",
    "until=min(time.monotonic()+seconds,ROUND_DEADLINE)")
assert 'ROUND_TIMEOUT_SEC=180.0' in driver_text and 'until=min(time.monotonic()+seconds,ROUND_DEADLINE)' in driver_text
driver.write_text(driver_text)
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
for rel in files:manifest['members'][rel]=hashlib.sha256((new/'candidate'/rel).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((old/'full/config.json').read_text());config['setups'].append(str(new/'install/local_setup.bash'))
config['python_paths'].insert(0,str(new/'candidate/wildos_port/repro_check/src/nebula2-wildos'));config['output_root']=str(full/'sessions')
(full/'config.json').write_text(json.dumps(config,indent=2))
shutil.copy2(root/'incoming/real_ros_review_v4.py',new/'real_ros_review.py')
shell=(old/'run_full.sh').read_text().replace(str(old),str(new))
shell=shell.replace('source '+str(root/'v2/install/local_setup.bash')+'\n','source '+str(root/'v2/install/local_setup.bash')+'\nsource '+str(root/'v3/install/local_setup.bash')+'\n')
(new/'run_full.sh').write_text(shell)
(new/'build.sh').write_text('''#!/usr/bin/env bash\nset -eo pipefail\nsource <(sed '\''/^exec /,$d'\'' /home/ly/execution_continuity_review_20261022/v3/run_full.sh)\nsource /home/ly/execution_continuity_review_20261022/v4/install/local_setup.bash\nexport PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH\ncd /home/ly/execution_continuity_review_20261022/v4\ncolcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_smac --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_smac --build-base build --install-base install --parallel-workers 2 >build.log 2>&1\nsource install/local_setup.bash\ncolcon test --build-base build --install-base install --packages-select wildos_ackermann_smac --event-handlers console_direct+ >smac_tests.log 2>&1\ncolcon test-result --test-result-base build/wildos_ackermann_smac --verbose >>smac_tests.log 2>&1\nexport ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1\nexport PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH\npython real_ros_review.py >real_ros_review.log 2>&1\n''')
print(json.dumps({'destination':str(new),'changed':list(files),'source_members':len(manifest['members'])}))
