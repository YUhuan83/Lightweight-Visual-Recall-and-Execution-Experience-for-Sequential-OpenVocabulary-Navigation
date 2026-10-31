import json,shutil
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v30_confirmation_history')
assert not list((root/'full').glob('trial_*'))
for n in ('real_ros_report.json','real_ros_review.log'):
    if (root/n).exists():shutil.copy2(root/n,root/(n+'.before_environment_fix'))
p=root/'build.sh';s=p.read_text()
old='export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH'
assert s.count(old)==1
s=s.replace(old,'export PYTHONPATH=$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH')
p.write_text(s)
print(json.dumps(dict(fixed='review-only PYTHONPATH',runtime_source_changed=False,field_started=False)))
