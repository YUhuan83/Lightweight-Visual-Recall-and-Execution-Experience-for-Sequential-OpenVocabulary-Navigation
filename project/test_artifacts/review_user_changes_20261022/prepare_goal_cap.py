from pathlib import Path
import shutil
root=Path('/home/ly/execution_continuity_review_20261022');new=root/'goal_cap_probe'
assert not new.exists();new.mkdir()
shutil.copytree(root/'v2/candidate/ackermann_smac',new/'ackermann_smac')
shutil.copy2(root/'incoming/test_planner.cpp',new/'ackermann_smac/test/test_planner.cpp')
