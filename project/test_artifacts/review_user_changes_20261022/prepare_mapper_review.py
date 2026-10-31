from pathlib import Path
import hashlib,json,shutil
root=Path('/home/ly/execution_continuity_review_20261022')
old=root/'v1';new=root/'v2'
assert not new.exists()
new.mkdir();shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
p='wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper'
assert hashlib.sha256((old/'candidate'/p/'src/mapper.cpp').read_bytes()).hexdigest()==hashlib.sha256(Path('/home/ly/graph_start_attachment_repair_20261020/src/graphnav_mapper/src/mapper.cpp').read_bytes()).hexdigest() if Path('/home/ly/graph_start_attachment_repair_20261020/src/graphnav_mapper/src/mapper.cpp').exists() else True
shutil.copy2(root/'incoming/test_mapper.cpp',new/'candidate'/p/'test/test_mapper.cpp')
print('Created isolated mapper counterexample build',flush=True)
