"""Use existing gtest build flags without mutating the deployment/build."""
import hashlib,os,shlex,subprocess
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022')
build=base/'v4/build/wildos_ackermann_smac';candidate=base/'v22_odometry_ingress/candidate/ackermann_smac'
# Reject silently using a different native source/library version.
for rel in ('src/planner.cpp','src/connector.cpp','include/wildos_ackermann_smac/planner.hpp','include/wildos_ackermann_smac/connector.hpp'):
    assert (candidate/rel).read_bytes()==(base/'v4/candidate/ackermann_smac'/rel).read_bytes(),rel
flags={}
for line in (build/'CMakeFiles/test_planner.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        key,value=line.split(' = ',1);flags[key]=shlex.split(value)
out=base/'fixed_heading_replay_20261024';out.mkdir(exist_ok=True)
obj=out/'heading.o';exe=out/'heading_replay'
cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c','/tmp/replay_fixed_heading_cases.cpp','-o',str(obj)]
subprocess.run(cmd,cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/test_planner.dir/link.txt').read_text());link[1]=str(obj);link[link.index('-o')+1]=str(exe)
subprocess.run(link,cwd=build,check=True)
print(str(exe),flush=True)
