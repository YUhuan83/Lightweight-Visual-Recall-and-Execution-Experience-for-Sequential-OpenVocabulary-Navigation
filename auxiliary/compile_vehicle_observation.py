import shlex,subprocess
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');build=base/'v4/build/wildos_ackermann_smac'
for rel in ('src/planner.cpp','src/connector.cpp','include/wildos_ackermann_smac/planner.hpp','include/wildos_ackermann_smac/connector.hpp'):
    assert (base/'v33_integrated_progress/candidate/ackermann_smac'/rel).read_bytes()==(base/'v4/candidate/ackermann_smac'/rel).read_bytes(),rel
flags={}
for line in (build/'CMakeFiles/test_planner.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        key,value=line.split(' = ',1);flags[key]=shlex.split(value)
out=base/'v33_integrated_progress/observation_pose_probe';out.mkdir(exist_ok=True)
obj=out/'probe.o';exe=out/'probe'
subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(base/'probe_vehicle_observation.cpp'),'-o',str(obj)],cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/test_planner.dir/link.txt').read_text());link[1]=str(obj);link[link.index('-o')+1]=str(exe)
subprocess.run(link,cwd=build,check=True)
