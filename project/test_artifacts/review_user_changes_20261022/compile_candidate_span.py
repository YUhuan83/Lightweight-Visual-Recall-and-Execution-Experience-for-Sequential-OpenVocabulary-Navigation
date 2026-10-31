from pathlib import Path
import shlex, subprocess
root=Path('/home/ly/execution_continuity_review_20261022/v4')
build=root/'build/wildos_ackermann_smac'
flags={k:v for line in (build/'CMakeFiles/test_planner.dir/flags.make').read_text().splitlines()
       if ' = ' in line for k,v in [line.split(' = ',1)]}
args=['/usr/bin/c++']+sum([shlex.split(flags[k]) for k in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS')],[])
args+=['-c',str(root.parent/'incoming/replay_candidate_span.cpp'),'-o',str(root/'candidate_span.o')]
subprocess.run(args,cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/test_planner.dir/link.txt').read_text())
link=[str(root/'candidate_span.o') if 'test_planner.cpp.o' in a else str(root/'candidate_span') if a=='test_planner' else a for a in link]
subprocess.run(link,cwd=build,check=True)
