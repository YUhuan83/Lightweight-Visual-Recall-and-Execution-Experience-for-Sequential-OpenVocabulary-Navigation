from pathlib import Path
import shlex, subprocess, json
root=Path('/home/ly/execution_continuity_review_20261022/v4')
build=root/'build/wildos_ackermann_smac'
command_files=[root/'build/compile_commands.json',build/'compile_commands.json']
command_file=next((p for p in command_files if p.exists()),None)
if command_file:
    entry=next(e for e in json.loads(command_file.read_text()) if e['file'].endswith('test_planner.cpp'))
    args=shlex.split(entry['command']);args=args[:args.index('-o')]
else:
    flags={k:v for line in (build/'CMakeFiles/test_planner.dir/flags.make').read_text().splitlines()
           if ' = ' in line for k,v in [line.split(' = ',1)]}
    args=['/usr/bin/c++']+sum([shlex.split(flags[k]) for k in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS')],[])
    entry={'directory':str(build)}
args+=['-c',str(root.parent/'incoming/replay_last_observation.cpp'),'-o',str(root/'observation_replay.o')]
subprocess.run(args,cwd=entry['directory'],check=True)
links=list((root/'build/wildos_ackermann_smac/CMakeFiles').glob('*planner*/link.txt'))
assert len(links)==1,links
link=shlex.split(links[0].read_text())
link=[str(root/'observation_replay.o') if 'test_planner.cpp.o' in a else str(root/'observation_replay') if a=='test_planner' else a for a in link]
subprocess.run(link,cwd=entry['directory'],check=True)
