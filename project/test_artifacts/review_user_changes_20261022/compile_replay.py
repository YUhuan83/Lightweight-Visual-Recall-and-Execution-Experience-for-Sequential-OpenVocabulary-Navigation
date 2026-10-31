from pathlib import Path
import json,shlex,subprocess,os
root=Path('/home/ly/execution_continuity_review_20261022/v2')
commands=json.loads((root/'build/graphnav_mapper/compile_commands.json').read_text())
entry=next(e for e in commands if e['file'].endswith('test_mapper.cpp'))
args=shlex.split(entry['command']);args=args[:args.index('-o')]
args=[a for a in args if not a.startswith('-std=')]
args += ['-std=c++20','-I/home/ly/beamng_ros2_0394_ws/install/graaf_vendor/include']
args += ['-I'+str(root/'candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include'),'-c',str(root.parent/'incoming/replay_mapper.cpp'),'-o',str(root/'mapper_replay.o')]
subprocess.run(args,cwd=entry['directory'],check=True)
link=shlex.split((root/'build/graphnav_mapper/CMakeFiles/graphnav_mapper_test_mapper.dir/link.txt').read_text())
link=[str(root/'mapper_replay.o') if 'test_mapper.cpp.o' in a else str(root/'mapper_replay') if a=='graphnav_mapper_test_mapper' else a for a in link]
link += ['/home/ly/frontier_absence_repair_20261020/install/graphnav_planner/lib/libgraphnav_planner_component.so']
subprocess.run(link,cwd=entry['directory'],check=True)
baseline=root/'baseline';baseline.mkdir(exist_ok=True)
mapper=next(e for e in commands if e['file'].endswith('/src/mapper.cpp'))
cmd=shlex.split(mapper['command']);cmd=cmd[:cmd.index('-o')]
cmd += ['-c',str(root.parent/'v1/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src/mapper.cpp'),'-o',str(baseline/'mapper.o')]
subprocess.run(cmd,cwd=mapper['directory'],check=True)
original=shlex.split((root/'build/graphnav_mapper/CMakeFiles/graphnav_mapper_component.dir/link.txt').read_text())
original=[str(baseline/'mapper.o') if a.endswith('/src/mapper.cpp.o') else str(baseline/'libgraphnav_mapper_component.so') if a=='libgraphnav_mapper_component.so' else a for a in original]
subprocess.run(original,cwd=mapper['directory'],check=True)
trial=root.parent/'v1/full/trial_20261022_155227'
row=min(json.loads((trial/'fluency_audit/graph_topology_audit.json').read_text()),key=lambda r:abs(r['elapsed']-264))
pairdir=trial/'sessions/session_20261022_155227/graph_pairs'
maps=trial/'fluency_audit/longstop_maps'
call=[str(root/'mapper_replay'),str(pairdir/row['files']['raw']['file']),str(maps/'265_local_terrain.cdr'),str(maps/'265_terrain_tf.cdr'),str(maps/'265_odom.cdr')]
print('BASELINE: original Mapper with identical recorded fixtures',flush=True)
subprocess.run(call,env=dict(os.environ,LD_LIBRARY_PATH=str(baseline)+':'+os.environ.get('LD_LIBRARY_PATH','')),check=True)
print('CANDIDATE: repaired Mapper with identical recorded fixtures',flush=True)
subprocess.run(call,check=True)
