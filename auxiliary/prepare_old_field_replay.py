from pathlib import Path
p=Path(__file__).with_name('replay_field_depth_geometry.py')
s=p.read_text(encoding='utf-8')
s=s.replace("s=s.replace(\"'new_depth_replay.json'\", \"'field_depth_replay.json'\")", "s=s.replace(\"'new_depth_replay.json'\", \"'field_depth_old_geometry.json'\")")
s=s.replace("exec(compile(s,", "s=s.replace('from visual_navigation.explorfm_triangulation import localization_geometry as geometry', '''import importlib.util\n_spec=importlib.util.spec_from_file_location('old_localization_geometry', '/home/ly/execution_continuity_review_20261022/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py')\ngeometry=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(geometry)''')\nexec(compile(s,")
Path('replay_field_old_geometry.py').write_text(s,encoding='utf-8')
shell=Path('replay_field_depth_geometry.sh').read_text(encoding='utf-8').replace('replay_field_depth_geometry.py','replay_field_old_geometry.py').replace('field_depth_replay','field_depth_old_geometry')
Path('replay_field_old_geometry.sh').write_text(shell,encoding='utf-8',newline='\n')
