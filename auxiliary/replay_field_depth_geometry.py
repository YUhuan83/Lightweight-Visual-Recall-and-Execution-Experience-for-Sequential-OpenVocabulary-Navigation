from pathlib import Path
s=Path(__file__).with_name('replay_new_depth_geometry.py').read_text(encoding='utf-8')
s=s.replace("root=Path('/home/ly/execution_continuity_review_20261022/v46_approach_feedback')", "root=Path('/home/ly/execution_continuity_review_20261022/v47_depth_geometry')")
s=s.replace("    n=env['ObjectMaskTriangulator']()", "    recorded_points=[dict(stamp_ns=k, data=v) for k,v in actual.items()]\n    n=env['ObjectMaskTriangulator']()")
s=s.replace("reports.append(dict(trial=trial.name", "reports.append(dict(recorded_points=recorded_points,trial=trial.name")
s=s.replace("'new_depth_replay.json'", "'field_depth_replay.json'")
exec(compile(s,'closed_v47_process_replay','exec'))
