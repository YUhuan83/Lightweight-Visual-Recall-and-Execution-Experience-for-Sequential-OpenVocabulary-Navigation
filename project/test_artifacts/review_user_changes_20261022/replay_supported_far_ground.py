"""Same recorded frames, baseline vs candidate, with unchanged clearing proof."""
import sys,importlib.util,time
from pathlib import Path
root=Path(__file__).resolve().parent
__file__=str(root/'replay_marked_surface.py')
exec(compile((root/'replay_marked_surface.py').read_text().split('results=[]')[0],__file__,'exec'))
spec=importlib.util.spec_from_file_location('candidate_cloud',root/'pointcloud_candidate.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
original=split_costmap_points
results=[]
for repeat in (1,2):
    baseline_clearing=None
    for label,fn in [('baseline',original),('candidate',candidate.split_costmap_points)]:
        split_costmap_points=fn
        began=time.perf_counter();regional,_=counts(body,base[:3,:3])
        marks,clearing=fn(body,**kwargs)
        elapsed=time.perf_counter()-began
        if label=='baseline':
            baseline_clearing=clearing
            assert len(regional)==marked['points']
        else:
            np.testing.assert_array_equal(clearing,baseline_clearing)
            assert len(regional)==0
        results.append(dict(repeat=repeat,version=label,regional_marks=len(regional),
                            total_marks=len(marks),two_calls_seconds=elapsed))
report=dict(trial=str(trial),results=results,clearing_unchanged=True)
(trial/'supported_far_ground_replay.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
