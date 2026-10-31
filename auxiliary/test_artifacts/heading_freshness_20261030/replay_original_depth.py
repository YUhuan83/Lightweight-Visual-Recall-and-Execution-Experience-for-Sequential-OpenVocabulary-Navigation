"""Compare original LiDAR median with current geometry on the SAME closed observations."""
from pathlib import Path

source=Path('/home/ly/execution_continuity_review_20261022/replay_v46_localization.py').read_text()
source=source.replace("root=Path('/home/ly/execution_continuity_review_20261022/v46_approach_feedback')",
                      "root=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')")
source=source.replace("receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']",'')
source=source.replace("sorted((root/'full').glob('trial_*'))", "sorted((root/'full').glob('trial_20261030_101938'))")
source=source.replace("localization_failure_replay.json", "original_depth_comparison.json")
source=source.replace("messages('/spot1/ouster/front/points_filtered')", "messages(config['lidar_topic'])")
source=source.replace("reports=[]", '''
original_proposals=[]
def compare_depth(points_cam,intrinsic,mask,*args,**kwargs):
    points=np.asarray(points_cam,float)
    points=points[np.all(np.isfinite(points),axis=1)&(points[:,2]>.1)]
    projected=points@intrinsic.T
    pixels=(projected[:,:2]/projected[:,2:]).astype(np.int32)
    h,w=mask.shape
    inside=(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)
    points,pixels=points[inside],pixels[inside]
    points=points[mask[pixels[:,1],pixels[:,0]]]
    proposal=dict(points=len(points))
    if len(points)>=args[0]:
        selected=np.argsort(np.linalg.norm(points,axis=1))[len(points)//2]
        proposal['original_position_cam']=points[selected].tolist()
    current=geometry.foreground_depth(points_cam,intrinsic,mask,*args,**kwargs)
    proposal['current_position_cam']=current['position'].tolist() if current is not None else None
    proposal['association_validated']=current.get('association_validated') if current else None
    original_proposals.append(proposal)
    return current
env['foreground_depth']=compare_depth
reports=[]
''')
source=source.replace("        trace=[]", "        original_proposals.clear()\n        trace=[]")
source=source.replace("row.update(source_age_s=", "row['original_depth_comparison']=list(original_proposals)\n        row.update(source_age_s=")
exec(compile(source,'current_vs_original_recorded_depth','exec'))
