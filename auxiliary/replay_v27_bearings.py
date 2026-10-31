"""Run the actual old/new process callbacks on v26 recorded bearing inputs."""
import ast, hashlib, json, sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np

root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
out=root/'fresh_bearing_repair_20261025'
visual='wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
sys.path.insert(0,str(out/'candidate'/visual))
from visual_navigation.explorfm_triangulation.localization_geometry import triangulate_bearings
data=json.loads((root/'arrival_decoupling_20261025/v26_bearing_replay.json').read_text())

def replay(candidate, rows):
    source=candidate/visual/'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
    tree=ast.parse(source.read_text(encoding='utf8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
    cls.bases=[];cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef)
        and n.name in ('process_observation','prune_views','stamp_ns')]
    ns=dict(np=np,triangulate_bearings=triangulate_bearings)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(source),'exec'),ns)
    n=ns['ObjectMaskTriangulator']()
    n.active_query='green residential house';n.query_started_ns=n.last_observation_ns=0
    n.views=[];n.bearing_views=[];n.prev_view_pos=None;n.observation_timeout=5.;n.min_view_distance=1.
    n.max_views=12;n.get_logger=Mock();n.publish_navigation_goal_and_marker=Mock()
    n.publish_goal_hypotheses=Mock();n.publish_localization_rejection=Mock()
    results=[]
    for row in rows:
        if not row['paired']:continue
        t=row['stamp_ns'];p=row['robot_position']
        msg=S(query=n.active_query,header=S(stamp=S(sec=t//10**9,nanosec=t%10**9)),
              odom=S(pose=S(pose=S(position=S(x=p[0],y=p[1],z=p[2])))))
        n.get_clock=lambda:S(now=lambda:S(nanoseconds=t+int(row['source_age_at_pair_s']*1e9)))
        n.msg_buffer=S(buffer=[1],pop_oldest_msg=lambda: (dict(obj_mask=msg,lidar=object()),None,None))
        n.extract_data_from_obj_mask_msg=lambda _:[]
        current=dict(origin=np.array(row['camera_origin']),ray=np.array(row['ray']),stamp_ns=t,angular_tolerance=.04)
        n.bearing_observation=lambda _:current
        def add(_):
            n.views.append(object());n.bearing_views.append(current)
            n.views=n.views[-n.max_views:];n.bearing_views=n.bearing_views[-n.max_views:]
        n.add_views=add;n.check_lidar_in_mask=lambda *_:None
        n.process_observation()
        if n.triangulated_position is not None:
            results.append(dict(stamp_ns=t,position=n.triangulated_position.tolist(),
                                keyframes=len(n.bearing_views)))
    return results

reports=[]
for item in data:
    before=replay(root/'arrival_decoupling_20261025/candidate',item['rows'])
    after=replay(out/'candidate',item['rows'])
    assert len(before)==1,before
    # These are localization proposals, without semantic ground truth or field actuation.
    reports.append(dict(trial=item['trial'],before=before,after=after,
                        before_count=len(before),after_count=len(after)))
encoded=json.dumps(reports,sort_keys=True).encode()
(out/'recorded_callback_replay.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),
    rounds=[dict(before=r['before_count'],after=r['after_count']) for r in reports])))
