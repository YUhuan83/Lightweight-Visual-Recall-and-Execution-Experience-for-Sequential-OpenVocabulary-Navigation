"""Extract bounded recorded input cases; published maps are not private Smac snapshots."""
import hashlib,json,math,sqlite3
from pathlib import Path
from nav_msgs.msg import Path as RosPath,OccupancyGrid
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
from wildos_nav2_adapter.motion_path import motion_sections
from wildos_nav2_adapter.input_quality import stamp_ns

root=Path('/home/ly/execution_continuity_review_20261022')
out=root/'fixed_heading_replay_20261024';out.mkdir(exist_ok=True)
trials=[root/'v21_planning_inputs_r2/full/trial_20261024_162748',
        root/'v22_odometry_ingress/full/trial_20261024_165941',
        root/'v22_odometry_ingress/full/trial_20261024_170417']
def pose(p):
    q=p.orientation
    return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
rows=[]
for trial in trials:
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    def read(topic,kind):
        tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
        return [(t,deserialize_message(b,kind)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    plans=sorted({d['stamp_ns']:d for _,m in read('/wildos/plan_status',String) for d in [json.loads(m.data)]}.items())
    full=read('/wildos/full_planned_path',RosPath);maps=read('/global_costmap/costmap',OccupancyGrid)
    db.close();selected=[]
    for bt,path in full:
        if not path.poses:continue
        points=[pose(p.pose) for p in path.poses]
        sections=motion_sections([p[:2] for p in points]);start=points[0]
        if len(sections)<2:continue
        related=[e for t,e in plans if t>=bt-50_000_000 and t-bt<1_500_000_000 and e.get('path_source_stamp_ns')==stamp_ns(path)]
        if not related:continue
        planned=related[0]
        requests=[e for t,e in plans if t<bt and bt-t<6_000_000_000 and e.get('requested_goal_pose') and e['replay_inputs']['navigation_epoch']==planned['replay_inputs']['navigation_epoch']]
        if not requests:continue
        request=requests[-1];target=request['requested_goal_pose']
        distance=math.dist(start[:2],target[:2]);bearing=math.atan2(target[1]-start[1],target[0]-start[0]);error=math.remainder(bearing-start[2],2*math.pi)
        if not 1<distance<9 or math.cos(error)<.5:continue
        # Exact same endpoint with a single-arc terminal heading, if its radius is feasible.
        arc_radius=distance/(2*abs(math.sin(error))) if abs(math.sin(error))>1e-8 else 1e20
        variants=[target,[*target[:2],math.remainder(2*bearing-start[2],2*math.pi)],[*target[:2],start[2]]]
        source=request['replay_inputs']['global_costmap_stamp_ns']
        map_matches=[(t,m) for t,m in maps if stamp_ns(m)==source]
        if not map_matches:continue
        _,grid=map_matches[-1]
        selected.append(dict(trial=str(trial),elapsed=(planned['stamp_ns']-plans[0][0])/1e9,request=request,actual_start=start,
          recorded_endpoint=points[-1],recorded_sections=len(sections),recorded_length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])),
          distance=distance,bearing_error=error,single_arc_radius=arc_radius,goals=variants,grid=grid))
    # Prefer geometries where relaxing non-observation yaw can plausibly remove a cusp.
    selected.sort(key=lambda d:(d['single_arc_radius']<8,-d['recorded_sections'],d['elapsed']))
    for row in selected[:3]:
        grid=row.pop('grid');idx=len(rows);fixture=out/f'case_{idx}.txt'
        with fixture.open('w') as f:
            f.write(' '.join(map(str,[grid.info.width,grid.info.height,grid.info.resolution,grid.info.origin.position.x,grid.info.origin.position.y,*row['actual_start'],len(row['goals'])]))+'\n')
            for p in row['goals']:f.write(' '.join(map(str,p))+'\n')
            for i,v in enumerate(grid.data):
                if v:f.write(f'{i} {v}\n')
        row['fixture']=str(fixture);row['fixture_sha256']=hashlib.sha256(fixture.read_bytes()).hexdigest();rows.append(row)
encoded=json.dumps(rows,sort_keys=True).encode();(out/'manifest.json').write_bytes(encoded)
print(json.dumps(dict(cases=len(rows),sha256=hashlib.sha256(encoded).hexdigest(),summary=[{k:r[k] for k in ('elapsed','distance','recorded_sections','single_arc_radius','fixture')} for r in rows])))
