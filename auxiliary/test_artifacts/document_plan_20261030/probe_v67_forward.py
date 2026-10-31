"""Actual Smac plans at the first covered local-map trigger; validate on that recorded local map."""
import bisect,json,math,os,shlex,sqlite3,subprocess
from pathlib import Path
from types import SimpleNamespace as S
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge
from wildos_nav2_adapter.graph_route import validate_local_prefix
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan');parent=root.parent
trial=parent/'v67_target_road_120s/full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
early=json.loads((root/'v67_local_ahead_replay.json').read_text())['runs'][0]['first_early'];assert early
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
query=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
first=next(e for e in events if e.get('stamp_ns'));origin=first['stamp_ns']-round((first['elapsed']-query)*1e9)
stamp=origin+round(early['q']*1e9)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def last(name):
    i,c=topics[name];t,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
    return t,deserialize_message(b,c)
gt,g=last('/global_costmap/costmap_raw');lt,local=last('/local_costmap/costmap');ot,odom=last('/spot1/odom');db.close()
confirmation=next(e['payload'] for e in reversed(events) if e.get('kind')=='object_confirmation'
    and e['elapsed']-query<=early['q'] and e.get('payload',{}).get('search_confirmed') is True)
target=confirmation['object_position'];start=early['start']
n=S(upstream_target_approach=True,object_destination=object(),expected=target[:2],memory_goal=None,memory_selecting=False,
    odom=odom,minimum_turning_radius=8.,get_parameter=lambda _:S(value=12.),
    goal_recently_failed=lambda *_:False,vehicle_pose_feasible=lambda *_:True)
candidates=ContinuousGraphPathBridge.target_forward_candidates(n,start[:2]);assert candidates
fixture=root/'v67_forward_probe.txt';meta=g.metadata
with fixture.open('w') as f:
    f.write(' '.join(map(str,[meta.size_x,meta.size_y,meta.resolution,meta.origin.position.x,meta.origin.position.y,*start,len(candidates)]))+'\n')
    for p in candidates:f.write(' '.join(map(str,p[:3]))+'\n')
    for i,cost in enumerate(g.data):
        if cost:f.write(f'{i} {cost}\n')
cpp=(parent/'probe_vehicle_observation.cpp').read_text()
needle='      std::cout<<"OBS_RESULT variant="'
insert='''      if(const auto folder=std::getenv("OBSERVATION_OUTPUT_DIRECTORY")) {
        std::ofstream csv(std::string(folder)+"/path_"+std::to_string(i)+".csv");
        csv<<std::setprecision(17);
        for(const auto & p:poses)csv<<p.x<<","<<p.y<<","<<p.yaw<<"\\n";
      }
'''
assert needle in cpp;cpp=cpp.replace(needle,insert+needle,1)
(root/'probe_smac_paths.cpp').write_text(cpp)
build=parent/'v4/build/wildos_ackermann_smac';flags={}
for line in (build/'CMakeFiles/test_planner.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        k,v=line.split(' = ',1);flags[k]=shlex.split(v)
out=root/'v67_smac_paths';out.mkdir(exist_ok=True);obj=out/'probe.o';exe=out/'probe'
subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
    '-c',str(root/'probe_smac_paths.cpp'),'-o',str(obj)],cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/test_planner.dir/link.txt').read_text());link[1]=str(obj);link[link.index('-o')+1]=str(exe)
subprocess.run(link,cwd=build,check=True)
env=os.environ.copy();env['OBSERVATION_FIXTURE']=str(fixture);env['OBSERVATION_OUTPUT_DIRECTORY']=str(out)
reports=[]
for repeat in (1,2):
    for p in out.glob('path_*.csv'):p.unlink()
    log=out/f'run_{repeat}.log'
    with log.open('w') as f:
        subprocess.run([str(exe),'--gtest_filter=ActualPlanner.RecordedObservationPoses'],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    plans=[]
    for index,candidate in enumerate(candidates):
        filename=out/f'path_{index}.csv'
        if not filename.exists():plans.append(dict(candidate=candidate,planned=False));continue
        rows=[[float(v) for v in line.split(',')] for line in filename.read_text().splitlines()]
        reason=validate_local_prefix([p[:2] for p in rows],[p[2] for p in rows],local,start[:2],start[2],8.,covered_only=True)
        plans.append(dict(candidate=candidate,planned=True,local_ahead_rejection=reason,poses=len(rows)))
    reports.append(dict(repeat=repeat,plans=plans,local_accepted=sum(p.get('planned') and p.get('local_ahead_rejection') is None for p in plans)))
report=dict(scope='Real unchanged 12m Smac; recorded map snapshots, current axle pose and user-approved bounded probes; not a driving counterfactual',
    q=early['q'],start=start,target=target,global_map_age_s=(stamp-gt)*1e-9,local_map_age_s=(stamp-lt)*1e-9,odom_age_s=(stamp-ot)*1e-9,runs=reports)
(root/'v67_smac_forward_replay.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(not all(r['local_accepted']>0 for r in reports))
