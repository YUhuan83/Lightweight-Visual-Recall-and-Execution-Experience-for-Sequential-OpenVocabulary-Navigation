"""Run native strict/explicit guidance twice on historical measured graphs."""
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
root=base.with_name('v73_document_plan')
cpp=(base/'trace_frontier.cpp').read_text()
cpp=cpp.replace('if(mode=="edge_time_now")g->header.stamp=rclcpp::Time(stamp).to_msg();','')
cpp=cpp.replace('p.plan_to_goal(goal,6.,rclcpp::Time(stamp))',
    'p.plan_to_goal(goal,6.,rclcpp::Time(stamp),mode=="explicit"?g:nullptr)')
(root/'trace_frontier.cpp').write_text(cpp)
s=(base/'prepare_trace.py').read_text()
s=s.replace("root=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')",f"root=Path('{root}')")
s=s.replace("trial=root/'full/trial_20261030_174941'",f"trial=Path('{base}/full/trial_20261030_174941')")
extra='''
line=next(x for x in Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/replay_v57_inputs.txt').read_text().splitlines() if 'pair_000325_' in x)
rawfile,scorefile,gx,gy,gz,clock=line.split()
g=deserialize_message(Path(rawfile).read_bytes(),NavigationGraph)
scored=deserialize_message(Path(scorefile).read_bytes(),NavigationGraph);g.nodes=scored.nodes
filename=out/'pair325.cdr';filename.write_bytes(serialize_message(g))
cases.append(dict(file=str(filename),goal=[float(gx),float(gy),float(gz)],stamp=int(clock),pair='v57_pair325',pair_age_s=None))
'''
s=s.replace("(out/'scope.json').write_text",extra+"\n(out/'scope.json').write_text",1)
start=s.index('for i,case in enumerate(cases):')
s=s[:start]+'''for repeat in (1,2):
 for i,case in enumerate(cases):
    for mode in ('baseline','explicit'):
        log=out/f'case_{i}_{mode}_{repeat}.log'
        with log.open('w') as f:
            subprocess.run([str(exe),case['file'],*map(str,case['goal']),str(case['stamp']),mode],stdout=f,stderr=subprocess.STDOUT,check=True)
        print('repeat',repeat,'case',i,mode,next(x for x in log.read_text().splitlines() if x.startswith('PATH')),flush=True)
'''
exec(compile(s,'native_explicit_guidance_replays','exec'))
