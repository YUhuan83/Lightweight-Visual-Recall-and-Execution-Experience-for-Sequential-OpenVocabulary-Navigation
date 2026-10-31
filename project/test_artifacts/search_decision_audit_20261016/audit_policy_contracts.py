"""Audit policy contracts against frozen source and recorded candidates.
Validation mocks isolate dispatch policy; this is not vehicle/map acceptance.
"""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import ast,json,math,sys,copy
import numpy as np
r=Path(__file__).resolve().parents[2];sys.path[:0]=[str(r/'nav2_adapter'),str(r/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from wildos_nav2_adapter.motion_path import motion_sections
from visual_navigation.wildos.graph_observation_store import GraphObservationStore

def method(path,name,env):
 tree=ast.parse(path.read_text(encoding='utf-8'));fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
 exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),env);return env[name]
p=Path(__file__).parent;trace=json.loads((p/'closed_decision_trace.json').read_text(encoding='utf-8'))
long=max((v for v in trace['full_candidates'] if v['execution_first_section_matched'] and math.dist(v['poses'][-1][:2],v['next_following']['graph_goal'])<1.),key=lambda v:v['length_m'])
path=S(header=S(frame_id='spot1/odom'),poses=[S(pose=S(position=S(x=x,y=y),orientation=S(x=0.,y=0.,z=math.sin(h/2),w=math.cos(h/2)))) for x,y,h in long['poses']])
target=tuple(long['next_following']['graph_goal']);h=1.96
ns=dict(math=math,xy=lambda p:(p.position.x,p.position.y),yaw=lambda p:2*math.atan2(p.orientation.z,p.orientation.w),GoalStatus=S(STATUS_SUCCEEDED=4),distance=math.dist,initial_maneuver_already_reached=lambda *a,**k:False,ACKERMANN_CUSP_POSITION_TOLERANCE_M=.2)
fn=method(r/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','on_plan_result',ns)
n=S(current_plan_request=lambda *a:True,input_freshness_issue=lambda:None,get_logger=lambda:Mock(),grid=S(header=S(frame_id='spot1/odom')),extending_path=False,odom=S(pose=S(pose=path.poses[0].pose)),candidates=[(*target,h,10.),(*target,h+.8,10.)],current_route_points=lambda:[(path.poses[0].pose.position.x,path.poses[0].pose.position.y),target],validate_path=lambda *a,**k:None,carrier_model='ackermann',initial_tracking_issue=lambda *a:None,full_path_pub=Mock(),memory_selecting=False,exploration_observation_heading=lambda:h,route_endpoint=target,memory_goal=None,normal_best=None,execute_checked_path=Mock(),try_candidate=Mock())
fn(n,Mock(result=lambda:S(status=4,result=S(path=path))),1,0,1)
assert n.execute_checked_path.call_count==1 and n.try_candidate.call_count==0
observation=dict(scope='Exact on_plan_result method with map/tracking validators mocked; recorded accepted multi-section path. Demonstrates observation-heading priority skips alternative comparison, not that a shorter alternative is feasible on this map.',recorded_candidate_length_m=long['length_m'],sections=long['sections'],executed_immediately=n.execute_checked_path.called,alternative_plans_requested=n.try_candidate.call_count)
store=GraphObservationStore(':memory:','audit');store.register_map('m','audit')
store.record_search_coverage('m','target','same-node',[0],1.,timestamp=1.)
store.record_search_coverage('m','target','same-node',[8],.1,timestamp=2.)
coverage=store.get_search_coverage('m','target','same-node')
assert coverage['negative_evidence']==1. and coverage['heading_mask']==257
crossbin=dict(recorded_evidence_per_direction={'0':1.,'8':.1},stored=dict(coverage),applied_to_bin_8=coverage['negative_evidence'],new_node_same_region_has_coverage=store.get_search_coverage('m','target','new-node') is not None,scope='Storage/policy representational counterexample; no archived per-node database was preserved to assert this caused the field revisit')
navsrc=r/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
fn=method(navsrc,'_record_negative_search',dict(np=np,torch=S(is_tensor=lambda _:False),wall_time=lambda:100.,GraphObservationStore=GraphObservationStore))
node=S(uuid='negative-node');odom=S(pose=S(pose=S(orientation=S(x=0.,y=0.,z=0.,w=1.))))
owner=S(visual_memory_map=store,object_search_mode=True,negative_search_interval=5.,last_negative_search_at=-math.inf,uuid_to_str=lambda x:x,num_angular_bins=16,memory_map_id='m',text_queries=['target'],num_cameras=1)
fn(owner,S(current_node_idx=0,nodes=[node]),[{}],np.array([[[[.99]]]]),odom)
negative=store.get_search_coverage('m','target','negative-node')
assert math.isclose(negative['negative_evidence'],.01)
negative_summary=dict(raw_cosine_max=.99,accepted_target_mask_assumed_absent=True,recorded_negative_evidence=negative['negative_evidence'],default_heading_penalty=.2*negative['negative_evidence'],cpp_keep_target_bonus=.15,scope='Exact negative-search method given raw similarity; assumes rejected mask as caller condition. Rejected/missing mask does not imply high probability of absence; demonstrates incompatible evidence scales, not calibrated probabilities.')
store.close()
fn=method(r/'nav2_adapter/wildos_nav2_adapter/execution_feedback.py','finish_execution_feedback',dict(json=json,String=S))
attempt=dict(start_ns=0,actuated=True,generation=1,observed_distance=2.,tracking_invalid=False,query='target',graph=S(nodes=[]))
producer=S(execution_attempt=attempt,query_generation=1,get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)),execution_event_pub=Mock(),execution_feedback_pub=Mock())
fn(producer,False,'collision')
message=json.loads(producer.execution_event_pub.publish.call_args.args[0].data)
assert producer.execution_feedback_pub.publish.call_count==0 and not message['learning_eligible']
failure_feedback=dict(scope='Exact producer method for failed attempt; verifies unconditional success-only eligibility, not permission to learn transient/reset failures as bad terrain',diagnostic_event=message,traversal_learning_messages=producer.execution_feedback_pub.publish.call_count)
out=dict(scope='Deterministic audit counterexamples, no production edit or new field test',failure_feedback_eligibility=failure_feedback,observation_priority=observation,coverage_direction_contamination=crossbin,raw_similarity_absence_scale=negative_summary)
(p/'policy_counterexamples.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(out,indent=2))
