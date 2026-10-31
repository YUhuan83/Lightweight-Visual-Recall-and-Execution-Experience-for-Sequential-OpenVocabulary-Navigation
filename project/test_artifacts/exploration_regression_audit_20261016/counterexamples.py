"""Run actual method bodies without ROS/model loading. Reproduces defects, not repair acceptance."""
from pathlib import Path
from types import SimpleNamespace as S
import ast
import importlib.util
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
WILDOS=ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos'
sys.path.insert(0,str(WILDOS.parent.parent))

def method(path,name,scope):
    tree=ast.parse(path.read_text(encoding='utf8'))
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    unit=ast.Module(body=[fn],type_ignores=[])
    exec(compile(unit,str(path),'exec'),scope)
    return scope[name]

append=method(WILDOS/'nav.py','_append_heading_candidates',dict(np=np,HeadingCandidate=S))
results=[]
for case,conf,feasible,physical,age,missing in [
    ('valid',.8,True,1.,1.,False),
    ('unknown_infeasible',0.,False,1.,1.,False),
    ('unknown_zero_physical',0.,True,0.,1.,False),
    ('stale_infeasible',.8,False,1.,20.,False),
    ('fresh_observed_blocked',.8,False,0.,1.,False),
    ('missing_assessment',0.,False,0.,0.,True),
]:
    assessment=S(robot_profile_id='default',physical_traversability=physical,
        minimum_clearance=0.,dynamic_risk=0.,confidence=conf,feasible=feasible)
    node=S(robot_traversability=[] if missing else [assessment],heading_candidates=[])
    owner=S(visual_memory_map=None,object_search_mode=True,text_queries=['target'],global_frame='map',
        heading_low_confidence_weight=.35,heading_turn_cost_weight=.05,
        heading_dynamic_risk_weight=.2,object_memory_heading_weight=.2,search_coverage_weight=.2)
    append(owner,node,np.full(16,.8),0.)
    # Corresponding C++ choose_exploration_goal physical gate, excluding heading veto.
    fresh=not missing and 0<=age<=5
    cpp_physical_veto=fresh and conf>=.7 and (not feasible or physical<=0)
    row=dict(case=case,assessment_age_s=age,confidence=conf,
        cpp_physical_veto=cpp_physical_veto,
        python_traversable_heading_count=sum(h.traversable for h in node.heading_candidates),
        final_selector_will_drop=not any(h.traversable for h in node.heading_candidates))
    results.append(row)
assert results[0]['python_traversable_heading_count']==16
assert results[1]['cpp_physical_veto'] is False and results[1]['final_selector_will_drop']
assert results[3]['cpp_physical_veto'] is False and results[3]['final_selector_will_drop']
assert results[4]['cpp_physical_veto'] is True

spec=importlib.util.spec_from_file_location('audit_graph_observation',WILDOS/'graph_observation_store.py')
store_module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=store_module;spec.loader.exec_module(store_module)
store=store_module.GraphObservationStore(':memory:','audit')
store.register_map('map')
store.record_search_coverage('map','target','node',[0],1.,timestamp=1.)
store.record_search_coverage('map','target','node',[8],.1,timestamp=2.)
coverage=store.get_search_coverage('map','target','node')
assert coverage['negative_evidence']==1. and coverage['heading_mask']==257
assert store.get_search_coverage('map','target','replacement-node') is None
store.close()

finish=method(ROOT/'nav2_adapter/wildos_nav2_adapter/execution_feedback.py','finish_execution_feedback',
    dict(String=lambda data:S(data=data),json=json))
published=[]
owner=S(execution_attempt=dict(start_ns=10**9,actuated=True,generation=1,observed_distance=3.,
    tracking_invalid=False,query='target',graph=None),query_generation=1,
    get_clock=lambda:S(now=lambda:S(nanoseconds=4*10**9)),
    execution_event_pub=S(publish=lambda m:published.append(json.loads(m.data))),
    input_freshness_issue=lambda:None,execution_transport_issue=lambda:None)
finish(owner,False,'execution_stalled')
assert len(published)==1 and published[0]['learning_eligible'] is False

report=dict(scope='Actual Python method/store bodies; synthetic inputs. Assertions verify existing defects reproduce. No deployed fix and no field success claimed.',
    heading_contract=results,coverage_direction_crosstalk=dict(actual=coverage,
        direction_8_supplied_strength=.1,direction_8_retrieved_strength=coverage['negative_evidence'],replacement_uuid_inherits=False),
    failure_learning_producer=published)
(OUT/'counterexamples.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
