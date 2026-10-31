"""Opt-in observation wrappers. Returns/exceptions and input objects are preserved."""
import functools,json,math,time,os
from pathlib import Path
import numpy as np

def observed(call, observe, errors):
 @functools.wraps(call)
 def run(*a,**kw):
  result=call(*a,**kw)
  try:observe(a,kw,result)
  except Exception as e:errors.append(repr(e))
  return result
 return run

def install(module,path):
 cls=module.WildOS_Nav;original=cls.__init__;nodes=[]
 def init(self,*a,**kw):
  original(self,*a,**kw);nodes.append(self)
  log=Path(path).open('x');errors=[]
  state=dict(retrieve_calls=0,retrieve_hits=0,feedback_received=0,feedback_stored=0,score_calls=0,
    visible_scoring_frames=0,memory_modified_scores=0,frame=0)
  self._audit=(log,state,errors);last=[0.];raw=[]
  def emit(kind,**values):
   log.write(json.dumps(dict(wall=time.time(),kind=kind,**values),allow_nan=False)+'\n');log.flush()
  def rows(store,table):
   return store._db.execute('SELECT count(*) FROM '+table).fetchone()[0] if store else None
  def snapshot():
   return dict(experiences=rows(self.visual_memory,'experiences'),
    node_observations=rows(self.visual_memory_map,'node_observations'),
    traversal_history=rows(self.visual_memory_map,'traversal_history'),
    search_coverage=rows(self.visual_memory_map,'search_coverage'),
    objects=rows(self.visual_memory_map,'semantic_objects'),pending_samples=len(self.pending_memory_samples),
    passive_landmarks=self.semantic_landmarks.count if self.semantic_landmarks else 0)
  self._audit_snapshot=lambda:emit('final',state=state.copy(),stores=snapshot(),errors=errors)
  emit('initial',stores=snapshot())
  if self.visual_memory:
   def retrieval(a,k,r):
    state['retrieve_calls']+=1;state['retrieve_hits']+=int(r[0] is not None)
   self.visual_memory.retrieve=observed(self.visual_memory.retrieve,retrieval,errors)
  def raw_score(a,k,r):
   state['score_calls']+=1;raw.append(np.array(r[0],copy=True))
  self.scorer.score_geofrontiers=observed(self.scorer.score_geofrontiers,raw_score,errors)
  def scored(a,k,r):
   graph,geo,data,position=a[:4];out=r[0];state['frame']+=1
   visible=sum(len(x.get('frontier_nodes',[])) for x in geo.values())
   state['visible_scoring_frames']+=int(visible>0)
   final_scores=[np.asarray(d['scores']) for d in data if 'scores' in d]
   changed=(len(raw)==len(final_scores) and any(not np.allclose(x,y) for x,y in zip(raw,final_scores)))
   state['memory_modified_scores']+=int(changed);raw.clear()
   if time.monotonic()-last[0]<1:return
   last[0]=time.monotonic();idx=graph.trav_classes.index(self.traversability_class)
   front=[n for n in graph.nodes if n.trav_properties[idx].is_frontier]
   ranges=[float(np.linalg.norm(np.asarray([n.pose.position.x,n.pose.position.y])-np.asarray(position).ravel()[:2])) for n in front]
   defaults=0;named=[];memory_utility_deltas=[]
   for n in out.nodes:
    props={p.key:list(p.value) for p in n.properties}
    if 'frontier_scores' not in props:continue
    defaults+=int(props.get('is_default_scored')==[1.])
    best=max((h.predicted_utility for h in n.heading_candidates if h.traversable),default=None)
    named.append(dict(id=self.uuid_to_str(n.uuid),xy=[n.pose.position.x,n.pose.position.y],default=props.get('is_default_scored'),max_score=max(props['frontier_scores']),utility=best))
    assess=next((x for x in n.robot_traversability if x.robot_profile_id=='default'),None)
    conf=float(np.clip(assess.confidence,0,1)) if assess else 0.
    for h in n.heading_candidates:
     base=h.frontier_score*(1.-min(.95,max(0.,self.heading_low_confidence_weight))*(1.-conf))-self.heading_turn_cost_weight*h.motion_cost-self.heading_dynamic_risk_weight*h.dynamic_risk
     if h.frontier_score<=1e-4:base=0.
     memory_utility_deltas.append(float(h.predicted_utility-np.clip(base,0,1)))
   emit('scores',stamp_ns=graph.header.stamp.sec*10**9+graph.header.stamp.nanosec,query=self.text_queries[0],
    graph_nodes=len(graph.nodes),frontiers=len(front),nearest_frontier_m=min(ranges) if ranges else None,
    within_config_range=sum(x<=self.geofrontier_to_image.frontiers_range for x in ranges),visible_frontiers=visible,
    scored_nodes=len(named),default_nodes=defaults,nondefault_nodes=len(named)-defaults,
    candidates=named,stores=snapshot(),state=state.copy(),
    max_memory_utility_delta=max(map(abs,memory_utility_deltas),default=0.),errors=list(errors))
  self.update_navgraph_with_scores=observed(self.update_navgraph_with_scores,scored,errors)
 cls.__init__=init
 feedback=cls.traversal_feedback_callback
 def on_feedback(self,msg):
  before=self.visual_memory._db.execute('SELECT count(*) FROM experiences').fetchone()[0] if self.visual_memory else 0
  pending=len(self.pending_memory_samples);result=feedback(self,msg)
  log,state,errors=self._audit;state['feedback_received']+=1
  after=self.visual_memory._db.execute('SELECT count(*) FROM experiences').fetchone()[0] if self.visual_memory else 0
  state['feedback_stored']+=max(0,after-before)
  log.write(json.dumps(dict(wall=time.time(),kind='feedback',pending_before=pending,pending_after=len(self.pending_memory_samples),rows_before=before,rows_after=after,duration=float(msg.duration),from_node=self.uuid_to_str(msg.from_node)))+'\n');log.flush()
  return result
 cls.traversal_feedback_callback=on_feedback
 return nodes

if __name__=='__main__':
 import visual_navigation.wildos.nav as module
 nodes=install(module,os.environ['WILDOS_AUDIT_FILE'])
 try:module.main()
 finally:
  for n in nodes:
   try:n._audit_snapshot()
   except Exception as e:print('AUDIT FINAL ERROR',repr(e),flush=True)
   n._audit[0].close()
