"""Regression contracts for the recorded no-score/no-experience failure."""
import ast,__future__,math,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from collections import deque
from unittest.mock import Mock,patch
import numpy as np
import yaml
from visual_navigation.wildos.frontier_score_state import score_state,fresh_visual
from visual_navigation.wildos.execution_scene_samples import append_scene_sample
from visual_navigation.wildos.feedback_matching import select_feedback_sample
from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore
from visual_navigation.wildos.frontier_history import RecentFrontierHistory

ROOT=Path(__file__).resolve().parents[6]
SRC=Path(__file__).parents[1]/'visual_navigation/wildos/nav.py'

def methods(path,cls,names,ns):
 tree=ast.parse(path.read_text(encoding='utf8'));c=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name==cls)
 c.bases=[];c.body=[x for x in c.body if isinstance(x,ast.FunctionDef) and x.name in names]
 exec(compile(ast.Module(body=[c],type_ignores=[]),str(path),'exec',flags=__future__.annotations.compiler_flag),ns)
 return ns[cls]

def node(x,uid,frontier=True):
 return S(uuid=S(id=[uid]),pose=S(position=S(x=x,y=0.,z=0.)),properties=[],heading_candidates=[],
          trav_properties=[S(is_frontier=frontier,frontier_points=[S(x=x+1.,y=0.,z=0.)])])
def stamp(v):return S(sec=int(v),nanosec=round((v-int(v))*1e9))
def odom(t=10.,x=0.):return S(header=S(frame_id='odom',stamp=stamp(t)),pose=S(pose=S(position=S(x=x,y=0.,z=0.))))
def graph(t=10.):return S(header=S(frame_id='odom',stamp=stamp(t)),trav_classes=['default'],current_node_idx=0,nodes=[node(5.,1),node(0.,2,False)])

class Contracts(unittest.TestCase):
 def test_shared_range_enables_recorded_distance_and_keeps_fov_and_bound(self):
  cfg=yaml.safe_load((ROOT/'official_adapter/config/beamng_wildos.yaml').read_text(encoding='utf8'))
  self.assertEqual(cfg['frontiers_range'],cfg['observation_range_m'])
  cls=methods(Path(__file__).parents[1]/'visual_navigation/geofrontier_nav/geofrontier_to_image.py','GeoFrontierToImage',
     {'__init__','extract_geofrontiers','is_within_range','get_frontier_heading','get_visible_frontiers'},{'np':np})
  g=graph();g.nodes=[node(14.,1),node(-14.,2),node(41.,3)]
  cam=dict(K=np.array([[500.,0,480],[0,500,320],[0,0,1]]),R_wc=np.array([[0,0,1],[-1,0,0],[0,-1,0]]),t_wc=np.array([[0.],[0.],[1.6]]),width=960,height=640)
  old=cls({},9.,'default',False,0.,True);new=cls({},cfg['frontiers_range'],'default',False,0.,True)
  self.assertEqual(old.extract_geofrontiers(odom(),g,[cam]),{0:{}})
  self.assertEqual([n.uuid.id for n in new.extract_geofrontiers(odom(),g,[cam])[0]['frontier_nodes']],[[1]])
  for v in (0.,float('nan'),float('inf')):
   with self.assertRaises(ValueError):cls({},v,'default',False)

 def test_no_frontier_still_supplies_experience_on_same_executor_anchor(self):
  q=deque(maxlen=512);g=graph()
  self.assertTrue(append_scene_sample(q,[1.,0.],g,odom(),20.,60.,16,lambda u:str(u.id)))
  self.assertEqual(q[0]['node_uuid'],'[2]') # current_node_idx points elsewhere
  self.assertIsNone(q[0]['heading_scores'])
  i=select_feedback_sample(list(q),10.1,'[2]',60.);self.assertEqual(i,0)
  memory=ExecutionExperienceStore(':memory:','test')
  try:
   self.assertTrue(memory.record_feedback(q[i]['embedding'],None,math.pi/2,0.,1,heading_count=16))
   prior,_=memory.retrieve([1.,0.],16);self.assertLess(prior[4],1.)
   np.testing.assert_allclose(np.delete(prior,4),np.ones(15))
  finally:memory.close()

 def test_sample_guards_duplicates_expiry_and_bounded_delayed_feedback(self):
  q=deque(maxlen=512)
  for i in range(220):
   t=10.+i*.26;append_scene_sample(q,[1.,0.],graph(t),odom(t),t,60.,16,lambda u:str(u.id))
  self.assertIsNotNone(select_feedback_sample(list(q),10.1,'[2]',60.))
  self.assertFalse(append_scene_sample(q,[1.,0.],graph(11.),odom(11.),11.,60.,16,lambda u:str(u.id)))
  g=graph(68.);g.header.frame_id='wrong';self.assertFalse(append_scene_sample(q,[1.,0.],g,odom(68.),68.,60.,16,lambda u:str(u.id)))
  append_scene_sample(q,[1.,0.],graph(80.),odom(80.),80.,60.,16,lambda u:str(u.id))
  self.assertIsNone(select_feedback_sample(list(q),10.1,'[2]',60.))
  self.assertLessEqual(len(q),512)

 def test_default_cache_never_counts_as_visual_and_first_visual_resets_stats(self):
  a=score_state(None,[.3,.3],default=True,now=1.,semantic=0.,beta=1.)
  a=score_state(a,[.3,.3],default=True,now=2.,semantic=0.,beta=1.)
  self.assertEqual(a['count'],0);self.assertFalse(fresh_visual(a,2.,5.))
  b=score_state(a,[.9,.1],default=False,now=3.,semantic=.8,beta=1.)
  self.assertEqual(b['count'],1);np.testing.assert_allclose(b['safe'],[.9,.1])
  self.assertTrue(fresh_visual(b,7.,5.));self.assertFalse(fresh_visual(b,9.,5.));self.assertFalse(fresh_visual(b,2.,5.))
  self.assertIsNone(score_state(b,[float('nan'),1],default=False,now=4.,semantic=1.,beta=1.))

 def test_actual_cache_publication_default_visual_expiry_and_no_duplicate_properties(self):
  clock=iter(10.+i*.001 for i in range(1000))
  ns=dict(np=np,wall_time=lambda:next(clock),score_state=score_state,fresh_visual=fresh_visual,KeyValue=S)
  cls=methods(SRC,'WildOS_Nav',{'update_navgraph_with_scores','_update_frontier_score','remove_old_frontiers'},ns)
  n=cls();n.frontier_uuid_to_scores={};n.frontier_score_stats={};n.frontier_uuid_to_scoring_distance={};n.frontier_last_scored_at={}
  n.uuid_to_str=lambda u:str(u.id);n.traversability_class='default';n.num_cameras=1;n.frontier_score_max_age=5.;n.frontier_uncertainty_beta=0.;n.frontier_rescore_interval=1.
  n.recent_frontiers=RecentFrontierHistory();n.min_frontier_separation=.75;n.std_for_default_scores=None;n.default_max_score=.3;n.std_for_frontier_heading=None
  n.scorer=Mock();n.scorer.get_default_scores.return_value=np.array([.3,.3]);n._append_heading_candidates=Mock()
  g=graph();g.nodes=g.nodes[:1]
  for _ in range(3):n.update_navgraph_with_scores(g,{0:{}},[{}],np.zeros((1,3)))
  p={p.key:p.value for p in g.nodes[0].properties};self.assertEqual(p['is_default_scored'],[1.]);self.assertEqual(p['frontier_score_observations'],[0.]);self.assertEqual(len(g.nodes[0].properties),6)
  geo={0:dict(frontier_nodes=g.nodes,frontier_headings=[np.array([1.,0.,0.])],frontier_pixel_coords=[(5,5)])}
  data=[dict(scores=np.array([[.9,.1]]),traversability=np.ones((10,10)),image=np.zeros((10,10,3)))]
  n.update_navgraph_with_scores(g,geo,data,np.zeros((1,3)))
  self.assertEqual({p.key:p.value for p in g.nodes[0].properties}['is_default_scored'],[0.])
  ns['wall_time']=lambda:16.;n.update_navgraph_with_scores(g,{0:{}},[{}],np.zeros((1,3)))
  self.assertEqual({p.key:p.value for p in g.nodes[0].properties}['is_default_scored'],[1.])
  n.frontier_uuid_to_scores.clear();n.frontier_score_stats.clear();n.update_navgraph_with_scores(g,{0:{}},[{}],np.zeros((1,3)))
  self.assertEqual(len(g.nodes[0].properties),6)

class FeedbackCallbackContracts(unittest.TestCase):
 def test_real_callback_matches_scene_without_frontier_and_keeps_capacity(self):
  constants=S(OUTCOME_UNKNOWN=0,OUTCOME_SUCCEEDED=1,OUTCOME_BLOCKED_DYNAMIC=3,OUTCOME_ABORTED=4,OUTCOME_BLOCKED_GEOMETRY=5,OUTCOME_COLLISION=6)
  cls=methods(SRC,'WildOS_Nav',{'traversal_feedback_callback'},dict(np=np,wall_time=lambda:20.,deque=deque,TraversalFeedback=constants,select_feedback_sample=select_feedback_sample))
  owner=cls();owner.visual_memory=ExecutionExperienceStore(':memory:','callback');owner.visual_memory_pending_ttl=60.;owner.num_angular_bins=16
  owner.pending_memory_samples=deque(maxlen=512);owner.uuid_to_str=lambda u:str(u.id);owner.get_logger=lambda:Mock();owner._record_memory_map_traversal=Mock()
  append_scene_sample(owner.pending_memory_samples,[1.,0.],graph(),odom(),19.,60.,16,owner.uuid_to_str)
  feedback=S(observations=[S(key='execution_source',value=[1.])],outcome=1,header=S(stamp=stamp(14.1)),duration=4.,from_node=S(id=[2]),commanded_distance=2.,observed_distance=2.,slip_ratio=0.,commanded_heading=0.)
  owner.traversal_feedback_callback(feedback)
  self.assertEqual(owner.visual_memory._db.execute('SELECT count(*) FROM experiences').fetchone()[0],1)
  self.assertEqual(owner.pending_memory_samples.maxlen,512)
  owner.traversal_feedback_callback(feedback) # duplicate cannot learn twice
  self.assertEqual(owner.visual_memory._db.execute('SELECT count(*) FROM experiences').fetchone()[0],1)
  owner.visual_memory.close()

if __name__=='__main__':unittest.main()
