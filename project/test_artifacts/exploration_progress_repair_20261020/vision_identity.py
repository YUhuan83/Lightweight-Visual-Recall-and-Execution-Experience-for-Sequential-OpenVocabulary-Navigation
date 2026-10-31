"""Record actual model source and active same-process stores; preserve inference behavior."""
from pathlib import Path
import hashlib
import json
import os
import visual_navigation.wildos.nav as module
from model_identity_helpers import database_snapshot, all_inprocess

original=module.WildOS_Nav.__init__
def initialize(self,*args,**kwargs):
    original(self,*args,**kwargs)
    path=Path(module.__file__).resolve()
    report=dict(module=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        stores=dict(ExecutionExperienceStore=self.visual_memory is not None,
                    GraphObservationStore=self.visual_memory_map is not None,
                    PassiveLandmarkIndex=self.semantic_landmarks is not None),
        confirmed_views_at_start=len(self.object_goal_memory.views),databases={})
    report['databases']=database_snapshot(dict(ExecutionExperienceStore=self.visual_memory,
                                              GraphObservationStore=self.visual_memory_map))
    Path(os.environ['WILDOS_FIELD_MODEL_IDENTITY']).write_text(json.dumps(report,indent=2)+'\n')
    assert all(report['stores'].values()),'Expected production memory stores active'
    assert report['confirmed_views_at_start']==0,'No preloaded target instructions'
    assert all_inprocess(report['databases']),'Expected in-process databases'
module.WildOS_Nav.__init__=initialize
module.main()
