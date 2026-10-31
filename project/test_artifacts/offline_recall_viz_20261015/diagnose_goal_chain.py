from pathlib import Path
import json,sys
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
from offline_goal_chain_replay import Session
session=Session('test_station_house_station_reuses_same_memory_and_can_finish_at_a_new_pose')
with patch('time.monotonic',side_effect=lambda:getattr(session,'wall',1000.)):
    session.setup_chain('ackermann')
    try:
        result=session.run_chain(False)
    except Exception as exc:
        result={'error':repr(exc),'trace':session.trace,'plans':session.plans,
            'bridge':{k:getattr(session.bridge,k,None) for k in (
                'object_found','expected','memory_goal','memory_active','last_status',
                'active_object_confirmation','confirmed_approach','confirmed_approach_goal',
                'following','pending','motion_queue')},
            'goal_state':session.standoff.__dict__ if hasattr(session,'standoff') else {}}
    finally:session.tearDown()
out=Path(__file__).with_name('goal_chain_diagnosis.json')
out.write_text(json.dumps(result,indent=2,default=repr))
print(json.dumps({k:v for k,v in result.items() if k not in ('trace','plans','goal_state')},default=repr))
