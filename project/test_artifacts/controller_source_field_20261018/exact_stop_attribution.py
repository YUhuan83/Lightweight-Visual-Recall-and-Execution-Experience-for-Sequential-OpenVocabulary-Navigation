"""Reuse closed exact source identity accounting; no ROS/DDS or active bag."""
from pathlib import Path
import importlib.util,json
out=Path(__file__).parent
closed=out/'closed_evidence'
driver=next(closed.glob('field_yaw_left90_search_*'))
assert (driver/'result.json').is_file()
session=next((closed/'field_results').glob('session_*'))
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
spec=importlib.util.spec_from_file_location('exact_closed_accounting',
    out.parent/'system_contract_audit_20261017/analyze_closed.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.OUT=out;module.CLOSED=closed;module.SESSION=session;module.main()
