"""Reuse the verified exact source/receipt accounting on this closed trial."""
from pathlib import Path
import importlib.util,json
out=Path(__file__).parent
session=out/'closed_evidence/field_results/session_20261018_083258'
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
driver=out/'closed_evidence/field_yaw_left90_search_20261018_083254'
assert (driver/'result.json').is_file()
spec=importlib.util.spec_from_file_location('exact_closed_accounting',
    out.parent/'system_contract_audit_20261017/analyze_closed.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.OUT=out;module.CLOSED=out;module.SESSION=session
module.main()
