from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
s=(base/'audit_body_arrival_field.py').read_text()
s=s.replace("root=Path('/home/ly/execution_continuity_review_20261022/v26_body_arrival')\ntrial=root/'full/trial_20261025_145355'",
    "import sys\nroot=Path(sys.argv[1])\ntrial=Path(json.loads((root/'field_result.json').read_text())['trial'])")
(base/'audit_arrival_round.py').write_text(s)
