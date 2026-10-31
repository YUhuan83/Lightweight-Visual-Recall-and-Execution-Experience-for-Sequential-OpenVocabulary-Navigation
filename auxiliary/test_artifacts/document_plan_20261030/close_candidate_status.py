import json
from pathlib import Path
parent=Path('/home/ly/execution_continuity_review_20261022')
for version,trial in (('v73_document_plan','trial_20261030_184621'),('v74_document_handoff','trial_20261030_190118')):
    root=parent/version;p=root/'candidate_scope.json';s=json.loads(p.read_text())
    r=json.loads((root/'full'/trial/'result.json').read_text());assert r['closed']
    s.update(field_acceptance=False,latest_trial=trial,latest_trial_closed=True,commit=None,
        acceptance_note='Full 120s search timed out; component verification is not complete navigation success.')
    p.write_text(json.dumps(s,indent=2)+'\n')
