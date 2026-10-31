from pathlib import Path
import json
import tarfile

root=Path('/home/ly/execution_continuity_review_20261022/v9')
with tarfile.open(root/'startup_failure_evidence.tar.gz','w:gz') as archive:
    for name in ('trial_20261023_091127','trial_20261023_091817','trial_20261023_092324'):
        trial=root/'full'/name
        assert json.loads((trial/'result.json').read_text())['closed']
        for path in trial.rglob('*'):
            if path.is_file() and path.suffix in ('.json','.log'):
                archive.add(path,arcname=str(path.relative_to(root/'full')))
print(root/'startup_failure_evidence.tar.gz')
