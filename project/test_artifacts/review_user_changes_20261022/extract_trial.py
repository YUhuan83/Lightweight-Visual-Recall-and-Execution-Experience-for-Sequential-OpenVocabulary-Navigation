import tarfile
from pathlib import Path
from shutil import copyfileobj
archive=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\review_user_changes_20261022\trial_20261022_173435_compact.tar.gz')
out=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\review_user_changes_20261022\v3_trial_20261022_173435')
out.mkdir(exist_ok=True)
with tarfile.open(archive,'r:gz') as t:
 for m in t.getmembers():
  rel=Path(m.name)
  assert not rel.is_absolute() and '..' not in rel.parts and m.isfile()
  target=out/rel
  assert target.resolve().is_relative_to(out.resolve())
  target.parent.mkdir(parents=True,exist_ok=True)
  with t.extractfile(m) as src, target.open('wb') as dst: copyfileobj(src,dst)
print(out)
