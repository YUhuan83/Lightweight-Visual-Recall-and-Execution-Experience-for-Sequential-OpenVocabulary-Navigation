"""Keep all test temporary files inside the authorized workspace."""
from pathlib import Path
import os,runpy,tempfile,shutil
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
tmp=(out/'tmp').resolve()
assert tmp.is_relative_to(root.resolve())
tmp.mkdir(exist_ok=True)
tempfile.tempdir=str(tmp)
os.environ['TEMP']=os.environ['TMP']=str(tmp)
reports=root/'test_artifacts/offline_navigation'
first=out/'first_run';first.mkdir(exist_ok=True)
for n in ('report.json','report.md','test_wildos_console.log','test_package_official_bundle.log',
          'test_dds_profile.log','test_visual_memory.log','test_visual_memory_map.log'):
    if (reports/n).exists() and not (first/n).exists():shutil.copyfile(reports/n,first/n)
runpy.run_path(str(root/'tools/offline_navigation_check.py'),run_name='__main__')
