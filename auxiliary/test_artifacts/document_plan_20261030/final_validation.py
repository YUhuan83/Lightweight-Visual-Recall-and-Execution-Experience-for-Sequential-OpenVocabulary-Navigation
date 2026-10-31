"""Complete the existing isolated suite with the correct visual test import path."""
import ast,json,os,subprocess,sys
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
base=json.loads((root/'isolated_ros_report.json').read_text())
missing=[r for r in base if not r['passed']]
assert all(r['module']=='test_scanline_depth' and r['tests']==0 for r in missing),missing
source=Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/real_ros_review.py').read_text()
t=ast.parse(source)
assignment=next(n for n in t.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='modules' for x in n.targets))
assignment.value=ast.Tuple(elts=[ast.Constant('test_scanline_depth')],ctx=ast.Load())
out=root/'scanline_correct_import';out.mkdir(exist_ok=True)
env=os.environ.copy();env['PYTHONPATH']=str(root/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test')+':'+env['PYTHONPATH']
with (out/'output.log').open('w') as log:
    p=subprocess.run([sys.executable,'-c',ast.unparse(ast.fix_missing_locations(t))],cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT)
report=json.loads((out/'real_ros_report.json').read_text()) if (out/'real_ros_report.json').exists() else []
passed=p.returncode==0 and bool(report) and all(x['passed'] for x in report)
fixed=dict(module='test_scanline_depth',returncode=p.returncode,tests=sum(x['tests'] for x in report),passed=passed)
final=[fixed if r['module']=='test_scanline_depth' else r for r in base]
summary=dict(passed=all(r['passed'] for r in final),tests=sum(r['tests'] for r in final),modules=final,
    scope='Existing final-source isolated suite plus the previously missing module with its correct import path; original import failure retained')
(root/'final_effective_regression.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='modules'}));print(json.dumps(fixed))
raise SystemExit(not summary['passed'])
