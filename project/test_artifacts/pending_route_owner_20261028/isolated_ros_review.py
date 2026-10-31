"""Run the existing ROS suites in separate processes, retaining crash evidence."""
import ast,json,subprocess,sys
from pathlib import Path
source=Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/real_ros_review.py').read_text()
tree=ast.parse(source)
modules=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='modules' for t in n.targets))
root=Path.cwd()/'isolated_ros';root.mkdir(exist_ok=True)
summary=[]
for name in modules:
 t=ast.parse(source)
 assignment=next(n for n in t.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='modules' for x in n.targets))
 assignment.value=ast.Tuple(elts=[ast.Constant(name)],ctx=ast.Load())
 program=ast.unparse(ast.fix_missing_locations(t))
 out=root/name;out.mkdir(exist_ok=True)
 with (out/'output.log').open('w') as log:
  p=subprocess.run([sys.executable,'-X','faulthandler','-c',program],cwd=out,stdout=log,stderr=subprocess.STDOUT)
 report=json.loads((out/'real_ros_report.json').read_text()) if (out/'real_ros_report.json').exists() else []
 item=dict(module=name,returncode=p.returncode,tests=sum(x['tests'] for x in report),passed=p.returncode==0 and bool(report) and all(x['passed'] for x in report))
 summary.append(item);print(json.dumps(item),flush=True)
Path('isolated_ros_report.json').write_text(json.dumps(summary,indent=2))
raise SystemExit(not all(x['passed'] for x in summary))
