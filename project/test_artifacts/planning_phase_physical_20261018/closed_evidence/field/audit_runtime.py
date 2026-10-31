from pathlib import Path
import hashlib,json,time
p=Path('/home/ly/planning_phase_physical_20261018/field');config=json.loads((p/'sensor_full_console.json').read_text());libraries={}
for root in Path('/proc').iterdir():
 if not root.name.isdigit():continue
 try:args=(root/'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
 if not args or Path(args[0]).name not in ('planner_node','planner_server','controller_server'):continue
 try:maps=(root/'maps').read_text().splitlines()
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
 for name,digest in config['library_sha256'].items():
  matches={line.split()[-1] for line in maps if line.endswith('/'+name)}
  for filename in matches:
   actual=hashlib.sha256(Path(filename).read_bytes()).hexdigest();assert actual==digest,(filename,actual,digest)
   libraries[name]=dict(pid=int(root.name),args=args,path=filename,sha256=actual)
assert set(libraries)==set(config['library_sha256']),libraries
report=dict(captured_wall=time.time(),mapped_libraries=libraries,expected_source='d9d2e27b035b46486cef3b3d74a381bc73b6bbececec8c39818ade6115d4a900')
(p/'loaded_runtime.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
