"""Prepare an isolated field harness from the previously exercised console driver."""
from pathlib import Path
import ast
import hashlib
import json

root = Path(__file__).resolve().parents[2]
old = root / 'test_artifacts/frontier_cost_repair_20261016/server/frontier_cost_repair_20261016/field'
out = Path(__file__).parent / 'field'
out.mkdir(exist_ok=True)
remote = '/home/ly/exploration_scope_repair_20261016'
previous = '/home/ly/frontier_cost_repair_20261016/field'
digest = '23675e2881fc797e2e65699cd045cfb7942cf7ae7bdf43785b6f0beb6dd2b769'

config = json.loads((old / 'sensor_full_console.json').read_text(encoding='utf8'))
config['setups'].append(remote + '/install/setup.bash')
config['output_root'] = remote + '/field_results'
config['library_sha256']['libgraphnav_planner_component.so'] = 'ff8840117a16f836e3779f44087633f8f65bc7f2c3a1a027b4b73b488a95ff51'
(out / 'sensor_full_console.json').write_text(json.dumps(config, indent=2) + '\n', encoding='utf8')
original_yaml = (root / 'official_adapter/config/beamng_wildos.yaml').read_text(encoding='utf8')
flags = ['visual_memory_enabled', 'visual_memory_map_enabled', 'semantic_landmark_memory_enabled']
for flag in flags:
    assert not any(line.startswith(flag + ':') for line in original_yaml.splitlines())
baseline_yaml = original_yaml.rstrip() + '\n\n# Controlled baseline: only newly added memory priors disabled.\n'
baseline_yaml += ''.join(flag + ': false\n' for flag in flags)
(out / 'baseline_no_memory.yaml').write_text(baseline_yaml, encoding='utf8')

for name in ('sensor_console.py', 'sensor_full_console.sh', 'drive_sensor_full.py', 'audit_runtime.py', 'progress.py', 'analyze_closed.py'):
    value = (old / name).read_text(encoding='utf8')
    value = value.replace(previous, remote + '/field')
    value = value.replace('/home/ly/wildos-console/bundle', remote + '/candidate')
    value = value.replace("Path('/home/ly/wildos-console')", "Path('" + remote + "')")
    value = value.replace('field_frontier_cost_', 'field_scope_baseline_')
    value = value.replace("root/'audit_results'", "root/'field_results'")
    value = value.replace("r/'audit_results'", "r/'field_results'")
    value = value.replace("console=root/'bundle/tools/wildos_console.py'", "console=root/'candidate/tools/wildos_console.py'")
    value = value.replace('e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb', digest)
    if name == 'sensor_full_console.sh':
        value = value.replace('export DISPLAY=', 'source ' + remote + '/install/setup.bash\nexport DISPLAY=')
    if name == 'sensor_console.py':
        value = value.replace(" if role=='vision':\n", " if role=='vision':\n  argv=list(argv)\n  argv[argv.index('--config')+1]='" + remote + "/field/baseline_no_memory.yaml'\n")
        value = value.replace('/home/ly/search_mechanism_audit_20261015/audit_vision.py', remote + '/field/audit_vision.py')
        value = value.replace('/home/ly/search_mechanism_audit_20261015/audit_preflight.py', remote + '/field/audit_preflight.py')
    if name.endswith('.py'):
        ast.parse(value)
    (out / name).write_text(value, encoding='utf8', newline='\n')

audit = (root / 'test_artifacts/search_mechanism_audit_20261015/audit_vision.py').read_text(encoding='utf8')
audit = audit.replace("  emit('initial',stores=snapshot())", "  assert self.visual_memory is None and self.visual_memory_map is None and self.semantic_landmarks is None\n  assert not self.object_goal_memory.views\n  emit('initial',stores=snapshot(),memory_priors_enabled=False,confirmed_viewpoints_initial=0)")
ast.parse(audit)
(out / 'audit_vision.py').write_text(audit, encoding='utf8', newline='\n')
preflight = """import sys,hashlib,json
from pathlib import Path
sys.path.insert(0,'REMOTE/candidate/tools')
import server_test_preflight as m
p=Path('REMOTE/field/audit_vision.py')
# Exact diagnostic role identity; original process, DDS and input gates retained.
m.ROLES['vision']=(str(p),)
assert p.is_file()
print(json.dumps(dict(diagnostic_vision=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())),flush=True)
raise SystemExit(m.main())
""".replace('REMOTE', remote)
ast.parse(preflight)
(out / 'audit_preflight.py').write_text(preflight, encoding='utf8', newline='\n')

report = dict(candidate_sha256=digest, memory_override_only=flags,
              production_bundle_replaced=False,
              files={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir()
                     if p.is_file() and (p.suffix in ('.py', '.sh', '.yaml') or p.name == 'sensor_full_console.json')})
(out / 'harness_manifest.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
print(json.dumps(report, indent=2))
