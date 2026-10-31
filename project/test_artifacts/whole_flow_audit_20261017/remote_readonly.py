from pathlib import Path
import hashlib, json, datetime

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

report = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              read_only=True, simulation_started=False)
for label, base, manifest_path, source in (
    ('last_field', Path('/home/ly/full_vehicle_acceptance_20261017'),
     'official_wildos_beamng_bundle.manifest.json', 'candidate'),
    ('daily', Path('/home/ly/wildos-console'), 'source_identity.json', 'bundle')):
    manifest = json.loads((base/manifest_path).read_text())
    differences = [name for name, want in manifest['members'].items()
                   if not (base/source/name).is_file() or digest(base/source/name) != want]
    report[label] = dict(bundle=manifest['sha256'], members=len(manifest['members']), differences=differences)
base = Path('/home/ly/motion_intent_handoff_repair_20261017/isolation')
manifest = json.loads((base/'component_manifest.json').read_text())
report['isolated_component'] = dict(members=len(manifest), differences=[name for name, item in manifest.items()
    if not (base/name).is_file() or digest(base/name) != item['sha256']])
report['relevant_processes'] = []
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        command=(process/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        comm=(process/'comm').read_text().strip()
        if any(token in command for token in ('BeamNG.tech', 'wildos_graph_path_bridge',
                'component_container', 'controller_server', 'planner_server', 'native_vehicle_control',
                'sensor_console.py', 'rviz2', 'record /spot1')):
            report['relevant_processes'].append(dict(pid=int(process.name), executable=comm))
    except (OSError, PermissionError):
        pass
print(json.dumps(report, indent=2))
