"""Seal the failed full trial, source identity and stopped runtime logs."""
import hashlib,json,time,zipfile
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
assert json.loads((BASE/'round_shutdown_attempt2.json').read_text())['passed']
assert not json.loads((BASE/'green_house_trial1/full_test.json').read_text())['passed']
archive=BASE/'full_dds_trial1_evidence.zip';assert not archive.exists()
files=set()
for directory in ('green_house_trial1','green_house_session1','physical_cusp_component12','physical_cusp_component12_observer','physical_component_session1'):
    files.update(p for p in (BASE/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
for name in ('installation_identity.json','changed_sources.json','deployment.log','round_shutdown.json',
             'round_shutdown_attempt2.json','brake_recovery1.json','green_house_trial1_driver.json',
             'dds_env.sh','prepare_dds_search.py','restart_visual_from_frozen_config.py','capture_dds_owners.py',
             'start_dds_full_trial.py','extract_dds_search_interfaces.py','brake_current_trial.py',
             'close_dds_trial.py','close_dds_trial_retry.py','analyze_dds_trial.py'):
    files.add(BASE/name)
manifest={'scope':'Failed full green residential house trial and precursor component; not arrival acceptance',
    'sealed_at_wall_ns':time.time_ns(),'members':{}}
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):
        assert p.resolve().is_relative_to(BASE.resolve()) and not p.is_symlink()
        name=p.relative_to(BASE).as_posix();data=p.read_bytes()
        manifest['members'][name]={'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)};z.writestr(name,data)
    z.writestr('evidence_manifest.json',json.dumps(manifest,indent=2))
assert zipfile.ZipFile(archive).testzip() is None
report={'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'bytes':archive.stat().st_size,
    'members':len(manifest['members']),'uncompressed_bytes':sum(v['bytes'] for v in manifest['members'].values())}
archive.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
