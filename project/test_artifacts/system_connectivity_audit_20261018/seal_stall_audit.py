"""Verify and seal diagnosis evidence. Passing is not repair acceptance."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent
repo = root.parents[1]
audit = json.loads((root/'current_audit.json').read_text())
assert audit['production_unchanged'] and audit['current_matches_frozen']
assert audit['members'] == 404
stall = json.loads((root/'stall_and_shifts.json').read_text())['stalls'][0]
assert stall['samples'] == stall['matched_source_controls'] == 100
assert stall['sent_brake']['maximum'] == 0
assert stall['actual_gear_counts'] == {'D': 100}
assert stall['watchdog_samples'] == 0 and stall['native_actuator_healthy'] and stall['native_telemetry_healthy']
assert stall['contacts']['snapshots'] == 4 and stall['contacts']['wheels'] == 16
assert stall['contacts']['propulsion_torque_abs']['minimum'] > 200
assert stall['rear_axle_cross_track_m']['maximum'] < .08
assert stall['visible_handoff']['approach_age_at_stall_sec'] > 10
assert 0 < stall['visible_handoff']['next_release_after_stall_sec'] < .25
gaps = json.loads((root/'visible_recovery_interaction.json').read_text())
assert gaps['expected_gap_reproduced'] and gaps['tests'] == 3 and not gaps['repair_verified']
height = json.loads((root/'height_reference_gap.json').read_text())
assert height['gap_reproduced'] and height['root_in_164222_proven'] is False
assert [r['graph_local_occupancy_value'] for r in height['cases']] == [100, 0, 100]
terrain = json.loads((root/'stall_connectivity_164222/terrain_review.json').read_text())
assert all(r['pose_source_skew_sec'] == 0 for r in terrain['snapshots'])
for line in (root/'stall_maps_server_sha256.txt').read_text(encoding='utf8').splitlines():
    want, filename = line.split(maxsplit=1)
    payload = root/'stall_connectivity_164222'/Path(filename.strip()).name
    assert hashlib.sha256(payload.read_bytes()).hexdigest() == want, payload.name
maps = json.loads((root/'stall_connectivity_164222/report.json').read_text())
assert maps['private_costmap_identity_proven'] is False and maps['raw_lidar_recorded'] is False
for snapshot in maps['snapshots']:
    for name in ('/local_costmap/costmap', '/global_costmap/costmap'):
        grid = snapshot['grids'][name]
        assert grid['current_footprint_blockers'] == [] and grid['current_forward_prefix_issue'] is None
assert json.loads((root/'readonly_audit_end_inventory.json').read_text(encoding='utf-8-sig')) == []
names = ['review_stall_and_shifts.py', 'stall_and_shifts.json', 'stall_and_shifts_run.log',
         'review_stall_maps.py', 'run_stall_maps.sh', 'review_stall_terrain.py',
         'reproduce_visible_recovery_interaction.py', 'visible_recovery_interaction.json',
         'visible_recovery_interaction.log', 'visible_recovery_initial.json', 'visible_recovery_initial.log',
         'visible_recovery_fixture_note.txt', 'reproduce_height_reference_gap.py',
         'height_reference_gap.json', 'height_reference_gap_run.log', 'plot_stall_maps.py',
         'stall_public_maps.png', 'stall_public_maps_initial_palette.png',
         'beamng_gear_api_sha256.txt', 'stall_maps_server_sha256.txt',
         'readonly_audit_end_inventory.json', 'current_audit.json', 'source_sha256.json',
         'current_after_stall_audit.log', 'seal_stall_audit.py', 'seal_stall_initial_failure.txt']
payloads = {name: root/name for name in names}
payloads.update({'stall_connectivity_164222/'+p.name: p for p in (root/'stall_connectivity_164222').glob('*.json')})
payloads['docs/PHYSICAL_STALL_HANDOFF_AUDIT_20261018.md'] = repo/'docs/PHYSICAL_STALL_HANDOFF_AUDIT_20261018.md'
manifest = {name: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for name, p in payloads.items()}
archive = root/'physical_stall_handoff_evidence.zip'
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as zf:
    for name, path in payloads.items(): zf.write(path, name)
    zf.writestr('manifest.json', json.dumps(manifest, indent=2)+'\n')
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for name, want in manifest.items():
        value = zf.read(name)
        assert len(value) == want['bytes'] and hashlib.sha256(value).hexdigest() == want['sha256']
result = dict(archive=str(archive), payloads=len(manifest), bytes=archive.stat().st_size,
              sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
              evidence_consistency_verified=True, server_export_hashes_match=True,
              production_unchanged=True, gap_reproduced=True, repair_verified=False,
              field_obstruction_identified=False, complete_navigation_passed=False)
(root/'physical_stall_handoff_identity.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
