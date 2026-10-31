"""Validate and seal map replays/source counterexamples without claiming graph execution."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

root=Path(__file__).resolve().parent
r=json.loads((root/'map_contract_review.json').read_text())
checks={}
checks['twenty_two_recorded_maps_matched']=len(r['snapshots'])==22 and not r['unmatched']
checks['actual_occupancy_and_terrain_replays_exact']=r['all_recorded_occupancy_replayed_exactly'] and r['all_recorded_terrain_replayed_exactly']
checks['contact_region_remained_free']=all(v['body_occupancy']==0 and v['terrain_contact']['traversability']==0 and v['terrain_contact']['unknown']==0 for v in r['snapshots'])
observed=[v for v in r['snapshots'] if v['terrain_contact']['elevation_valid']>.5]
checks['observed_height_then_missing_height_separated']=len(observed)==6 and len(r['snapshots'])-len(observed)==16
checks['measured_grid_patch_has_small_residual_step']=all(v['patch_metrics']['observed'] and .07<v['patch_metrics']['step']<.09 and v['patch_metrics']['within_source_default_terrain_limits'] is True for v in observed)
c=r['cpp_controls']['abrupt_17cm_half_plane']
checks['known_seventeen_cm_step_underestimated_and_admitted_by_source_defaults']=c['known_input_step_m']==.17 and abs(c['step']-.085)<1e-10 and c['contract_gap_present'] and c['within_source_default_terrain_limits'] is True
checks['smooth_ramp_and_obstacle_controls_distinguish_behavior']=r['cpp_controls']['smooth_17percent_ramp']['step']<1e-10 and r['cpp_controls']['isolated_30cm_step']['within_source_default_terrain_limits'] is False and r['cpp_controls']['insufficient_samples']['observed'] is False
checks['analysis_not_misrepresented_as_runtime_graph']=r['actual_graph_node_or_edge_replayed'] is False and r['source_default_thresholds_not_live_parameter_read'] is True and not r['production_modified'] and not r['complete_navigation_passed']
checks['mapping_analysis_source_matches_frozen']=all(v['equals_frozen'] for v in r['source_identity'].values())
assert all(checks.values()),checks
(root/'consistency.json').write_text(json.dumps(dict(checks=checks,passed=True,scope=__doc__,complete_navigation_passed=False),indent=2)+'\n')
frozen=Path('/home/ly/smoothing_boundary_repair_20261018')
manifest=json.loads((frozen/'official_wildos_beamng_bundle.manifest.json').read_text())
snapshot=root/'source_snapshot';snapshot.mkdir(exist_ok=False)
sources={}
members=['wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/include/graphnav_mapper/terrain_metrics.hpp',
 'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src/mapper_node.cpp',
 'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/src/mapper.cpp',
 'wildos_port/repro_check/src/nebula2-wildos/graphnav_msgs/msg/RobotMotionConstraints.msg',
 'official_adapter/beamng_wildos_adapter/mapping.py','official_adapter/beamng_wildos_adapter/terrain.py',
 'official_adapter/beamng_wildos_adapter/node.py','official_adapter/launch/official_graph_stack.launch.py']
for member in members:
    source=frozen/'candidate'/member;data=source.read_bytes();sha=hashlib.sha256(data).hexdigest()
    assert sha==manifest['members'][member],member
    target=snapshot/member;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    sources[member]=sha
provenance=dict(frozen_bundle_sha256=manifest['sha256'],sources=sources,
    compiler=subprocess.run(['g++','--version'],capture_output=True,text=True,check=True).stdout,
    probe_binary_sha256=hashlib.sha256((root/'terrain_metrics_probe').read_bytes()).hexdigest(),
    cpp_scope='Compiled frozen header in isolated diagnostic, not the running Mapper component',
    source_database_external=dict(path='/home/ly/contact_frame_audit_20261019/frame_20261019_084254/interfaces/interfaces_0.db3',sha256=r['database_sha256'],local_downloaded=False))
(root/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
payloads={str(p.relative_to(root)):p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.zip' and p.name!='evidence_identity.json'}
inventory={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in payloads.items()}
archive=root/'terrain_contract_evidence.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as zf:
    for name,p in payloads.items():zf.write(p,name)
    zf.writestr('manifest.json',json.dumps(inventory,indent=2)+'\n')
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for name,want in inventory.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256']
identity=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payloads=len(inventory),complete_navigation_passed=False)
(root/'evidence_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(dict(consistency=checks,**identity),indent=2))
