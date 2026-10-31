"""Refresh the existing deployment manifest plus current adapter source files."""
from pathlib import Path
import hashlib
import json
import os
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def write_bundle(root, bundle, names):
    """Publish only after archive bytes match the frozen source inventory."""
    root, bundle = root.resolve(), bundle.resolve()
    if not bundle.is_relative_to(root):
        raise ValueError('Bundle output must stay inside workspace')
    sources = {}
    for name in sorted(names):
        path = (root / name).resolve()
        if (Path(name).as_posix() != name or '..' in Path(name).parts
                or Path(name).is_absolute() or not path.is_relative_to(root)
                or not path.is_file()):
            raise ValueError(f'Invalid bundle source: {name}')
        sources[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    temp = bundle.with_suffix('.next.zip')
    with zipfile.ZipFile(temp, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(root / name, name)
    with zipfile.ZipFile(temp) as archive:
        bad = archive.testzip()
        if bad:
            raise ValueError(f'Corrupt entry: {bad}')
        if set(archive.namelist()) != set(sources):
            raise ValueError('Archive inventory changed')
        for name, digest in sources.items():
            if (hashlib.sha256(archive.read(name)).hexdigest() != digest
                    or hashlib.sha256((root / name).read_bytes()).hexdigest() != digest):
                raise ValueError(f'Source changed while packaging: {name}')
    digest = hashlib.sha256(temp.read_bytes()).hexdigest()
    manifest = dict(schema_version=1, sha256=digest, members=sources,
                    acceptance='Offline frozen candidate; field validation pending')
    manifest_temp = bundle.with_suffix('.manifest.next.json')
    sha_temp = bundle.with_suffix('.sha256.next')
    manifest_temp.write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf8')
    sha_temp.write_text(f'{digest.upper()}  {bundle.name}\n', encoding='ascii')
    os.replace(temp, bundle)
    os.replace(manifest_temp, bundle.with_suffix('.manifest.json'))
    os.replace(sha_temp, bundle.with_suffix('.sha256'))
    return manifest


def bundle_sources(root, bundle):
    with zipfile.ZipFile(bundle) as old:
        names = {n for n in old.namelist() if not n.endswith("/")}
    for directory in ("official_adapter", "nav2_adapter", "ackermann_rpp", "ackermann_smac",
                      "wildos_port/repro_check/src/nebula2-wildos/graphnav_msgs",
                      "wildos_port/repro_check/src/nebula2-wildos/object_search_msgs",
                      "wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper",
                      "wildos_port/repro_check/src/nebula2-wildos/graphnav_planner",
                      "wildos_port/repro_check/src/nebula2-wildos/visual_navigation"):
        names.update(p.relative_to(root).as_posix() for p in (root / directory).rglob("*")
                     if p.is_file() and "__pycache__" not in p.parts and p.suffix not in (".pyc", ".bak"))
    names.update({"tools/official_planner_probe.py", "tools/offline_navigation_check.py",
                  "tools/diagnose_town_follow_path.py",
                  "tools/offline_callback_check.py", "tools/offline_follow_path_replay.py",
                  "tools/offline_interface_replay.py", "tools/test_offline_interface_replay.py",
                  "tools/test_beamng_kinematic_math.py",
                  "tools/verify_beamng_math_source.py",
                  "tools/package_official_bundle.py", "tools/NAVIGATION_CHECK_FLOW.md",
                  "tools/beamng_extensions/wildosKinematic.lua"})
    names.update({"tools/BEAMNG_RVIZ_BRIDGE.md", "tools/patch_beamng_sensor_lifecycle.py",
                  "test_artifacts/server_validation_20261002/body_collision_bridge_vehicle.py"})
    names.update({"tools/official_wildos_window_demo.sh", "tools/wildos_server_window_demo.sh",
                  "tools/wildos_full_run.sh"})
    names.update({"test_artifacts/server_validation_20261002/accepted_memory_prefix_snapshot.npz",
                  "test_artifacts/server_validation_20261002/prefix_replay.json",
                  "test_artifacts/server_validation_20261002/house_localization_replay.json",
                  "test_artifacts/server_validation_20261002/full_town_memory_test.py"})
    names.add("test_artifacts/server_validation_20261006/green_house_station_arrival_kinematic_test.py")
    names.update({"tools/server_test_preflight.py", "tools/restore_physical_vehicle.py", "tools/check_physical_launch.py", "tools/check_mapper_tf_delivery.py", "tools/attach_current_vehicle_sensors.py", "tools/audit_stability_boundaries.py",
        "captures/20261008_B_green_house_station_trial12_repeat_geometry/stationary_sensor_calibration.json"})
    names.update({'tools/'+name for name in ('town_trial_harness.py',
        'town_physical_single_target_trial.py','failure_video_capture.py',
        'test_town_trial_harness.py','test_failure_video_capture.py',
        'town_sequential_memory_trial.py','continue_town_memory_trial.py',
        'check_ackermann_cusp_execution.py','check_execution_braking_dds.py',
        'check_rviz_image_delivery.py','image_stream_health.py','test_image_stream_health.py')})
    names.update({'tools/'+name for name in ('offline_extension_ownership_audit.py',
        'test_offline_extension_ownership_audit.py', 'offline_execution_contract_audit.py',
        'offline_goal_chain_replay.py', 'test_offline_goal_chain_replay.py',
        'test_package_official_bundle.py','offline_navigation_state_audit.py',
        'test_offline_navigation_state_audit.py')})
    names.update({'config/sbr_physical_sensor_rig.json',
                  'test_artifacts/server_validation_20261012/direct_steering_evidence.zip'})
    names.update({'test_artifacts/server_validation_20261004/unseeded_house_station_2_generic_20261004/'+name
                  for name in ('full_test.json','events.jsonl')})
    return names


def main():
    bundle = ROOT / 'official_wildos_beamng_bundle.zip'
    names = bundle_sources(ROOT, bundle)
    # Preserve the prior bytes under their content identity before publishing.
    digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
    backup = bundle.with_name(bundle.stem+'.rollback.'+digest+'.zip')
    if not backup.exists():
        backup.write_bytes(bundle.read_bytes())
    elif hashlib.sha256(backup.read_bytes()).hexdigest() != digest:
        raise ValueError('Rollback archive does not match its identity')
    manifest = write_bundle(ROOT, bundle, names)
    print(f"Packaged {len(names)} files: {manifest['sha256']}")
    print(f'Rollback: {backup}')

if __name__ == "__main__": main()
