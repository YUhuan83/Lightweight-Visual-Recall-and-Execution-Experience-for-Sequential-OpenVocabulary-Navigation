"""Verify live source against the final bundle; update test fixtures only."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path('/home/ly/town_memory_validation_20261003')
WS=Path('/home/ly/beamng_ros2_0394_ws')
bundle=ROOT/'official_wildos_beamng_bundle.zip'
digest=hashlib.sha256(bundle.read_bytes()).hexdigest()
assert digest.upper()==(ROOT/'official_wildos_beamng_bundle.sha256').read_text().split()[0].upper()
prefixes={'nav2_adapter/':WS/'src/wildos_nav2_adapter',
          'official_adapter/':WS/'src/beamng_wildos_adapter',
          'wildos_port/repro_check/src/nebula2-wildos/':WS/'src/nebula2-wildos'}
files={};mismatches=[];updated_tests=[]
with zipfile.ZipFile(bundle) as archive:
    assert archive.testzip() is None
    for name in archive.namelist():
        match=next(((prefix,base) for prefix,base in prefixes.items() if name.startswith(prefix)),None)
        if match is None:continue
        prefix,base=match
        target=(base/name[len(prefix):]).resolve()
        assert target.is_relative_to(base.resolve())
        data=archive.read(name)
        if not target.is_file() or target.read_bytes()!=data:
            if '/test/' in name:
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(data)
                updated_tests.append(str(target))
            else:
                mismatches.append(str(target))
        if target.is_file():files[str(target)]=hashlib.sha256(target.read_bytes()).hexdigest()
report={'bundle_sha256':digest,'files':files,'runtime_source_mismatches':mismatches,
        'updated_test_files':updated_tests,'all_selected_sources_match':not mismatches}
(ROOT/'deployment_manifest_final.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'bundle_sha256':digest,'verified_files':len(files),
                  'runtime_source_mismatches':mismatches,'updated_test_count':len(updated_tests)},indent=2))
assert not mismatches,'Live sources differ from the final bundle'
