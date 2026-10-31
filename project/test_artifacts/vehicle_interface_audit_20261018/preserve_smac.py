"""Verify the immutable Smac candidate before independent Native work."""
import hashlib
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[2]
frozen = root / 'deploy/20261018_smoothing_boundary_candidate'
manifest = json.loads((frozen / 'official_wildos_beamng_bundle.manifest.json').read_text())
archive = frozen / 'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['sha256']
assert manifest['sha256'] == '6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, digest in manifest['members'].items():
        assert hashlib.sha256(z.read(name)).hexdigest() == digest, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
out = root / 'test_artifacts/native_delay_20261018'
out.mkdir(exist_ok=True)
record = dict(smac_candidate=manifest['sha256'], members=len(manifest['members']),
              frozen_path=str(frozen), archive_and_sources_verified=True,
              native_candidate=None, merged=False,
              policy='Keep Smac frozen; diagnose Native independently; merge after Native validation.')
(out / 'preserved_smac.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record))
