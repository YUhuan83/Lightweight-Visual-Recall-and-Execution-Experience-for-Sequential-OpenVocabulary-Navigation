"""Create a separate source candidate from a verified baseline plus a bounded delta."""
from pathlib import Path,PurePosixPath
import hashlib
import json
import zipfile
import argparse

parser=argparse.ArgumentParser();parser.add_argument('--portable',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parent;candidate=root/('candidate_portable' if args.portable else 'candidate')
assert not candidate.exists();candidate.mkdir()
baseline=Path('/home/ly/smoothing_boundary_repair_20261018/official_wildos_beamng_bundle.zip')
assert hashlib.sha256(baseline.read_bytes()).hexdigest()=='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
def unpack(archive):
    with zipfile.ZipFile(archive) as zf:
        assert zf.testzip() is None
        for name in zf.namelist():
            relative=PurePosixPath(name)
            assert not relative.is_absolute() and '..' not in relative.parts and '\\' not in name
            if name.endswith('/'):continue
            path=candidate.joinpath(*relative.parts)
            assert path.resolve().is_relative_to(candidate.resolve())
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(zf.read(name))
unpack(baseline)
baseline_manifest=json.loads(baseline.with_suffix('.manifest.json').read_text())
for name,want in baseline_manifest['members'].items():assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==want,name
delta_zip=root/('reset_intent_delta.portable.zip' if args.portable else 'reset_intent_delta.zip')
with zipfile.ZipFile(delta_zip) as zf:delta=json.loads(zf.read('delta_manifest.json'))
for name,want in delta['before'].items():assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==want,name
unpack(delta_zip)
for name,want in delta['after'].items():assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==want,name
for name,want in baseline_manifest['members'].items():
    if name not in delta['after']:assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==want,name
(root/('staged_identity.portable.json' if args.portable else 'staged_identity.json')).write_text(json.dumps(dict(baseline_members=len(baseline_manifest['members']),**delta),indent=2)+'\n')
print(json.dumps(dict(candidate=str(candidate),baseline_members=len(baseline_manifest['members']),delta_files=len(delta['after']),production_deployment=False)))
