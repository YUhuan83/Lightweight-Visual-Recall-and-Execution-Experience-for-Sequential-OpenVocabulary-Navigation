"""Extract a verified immutable base and a bounded delta into an independent root."""
from pathlib import Path,PurePosixPath
import hashlib,json,zipfile
root=Path(__file__).resolve().parent;candidate=root/'candidate'
assert not candidate.exists();candidate.mkdir()
baseline=Path('/home/ly/smoothing_boundary_repair_20261018/official_wildos_beamng_bundle.zip')
assert hashlib.sha256(baseline.read_bytes()).hexdigest()=='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b'
def unpack(archive):
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            rel=PurePosixPath(name);assert not rel.is_absolute() and '..' not in rel.parts and '\\' not in name
            if name.endswith('/'):continue
            p=candidate.joinpath(*rel.parts);assert p.resolve().is_relative_to(candidate.resolve())
            p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
def verify(members):
    for name,want in members.items():assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==want,name
unpack(baseline)
original=json.loads(baseline.with_suffix('.manifest.json').read_text())['members'];verify(original)
with zipfile.ZipFile(root/'candidate_delta.zip') as z:delta=json.loads(z.read('candidate_delta.json'))
unpack(root/'candidate_delta.zip');verify(delta['members'])
verify({k:v for k,v in original.items() if k not in delta['members']})
(root/'candidate_delta.json').write_text(json.dumps(delta,indent=2)+'\n')
identity=dict(baseline_members=len(original),candidate=str(candidate),**delta)
(root/'staged_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(dict(baseline_members=len(original),delta_files=len(delta['members']))))
