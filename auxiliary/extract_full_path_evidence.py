from pathlib import Path,PurePosixPath
import tarfile,sys
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\full_path_control_20261026\field')
dest=root/'closed_core';dest.mkdir(exist_ok=False)
archives=list(root.glob('trial_*_core.tar.gz'));assert len(archives)==1
with tarfile.open(archives[0]) as archive:
    for m in archive.getmembers():
        p=PurePosixPath(m.name)
        assert not p.is_absolute() and '..' not in p.parts and ':' not in m.name and '\\' not in m.name
        target=dest.joinpath(*p.parts)
        if m.isdir():target.mkdir(parents=True,exist_ok=True);continue
        assert m.isfile(),m.name
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(archive.extractfile(m).read())
print(dest)
print('\n'.join(p.relative_to(dest).as_posix() for p in dest.rglob('*') if p.is_file()))
