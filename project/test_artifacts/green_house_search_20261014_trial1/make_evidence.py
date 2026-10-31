"""Freeze finished logs and hash each archived file; exclude the active engine log."""
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

ROOT=Path('/home/ly/town_memory_validation_20261014')
ARCHIVE=Path('/home/ly/town_memory_validation_20261014_evidence.tar.gz')


def main():
    if ARCHIVE.exists(): raise FileExistsError(ARCHIVE)
    shutdown=json.loads((ROOT/'round_shutdown.json').read_text())
    if shutdown.get('passed') is not True: raise RuntimeError('Test owners not safely closed')
    shutil.copyfile(ROOT/'beamng_headless.log',ROOT/'beamng_headless_after_trial.log')
    files=[p for p in ROOT.rglob('*') if p.is_file() and not p.is_symlink()
           and '__pycache__' not in p.parts and p.name not in ('beamng_headless.log','evidence_manifest.json')]
    manifest={'root':str(ROOT),'files':[{'path':p.relative_to(ROOT).as_posix(),
        'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(files)]}
    manifest['total_bytes']=sum(p['bytes'] for p in manifest['files'])
    target=ROOT/'evidence_manifest.json'
    target.write_text(json.dumps(manifest,indent=2)+'\n')
    with tarfile.open(ARCHIVE,'w:gz') as archive:
        for p in sorted(files)+[target]: archive.add(p,arcname=p.relative_to(ROOT).as_posix(),recursive=False)
    result={'archive':str(ARCHIVE),'bytes':ARCHIVE.stat().st_size,
            'sha256':hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),'raw_files':len(files),
            'raw_bytes':manifest['total_bytes']}
    print(json.dumps(result),flush=True)


if __name__=='__main__': main()
