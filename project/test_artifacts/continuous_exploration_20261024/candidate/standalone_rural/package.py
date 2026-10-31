"""Portable allowlisted delivery, excluding native format excerpts and all game assets."""
import hashlib
import json
from pathlib import Path
import zipfile
from install import ROOT, MOD_NAME, validate


def package():
    manifest=validate()
    report=json.loads((ROOT/'generated/verification.json').read_text())
    if not report['successful']: raise ValueError('Offline verification must pass first')
    names=['README.md','requirements.txt','mesh.py','build.py','install.py','run.py',
           'test_standalone.py','verify.py','package.py','inspect_native.py',
           'generated/'+MOD_NAME,'generated/build_manifest.json','generated/verification.json','generated/tests.txt']
    out=ROOT/'dist'
    out.mkdir(exist_ok=True)
    target=out/'nizhen_countryside_portable.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for name in names:
            info=zipfile.ZipInfo('standalone_rural/'+name,(2026,9,11,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED
            z.writestr(info,(ROOT/name).read_bytes())
    with zipfile.ZipFile(target) as z:
        if z.testzip(): raise ValueError('Portable ZIP CRC failed')
    checksum=hashlib.sha256(target.read_bytes()).hexdigest()
    (out/'nizhen_countryside_portable.sha256').write_text(checksum+'  '+target.name+'\n',encoding='ascii')
    print(json.dumps(dict(portable=str(target),bytes=target.stat().st_size,sha256=checksum,
                          mod_bytes=manifest['zip_bytes']),indent=2,ensure_ascii=True))


if __name__=='__main__':
    package()
