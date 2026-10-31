"""Portable explicit allowlist, including evaluator-only metadata, never game resources."""
import hashlib
import json
import zipfile
from install import ROOT, MOD_NAME, validate


def package():
    manifest=validate()
    report=json.loads((ROOT/'generated/verification.json').read_text())
    if not report['successful'] or report['zip_sha256']!=manifest['zip_sha256']:
        raise ValueError('Verify current build first')
    names=['README.md','requirements.txt','mesh.py','base.py','build.py','install.py','run.py',
           'test_original.py','test_terraced.py','verify.py','package.py','reproduce.py','helper_provenance.json',
           'generated/'+MOD_NAME,'generated/build_manifest.json','generated/verification.json','generated/tests.txt',
           'generated/reproducibility.json','generated/evaluator/ground_truth.json',
           'evidence/offline_runtime.prefab.json','evidence/offline_scenario.json']
    out=ROOT/'dist'; out.mkdir(exist_ok=True)
    target=out/'terraced_countryside_portable.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for name in names:
            zi=zipfile.ZipInfo('terraced_rural/'+name,(2026,9,11,0,0,0)); zi.compress_type=zipfile.ZIP_DEFLATED
            z.writestr(zi,(ROOT/name).read_bytes())
    with zipfile.ZipFile(target) as z:
        if z.testzip(): raise ValueError('Portable CRC failure')
    checksum=hashlib.sha256(target.read_bytes()).hexdigest()
    (out/'terraced_countryside_portable.sha256').write_text(checksum+'  '+target.name+'\n',encoding='ascii')
    print(json.dumps(dict(portable=str(target),bytes=target.stat().st_size,sha256=checksum,mod_bytes=manifest['zip_bytes']),indent=2))


if __name__=='__main__': package()
