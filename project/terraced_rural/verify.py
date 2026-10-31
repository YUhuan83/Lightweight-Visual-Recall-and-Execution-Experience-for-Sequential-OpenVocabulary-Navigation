"""Offline installed-resource evidence and complete test report; never connects to BeamNG."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import unittest
import zipfile
import build
import test_terraced


def verify(home):
    refs=['art/sky_gradients/default/gradient_'+n+'.png' for n in ('ambient','colorize','fog','sunscale')]
    refs+=['art/skies/clouds/clouds_normal_displacement','vehicles/pickup/d15_4wd_A.pc']
    found={}
    archives=list((home/'content').glob('*.zip'))+list((home/'content/vehicles').glob('pickup.zip'))
    archives += [home/'gameengine.zip',home/'content/assets/materials/sky.zip']
    for p in archives:
        with zipfile.ZipFile(p) as z:
            names=set(z.namelist())
            for ref in refs:
                candidates=[ref] if Path(ref).suffix else [ref+s for s in ('','.dds','.png')]
                for member in candidates:
                    if member in names:
                        found[ref]={'archive':p.name,'member':member,'bytes':z.getinfo(member).file_size}
            # Confirm the named common environment cubemap is defined, not copied.
            for info in z.infolist():
                if info.file_size<200000 and 'sky' in info.filename.lower() and info.filename.endswith(('.cs','.json')):
                    if b'BNG_Sky_01_cubemap' in z.read(info):
                        found['BNG_Sky_01_cubemap']={'archive':p.name,'member':info.filename}
    for ref in refs:
        if (home/ref).is_file(): found[ref]={'loose':ref}
    missing=set(refs+['BNG_Sky_01_cubemap'])-set(found)
    source_evidence={}
    for relative in ('lua/ge/extensions/core/levels.lua','lua/ge/spawn.lua','lua/ge/extensions/core/vehicles.lua'):
        p=home/relative
        data=p.read_bytes()
        source_evidence[relative]=hashlib.sha256(data).hexdigest()
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.loadTestsFromModule(test_terraced)
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    print(stream.getvalue())
    native_evidence={}
    with zipfile.ZipFile(home/'content/levels/smallgrid.zip') as z:
        for suffix in ('info.json','main/items.level.json','main/MissionGroup/PlayerDropPoints/items.level.json','main/MissionGroup/Sky/items.level.json'):
            member='levels/smallgrid/'+suffix
            data=z.read(member)
            native_evidence[member]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))
        sky=z.read('levels/smallgrid/main/MissionGroup/Sky/items.level.json').decode()
        assert all(name in sky for name in ('ScatterSky','CloudLayer','LevelInfo','TimeOfDay'))
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                zip_sha256=json.loads((build.ROOT/'generated/build_manifest.json').read_text())['zip_sha256'],
                native_format_evidence=native_evidence,
                successful=result.wasSuccessful() and not missing,missing_builtin_refs=sorted(missing),
                installed_resource_refs=found,native_loader_source_sha256=source_evidence,
                runtime_verified=False,menu_registration='Native entry structure checked against installed core/levels.lua',
                vehicle_spawn='Native SpawnSphere + FreeRoam selected vehicle; script spawns one pickup',
                visual_review=False,license_files_accessed=False,game_connected=False)
    build.write_json(build.ROOT/'generated/verification.json',report)
    (build.ROOT/'generated/tests.txt').write_text(stream.getvalue(),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not report['successful']: raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home',type=Path,required=True)
    verify(p.parse_args().home)
    