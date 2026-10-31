"""Conservative owned-mod installer. Never edits modmanager DB or game settings."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import time
import zipfile

ROOT=Path(__file__).resolve().parent
LEVEL='nizhen_countryside'
PREFIX='levels/'+LEVEL+'/'
MOD_NAME='nizhen_countryside_standalone.zip'
OWNER='nizhen-countryside-standalone-v1'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(root=ROOT):
    manifest=json.loads((root/'generated/build_manifest.json').read_text(encoding='utf-8'))
    archive=root/'generated'/MOD_NAME
    if manifest['level']!=LEVEL or digest(archive)!=manifest['zip_sha256']:
        raise ValueError('Build archive digest mismatch')
    with zipfile.ZipFile(archive) as z:
        names=z.namelist()
        if len(names)!=len(set(names)) or set(names)!=set(manifest['files']) or z.testzip():
            raise ValueError('ZIP members or CRC mismatch')
        for i in z.infolist():
            name=i.filename
            if not name.startswith(PREFIX) or '..' in PurePosixPath(name).parts or '\\' in name or ':' in name:
                raise ValueError('Unsafe ZIP member')
            if i.file_size>30_000_000 or hashlib.sha256(z.read(i)).hexdigest()!=manifest['files'][name]:
                raise ValueError('Member integrity mismatch: '+name)
        for suffix in ('info.json','main/items.level.json','main/MissionGroup/Geometry/items.level.json'):
            if PREFIX+suffix not in names:
                raise ValueError('Missing native entry point')
    return manifest


def effective_user(user):
    user=Path(user).expanduser().resolve()
    if not user.is_dir():
        raise ValueError('User directory must already exist')
    if user.name.lower()!='current' and (user/'current').is_dir():
        user=user/'current'
    return user


def inventories(path, mode):
    if mode=='packed':
        return {'zip':digest(path)}
    if path.is_symlink() or any(p.is_symlink() for p in path.rglob('*')):
        raise ValueError('Symlinked unpacked mod refused')
    # Native mesh caches are engine-generated, not authored content.
    return {p.relative_to(path).as_posix():digest(p) for p in path.rglob('*')
            if p.is_file() and p.suffix not in ('.cdae','.cached.dts') and not p.name.endswith('.cached.dts')}


def check_conflicts(user, target):
    loose=user/'levels'/LEVEL
    if (loose/'info.json').exists() or (loose/'main').exists() or (loose/'main.level.json').exists():
        raise ValueError('Loose editor level shadows the mod; preserve/move it yourself: '+str(loose))
    mods=user/'mods'
    for p in mods.rglob('info.json'):
        if p.parent.name==LEVEL and p!=target/PREFIX/'info.json':
            raise ValueError('Another unpacked level shadows this map: '+str(p))
    for p in mods.rglob('*.zip'):
        if p==target:
            continue
        try:
            with zipfile.ZipFile(p) as z:
                if PREFIX+'info.json' in z.namelist():
                    raise ValueError('Another ZIP contains this level: '+str(p))
        except zipfile.BadZipFile:
            continue


def installed(user,root=ROOT):
    manifest=validate(root)
    user=effective_user(user)
    packed=user/'mods'/MOD_NAME
    unpacked=user/'mods/unpacked'/Path(MOD_NAME).stem
    candidates=[(packed,'packed'),(unpacked,'unpacked')]
    found=[(p,m) for p,m in candidates if p.exists()]
    if len(found)!=1:
        raise ValueError('Expected exactly one packed or unpacked installation')
    target,mode=found[0]
    expected={'zip':manifest['zip_sha256']} if mode=='packed' else manifest['files']
    if inventories(target,mode)!=expected:
        raise ValueError('Installed mod differs from tested build')
    check_conflicts(user,target)
    return target


def install(user,mode='packed',replace=False,root=ROOT):
    manifest=validate(root)
    user=effective_user(user)
    mods=user/'mods'
    mods.mkdir(exist_ok=True)
    target=mods/MOD_NAME if mode=='packed' else mods/'unpacked'/Path(MOD_NAME).stem
    target.parent.mkdir(exist_ok=True)
    check_conflicts(user,target)
    receipt_dir=user/'settings'/OWNER
    receipt_path=receipt_dir/'install.json'
    expected={'zip':manifest['zip_sha256']} if mode=='packed' else manifest['files']
    if target.exists():
        if not receipt_path.is_file():
            raise ValueError('Existing target has no ownership receipt; refusing overwrite')
        receipt=json.loads(receipt_path.read_text(encoding='utf-8'))
        if receipt.get('owner')!=OWNER or receipt.get('target')!=str(target) or receipt.get('mode')!=mode:
            raise ValueError('Ownership receipt mismatch')
        actual=inventories(target,mode)
        if actual!=receipt['inventory']:
            raise ValueError('Installed content was modified; refusing overwrite even with --replace')
        if actual==expected:
            print('Already installed and verified: '+str(target))
            return target
        if not replace:
            raise ValueError('Owned previous build exists; --replace required (backup retained)')
        backup=receipt_dir/'backups'/str(time.time_ns())
        backup.mkdir(parents=True)
        if mode=='packed': shutil.copy2(target,backup/target.name)
        else: shutil.copytree(target,backup/target.name)
    receipt_dir.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='nc_install_',dir=target.parent) as tmp:
        staged=Path(tmp)/target.name
        if mode=='packed':
            shutil.copy2(root/'generated'/MOD_NAME,staged)
        else:
            staged.mkdir()
            with zipfile.ZipFile(root/'generated'/MOD_NAME) as z:
                z.extractall(staged)
        if inventories(staged,mode)!=expected:
            raise ValueError('Staged copy failed verification')
        if mode=='unpacked' and target.exists():
            # Ownership and backup were checked above; never remove unknown files.
            target.rename(Path(tmp)/'previous')
        staged.replace(target)
    receipt=dict(owner=OWNER,target=str(target),mode=mode,inventory=expected,installed_at=time.time())
    receipt_path.write_text(json.dumps(receipt,indent=2),encoding='utf-8')
    print('Installed and verified: '+str(installed(user,root)))
    return target


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--user',type=Path,required=True,help='Effective current user folder or its parent')
    p.add_argument('--unpacked',action='store_true')
    p.add_argument('--replace',action='store_true')
    args=p.parse_args()
    install(args.user,'unpacked' if args.unpacked else 'packed',args.replace)
