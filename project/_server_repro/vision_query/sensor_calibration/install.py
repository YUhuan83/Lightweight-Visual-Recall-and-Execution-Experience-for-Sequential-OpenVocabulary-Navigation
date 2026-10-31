"""Install only our ZIP, refusing any differing destination; never edit mod DB."""
import argparse
import hashlib
import json
from pathlib import Path
from fixture import ROOT, MOD


def user_paths(value):
    user = Path(value).expanduser().resolve()
    parent = user.parent if user.name.lower() == 'current' else user
    effective = parent/'current'
    if not parent.is_dir() or not effective.is_dir():
        raise ValueError('Expected existing userpath parent and its current directory: '+str(effective))
    return parent, effective


def verified_mod():
    path = ROOT/'dist'/MOD
    info = json.loads((ROOT/'generated/package.json').read_text(encoding='utf-8'))
    if hashlib.sha256(path.read_bytes()).hexdigest() != info['sha256']:
        raise ValueError('Package hash mismatch; rebuild')
    return path


def install(user):
    _, effective = user_paths(user)
    source = verified_mod()
    target = effective/'mods'/MOD
    target.parent.mkdir(exist_ok=True)
    if target.exists():
        if target.read_bytes() != source.read_bytes():
            raise ValueError('Refusing differing target; manually review/remove only this own mod: '+str(target))
        return target
    with target.open('xb') as out:
        out.write(source.read_bytes())
    return target


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--user', required=True)
    print(install(p.parse_args().user))
