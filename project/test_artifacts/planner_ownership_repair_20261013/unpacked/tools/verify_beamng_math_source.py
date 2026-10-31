"""Verify the quaternion reference against a local BeamNG ZIP, without running it."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile


def verify(archive):
    sources = {}
    with zipfile.ZipFile(archive) as package:
        for suffix in ('lua/common/mathlib.lua', 'lua/ge/extensions/tech/techCore.lua'):
            names = [n for n in package.namelist() if n.endswith('/'+suffix)]
            if len(names) != 1:
                raise ValueError(f'Expected one {suffix}, found {len(names)}')
            raw = package.read(names[0])
            sources[suffix] = dict(entry=names[0], sha256=hashlib.sha256(raw).hexdigest(),
                                   text=raw.decode('utf8').replace('\r', ''))
    mathlib, core = (sources[s]['text'] for s in sources)
    compact = lambda text: re.sub(r'\s+', '', text)
    checks = {
        'passive_vector_action': 'qv:set(push3(b)-push3(qv)*a.w+push3(a):cross(qv))' in compact(mathlib),
        'hamilton_product': compact('''newLuaQuatxyzw(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,
            a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x,
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z)''') in compact(mathlib),
        'axis_angle_sign': compact('self.x, self.y, self.z, self.w = fsin * axle.x, fsin * axle.y, fsin * axle.z, math.cos(angleRad)') in compact(mathlib),
        'forward_up_axes': 'returnself*vec3(0,1,0),self*vec3(0,0,1)' in compact(mathlib),
        'absolute_to_relative_setter': 'localdiffRot=vehRot:inversed()*rot' in compact(core)
            and 'veh:setClusterPosRelRot(veh:getRefNodeId(),pos.x,pos.y,pos.z,diffRot.x,diffRot.y,diffRot.z,diffRot.w)' in compact(core),
    }
    if not all(checks.values()):
        raise ValueError('Shipped math source differs from the reference: '+json.dumps(checks))
    return dict(archive=str(archive.resolve()), source_checks=checks,
        sources={name:{k:v for k,v in source.items() if k != 'text'} for name,source in sources.items()},
        scope='Official shipped Lua source inspection only; no Lua VM or BeamNG engine execution.',
        consequence='Q(a*b)=Q(b)Q(a); current*quatZ(-turn) applies positive world-Z yaw to a tilted carrier.')


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', type=Path,
        default=root/'test_artifacts/offline_navigation/beamng_math_source_20261006.json')
    args = parser.parse_args()
    result = verify(args.archive)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
