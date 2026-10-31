"""Verify the sealed server survey and describe geometry without naming an asset."""
from pathlib import Path, PurePosixPath
import hashlib
import json
import tarfile

import numpy as np

OUT = Path(__file__).resolve().parent


def main():
    archive = OUT / 'evidence.tgz'
    want = (OUT / 'evidence.sha256').read_text().split()[0]
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == want
    dest = OUT / 'server_evidence'
    dest.mkdir(exist_ok=False)
    with tarfile.open(archive, 'r:gz') as package:
        members = package.getmembers()
        assert len({m.name for m in members}) == len(members)
        for member in members:
            name = PurePosixPath(member.name)
            assert member.isfile() and not name.is_absolute() and '..' not in name.parts
            target = dest.joinpath(*name.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(package.extractfile(member).read())
    manifest = json.loads((dest / 'manifest.json').read_text())
    for name, expected in manifest.items():
        data = dest.joinpath(*PurePosixPath(name).parts).read_bytes()
        assert len(data) == expected['bytes']
        assert hashlib.sha256(data).hexdigest() == expected['sha256'], name
    runs = sorted(dest.glob('static_*/result.json'))
    assert len(runs) == 1
    result = json.loads(runs[0].read_text())
    assert result['completed'] and not result['errors']
    records = json.loads(runs[0].with_name('surface_samples.json').read_text())
    assert len(records) == result['samples'] == 61 * 61
    assert all(r['hasStatic'] and 'terrainZ' in r for r in records)
    static = np.array([r['staticZ'] for r in records]).reshape(61, 61)
    terrain = np.array([r['terrainZ'] for r in records]).reshape(61, 61)
    difference = static - terrain
    gx, gy = np.gradient(static, .1, .1)
    centre = result['centre']
    report = dict(
        scope=__doc__, archive_sha256=want, verified_payloads=len(manifest),
        sample_count=len(records), completed=True,
        contact_static_height_m=centre['staticZ'],
        contact_terrain_height_m=centre['terrainZ'],
        contact_static_above_terrain_m=centre['staticZ'] - centre['terrainZ'],
        contact_static_minus_historical_contact_z_m=centre['staticZ'] - result['contact_world'][2],
        static_above_terrain_p95_m=float(np.percentile(difference, 95)),
        static_surface_grade_p95=float(np.percentile(np.hypot(gx, gy), 95)),
        static_surface_grade_max=float(np.hypot(gx, gy).max()),
        central_static_grade=float(np.hypot(gx[30, 30], gy[30, 30])),
        static_above_terrain_over_10cm_cells=int((difference > .1).sum()),
        complete_navigation_passed=False, asset_identity_proven=False,
        limits='Static ray height is the uppermost hit from above. A fresh scene does not replay historical deformation. Terrain/static disagreement alone does not prove an undrivable obstacle.',
    )
    (OUT / 'static_geometry_review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
