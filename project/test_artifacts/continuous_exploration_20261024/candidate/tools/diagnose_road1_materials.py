#!/usr/bin/env python3
"""Read-only ZIP diagnosis; optional JSON report, no extraction or game access."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import posixpath
import re
import zipfile


SPAWN = (-2326.96729, 1479.02979, -547.536377)
QUAT = (.0002917, -.0001847, .5349686, .8448718)
MAX_CONFIG = 64 * 1024 * 1024


def segment_distance_xy(point, a, b):
    """Return distance, clamped segment fraction, and closest XY point."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                          (point[1] - a[1]) * dy) / length2)) if length2 else 0.0
    closest = (a[0] + t * dx, a[1] + t * dy)
    return math.hypot(point[0] - closest[0], point[1] - closest[1]), t, closest


def clean_comments(text):
    return re.sub(r'"(?:\\.|[^"\\])*"|//[^\r\n]*|/\*[\s\S]*?\*/',
                  lambda m: m[0] if m[0].startswith('"') else ' ', text)


def parse_json(text):
    text = clean_comments(text)
    text = re.sub(r'("(?:\\.|[^"\\])*")|,\s*(?=[}\]])',
                  lambda m: m[1] or '', text)
    decoder = json.JSONDecoder()
    offset = 0
    while offset < len(text):
        if text[offset].isspace():
            offset += 1
            continue
        value, offset = decoder.raw_decode(text, offset)
        yield value


def walk(value, key=''):
    if isinstance(value, dict):
        yield key, value
        for name, child in value.items():
            yield from walk(child, name)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from walk(child, str(index))


def valid_node(node):
    return (isinstance(node, list) and len(node) >= 4 and
            all(isinstance(v, (int, float)) and math.isfinite(v) for v in node[:4]))


def road_record(obj, source):
    nodes = obj.get('nodes', [])
    segments = [(segment_distance_xy(SPAWN, a, b), i, a, b)
                for i, (a, b) in enumerate(zip(nodes, nodes[1:]))
                if valid_node(a) and valid_node(b)]
    if not segments:
        return None
    (distance, t, closest), index, a, b = min(segments, key=lambda s: s[0][0])
    position = obj.get('position')
    first = nodes[0] if nodes and valid_node(nodes[0]) else None
    comparable = (isinstance(position, list) and len(position) >= 3 and first and
                  all(isinstance(v, (int, float)) for v in position[:3]))
    materials = {k: v for k, v in obj.items()
                 if k.lower() in ('material', 'topmaterial', 'bottommaterial', 'sidematerial')}
    return {
        'class': obj['class'], 'name': obj.get('name'),
        'uuid': obj.get('persistentId', obj.get('uuid')), 'source_filepath': source,
        'materials': materials, 'object_position': position,
        'rotationMatrix': obj.get('rotationMatrix'), 'scale': obj.get('scale'),
        'first_node': first, 'node_count': len(nodes),
        'node_widths_m': [n[3] if valid_node(n) else None for n in nodes],
        'first_node_position_delta_xyz': [first[i] - position[i] for i in range(3)]
        if comparable else None,
        'min_distance_xy_m': distance, 'nearest_segment_index': index,
        'nearest_segment_nodes': [a, b], 'segment_fraction': t,
        'closest_xy': closest, 'interpolated_node_z': a[2] + t * (b[2] - a[2]),
        'interpolated_width_m': a[3] + t * (b[3] - a[3]),
        'position_added_min_distance_xy_m': min(
            segment_distance_xy(SPAWN,
                                [s[2][j] + position[j] for j in range(2)],
                                [s[3][j] + position[j] for j in range(2)])[0]
            for s in segments) if comparable else None,
    }


def texture_refs(value, field=''):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from texture_refs(child, f'{field}.{key}' if field else key)
    elif isinstance(value, list):
        for i, child in enumerate(value):
            yield from texture_refs(child, f'{field}[{i}]')
    elif isinstance(value, str) and value.strip():
        # Includes baseColorMap, colorMap, diffuseMap, normalMap and detail maps.
        if re.search(r'(?:map|tex)(?:\[\d+\])?$', field, re.I):
            yield field, value


def resolve_reference(reference, source, names):
    path = reference.replace('\\', '/').lstrip('/')
    rooted = path.startswith(('levels/', 'art/', 'core/', 'vehicles/'))
    candidates = [path] if rooted else [posixpath.normpath(
        posixpath.join(posixpath.dirname(source), path)), path]
    candidates = list(dict.fromkeys(candidates))
    exact = [p for p in candidates if p in names]
    links = [p + '.link' for p in candidates if p + '.link' in names]
    folded = {p.casefold() for p in candidates}
    case_matches = sorted(p for p in names if p.casefold() in folded and p not in exact)
    return {'reference': reference, 'candidate_paths': candidates,
            'exact_in_mod': exact, 'link_entries_in_mod': links,
            'case_only_matches': case_matches,
            'status': 'exact_in_mod' if exact else 'link_entry_in_mod_target_unverified'
            if links else 'unresolved_in_archive_not_confirmed_missing'}


def diagnose(path):
    before = path.stat()
    roads, definitions, warnings = [], [], []
    grid_mentions = []
    cs_road_sources = []
    with zipfile.ZipFile(path, 'r') as archive:
        infos = archive.infolist()
        names = {i.filename for i in infos if not i.is_dir()}
        duplicates = [name for name, count in Counter(i.filename for i in infos).items()
                      if count > 1]
        for info in infos:
            if not info.filename.lower().endswith(('.json', '.cs')):
                continue
            if info.file_size > MAX_CONFIG:
                warnings.append({'source': info.filename, 'reason': 'exceeds 64 MiB read limit'})
                continue
            text = archive.read(info).decode('utf-8-sig', errors='replace')
            for number, line in enumerate(text.splitlines(), 1):
                for mention in re.finditer(r'Grid512\w*|512_red', line, re.I):
                    grid_mentions.append({'source_filepath': info.filename, 'line': number,
                                          'context': line[max(0, mention.start() - 100):
                                                          mention.end() + 160]})
            if info.filename.lower().endswith('.json'):
                try:
                    for root in parse_json(text):
                        for key, obj in walk(root):
                            if obj.get('class') in ('DecalRoad', 'MeshRoad'):
                                record = road_record(obj, info.filename)
                                if record:
                                    roads.append(record)
                                else:
                                    warnings.append({'source': info.filename,
                                                     'reason': 'road has no valid segments',
                                                     'uuid': obj.get('persistentId')})
                            if obj.get('class') == 'Material' or (
                                    'materials' in info.filename.lower() and
                                    ('Stages' in obj or 'mapTo' in obj)):
                                definitions.append((key, obj, info.filename, None))
                except (ValueError, TypeError) as error:
                    warnings.append({'source': info.filename, 'reason': str(error)})
            else:
                clean = clean_comments(text)
                if re.search(r'\b(?:new|singleton)\s+(?:DecalRoad|MeshRoad)\s*\(', clean):
                    cs_road_sources.append(info.filename)
                for match in re.finditer(
                        r'\b(?:singleton|new)\s+Material\s*\(\s*([^):\s]+)'
                        r'(?:\s*:\s*([^)]*))?\s*\)\s*\{([^{}]*)\}', clean, re.S):
                    obj = dict(re.findall(r'(\w+(?:\[\d+\])?)\s*=\s*"((?:\\.|[^"\\])*)"\s*;',
                                          match[3]))
                    definitions.append((match[1], obj, info.filename, match[2]))
        nearby_all = sorted((r for r in roads if r['min_distance_xy_m'] <= 100),
                            key=lambda r: r['min_distance_xy_m'])
        nearby = nearby_all[:10]
        targets = {v for r in nearby_all for v in r['materials'].values() if isinstance(v, str)}
        targets.add('Grid512_Red_Mat')
        matches = []
        found = set()
        for key, obj, source, parent in definitions:
            aliases = {v for v in (key, obj.get('name'), obj.get('mapTo'), obj.get('internalName'))
                       if isinstance(v, str)}
            matched = targets & aliases
            if not matched and not any('grid512' in a.lower() for a in aliases):
                continue
            found.update(matched)
            refs = [dict(field=field, **resolve_reference(ref, source, names))
                    for field, ref in texture_refs(obj)]
            matches.append({'name': key, 'source_filepath': source,
                            'aliases': sorted(aliases), 'matched_road_or_grid_names': sorted(matched),
                            'inherits': parent, 'texture_references': refs,
                            'render_settings': {k: v for k, v in obj.items()
                                                if k not in ('Stages',) and
                                                not isinstance(v, (dict, list)) and
                                                not re.search(r'map(?:\[\d+\])?$', k, re.I)}})
    conventions = {}
    for cls in ('DecalRoad', 'MeshRoad'):
        group = [r for r in roads if r['class'] == cls]
        comparable = [r for r in group if r['first_node_position_delta_xyz'] is not None]
        equal = [r for r in comparable if max(abs(v) for v in
                                             r['first_node_position_delta_xyz']) < .02]
        conventions[cls] = {'road_count': len(group), 'position_comparable_count': len(comparable),
                            'first_node_matches_position_within_2cm': len(equal),
                            'interpretation': 'Node coordinates used directly as world coordinates; '
                            'object position is not added. Equality with the first node is archive '
                            'evidence, not an independent runtime transform verification.'}
    after = path.stat()
    return {
        'archive': str(path), 'archive_size_bytes': before.st_size,
        'archive_size_and_mtime_unchanged': (before.st_size, before.st_mtime_ns) ==
        (after.st_size, after.st_mtime_ns), 'archive_open_mode': 'r',
        'spawn_xyz': SPAWN, 'quaternion_xyzw': QUAT,
        'user_reported_facing_xy': [-.904, -.428],
        'distance_method': 'Closest XY point on each consecutive node segment; centerline, '
        'not road edge, spline tessellation, terrain projection, or visibility test.',
        'coordinate_conventions': conventions, 'radius_m': 100,
        'roads_within_radius_count': len(nearby_all), 'closest_roads': nearby,
        'material_targets_all_roads_within_radius': sorted(targets),
        'material_definitions': matches, 'unresolved_material_names': sorted(targets - found),
        'grid_asset_entries': sorted(n for n in names if re.search(r'grid512|512_red', n, re.I)),
        'grid_text_mentions': grid_mentions,
        'duplicate_zip_members': duplicates, 'warnings': warnings,
        'unsupported_cs_road_sources': cs_road_sources,
        'limitations': [
            'Only archive evidence; external game and other-mod assets are not inspected.',
            '.dds.link entries are not treated as invalid; targets and runtime resolution are unverified.',
            'File presence does not establish image validity, material loading, or rendering behavior.',
            'Material duplicates/load order, CS expressions/inheritance and runtime overrides are not resolved.',
            'CS road geometry is not parsed; any such sources are explicitly listed.',
            'Grid512 and ECA detail warnings do not establish the cause of the white road.',
            'The actual visible surface and white-road cause remain unknown.'
        ],
        'manual_investigation_plan': [
            'Identify the visible surface against the closest road UUIDs and segment coordinates.',
            'Check that surface material and active stages in the running game only when separately authorized.',
            'Resolve external/link assets and duplicate material load order before classifying references as missing.',
            'Correlate a surface-specific render warning or controlled experiment before proposing a fix.'
        ]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path,
                        default=Path('~/beamng-maps/american_road_0.39.zip'))
    parser.add_argument('--output', type=Path, help='Optional derived JSON report; otherwise stdout')
    args = parser.parse_args()
    archive = args.archive.expanduser().resolve(strict=True)
    if args.output:
        output = args.output.expanduser().resolve()
        if output == archive or (output.exists() and output.samefile(archive)):
            parser.error('Report output must not be the source archive')
        if output.suffix.lower() != '.json' or not output.parent.is_dir():
            parser.error('Report must be a .json file in an existing directory')
    report = diagnose(archive)
    text = json.dumps(report, indent=2, ensure_ascii=True, allow_nan=False) + '\n'
    if args.output:
        output.write_text(text, encoding='utf-8')
        print(f'Report: {output}')
        print(f'Roads within 100 m: {report["roads_within_radius_count"]}')
        for road in report['closest_roads']:
            print(f'{road["min_distance_xy_m"]:.3f} m {road["class"]} '
                  f'{road["uuid"]} {road["materials"]}')
        print('Coordinate evidence: ' + json.dumps(report['coordinate_conventions']))
        for material in report['material_definitions']:
            print(f'Material {material["name"]}: {material["source_filepath"]}')
            for ref in material['texture_references']:
                print(f'  {ref["field"]}: {ref["reference"]}: {ref["status"]}')
        print(f'Warnings: {len(report["warnings"])}; '
              f'unsupported CS road sources: {len(report["unsupported_cs_road_sources"])}')
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
