"""Stdlib-only, read-only checkpoint audit; never deserialize pickle weights."""
import argparse
import collections
import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key: ' + key)
        result[key] = value
    return result


def header_audit(path):
    sizes = {'BOOL': 1, 'U8': 1, 'I8': 1, 'F8_E4M3': 1, 'F8_E5M2': 1,
             'I16': 2, 'U16': 2, 'F16': 2, 'BF16': 2, 'I32': 4, 'U32': 4,
             'F32': 4, 'I64': 8, 'U64': 8, 'F64': 8}
    length = path.stat().st_size
    with path.open('rb') as stream:
        prefix = stream.read(8)
        if len(prefix) != 8:
            raise ValueError('Missing 8-byte header length')
        header_length = struct.unpack('<Q', prefix)[0]
        if not 2 <= header_length <= min(100_000_000, length - 8):
            raise ValueError('Invalid or excessively large header length: ' + str(header_length))
        raw = stream.read(header_length)
    if not raw.startswith(b'{'):
        raise ValueError('Header does not begin with JSON object')
    header = json.loads(raw, object_pairs_hook=unique_object)
    if not isinstance(header, dict):
        raise ValueError('Header is not an object')
    errors, intervals, tensors = [], [], []
    dtypes = collections.Counter()
    payload_size = length - 8 - header_length
    metadata = header.pop('__metadata__', {})
    if not isinstance(metadata, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in metadata.items()):
        errors.append('Invalid __metadata__')
    for name, value in header.items():
        try:
            if not isinstance(value, dict) or set(value) != {'dtype', 'shape', 'data_offsets'}:
                raise ValueError('Invalid tensor descriptor fields')
            dtype, shape, offsets = value['dtype'], value['shape'], value['data_offsets']
            if dtype not in sizes:
                raise ValueError('Unsupported dtype: ' + str(dtype))
            if not isinstance(shape, list) or not all(type(n) is int and n >= 0 for n in shape):
                raise ValueError('Invalid shape')
            if not isinstance(offsets, list) or len(offsets) != 2 or not all(type(n) is int for n in offsets):
                raise ValueError('Invalid data_offsets')
            start, end = offsets
            expected = math.prod(shape) * sizes[dtype]
            if not 0 <= start <= end <= payload_size:
                raise ValueError('Offsets outside file payload')
            if end - start != expected:
                raise ValueError(f'Shape/dtype require {expected} bytes, offsets contain {end - start}')
            intervals.append((start, end, name))
            dtypes[dtype] += 1
            tensors.append({'name': name, **value, 'elements': math.prod(shape), 'bytes': expected})
        except (ValueError, TypeError, KeyError) as exc:
            errors.append(f'{name}: {exc}')
    cursor = 0
    for start, end, name in sorted(intervals):
        if start < cursor:
            errors.append(f'{name}: overlapping data range')
        elif start > cursor:
            errors.append(f'{name}: unindexed payload gap [{cursor}, {start})')
        cursor = max(cursor, end)
    if cursor != payload_size:
        errors.append(f'Indexed payload ends at {cursor}, actual payload length {payload_size}')
    return {'valid': not errors, 'errors': errors, 'header_bytes': header_length,
            'payload_bytes': payload_size, 'tensor_count': len(tensors),
            'elements': sum(t['elements'] for t in tensors), 'dtypes': dict(dtypes),
            'metadata': metadata, 'tensors': tensors}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(r'C:\Users\Yu\ckpts'))
    args = parser.parse_args()
    output = Path(__file__).resolve().parent / 'audit_results.json'
    report = {'started_utc': now(), 'command': [sys.executable, '-I', '-B', *sys.argv],
              'python': sys.version, 'root': str(args.root), 'files': [],
              'safetensors': [], 'json_checks': [], 'skipped': [],
              'scope': 'SHA256 is a measured fingerprint, not publisher authenticity verification. Pickle-based checkpoints are hashed only; never deserialized.'}
    started = time.monotonic()
    aliases = {}
    for path in sorted(args.root.rglob('*')):
        if not path.is_file():
            continue
        relative = path.relative_to(args.root)
        if any(part in {'.cache', '.locks'} for part in relative.parts) or path.suffix in {'.onnx', '.lock', '.incomplete'}:
            report['skipped'].append({'path': str(path), 'reason': 'Cache bookkeeping, incomplete download, lock, or optional ONNX excluded'})
            continue
        resolved = path.resolve()
        if resolved in aliases:
            report['files'].append({'path': str(path), 'same_resolved_file_as': aliases[resolved]})
        else:
            before = path.stat()
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                while chunk := stream.read(8 * 1024 * 1024):
                    digest.update(chunk)
            after = path.stat()
            item = {'path': str(path), 'resolved_path': str(resolved), 'bytes': after.st_size,
                    'sha256': digest.hexdigest(), 'stable_during_hash': (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)}
            # Hugging Face LFS blob filenames are expected SHA256 values.
            if path.parent.name == 'blobs' and len(path.name) == 64 and all(c in '0123456789abcdef' for c in path.name):
                item['matches_blob_sha256_name'] = path.name == digest.hexdigest()
            report['files'].append(item)
            aliases[resolved] = str(path)
            print(json.dumps(item), flush=True)
        if path.suffix == '.safetensors':
            try:
                check = header_audit(path)
            except Exception as exc:
                check = {'valid': False, 'errors': [f'{type(exc).__name__}: {exc}']}
            report['safetensors'].append({'path': str(path), **check})
        if path.suffix == '.json':
            try:
                if path.stat().st_size > 100_000_000:
                    raise ValueError('JSON exceeds bounded 100 MB parsing limit')
                with path.open('r', encoding='utf-8') as stream:
                    data = json.load(stream, object_pairs_hook=unique_object)
                check = {'path': str(path), 'valid_json': True, 'top_level_type': type(data).__name__}
                if path.name in {'config.json', 'preprocessor_config.json'}:
                    check['configuration'] = data
                elif isinstance(data, dict):
                    check['keys'] = sorted(data)
                report['json_checks'].append(check)
            except Exception as exc:
                report['json_checks'].append({'path': str(path), 'valid_json': False, 'error': f'{type(exc).__name__}: {exc}'})
    report['finished_utc'] = now()
    report['elapsed_seconds'] = time.monotonic() - started
    report['passed'] = (bool(report['safetensors']) and all(c['valid'] for c in report['safetensors'])
                        and all(c['valid_json'] for c in report['json_checks'])
                        and all(f.get('stable_during_hash', True) and f.get('matches_blob_sha256_name', True) for f in report['files']))
    output.write_text(json.dumps(report, indent=2, ensure_ascii=True) + '\n', encoding='utf-8')
    print(json.dumps({'report': str(output), 'passed': report['passed'], 'elapsed_seconds': report['elapsed_seconds']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
