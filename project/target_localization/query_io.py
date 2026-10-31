"""Read vocabulary_query/query.py outputs using only stdlib and numpy."""

import hashlib
import json
from pathlib import Path
import re
import struct
import zlib

import numpy as np


class Invalid(ValueError):
    pass


def require(ok, message):
    if not ok:
        raise Invalid(message)


def fields(value, names):
    require(type(value) is dict and set(value) == set(names.split()),
            "Missing or unknown fields: " + names)


def text(value):
    require(type(value) is str and bool(value.strip()), "Nonempty text required")
    return value


def number(value):
    require(type(value) in (int, float), "Numeric value required (not bool/null)")
    try:
        valid = np.isfinite(value)
    except (TypeError, OverflowError):
        valid = False
    require(bool(valid), "Finite numeric value required")
    return value


def read_json(path):
    def pairs(items):
        out = {}
        for key, value in items:
            require(key not in out, "Duplicate JSON key: " + key)
            out[key] = value
        return out

    def bad(value):
        raise Invalid("Nonfinite JSON constant: " + value)

    value = json.loads(Path(path).read_text(encoding="utf-8"),
                       object_pairs_hook=pairs, parse_constant=bad)

    def finite(item):
        if type(item) is float:
            number(item)
        elif type(item) is dict:
            for child in item.values():
                finite(child)
        elif type(item) is list:
            for child in item:
                finite(child)
    finite(value)
    return value


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array(value, shape=None):
    # Reject ragged rows, strings, bools, object arrays and nonfinite returns.
    if isinstance(value, list):
        def numeric(item):
            if type(item) is list:
                for child in item:
                    numeric(child)
            else:
                number(item)
        numeric(value)
    try:
        out = np.asarray(value)
    except (ValueError, TypeError) as exc:
        raise Invalid("Invalid numeric rows") from exc
    require(out.dtype.kind in "fiu" and np.isfinite(out).all(), "Finite numeric array required")
    require(shape is None or out.shape == shape, "Array shape mismatch")
    return out.astype(np.float64)


def read_mask_png(path):
    """Strict subset emitted by Image.fromarray(uint8 mask): gray8, noninterlaced."""
    data = Path(path).read_bytes()
    require(data[:8] == b"\x89PNG\r\n\x1a\n", "PNG signature required")
    pos, compressed, header, ended = 8, bytearray(), None, False
    while pos < len(data):
        require(pos + 12 <= len(data), "Truncated PNG chunk")
        size = struct.unpack(">I", data[pos:pos+4])[0]
        kind = data[pos+4:pos+8]
        body = data[pos+8:pos+8+size]
        require(pos + size + 12 <= len(data), "Truncated PNG payload")
        crc = struct.unpack(">I", data[pos+8+size:pos+12+size])[0]
        require(zlib.crc32(kind + body) & 0xffffffff == crc, "PNG CRC mismatch")
        if kind == b"IHDR":
            require(header is None and pos == 8 and size == 13, "Invalid PNG header")
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            require(header is not None, "PNG missing header")
            compressed.extend(body)
        elif kind == b"IEND":
            require(size == 0 and pos + 12 == len(data), "Invalid PNG end")
            ended = True
        else:
            require(kind[0] & 32, "Unsupported critical PNG chunk")
        pos += size + 12
    require(header is not None and ended, "Incomplete PNG")
    w, h, depth, color, compression, filtering, interlace = header
    require(0 < w*h <= 32_000_000 and (depth, color, compression, filtering, interlace) == (8, 0, 0, 0, 0),
            "Only bounded gray8 noninterlaced region PNG supported")
    decoder = zlib.decompressobj()
    raw = decoder.decompress(bytes(compressed), (w+1)*h + 1)
    require(len(raw) == (w+1)*h and decoder.eof and not decoder.unused_data,
            "PNG decompressed size mismatch")
    result = np.zeros((h, w), dtype=np.uint8)
    for y in range(h):
        mode = raw[y*(w+1)]
        require(mode <= 4, "Unknown PNG filter")
        for x in range(w):
            a = int(result[y, x-1]) if x else 0
            b = int(result[y-1, x]) if y else 0
            c = int(result[y-1, x-1]) if x and y else 0
            p = a+b-c
            distances = (abs(p-a), abs(p-b), abs(p-c))
            paeth = (a, b, c)[distances.index(min(distances))]
            predictor = (0, a, b, (a+b)//2, paeth)[mode]
            result[y, x] = (raw[y*(w+1)+1+x] + predictor) & 255
    require(np.isin(result, [0, 255]).all(), "Binary region mask required")
    return result != 0


def components(mask):
    labels = np.zeros(mask.shape, dtype=np.int32)
    groups = []
    h, w = mask.shape
    for y, x in zip(*np.nonzero(mask)):
        if labels[y, x]:
            continue
        label = len(groups)+1
        labels[y, x] = label
        stack, pixels = [(int(y), int(x))], []
        while stack:
            yy, xx = stack.pop()
            pixels.append((yy, xx))
            for ny in range(max(0, yy-1), min(h, yy+2)):
                for nx in range(max(0, xx-1), min(w, xx+2)):
                    if mask[ny, nx] and not labels[ny, nx]:
                        labels[ny, nx] = label
                        stack.append((ny, nx))
        groups.append(np.asarray(pixels))
    return labels, groups


def associate(report, prefix, dense, mask):
    require(report.get("status") == "completed" and report.get("inference_ran") is True,
            "Completed vocabulary inference required")
    require(report.get("box_convention") == "xyxy, exclusive right/bottom, EXIF-oriented original image pixels",
            "Unknown query pixel convention")
    size = report.get("image_size")
    require(type(size) is list and len(size) == 2 and
            all(type(v) is int and v > 0 for v in size), "Original image size required")
    small = report.get("input_size")
    require(type(small) is list and len(small) == 2 and
            all(type(v) is int and 0 < v <= size[i] for i, v in enumerate(small)),
            "Known resized inference dimensions required")
    dense = array(dense, (size[1], size[0])).astype(np.float32)
    require(np.isfinite(dense).all() and ((dense >= -1) & (dense <= 1)).all(), "Raw cosine out of range")
    require(isinstance(mask, np.ndarray) and mask.dtype == np.bool_ and mask.shape == dense.shape,
            "Full resolution binary mask required; no implicit resize")
    threshold = number(report.get("threshold"))
    require(-1 <= threshold <= 1, "Invalid raw cosine threshold")
    require(type(report.get("results")) is list, "Query results required")
    matches = [r for r in report["results"] if type(r) is dict and r.get("prefix") == prefix]
    require(len(matches) == 1, "Unique query prefix required")
    result = matches[0]
    fields(result, "query prefix raw_min raw_max regions")
    text(result["query"])
    require(type(result["regions"]) is list, "Region list required")
    for key, actual in (("raw_min", dense.min()), ("raw_max", dense.max())):
        require(abs(number(result[key])-float(actual)) <= 1e-6, "Raw cosine summary mismatch")
    require(not np.any(mask & ~(dense > threshold)), "Mask includes subthreshold pixels")
    labels, groups = components(dense > threshold)
    retained = []
    for i, pixels in enumerate(groups, 1):
        ys, xs = pixels.T
        kept = mask[ys, xs]
        require(not kept.any() or kept.all(), "Partial connected component in region mask")
        if kept.all():
            values = dense[ys, xs]
            retained.append((i, {"box_xyxy": [int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)],
                                 "area_pixels": len(pixels), "mean_cosine": float(values.mean()),
                                 "max_cosine": float(values.max())}))
    require(len(retained) == len(result["regions"]), "Region count mismatch")
    # Associate by geometry, not max-score ordering (OpenCV label order can differ).
    ordered = []
    previous = float("inf")
    for region in result["regions"]:
        fields(region, "box_xyxy area_pixels mean_cosine max_cosine")
        box = region["box_xyxy"]
        require(type(box) is list and len(box) == 4 and all(type(v) is int for v in box), "Integer xyxy required")
        require(type(region["area_pixels"]) is int, "Integer region area required")
        found = [(j, pair) for j, pair in enumerate(retained)
                 if pair[1]["box_xyxy"] == box and pair[1]["area_pixels"] == region["area_pixels"]
                 and all(abs(number(region[k])-pair[1][k]) <= 1e-6 for k in ("mean_cosine", "max_cosine"))]
        require(len(found) == 1, "Ambiguous or mismatched region association")
        j, (label, actual) = found[0]
        require(region["max_cosine"] <= previous, "Regions not sorted by maximum cosine")
        previous = region["max_cosine"]
        retained.pop(j)
        ordered.append({**actual, "mask": labels == label})
    return {"query": result["query"], "prefix": prefix, "image_size": size,
            "input_size": small, "threshold": threshold, "regions": ordered,
            "dense": dense, "mask": mask}


def load_query(result_path, prefix):
    require(type(prefix) is str and re.fullmatch(r"query_[0-9]{3,}", prefix) is not None,
            "Invalid vocabulary query prefix")
    path = Path(result_path)
    cosine = path.parent / (prefix + "_cosine.npy")
    mask = path.parent / (prefix + "_region_mask.png")
    query = associate(read_json(path), prefix, np.load(cosine, allow_pickle=False), read_mask_png(mask))
    query["artifact_sha256"] = {"result": sha256(path), "cosine": sha256(cosine), "mask": sha256(mask)}
    return query
