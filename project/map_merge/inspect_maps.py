"""Read-only BeamNG ZIP inspection; writes derived metadata only beside this script.

No extraction, installation, game launch, third-party libraries, or .ter decoding.
"""
import argparse
import collections
import hashlib
import io
import json
import math
import re
import stat
import statistics
import sys
import unittest
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

SOURCES = [Path(r"C:\Users\Yu\Desktop\Beamng\2K-Shomaru.zip"),
           Path(r"C:\Users\Yu\Desktop\Beamng\american_road_0.39.zip")]
OUT = Path(__file__).resolve().parent
MAX_MEMBER = 64 * 1024 * 1024
MAX_TOTAL = 384 * 1024 * 1024
REF = re.compile(r"(?i)(?:/?(?:levels|art|core|vehicles)/[^\s\"<>;]+|[a-z]:[\\/][^\s\"<>;]+)")


def safe_name(name):
    normalized = name.replace("\\", "/")
    return bool(normalized and not normalized.startswith("/") and
                not re.match(r"^[a-zA-Z]:", normalized) and
                not any(p in ("..", ".") for p in normalized.split("/")) and
                not any(ord(c) < 32 for c in normalized) and ":" not in normalized)


def distribution(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values:
        return {"count": 0}
    def percentile(p):
        i = (len(values) - 1) * p
        lo = int(i)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (i - lo)
    return {"count": len(values), "min": values[0], "p10": percentile(.1),
            "p50": percentile(.5), "p90": percentile(.9), "p95": percentile(.95),
            "max": values[-1], "mean": statistics.fmean(values)}


def parse_objects(text):
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, list) else [obj], "json"
    except (ValueError, RecursionError):
        # BeamNG metadata may use JSON comments and trailing commas. Preserve strings.
        cleaned = re.sub(r'"(?:\\.|[^"\\])*"|//[^\r\n]*|/\*[\s\S]*?\*/',
                         lambda m: m.group(0) if m.group(0).startswith('"') else ' ', text)
        cleaned = re.sub(r'("(?:\\.|[^"\\])*")|,\s*(?=[}\]])',
                         lambda m: m.group(1) or '', cleaned)
        try:
            obj = json.loads(cleaned)
            return obj if isinstance(obj, list) else [obj], "json_with_comments_or_trailing_commas"
        except (ValueError, RecursionError):
            pass
        result = []
        failures = 0
        for line in text.splitlines():
            if line.strip():
                try:
                    result.append(json.loads(line))
                except (ValueError, RecursionError):
                    failures += 1
        return result, "ndjson" if failures == 0 else "partial_ndjson:%d" % failures


def walk(value):
    if isinstance(value, dict):
        yield value
        for sub in value.values():
            yield from walk(sub)
    elif isinstance(value, list):
        for sub in value:
            yield from walk(sub)


def road_metrics(obj, source):
    nodes = obj.get("nodes", [])
    valid = []
    for node in nodes:
        if isinstance(node, list) and len(node) >= 4 and all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in node[:4]):
            valid.append(node[:4])
        else:
            valid.append(None)
    segments = []
    degenerate = 0
    for a, b in zip(valid, valid[1:]):
        if a is None or b is None:
            continue
        horizontal = math.hypot(b[0] - a[0], b[1] - a[1])
        if horizontal < .01:
            degenerate += 1
            continue
        dz = b[2] - a[2]
        segments.append({"horizontal_m": horizontal, "grade_abs_percent": abs(dz / horizontal) * 100,
                         "grade_signed_percent": dz / horizontal * 100,
                         "width_m": (a[3] + b[3]) / 2})
    usable = [n for n in valid if n is not None]
    total = sum(s["horizontal_m"] for s in segments)
    # Node-height grades are design hints, not measured draped terrain gradients.
    return {"source": source, "name": obj.get("name"), "persistentId": obj.get("persistentId"),
            "material": obj.get("Material", obj.get("material", obj.get("topMaterial"))),
            "position": obj.get("position"), "rotationMatrix": obj.get("rotationMatrix"),
            "scale": obj.get("scale"), "node_count": len(nodes), "valid_node_count": len(usable),
            "degenerate_segments": degenerate, "horizontal_length_m": total,
            "width_m": distribution([n[3] for n in usable]),
            "node_z": distribution([n[2] for n in usable]),
            "abs_node_grade_percent": distribution([s["grade_abs_percent"] for s in segments]),
            "length_weighted_abs_node_grade_percent":
                sum(s["horizontal_m"] * s["grade_abs_percent"] for s in segments) / total if total else None,
            "length_fraction_width_le_6m": sum(s["horizontal_m"] for s in segments if 0 < s["width_m"] <= 6) / total if total else None,
            "length_fraction_node_grade_3_to_15pct": sum(s["horizontal_m"] for s in segments if 3 <= s["grade_abs_percent"] <= 15) / total if total else None,
            "bounds_xyz": [[min(n[i] for n in usable), max(n[i] for n in usable)] for i in range(3)] if usable else None,
            "node_samples": [{"index": i, "node": valid[i]} for i in sorted(set(
                round(j * (len(valid) - 1) / min(15, len(valid) - 1)) for j in range(min(16, len(valid))))) ] if len(valid) > 1 else valid,
            "settings": {k: v for k, v in obj.items() if k not in ("nodes", "__parent")}}


def inspect(path):
    result = {"archive": str(path), "archive_bytes": path.stat().st_size,
              "archive_modified_ns": path.stat().st_mtime_ns, "errors": [], "skipped": [],
              "metadata": [], "terrain_files": [], "terrain_blocks": [], "roads": [],
              "materials": [], "forest_objects": [], "mesh_objects": [], "license_readme": [],
              "reference_resolution": {}, "class_counts": {}, "forest_data": [], "dae_metadata": []}
    refs = collections.defaultdict(set)
    classes = collections.Counter()
    parsed_bytes = 0
    with zipfile.ZipFile(path, "r") as archive:
        infos = archive.infolist()
        names = {i.filename for i in infos}
        folded = collections.defaultdict(list)
        for name in names:
            folded[name.casefold()].append(name)
        result["entry_count"] = len(infos)
        result["total_uncompressed_bytes"] = sum(i.file_size for i in infos)
        result["extension_counts"] = dict(collections.Counter(PurePosixPath(i.filename).suffix.lower() for i in infos if not i.is_dir()))
        result["level_ids"] = sorted({i.filename.split("/")[1] for i in infos if i.filename.startswith("levels/") and len(i.filename.split("/")) > 2})
        result["top_level_paths"] = sorted({i.filename.split("/")[0] for i in infos})
        result["duplicate_entries"] = [n for n, c in collections.Counter(i.filename for i in infos).items() if c > 1]
        result["case_collisions"] = [v for v in folded.values() if len(v) > 1]
        result["unsafe_entries"] = [i.filename for i in infos if not safe_name(i.filename)]
        result["inventory"] = [{"path": i.filename, "bytes": i.file_size, "compressed_bytes": i.compress_size,
                                "crc32": "%08x" % i.CRC} for i in infos if not i.is_dir()]
        for info in infos:
            name = info.filename
            lower = name.lower()
            if info.is_dir() or not safe_name(name):
                continue
            if stat.S_ISLNK(info.external_attr >> 16) or info.flag_bits & 1:
                result["skipped"].append({"path": name, "reason": "symlink_or_encrypted"})
                continue
            if lower.endswith(".ter"):
                with archive.open(info) as stream:
                    header = stream.read(128)
                result["terrain_files"].append({"path": name, "bytes": info.file_size,
                    "crc32_declared": "%08x" % info.CRC, "header_128_hex": header.hex(),
                    "decoded": False, "crc_verified": False})
                continue
            selected_dae = ("/2k_shomaru/art/shapes/" in lower and lower.endswith(".dae"))
            selected = (selected_dae or lower.endswith((".json", ".cs", ".lua", ".mis")) or
                        any(word in PurePosixPath(lower).name for word in ("readme", "license", "licence", "copyright", "credits")))
            if not selected:
                continue
            if info.file_size > MAX_MEMBER or parsed_bytes + info.file_size > MAX_TOTAL or info.file_size / max(1, info.compress_size) > 1000:
                result["skipped"].append({"path": name, "bytes": info.file_size, "reason": "metadata_safety_budget"})
                continue
            try:
                with archive.open(info) as stream:
                    data = stream.read(MAX_MEMBER + 1)
                if len(data) > MAX_MEMBER:
                    raise ValueError("metadata read limit")
                parsed_bytes += len(data)
                text = data.decode("utf-8-sig", errors="replace")
                record = {"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(), "crc_verified": True}
                if selected_dae:
                    if b"<!DOCTYPE" in data.upper() or b"<!ENTITY" in data.upper():
                        raise ValueError("XML DTD/entities not accepted")
                    dae = {"path": name, "geometry_ids": [], "scene_nodes": [], "material_bindings": [],
                           "image_refs": [], "triangle_count_declared": 0}
                    for _, element in ET.iterparse(io.BytesIO(data), events=("end",)):
                        tag = element.tag.rsplit("}", 1)[-1]
                        if tag == "geometry":
                            dae["geometry_ids"].append(dict(element.attrib))
                        elif tag == "node":
                            dae["scene_nodes"].append(dict(element.attrib))
                        elif tag == "instance_material":
                            dae["material_bindings"].append(dict(element.attrib))
                        elif tag == "init_from" and element.text:
                            dae["image_refs"].append(element.text)
                        elif tag == "triangles":
                            dae["triangle_count_declared"] += int(element.attrib.get("count", 0))
                        elif tag in ("unit", "up_axis"):
                            dae[tag] = dict(element.attrib) or element.text
                        element.clear()
                    result["dae_metadata"].append(dae)
                for match in REF.finditer(text):
                    refs[match.group(0).rstrip(",)'")].add(name)
                if any(word in PurePosixPath(lower).name for word in ("readme", "license", "licence", "copyright", "credits")):
                    result["license_readme"].append({"path": name, "text": text[:20000], "truncated": len(text) > 20000})
                if lower.endswith(".json"):
                    objects, mode = parse_objects(text)
                    record["parse_mode"] = mode
                    if PurePosixPath(lower).name in ("info.json", "main.level.json") or "groundmodel" in lower:
                        record["content"] = objects
                        record["text"] = text[:16000]
                    forest_count = 0
                    forest_types = collections.Counter()
                    for root in objects:
                        for obj in walk(root):
                            cls = obj.get("class")
                            if isinstance(cls, str):
                                classes[cls] += 1
                            if cls == "TerrainBlock":
                                result["terrain_blocks"].append({"source": name, "object": obj})
                            elif cls in ("DecalRoad", "MeshRoad"):
                                road = road_metrics(obj, name)
                                road["class"] = cls
                                result["roads"].append(road)
                            elif cls in ("TSStatic", "GroundCover"):
                                result["mesh_objects"].append({"source": name, "object": obj})
                            elif cls and "Forest" in cls:
                                result["forest_objects"].append({"source": name, "object": obj})
                            if cls in ("Material", "material", "TerrainMaterial") or "groundType" in obj or "friction" in obj:
                                result["materials"].append({"source": name, "object": obj})
                            if "/forest/" in lower and "type" in obj:
                                forest_count += 1
                                forest_types[str(obj["type"])] += 1
                    if "/forest/" in lower:
                        result["forest_data"].append({"path": name, "root_count": len(objects), "instance_count_with_type": forest_count,
                                                      "types": dict(forest_types), "sample": objects[:2]})
                result["metadata"].append(record)
            except (OSError, ValueError, RuntimeError, zipfile.BadZipFile, ET.ParseError) as error:
                result["errors"].append({"path": name, "error": str(error)})
        result["class_counts"] = dict(classes)
        result["metadata_bytes_read"] = parsed_bytes
        for ref, sources in sorted(refs.items()):
            normalized = ref.lstrip("/").replace("\\", "/")
            resolution = "in_archive" if normalized in names else "case_only_match" if normalized.casefold() in folded else "not_in_archive"
            if re.match(r"^[a-zA-Z]:", ref):
                resolution = "absolute_host_path"
            result["reference_resolution"][ref] = {"status": resolution, "sources": sorted(sources)}
    result["road_summary"] = {"road_count": len(result["roads"]),
        "total_horizontal_length_m": sum(r["horizontal_length_m"] for r in result["roads"]),
        "all_road_node_counts": distribution([r["node_count"] for r in result["roads"]]),
        "note": "Every valid node and adjacent segment in parsed roads is measured; only display samples are limited to 16. Node Z is not a terrain-surface measurement. Overlapping roads are not deduplicated."}
    return result


class Tests(unittest.TestCase):
    def test_generated_plan_matches_report(self):
        report_path = OUT / "archive_report.json"
        plan_path = OUT / "proposed_merge_plan.json"
        if not report_path.exists() or not plan_path.exists():
            self.skipTest("Assessment artifacts not generated yet")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
        self.assertIs(plan["ready"], False)
        for candidate in plan["route_candidates"]:
            archive = next(a for a in report["archives"] if candidate["level"] in a["level_ids"])
            road = next(r for r in archive["roads"] if r["persistentId"] == candidate["persistentId"])
            self.assertEqual(candidate["source"], road["source"])
            self.assertEqual(candidate["material"], road["material"])
            self.assertEqual(candidate["node_count"], road["node_count"])
            self.assertEqual(candidate["original_position"], road["position"])
            self.assertAlmostEqual(candidate["horizontal_chord_length_m"], road["horizontal_length_m"], places=5)
            for p in ("p50", "p90", "max"):
                self.assertAlmostEqual(candidate["absolute_node_grade_" + p + "_percent"], road["abs_node_grade_percent"][p], places=5)

    def test_paths(self):
        for name in ("../bad", "/bad", "C:/bad", "levels/../bad", "..\\bad", "a:b", "x\x00"):
            self.assertFalse(safe_name(name), name)
        self.assertTrue(safe_name("levels/test/info.json"))

    def test_json(self):
        self.assertEqual(parse_objects('{"a":1}\n{"a":2}')[1], "ndjson")
        self.assertEqual(parse_objects('{"a":1}\nbad')[1], "partial_ndjson:1")
        self.assertEqual(parse_objects('{"url":"http://a",/*test*/"b":2,}')[0][0]["b"], 2)

    def test_roads(self):
        road = road_metrics({"nodes": [[0, 0, 0, 4], [10, 0, 1, 6], [20, 0, 1, 8]]}, "test")
        self.assertEqual(road["horizontal_length_m"], 20)
        self.assertEqual(road["length_weighted_abs_node_grade_percent"], 5)
        self.assertEqual(road["length_fraction_width_le_6m"], .5)
        self.assertEqual(len(road["node_samples"]), 3)

    def test_invalid_gap(self):
        road = road_metrics({"nodes": [[0, 0, 0, 4], None, [20, 0, 1, 8]]}, "test")
        self.assertEqual(road["horizontal_length_m"], 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--details", action="store_true", help="Summarize the generated report without reopening source archives")
    parser.add_argument("--focus", action="store_true", help="Print compact evidence and candidate catalog")
    parser.add_argument("--verify-terrain", action="store_true", help="Stream terrain bytes for CRC and SHA256, without decoding or extraction")
    args = parser.parse_args()
    if args.test:
        unittest.main(argv=[sys.argv[0]])
        return
    if args.focus:
        report = json.loads((OUT / "archive_report.json").read_text(encoding="utf-8"))
        for a in report["archives"]:
            print(a["level_ids"])
            print("FOREST", sum(x["instance_count_with_type"] for x in a["forest_data"]), "files", sum(x["path"].endswith(".forest4.json") for x in a["forest_data"]))
            print("SAFETY", {k: a[k] for k in ("unsafe_entries", "duplicate_entries", "case_collisions", "errors", "skipped")})
            print("TERRAIN_MATERIALS", [(m["object"].get("internalName"), m["object"].get("groundmodelName")) for m in a["materials"] if m["object"].get("class") == "TerrainMaterial"])
            print("DAE", json.dumps(a["dae_metadata"], ensure_ascii=True))
            if "2k_shomaru" in a["level_ids"]:
                print("STATIC", json.dumps([x for x in a["mesh_objects"] if x["object"].get("class") == "TSStatic"]))
            print("RELEVANT_MATERIALS", json.dumps([m for m in a["materials"] if any(t in str(m).lower() for t in ("cobble", "groundmodel", "road_light_nolines", "brightg3", "mymap_concrete1"))], ensure_ascii=True))
            print("EXTERNAL", [k for k, v in a["reference_resolution"].items() if v["status"] == "absolute_host_path" or "2k_yabitsu" in k or (v["status"] != "in_archive" and "american_road" in a["level_ids"])])
            for cls in ("DecalRoad", "MeshRoad"):
                roads = [r for r in a["roads"] if r["class"] == cls]
                print(cls, "count", len(roads), "nodes", sum(r["node_count"] for r in roads), "length_m", sum(r["horizontal_length_m"] for r in roads))
                print("TOP_MATERIALS", collections.Counter(str(r["material"]) for r in roads).most_common(10))
            for r in a["roads"]:
                if (r["class"] == "DecalRoad" and r["horizontal_length_m"] >= 300 and
                    2.5 <= r["width_m"].get("p50", 0) <= 6 and
                    (r["material"] in ("road_light_nolines", "mymap_concrete1", "BrightG3", "DefaultDecalRoadMaterial"))):
                    print("ROUTE", json.dumps({k: r[k] for k in ("persistentId", "material", "position", "node_count", "horizontal_length_m", "width_m", "abs_node_grade_percent", "length_weighted_abs_node_grade_percent")}))
        return
    if args.details:
        report = json.loads((OUT / "archive_report.json").read_text(encoding="utf-8"))
        for a in report["archives"]:
            print("\nARCHIVE " + a["archive"])
            for key in ("license_readme", "forest_data"):
                value = a[key]
                print(key, json.dumps(value if key == "license_readme" else [{k: v for k, v in x.items() if k != "sample"} for x in value], ensure_ascii=True))
            print("INFO", json.dumps([m for m in a["metadata"] if m["path"].endswith("/info.json")], ensure_ascii=True))
            print("PARSE_ISSUES", json.dumps([m["path"] for m in a["metadata"] if m.get("parse_mode", "").startswith("partial")]))
            print("TERRAIN_MATERIALS", json.dumps([m for m in a["materials"] if m["object"].get("class") == "TerrainMaterial"], ensure_ascii=True))
            print("GROUND_TYPES", dict(collections.Counter(str(m["object"].get("groundType")) for m in a["materials"])))
            print("MESH_SHAPES", json.dumps(collections.Counter(str(m["object"].get("shapeName")) for m in a["mesh_objects"]), ensure_ascii=True))
            print("FOREST_OBJECTS", json.dumps([m for m in a["forest_objects"] if m["object"].get("class") == "Forest"], ensure_ascii=True))
            print("REF_STATUS", dict(collections.Counter(v["status"] for v in a["reference_resolution"].values())))
            external = {k: v for k, v in a["reference_resolution"].items() if v["status"] != "in_archive"}
            print("EXTERNAL_REF_PREFIXES", dict(collections.Counter('/'.join(k.split('/')[:4]) for k in external)))
            for cls in ("DecalRoad", "MeshRoad"):
                roads = [r for r in a["roads"] if r["class"] == cls]
                print(cls, "materials", dict(collections.Counter(str(r["material"]) for r in roads)))
                candidates = [r for r in roads if r["width_m"].get("p50", 0) >= 2.5 and r["width_m"].get("p50", 100) <= 7 and r["horizontal_length_m"] >= 150]
                candidates.sort(key=lambda r: r["horizontal_length_m"], reverse=True)
                print("CANDIDATES", json.dumps([{k: v for k, v in r.items() if k not in ("settings", "node_samples")} for r in candidates[:15]], ensure_ascii=True))
        return
    report = {"schema_version": 1, "ready": False,
              "method": {"read_only_sources": True, "no_extraction": True, "max_metadata_member_bytes": MAX_MEMBER,
                         "max_metadata_bytes_per_archive": MAX_TOTAL, "terrain_header_only": True,
                         "crc_scope": "Fully read selected metadata only; ZIP directory CRCs for all other members are declarations, not verification."},
              "archives": []}
    for source in SOURCES:
        print("Inspecting " + source.name, flush=True)
        report["archives"].append(inspect(source))
    if args.verify_terrain:
        report["method"]["terrain_header_only"] = False
        for a in report["archives"]:
            with zipfile.ZipFile(a["archive"], "r") as archive:
                for terrain in a["terrain_files"]:
                    if terrain["bytes"] > 600 * 1024 * 1024:
                        continue
                    digest = hashlib.sha256()
                    with archive.open(terrain["path"]) as stream:
                        while True:
                            chunk = stream.read(1024 * 1024)
                            if not chunk:
                                break
                            digest.update(chunk)
                    terrain.update(crc_verified=True, sha256=digest.hexdigest())
        report["method"]["crc_scope"] = "Selected metadata plus fully streamed terrain files; all other CRCs are directory declarations."
    for a in report["archives"]:
        current = Path(a["archive"]).stat()
        a["source_stat_unchanged"] = current.st_size == a["archive_bytes"] and current.st_mtime_ns == a["archive_modified_ns"]
    target = OUT / "archive_report.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for item in report["archives"]:
        summary = {k: item[k] for k in ("archive", "archive_bytes", "entry_count", "level_ids", "terrain_files", "terrain_blocks", "class_counts", "road_summary", "metadata_bytes_read", "skipped", "errors")}
        summary["info"] = [m for m in item["metadata"] if PurePosixPath(m["path"]).name == "info.json"]
        print(json.dumps(summary, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
