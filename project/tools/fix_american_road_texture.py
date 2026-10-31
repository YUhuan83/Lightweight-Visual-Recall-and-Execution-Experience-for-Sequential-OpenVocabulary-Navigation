#!/usr/bin/env python3
"""Offline, opt-in American Road texture path repair. Never installs a mod.

Dry-run is the default. --output must name a nonexistent, separate ZIP.
Only the exact stale wood-grunge B path in .json/.cs files is replaced.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import zipfile


OLD = b"levels/east_coast_usa/art/shapes/buildings/eca/eca_grungewood_b.dds"
NEW = b"levels/american_road/art/shapes/buildings/eca/eca_grungewood_b.dds"
MAX_CONFIG = 16 * 1024 * 1024
GRID = re.compile(rb"Grid512_Red_Mat|512_red", re.IGNORECASE)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_config(archive, info):
    if info.file_size > MAX_CONFIG:
        raise ValueError(f"Config exceeds safety limit ({MAX_CONFIG} bytes): {info.filename}")
    with archive.open(info) as stream:
        data = stream.read(MAX_CONFIG + 1)
    if len(data) > MAX_CONFIG:
        raise ValueError(f"Config exceeds safety limit: {info.filename}")
    return data


def report_context(data, pattern, limit=None):
    lines = data.decode("utf-8", errors="replace").splitlines()
    matches = [number for number, line in enumerate(lines)
               if pattern.search(line.encode("utf-8"))]
    if limit is not None:
        print("  All matching lines: " + ", ".join(str(number + 1) for number in matches))
        print(f"  Nearby context for first {limit} matching lines:")
    for number, line in enumerate(lines):
        if number in (matches if limit is None else matches[:limit]):
            print(f"  match line {number + 1}:")
            for index in range(max(0, number - 2), min(len(lines), number + 3)):
                print(f"    {index + 1}: {lines[index]}")


def report_materials(data):
    # Parsing is for reporting only; the file itself is always edited as bytes.
    try:
        value = json.loads(data)
    except (ValueError, UnicodeError):
        declarations = re.findall(
            rb"(?:singleton|new)\s+Material\s*\(\s*([^)]*)\)", data
        )
        if declarations:
            print("  CS material declarations (file scope): " + ", ".join(
                item.decode("utf-8", errors="replace") for item in declarations
            ))
        else:
            print("  Material name: see source context (non-standard JSON/CS).")
        return
    if isinstance(value, dict):
        for name, material in value.items():
            if OLD.decode("ascii") in json.dumps(material, ensure_ascii=True):
                print(f"  material: {name}")


def patch_archive(source, output, apply=False):
    source = Path(source).expanduser().resolve(strict=True)
    output = Path(output).expanduser().absolute()
    if source == output.resolve() or os.path.lexists(output):
        raise ValueError("Output must be a new, separate path; existing outputs are never overwritten.")
    if not output.parent.is_dir():
        raise ValueError(f"Output parent directory does not exist: {output.parent}")
    if output.suffix.lower() != ".zip":
        raise ValueError("Output must have a .zip extension.")

    print(f"Mode: {'APPLY' if apply else 'DRY-RUN (no archive written)'}")
    print(f"Source: {source}")
    print(f"Output: {output}")
    print(f"Exact old path: {OLD.decode('ascii')}")
    print(f"Exact new path: {NEW.decode('ascii')}")
    with zipfile.ZipFile(source) as original:
        infos = original.infolist()
        names = [info.filename for info in infos]
        if len(set(names)) != len(names):
            raise ValueError("Duplicate ZIP member names; refusing an ambiguous archive.")
        target = NEW.decode("ascii")
        if target not in names or original.getinfo(target).is_dir():
            raise ValueError(f"Corrected texture target is missing: {target}")
        # Read the texture without extracting it, checking its decompression and CRC.
        target_hash = hashlib.sha256()
        with original.open(target) as stream:
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                target_hash.update(chunk)
        print(f"Validated target: {target}; SHA256={target_hash.hexdigest()}")

        changes = {}
        replacements = 0
        grid_files = 0
        for info in infos:
            if info.is_dir() or Path(info.filename).suffix.lower() not in (".json", ".cs"):
                continue
            if info.file_size > MAX_CONFIG:
                # Large forest/scene JSON need not be loaded to rule out matches.
                tail = b""
                with original.open(info) as stream:
                    while True:
                        chunk = stream.read(1024 * 1024)
                        if not chunk:
                            break
                        window = tail + chunk
                        if OLD in window or GRID.search(window):
                            raise ValueError(f"Matching config exceeds safety limit: {info.filename}")
                        tail = window[-max(len(OLD), 32):]
                continue
            data = read_config(original, info)
            count = data.count(OLD)
            if not count and b"eca_grungewood_b" in data:
                print(f"REPORT ONLY existing wood-grunge B member: {info.filename}")
                report_context(data, re.compile(rb"eca_grungewood_b|eca_bld_woodcladding_01_bare_mat"))
            if count:
                updated = data.replace(OLD, NEW)
                changes[info.filename] = (digest(data), digest(updated), count)
                replacements += count
                print(f"PATCH member: {info.filename}; replacements={count}")
                report_materials(data)
                report_context(data, re.compile(re.escape(OLD)))
                print(f"  SHA256 before={digest(data)} after={digest(updated)}")
            if GRID.search(data):
                grid_files += 1
                print(f"REPORT ONLY Grid512/512_red member: {info.filename}; "
                      f"occurrences={len(GRID.findall(data))}")
                report_context(data, GRID, limit=3)

        grid_assets = [name for name in names if GRID.search(name.encode("utf-8"))]
        for name in grid_assets:
            print(f"REPORT ONLY Grid512/512_red asset/member: {name}")
        print(f"Summary: {replacements} exact replacements in {len(changes)} configs; "
              f"Grid512/512_red references in {grid_files} configs, "
              f"{len(grid_assets)} matching member names.")
        if not changes:
            print("No stale exact path found; no output created.")
            return 0
        if not apply:
            print("Dry-run complete. Use --apply to create the separate patched ZIP.")
            return replacements

        created = False
        try:
            # Exclusive creation also protects against a race after preflight.
            with output.open("xb") as raw_output:
                created = True
                with zipfile.ZipFile(raw_output, "w", allowZip64=True) as patched:
                    patched.comment = original.comment
                    for info in infos:
                        metadata = copy.copy(info)
                        if info.filename in changes:
                            data = read_config(original, info)
                            if digest(data) != changes[info.filename][0]:
                                raise ValueError(f"Source changed during processing: {info.filename}")
                            patched.writestr(metadata, data.replace(OLD, NEW))
                        else:
                            with original.open(info) as reader, patched.open(metadata, "w") as writer:
                                shutil.copyfileobj(reader, writer, length=1024 * 1024)

            # Verify every rebuilt member without loading large assets into memory.
            with zipfile.ZipFile(output) as patched:
                if patched.namelist() != names or patched.comment != original.comment:
                    raise ValueError("Output ZIP directory/comment verification failed.")
                for info in infos:
                    result = hashlib.sha256()
                    with patched.open(info.filename) as stream:
                        while True:
                            chunk = stream.read(1024 * 1024)
                            if not chunk:
                                break
                            result.update(chunk)
                    actual = patched.getinfo(info.filename)
                    if info.filename in changes:
                        if result.hexdigest() != changes[info.filename][1]:
                            raise ValueError(f"Patched member hash mismatch: {info.filename}")
                    elif (actual.CRC, actual.file_size) != (info.CRC, info.file_size):
                        raise ValueError(f"Unchanged member CRC/size mismatch: {info.filename}")
            print(f"Created and CRC-verified: {output}")
            print("Original was opened read-only. No mod was installed or merged.")
        except BaseException:
            if created:
                output.unlink()
            raise
    return replacements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path,
                        default=Path("~/beamng-maps/american_road_0.39.zip"))
    parser.add_argument("--output", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Inspect only (default)")
    mode.add_argument("--apply", action="store_true", help="Create and verify a new patched ZIP")
    args = parser.parse_args()
    try:
        patch_archive(args.archive, args.output, apply=args.apply)
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile, NotImplementedError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
