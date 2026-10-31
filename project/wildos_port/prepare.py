"""Read-only working-tree export. No installs, model imports, or weight access."""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess

ROOT = Path(__file__).resolve().parent
DEFAULT_DEST = ROOT / "workspace/src/nebula2-wildos"
SELECT = {"explorfm", "nvidia_radio", "visual_navigation", "triangulation3d",
          "graphnav_mapper", "graphnav_planner", "graphnav_msgs", "pose_graph_msgs",
          "object_search_msgs", "gps_visualization", "thirdparty"}
ROOT_FILES = {"LICENSE", "README.md", "requirements.txt", ".gitattributes", ".gitignore"}
EXCLUDE = {".git", ".venv", "venv", "env", "ckpts", "checkpoints", "weights",
           "cache", ".cache", "__pycache__", ".pytest_cache", "build", "install",
           "log", "logs", "node_modules", "assets", "notebooks", ".mypy_cache"}
SUFFIXES = {".py", ".xml", ".txt", ".md", ".yaml", ".yml", ".json", ".cfg",
            ".hpp", ".h", ".cpp", ".cc", ".c", ".cmake", ".msg", ".srv",
            ".action", ".sh", ".toml", ".ini"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def safe_path(path):
    path = Path(os.path.abspath(path))
    for item in (path, *path.parents):
        if item.is_symlink() or (item.exists() and
                getattr(item.lstat(), "st_file_attributes", 0) & 0x400):
            raise ValueError(f"Symlink/reparse point forbidden: {item}")
    return path


def git(source, *args):
    return subprocess.check_output(
        ["git", "--no-optional-locks", "-C", str(source), *args],
        env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"}).decode("utf-8")


def snapshot(source):
    files, skipped = {}, []
    def visit(folder):
        for path in sorted(folder.iterdir()):
            rel = path.relative_to(source).as_posix()
            info = path.lstat()
            if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
                skipped.append([rel, "link/reparse"])
                continue
            if path.name.lower() in EXCLUDE:
                skipped.append([rel, "excluded directory/file"])
                continue
            if stat.S_ISDIR(info.st_mode):
                visit(path)
            elif stat.S_ISREG(info.st_mode):
                if (path.suffix.lower() in SUFFIXES or path.name in ROOT_FILES or
                        path.name == "LICENSE" or path.parent.name == "resource"):
                    if info.st_size > 8 * 1024 * 1024:
                        raise ValueError(f"Oversize selected source file: {rel}")
                    files[rel] = {"sha256": digest(path.read_bytes()),
                                  "mode": stat.S_IMODE(info.st_mode)}
                else:
                    skipped.append([rel, "not selected source type"])
    for path in sorted(source.iterdir()):
        if path.name in SELECT:
            safe_path(path)
            visit(path)
        elif path.name in ROOT_FILES:
            safe_path(path)
            files[path.name] = {"sha256": digest(path.read_bytes()),
                                "mode": stat.S_IMODE(path.stat().st_mode)}
        else:
            skipped.append([path.name, "not selected top level"])
    return dict(sorted(files.items())), skipped


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def verify_source(source, manifest):
    files, _ = snapshot(source)
    if files != manifest["source_files"]:
        raise ValueError("Source content/mode/selection changed")
    if git(source, "rev-parse", "HEAD").strip() != manifest["source_head"]:
        raise ValueError("Source HEAD changed")
    if git(source, "status", "--porcelain=v1", "--untracked-files=all") != manifest["source_status"]:
        raise ValueError("Source git status changed")


def prepare(source, dest, pristine=False):
    source, dest = safe_path(source), safe_path(dest)
    if not dest.is_relative_to(ROOT) or dest == ROOT or source.is_relative_to(dest) or dest.is_relative_to(source):
        raise ValueError("Destination must be inside wildos_port and disjoint from source")
    if dest.exists():
        raise ValueError("Destination exists; refusing to overwrite. Use a new --dest.")
    provenance = safe_path(dest.parent / (dest.name + ".provenance.json"))
    if provenance.exists():
        raise ValueError("Provenance already exists; refusing to overwrite")
    head = git(source, "rev-parse", "HEAD").strip()
    status = git(source, "status", "--porcelain=v1", "--untracked-files=all")
    files, skipped = snapshot(source)
    manifest = {"source": str(source), "source_head": head, "source_status": status,
                "source_dirty": bool(status), "source_files": files, "skipped": skipped}
    overlay_manifest = ROOT / "patches/manifest.json"
    patches = [] if pristine else json.loads(overlay_manifest.read_text(encoding="utf-8"))["files"]
    for item in patches:
        rel = item["path"]
        if Path(rel).is_absolute() or ".." in Path(rel).parts:
            raise ValueError("Unsafe patch path")
        if files.get(rel, {}).get("sha256") != item["before"]:
            raise ValueError(f"Patch base mismatch: {rel}")
        overlay = safe_path(ROOT / "patches/overlay" / rel)
        if digest(overlay.read_bytes()) != item["after"]:
            raise ValueError(f"Overlay hash mismatch: {rel}")
    dest.mkdir(parents=True)
    for rel, info in files.items():
        data = safe_path(source / rel).read_bytes()
        if digest(data) != info["sha256"]:
            raise ValueError(f"Source changed during copy: {rel}")
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        target.chmod(info["mode"])
    for item in patches:
        target = dest / item["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / "patches/overlay" / item["path"]).read_bytes())
    verify_source(source, manifest)
    manifest["output_files"] = {p.relative_to(dest).as_posix(): digest(p.read_bytes())
                                for p in sorted(dest.rglob("*")) if p.is_file()}
    manifest["patched"] = not pristine
    write_json(provenance, manifest)
    print(f"Exported {len(files)} source files; dirty={bool(status)}; patches={len(patches)}")
    return manifest


def seal(source, dest):
    manifest_path = dest.parent / (dest.name + ".provenance.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verify_source(source, manifest)
    patch_dir = safe_path(ROOT / "patches")
    if patch_dir.exists():
        raise ValueError("Patches already sealed")
    patch_dir.mkdir()
    patches, diffs = [], []
    for path in sorted(dest.rglob("*")):
        safe_path(path)
        if not path.is_file():
            continue
        rel = path.relative_to(dest).as_posix()
        data = path.read_bytes()
        before = manifest["source_files"].get(rel, {}).get("sha256")
        if digest(data) == before:
            continue
        overlay = patch_dir / "overlay" / rel
        overlay.parent.mkdir(parents=True, exist_ok=True)
        overlay.write_bytes(data)
        old = (source / rel).read_bytes() if before else b""
        diffs.extend(difflib.unified_diff(old.decode("utf-8").splitlines(True),
                     data.decode("utf-8").splitlines(True), fromfile="a/" + rel, tofile="b/" + rel))
        patches.append({"path": rel, "before": before, "after": digest(data)})
    if any(not (dest / rel).is_file() for rel in manifest["source_files"]):
        raise ValueError("Deleted source files are not supported")
    write_json(patch_dir / "manifest.json", {"source_head": manifest["source_head"], "files": patches})
    (patch_dir / "isolated.patch").write_text("".join(diffs), encoding="utf-8")
    manifest["output_files"] = {p.relative_to(dest).as_posix(): digest(p.read_bytes())
                                for p in sorted(dest.rglob("*")) if p.is_file()}
    manifest["patched"] = True
    write_json(manifest_path, manifest)
    print(f"Sealed {len(patches)} patched/new files; source unchanged")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--dest", type=Path, default=DEFAULT_DEST)
    parser.add_argument("--action", choices=("prepare", "seal", "verify"), default="prepare")
    parser.add_argument("--pristine", action="store_true", help="Initial export before manual apply_patch edits")
    args = parser.parse_args()
    source, dest = safe_path(args.source), safe_path(args.dest)
    if not dest.is_relative_to(ROOT) or dest == ROOT or dest.is_relative_to(source) or source.is_relative_to(dest):
        raise ValueError("Unsafe source/destination")
    if args.action == "prepare":
        prepare(source, dest, args.pristine)
    elif args.action == "seal":
        seal(source, dest)
    else:
        manifest = json.loads((dest.parent / (dest.name + ".provenance.json")).read_text(encoding="utf-8"))
        verify_source(source, manifest)
        actual = {}
        for path in dest.rglob("*"):
            safe_path(path)
            if path.is_file():
                actual[path.relative_to(dest).as_posix()] = digest(path.read_bytes())
        if actual != manifest["output_files"]:
            raise ValueError("Export content changed")
        print(f"Source and export hashes verified; source_dirty={manifest['source_dirty']}")


if __name__ == "__main__":
    main()
