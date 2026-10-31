"""Reproducible allowlisted source-only ZIP for the object_search module."""
import hashlib
import json
import zipfile
from pathlib import Path

FILES = (
    "__init__.py", "triangulation.py", "run_object_search.py",
    "test_triangulation.py", "export_kit.py", "README_验收.md",
)


def build():
    root = Path(__file__).resolve().parent
    sources = {name: (root / name).read_bytes() for name in sorted(FILES)}
    manifest = {"schema": "object-search-source-package/1",
                "scope": "owned source only; no weights, upstream source or captures",
                "reference": "wildos_port triangulation3d + obj_mask_triangulation (ROS-free port)",
                "files": [{"path": name, "bytes": len(data),
                           "sha256": hashlib.sha256(data).hexdigest()}
                          for name, data in sources.items()]}
    (root / "source_manifest.json").write_bytes(
        (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode("utf-8"))
    sources["source_manifest.json"] = (root / "source_manifest.json").read_bytes()
    output = root / "object_search_kit.zip"
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED) as archive:
        for name, data in sorted(sources.items()):
            info = zipfile.ZipInfo("object_search/" + name, (1980, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    return {"zip": str(output), "source_files": len(FILES), "archive_entries": len(sources),
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest()}


if __name__ == "__main__":
    print(json.dumps(build(), indent=2))
