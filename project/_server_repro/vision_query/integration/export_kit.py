"""Reproducible allowlisted source-only ZIP for the integration driver."""
import hashlib
import json
import zipfile
from pathlib import Path

FILES = (
    "__init__.py", "common.py", "make_calibration.py", "build_frame.py",
    "capture_frames.py", "run_e2e.py", "test_integration.py", "export_kit.py",
    "README_验收.md",
)


def build():
    root = Path(__file__).resolve().parent
    sources = {name: (root / name).read_bytes() for name in sorted(FILES)}
    manifest = {"schema": "integration-source-package/1",
                "scope": "owned integration source only; no weights, upstream source or captures",
                "requires": ["target_localization (query_io, core)", "vocabulary_query/query.py"],
                "files": [{"path": name, "bytes": len(data),
                           "sha256": hashlib.sha256(data).hexdigest()}
                          for name, data in sources.items()]}
    manifest_bytes = (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode("utf-8")
    (root / "source_manifest.json").write_bytes(manifest_bytes)
    sources["source_manifest.json"] = manifest_bytes
    output = root / "integration_kit.zip"
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED) as archive:
        for name, data in sorted(sources.items()):
            info = zipfile.ZipInfo("integration/" + name, (1980, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    return {"zip": str(output), "source_files": len(FILES), "archive_entries": len(sources),
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest()}


if __name__ == "__main__":
    print(json.dumps(build(), indent=2))
