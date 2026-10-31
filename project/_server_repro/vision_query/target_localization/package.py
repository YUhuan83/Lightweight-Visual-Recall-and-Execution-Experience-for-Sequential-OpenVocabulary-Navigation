"""Reproducible allowlisted source-only ZIP; no model, upstream source or captures."""

import hashlib
import json
from pathlib import Path
import zipfile


FILES = (
    "__init__.py", "__main__.py", "core.py", "query_io.py", "package.py",
    "README_zh.md", "CONTRACT.md", "requirements.txt", "manifest.draft.json",
    "calibration.draft.json", "tests/__init__.py", "tests/test_localization.py",
)


def build():
    root = Path(__file__).resolve().parent
    sources = {name: (root / name).read_bytes() for name in sorted(FILES)}
    manifest = {"schema": "target-source-package/1", "scope": "owned source only; no weights or upstream source",
                "files": [{"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
                          for name, data in sources.items()]}
    manifest_bytes = (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode("utf-8")
    (root / "source_manifest.json").write_bytes(manifest_bytes)
    sources["source_manifest.json"] = manifest_bytes
    output = root / "target_localization_kit.zip"
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED) as archive:
        for name, data in sorted(sources.items()):
            info = zipfile.ZipInfo("target_localization/" + name, (1980, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    return {"zip": str(output), "source_files": len(FILES), "archive_entries": len(sources),
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest()}


if __name__ == "__main__":
    print(json.dumps(build(), indent=2))
