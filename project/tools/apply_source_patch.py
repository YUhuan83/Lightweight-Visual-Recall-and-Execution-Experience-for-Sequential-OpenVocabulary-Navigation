"""Apply/verify the required WildOS source patch (offline SigLIP2 local_files_only).

query.py refuses any source-root whose nvidia_radio/radio/siglip2_adaptor.py does
not guard AutoModel/AutoProcessor.from_pretrained with local_files_only=True.
This tool applies that patch idempotently and verifies it with the same AST check,
so a clean checkout becomes reproducible.

    python tools/apply_source_patch.py --source C:\\Users\\Yu\\nebula2-wildos
    python tools/apply_source_patch.py --source ... --check
"""
import argparse
import ast
import sys
from pathlib import Path

REL = Path("nvidia_radio/radio/siglip2_adaptor.py")
REPLACEMENTS = [
    ("AutoModel.from_pretrained(str(ms_dir), trust_remote_code=True)",
     "AutoModel.from_pretrained(str(ms_dir), trust_remote_code=True, local_files_only=True)"),
    ("AutoProcessor.from_pretrained(str(ms_dir), trust_remote_code=True)",
     "AutoProcessor.from_pretrained(str(ms_dir), trust_remote_code=True, local_files_only=True)"),
    ("AutoModel.from_pretrained(version, trust_remote_code=True, cache_dir=Path(ckpt_path))",
     "AutoModel.from_pretrained(version, trust_remote_code=True, cache_dir=Path(ckpt_path), local_files_only=True)"),
    ("AutoProcessor.from_pretrained(version, trust_remote_code=True, cache_dir=Path(ckpt_path))",
     "AutoProcessor.from_pretrained(version, trust_remote_code=True, cache_dir=Path(ckpt_path), local_files_only=True)"),
]


def guarded_api(path):
    """Mirror query.py's check: {AutoModel, AutoProcessor} must be guarded."""
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    guarded = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "from_pretrained":
            if any(k.arg == "local_files_only" and isinstance(k.value, ast.Constant)
                   and k.value.value is True for k in node.keywords):
                guarded.add(ast.unparse(node.func.value))
    return guarded


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--check", action="store_true", help="Verify only; do not modify")
    a = p.parse_args(argv)

    path = a.source / REL
    if not path.is_file():
        print("MISSING", path)
        return 2
    text = path.read_text(encoding="utf-8")
    applied = 0
    for old, new in REPLACEMENTS:
        if new in text:
            continue
        if old in text:
            text = text.replace(old, new)
            applied += 1
    if applied and not a.check:
        path.write_text(text, encoding="utf-8")
    guarded = guarded_api(path)
    ok = {"AutoModel", "AutoProcessor"}.issubset(guarded)
    print("applied=%d guarded=%s ok=%s" % (applied, sorted(guarded), ok))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
