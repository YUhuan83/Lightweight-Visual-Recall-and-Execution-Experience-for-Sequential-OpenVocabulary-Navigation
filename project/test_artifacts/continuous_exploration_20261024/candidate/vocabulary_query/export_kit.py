"""Package portable entry code and shared manifest, not weights or local deps."""
import argparse
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "vocabulary_query_kit.zip")
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(HERE) or not output.name.endswith(".zip"):
        parser.error("Output must be a .zip file under vocabulary_query")
    output.parent.mkdir(parents=True, exist_ok=True)
    names = ["query.py", "explorfm_heads.py", "manifest.json", "requirements.txt", "requirements-overlay.txt",
             "test_postprocess.py", "test_explorfm_heads.py", "run_linux.sh", "export_kit.py", "README.md"]
    if output.exists():
        output.unlink()
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            archive.write(HERE / name, "vocabulary_query/" + name)
    print(output)


if __name__ == "__main__":
    main()
