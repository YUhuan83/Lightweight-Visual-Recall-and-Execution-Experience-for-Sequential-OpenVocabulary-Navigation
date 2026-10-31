"""Run from the repository root: python -B -m algorithm_mode.validate_config."""

import argparse
import json
from pathlib import Path

from .contract import Invalid, load_config, validate_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default=str(Path(__file__).with_name("config_template.json")))
    parser.add_argument("--require-ready", action="store_true")
    args = parser.parse_args()
    try:
        print(json.dumps(validate_config(load_config(args.path), ready=args.require_ready)))
    except (Invalid, ValueError, OSError) as exc:
        parser.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    main()
