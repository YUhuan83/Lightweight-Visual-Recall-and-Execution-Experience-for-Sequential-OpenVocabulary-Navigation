"""Resolve explicit deployment paths without importing ROS or model libraries."""
from pathlib import Path


def model_paths(config, triangulation=False):
    root = Path(config.get("ckpt_root", Path.home() / "ckpts")).expanduser()
    version = config["model_version"]
    model = root / version if triangulation else version
    adaptor = config.get("adaptor_ckpt_path", root / config["adaptor_version"]
                         if triangulation else root)
    adaptor = Path(adaptor).expanduser()
    if config.get("local_files_only", False):
        model = Path(version).expanduser()
        if not model.is_absolute():
            model = root / model
        if not model.is_file():
            raise FileNotFoundError(f"Local RADIO checkpoint required: {model}")
        if config.get("adaptor_version") is not None:
            candidates = (adaptor, adaptor / "siglip2_ms", Path(f"{adaptor}_ms"))
            adaptor = next((p for p in candidates if (p / "config.json").is_file()), None)
            if adaptor is None:
                raise FileNotFoundError("Local SigLIP2 config.json required; no network fallback")
    return root, model, adaptor
