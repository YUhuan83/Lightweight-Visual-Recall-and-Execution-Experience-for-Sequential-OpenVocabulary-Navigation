"""Offline ExploRFM traversability and visual-frontier head inference.

Companion to query.py: same offline/network guarantees and model wiring, but loads
frontier_head.ckpt and trav_head.ckpt and emits the two dense maps that WildOS uses
for navigation-graph scoring. Object-similarity querying stays in query.py.

    python -B explorfm_heads.py --image IMG --checkpoint-root CKPT --source-root SRC \
        --output OUT [--max-side 640] [--selfcheck]
"""
import argparse
import contextlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import traceback

for _key in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE", "HF_HUB_DISABLE_TELEMETRY"):
    os.environ[_key] = "1"
sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent


@contextlib.contextmanager
def tolerant_torch_load():
    """Load checkpoints whose pickle references omegaconf even if it is not installed.

    frontier_head.ckpt stores an OmegaConf config object alongside 'state_dict'. Only the
    tensors are needed, so unpickling maps omegaconf.* classes to an inert stub. If
    omegaconf is installed this is a no-op.
    """
    if importlib.util.find_spec("omegaconf") is not None:
        yield
        return
    import torch

    original = torch.load
    _, shim = make_stub_unpickler()

    def patched(stream, *args, **kwargs):
        if kwargs.get("weights_only", None) is False and "pickle_module" not in kwargs:
            kwargs["pickle_module"] = shim
        return original(stream, *args, **kwargs)

    torch.load = patched
    try:
        yield
    finally:
        torch.load = original


def make_stub_unpickler():
    """Return (Unpickler, shim) that turn omegaconf.* classes into an inert stub."""
    import pickle

    class _Stub:
        def __init__(self, *unused_args, **unused_kwargs):
            pass

        def __setstate__(self, state):
            if isinstance(state, dict):
                self.__dict__.update(state)

    class _Unpickler(pickle.Unpickler):
        def find_class(self, module, name):
            if module.split(".")[0] == "omegaconf":
                return _Stub
            return super().find_class(module, name)

    class _Shim:
        Unpickler = _Unpickler

        @staticmethod
        def load(stream, **kwargs):
            return _Unpickler(stream, **kwargs).load()

    return _Unpickler, _Shim


def load_query_module():
    spec = importlib.util.spec_from_file_location("vc_query", HERE / "query.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--checkpoint-root", required=True, type=Path)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New, empty directory under vocabulary_query")
    parser.add_argument("--deps", type=Path)
    parser.add_argument("--max-side", type=int, default=640)
    parser.add_argument("--heatmap", choices=("relative", "fixed"), default="relative")
    parser.add_argument("--smooth", action="store_true")
    parser.add_argument("--selfcheck", action="store_true", help="Check files/deps only; no torch import")
    args = parser.parse_args(argv)

    out = args.output.resolve()
    if not out.is_relative_to(HERE) or out == HERE:
        parser.error("--output must be a child directory of vocabulary_query")
    if out.exists() and any(out.iterdir()):
        parser.error("--output must be empty (prevents stale results)")
    out.mkdir(parents=True, exist_ok=True)

    root = args.checkpoint_root.resolve()
    source = args.source_root.resolve()
    heads = {"frontier_head.ckpt": root / "frontier_head.ckpt", "trav_head.ckpt": root / "trav_head.ckpt"}
    report = {"status": "started", "inference_ran": False, "offline": True,
              "scope": "ExploRFM traversability and visual-frontier dense maps; not navigation",
              "head_files": {name: {"present": path.is_file(), "bytes": path.stat().st_size if path.is_file() else None}
                             for name, path in heads.items()}}
    try:
        missing = [str(path) for path in heads.values() if not path.is_file()]
        if missing:
            raise FileNotFoundError("Missing head checkpoints:\n" + "\n".join(missing))
        if args.deps:
            deps = args.deps.resolve()
            if not deps.is_dir():
                raise FileNotFoundError(deps)
            if any((deps / n).exists() for n in ("torch", "torchvision")):
                raise RuntimeError("Overlay must not shadow the existing CUDA torch/torchvision")
            sys.path.insert(0, str(deps))
        if args.selfcheck:
            report["status"] = "selfcheck_passed"
            return 0
        if not args.image:
            parser.error("inference requires --image")

        import socket
        def deny_network(*unused_args, **unused_kwargs):
            raise RuntimeError("Network access is disabled in this local-only inference process")
        socket.socket.connect = deny_network
        socket.socket.connect_ex = deny_network
        socket.create_connection = deny_network

        import numpy as np
        import torch
        from PIL import Image, ImageOps
        query = load_query_module()
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is required by official NA-CLIP; no CPU/dummy fallback")
        free, total = torch.cuda.mem_get_info()
        report["memory"] = {"free_vram_bytes": free, "total_vram_bytes": total}
        if free < 5 * 2**30:
            raise RuntimeError("Refusing model load: need >=5 GiB free VRAM")
        torch.cuda.set_per_process_memory_fraction(min(0.80, (free - 1024**3) / total))

        sys.path.insert(0, str(source))
        from explorfm.explorfm_model import ExploRFMInference

        with Image.open(args.image) as opened:
            image = ImageOps.exif_transpose(opened).convert("RGB")
        original_size = image.size
        small = image.copy()
        small.thumbnail((args.max_side, args.max_side), Image.Resampling.BILINEAR)
        report.update(image=str(args.image.resolve()), image_size=list(original_size), input_size=list(small.size))

        model = None
        with tolerant_torch_load():
            model = ExploRFMInference(
                frontier_ckpt=str(heads["frontier_head.ckpt"]),
                traversability_ckpt=str(heads["trav_head.ckpt"]),
                model_version=str(root / "c-radio_v3-b_half.pth.tar"), adaptor_version="siglip2",
                adaptor_ckpt_path=str(root / "siglip2_ms"), use_naclip=True,
                use_summary_for_spatial=True, static_scale_factor=1.0,
                model_precision="FP16", device="cuda")
        with torch.inference_mode():
            traversability, frontiers, _ = model.forward_on_numpy(np.asarray(small))
        if traversability is None or frontiers is None:
            raise RuntimeError("ExploRFM did not return traversability/frontier maps")
        trav = traversability[0, 0].float().cpu().numpy()
        front = frontiers[0, 0].float().cpu().numpy()
        report.update(inference_ran=True, traversability_shape=list(trav.shape),
                      frontier_shape=list(front.shape),
                      traversability_range=[float(trav.min()), float(trav.max())],
                      frontier_range=[float(front.min()), float(front.max())])
        np.save(out / "traversability.npy", trav, allow_pickle=False)
        np.save(out / "frontier.npy", front, allow_pickle=False)
        _, _, trav_heat = query.heatmap_colors(trav, args.heatmap, smooth=args.smooth)
        _, _, front_heat = query.heatmap_colors(front, args.heatmap, smooth=args.smooth)
        trav_heat.save(out / "traversability_heatmap.png")
        front_heat.save(out / "frontier_heatmap.png")
        for name, score in (("traversability", trav), ("frontier", front)):
            blend = Image.blend(small, query.heatmap_colors(score, args.heatmap, smooth=args.smooth)[2], 0.45)
            blend.save(out / ("%s_blend.png" % name))
        report.update(status="completed", threshold_note="sigmoid head outputs in [0,1]")
        return 0
    except Exception as exc:
        report.update(status="blocked", error_type=type(exc).__name__, error=str(exc),
                      traceback=traceback.format_exc())
        return 1
    finally:
        (out / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
