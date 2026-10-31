"""Offline ExploRFM dense text query. No ROS or network required."""
import argparse
import ast
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import traceback

# Must precede every ML import, including imports from the source tree.
for _key in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE", "HF_HUB_DISABLE_TELEMETRY"):
    os.environ[_key] = "1"
sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
NAV = "visual_navigation/visual_navigation"


def official_code(source):
    nav = source / NAV / "wildos/nav.py"
    tree = ast.parse(nav.read_text(encoding="utf-8-sig"))
    config = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == "object_search_config":
                    config = ast.literal_eval(value)
                    break
    if config is None:
        raise ValueError("Official object_search_config not found")
    path = source / NAV / "utils/object_search_utils.py"
    nodes = ast.parse(path.read_text(encoding="utf-8-sig")).body
    fn = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == "localize_query")
    # Compile only this function, leaving all ROS imports unexecuted.
    code = compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec")
    return config, code


def inspect_inputs(source, root):
    spec = json.loads((HERE / "manifest.json").read_text(encoding="utf-8"))
    missing = []
    entries = []
    for rel in spec["required_checkpoints"] + spec["optional_checkpoints"]:
        p = root / rel
        required = rel in spec["required_checkpoints"]
        exists = p.is_file() and p.stat().st_size > 0
        entries.append({"path": rel, "required": required, "present": exists,
                        "bytes": p.stat().st_size if exists else None})
        if required and not exists:
            missing.append(str(p))
    hashes = {}
    for rel in spec["source_files"]:
        p = source / rel
        if not p.is_file():
            missing.append(str(p))
        else:
            hashes[rel] = hashlib.sha256(p.read_bytes()).hexdigest()
    if missing:
        raise FileNotFoundError("Missing required files:\n" + "\n".join(missing))
    # Require the explicit local directory branch, never the cache/network fallback.
    adaptor = ast.parse((source / "nvidia_radio/radio/siglip2_adaptor.py").read_text(encoding="utf-8-sig"))
    guarded = set()
    for node in ast.walk(adaptor):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "from_pretrained":
            if any(k.arg == "local_files_only" and isinstance(k.value, ast.Constant) and k.value.value is True for k in node.keywords):
                guarded.add(ast.unparse(node.func.value))
    if not {"AutoModel", "AutoProcessor"}.issubset(guarded):
        raise RuntimeError("Source lacks patched local_files_only AutoModel/AutoProcessor API; use patched source-root")
    return {"checkpoint_files": entries, "source_sha256": hashes,
            "weight_integrity": "existence and size only; no full weight checksum"}


def heatmap_colors(dense, mode, smooth=False):
    """TURBO colors. 'relative' uses this map's own range; 'fixed' keeps [-1,1].

    smooth only changes DISPLAY upsampling (bilinear); saved scores are untouched.
    """
    import numpy as np
    import cv2
    from PIL import Image
    dense = np.asarray(dense, dtype=np.float32)
    if mode == "relative":
        vmin, vmax = float(dense.min()), float(dense.max())
        if not math.isfinite(vmin) or not math.isfinite(vmax) or vmax <= vmin:
            raise ValueError("relative heatmap needs a finite non-degenerate score range")
        scaled = (dense - vmin) / (vmax - vmin)
    elif mode == "fixed":
        vmin, vmax = -1.0, 1.0
        scaled = np.clip((dense - vmin) / (vmax - vmin), 0.0, 1.0)
    else:
        raise ValueError("heatmap mode must be 'relative' or 'fixed'")
    if smooth:
        scaled = cv2.resize(scaled, (dense.shape[1] * 8, dense.shape[0] * 8),
                            interpolation=cv2.INTER_LINEAR)
    color = cv2.applyColorMap(np.round(scaled * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    if smooth:
        color = cv2.resize(color, (dense.shape[1], dense.shape[0]), interpolation=cv2.INTER_AREA)
    return vmin, vmax, Image.fromarray(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))


def postprocess(dense, threshold, min_area):
    import numpy as np
    import cv2
    dense = np.asarray(dense, dtype=np.float32)
    if dense.ndim != 2 or not dense.size or not np.isfinite(dense).all():
        raise ValueError("Dense map must be a finite nonempty HxW array")
    if not math.isfinite(threshold) or not -1 <= threshold <= 1 or min_area < 1:
        raise ValueError("threshold must be in [-1,1]; min_area must be positive")
    mask = (dense > threshold).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    regions = []
    kept = np.zeros_like(mask)
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < min_area:
            continue
        values = dense[labels == i]
        kept[labels == i] = 1
        regions.append({"box_xyxy": [x, y, x + w, y + h], "area_pixels": area,
                        "mean_cosine": float(values.mean()), "max_cosine": float(values.max())})
    regions.sort(key=lambda r: r["max_cosine"], reverse=True)
    return mask, kept, regions


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--image", type=Path)
    p.add_argument("--query", action="append", help="Repeat for multiple queries; Unicode supported")
    p.add_argument("--checkpoint-root", required=True, type=Path)
    p.add_argument("--source-root", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path, help="New, empty directory under vocabulary_query")
    p.add_argument("--deps", type=Path, help="Explicit process-local dependency overlay")
    p.add_argument("--selfcheck", action="store_true", help="No ML imports/model allocation")
    p.add_argument("--threshold", type=float, help="Raw cosine; default read from official WildOS config")
    p.add_argument("--min-area", type=int, default=64)
    p.add_argument("--max-side", type=int, default=640)
    p.add_argument("--heatmap", choices=("relative", "fixed"), default="relative",
                   help="relative: this map's own range (default); fixed: raw cosine scale")
    p.add_argument("--region-overlay", action="store_true",
                   help="Blend thresholded regions only, keep background uncolored")
    p.add_argument("--smooth", action="store_true",
                   help="Bilinear display upsampling of the heatmap; scores unchanged")
    args = p.parse_args(argv)
    out = args.output.resolve()
    if not out.is_relative_to(HERE) or out == HERE:
        p.error("--output must be a child directory of vocabulary_query")
    if out.exists() and any(out.iterdir()):
        p.error("--output must be empty (prevents stale results)")
    if not args.selfcheck and (not args.image or not args.query or any(not q.strip() for q in args.query)):
        p.error("inference requires --image and nonempty --query")
    if args.max_side < 32 or args.max_side > 640 or args.min_area < 1:
        p.error("--max-side must be 32..640 and --min-area >= 1")
    out.mkdir(parents=True, exist_ok=True)
    report = {"status": "started", "inference_ran": False, "stage": "preflight",
              "queries": args.query, "offline": True, "calibration": "uncalibrated",
              "scope": "2D dense similarity candidates only; not object confidence, 3D or navigation"}
    try:
        if args.deps:
            deps = args.deps.resolve()
            if not deps.is_dir():
                raise FileNotFoundError(deps)
            if any((deps / n).exists() for n in ("torch", "torchvision")):
                raise RuntimeError("Overlay must not shadow the existing CUDA torch/torchvision")
            sys.path.insert(0, str(deps))
        source, root = args.source_root.resolve(), args.checkpoint_root.resolve()
        report.update(inspect_inputs(source, root))
        config, code = official_code(source)
        threshold = config["mask_threshold"] if args.threshold is None else args.threshold
        if not math.isfinite(threshold) or not -1 <= threshold <= 1:
            raise ValueError("threshold must be finite and in [-1,1]")
        report.update(threshold=threshold, threshold_origin="official WildOS config" if args.threshold is None else "CLI",
                      pixel_level_seg=config["pixel_level_seg"], source_root=str(source), checkpoint_root=str(root))
        report["dependencies"] = {n: importlib.util.find_spec(n) is not None for n in
                                  ("torch", "torchvision", "transformers", "timm", "einops", "numpy", "PIL", "cv2", "psutil", "sentencepiece")}
        if args.selfcheck:
            report["status"] = "selfcheck_passed" if all(report["dependencies"].values()) else "dependencies_missing"
            return 0 if report["status"] == "selfcheck_passed" else 1
        report["stage"] = "runtime_import"
        # HF flags do not cover torch.hub or arbitrary remote-code requests.
        import socket
        def deny_network(*unused_args, **unused_kwargs):
            raise RuntimeError("Network access is disabled in this local-only inference process")
        socket.socket.connect = deny_network
        socket.socket.connect_ex = deny_network
        socket.create_connection = deny_network
        import numpy as np
        import cv2
        import psutil
        from PIL import Image, ImageOps, ImageDraw
        import torch
        import torch.nn.functional as F
        sys.path.insert(0, str(source))
        from explorfm.explorfm_model import ExploRFMInference
        report["torch"] = torch.__version__
        report["stage"] = "memory_guard"
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is required by official NA-CLIP; no CPU/dummy fallback")
        free, total = torch.cuda.mem_get_info()
        ram = psutil.virtual_memory().available
        report["memory"] = {"free_vram_bytes": free, "total_vram_bytes": total, "available_ram_bytes": ram}
        if free < 5 * 2**30 or ram < 6 * 2**30:
            raise RuntimeError("Refusing model load: need >=5 GiB free VRAM and >=6 GiB available RAM (official loader builds full SigLIP2 on CPU)")
        torch.cuda.set_per_process_memory_fraction(min(0.80, (free - 1024**3) / total))
        with Image.open(args.image) as opened:
            image = ImageOps.exif_transpose(opened).convert("RGB")
        original_size = image.size
        small = image.copy()
        small.thumbnail((args.max_side, args.max_side), Image.Resampling.BILINEAR)
        report.update(image=str(args.image.resolve()), image_size=list(original_size), input_size=list(small.size))
        report["stage"] = "model_load"
        model = ExploRFMInference(frontier_ckpt=None, traversability_ckpt=None,
                    model_version=str(root / "c-radio_v3-b_half.pth.tar"), adaptor_version="siglip2",
                    adaptor_ckpt_path=str(root / "siglip2_ms"), use_naclip=True,
                    use_summary_for_spatial=True, static_scale_factor=1.0,
                    model_precision="FP16", device="cuda")
        namespace = {"torch": torch, "np": np, "F": F}
        exec(code, namespace)
        report["stage"] = "dense_forward"
        with torch.inference_mode():
            _, _, spatial = model.forward_on_numpy(np.asarray(small).copy())
            text = model.forward_on_text(args.query)
            if spatial is None or spatial.ndim != 4 or text.ndim != 2 or spatial.shape[1] != text.shape[1]:
                raise RuntimeError("Official dense/text feature contract mismatch")
            # FP32 similarity avoids half precision reduction error; model stays FP16.
            maps, _ = namespace["localize_query"](text.float(), spatial.float(),
                (image.height, image.width), config["pixel_level_seg"], threshold)
        report.update(inference_ran=True, spatial_shape=list(spatial.shape), text_shape=list(text.shape))
        report["stage"] = "save_outputs"
        results = []
        for i, query in enumerate(args.query):
            dense = maps[0, i]
            mask, kept, regions = postprocess(dense, threshold, args.min_area)
            prefix = f"query_{i:03d}"
            np.save(out / f"{prefix}_cosine.npy", dense, allow_pickle=False)
            Image.fromarray(mask * 255).save(out / f"{prefix}_threshold_mask.png")
            Image.fromarray(kept * 255).save(out / f"{prefix}_region_mask.png")
            # Raw scores and threshold logic never depend on the display scale.
            vmin, vmax, heat = heatmap_colors(dense, args.heatmap, smooth=args.smooth)
            heat.save(out / f"{prefix}_heatmap.png")
            if args.region_overlay:
                overlay = image.copy()
                tint = np.zeros_like(np.asarray(overlay), dtype=np.uint8)
                tint[kept > 0] = (255, 96, 0)
                overlay = Image.blend(overlay, Image.fromarray(tint), 0.35)
                draw = ImageDraw.Draw(overlay)
                for r in regions:
                    x1, y1, x2, y2 = r["box_xyxy"]
                    draw.rectangle((x1, y1, x2 - 1, y2 - 1), outline="white", width=2)
                blend = overlay
            else:
                blend = Image.blend(image, heat, 0.45)
                draw = ImageDraw.Draw(blend)
                for r in regions:
                    x1, y1, x2, y2 = r["box_xyxy"]
                    draw.rectangle((x1, y1, x2 - 1, y2 - 1), outline="white", width=2)
                overlay = None
            blend.save(out / f"{prefix}_blend.png")
            if overlay is not None:
                overlay.save(out / f"{prefix}_region_overlay.png")
            results.append({"query": query, "prefix": prefix, "raw_min": float(dense.min()),
                            "raw_max": float(dense.max()), "regions": regions})
        report.update(status="completed", results=results,
                      heatmap_mode=args.heatmap,
                      heatmap_range=[-1, 1] if args.heatmap == "fixed" else None,
                      box_convention="xyxy, exclusive right/bottom, EXIF-oriented original image pixels")
        return 0
    except Exception as exc:
        report.update(status="blocked", error_type=type(exc).__name__, error=str(exc), traceback=traceback.format_exc())
        return 1
    finally:
        (out / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
