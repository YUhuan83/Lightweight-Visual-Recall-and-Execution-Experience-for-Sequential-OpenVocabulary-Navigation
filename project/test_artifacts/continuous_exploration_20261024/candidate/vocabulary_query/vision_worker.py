"""Persistent vision worker: load ExploRFM (SigLIP2 + C-RADIO + heads) once.

Serves both the WildOS text query (dense similarity -> regions) and the ExploRFM
traversability/frontier heads from a single model instance, over a JSON-lines
protocol on stdin/stdout. This removes the per-cycle model reload of query.py /
explorfm_heads.py.

Request: {"image": path, "query": text, "out": dir, "max_side": 640, "min_area": 64}
Response: {"status":"ok", "regions":[...], "traversability_range":[...], "frontier_range":[...]}

    python -B vision_worker.py --checkpoint-root CKPT --source-root SRC            # serve
    python -B vision_worker.py ... --image IMG --query "house" --out DIR           # one shot
"""
import argparse
import importlib.util
import json
import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Worker:
    def __init__(self, ckpt_root, source_root, max_side=640, min_area=64):
        self.query = load_module("vc_query", HERE / "query.py")
        self.heads = load_module("vc_heads", HERE / "explorfm_heads.py")
        self.ckpt = Path(ckpt_root).resolve()
        self.source = Path(source_root).resolve()
        self.max_side = max_side
        self.min_area = min_area
        self.query.inspect_inputs(self.source, self.ckpt)
        self.config, code = self.query.official_code(self.source)
        self.threshold = self.config["mask_threshold"]
        import torch
        import torch.nn.functional as F
        import numpy as np
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA required")
        free, total = torch.cuda.mem_get_info()
        torch.cuda.set_per_process_memory_fraction(min(0.80, (free - 1024**3) / total))
        adaptor = self._resolve_adaptor()
        self.config, code = self.query.official_code(self.source)
        sys.path.insert(0, str(self.source))
        from explorfm.explorfm_model import ExploRFMInference
        with self.heads.tolerant_torch_load():
            self.model = ExploRFMInference(
                frontier_ckpt=str(self.ckpt / "frontier_head.ckpt"),
                traversability_ckpt=str(self.ckpt / "trav_head.ckpt"),
                model_version=str(self.ckpt / "c-radio_v3-b_half.pth.tar"),
                adaptor_version="siglip2", adaptor_ckpt_path=str(adaptor),
                use_naclip=True, use_summary_for_spatial=True, static_scale_factor=1.0,
                model_precision="FP16", device="cuda")
        namespace = {"torch": torch, "np": np, "F": F}
        exec(code, namespace)
        self.localize_query = namespace["localize_query"]
        self.torch = torch
        self.np = np

    def _resolve_adaptor(self):
        """Locate the SigLIP2 adaptor directory. Deployments store it either as
        ``siglip2`` (paper layout) or ``siglip2_ms`` (Ubuntu server layout)."""
        for name in ("siglip2", "siglip2_ms"):
            candidate = self.ckpt / name
            if candidate.is_dir():
                return candidate
        raise RuntimeError("adaptor 'siglip2'/'siglip2_ms' not found under %s" % self.ckpt)

    def infer(self, image_path, text, out_dir):
        from PIL import Image, ImageOps
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        with Image.open(image_path) as opened:
            image = ImageOps.exif_transpose(opened).convert("RGB")
        small = image.copy()
        small.thumbnail((self.max_side, self.max_side), Image.Resampling.BILINEAR)
        with self.torch.inference_mode():
            trav, front, spatial = self.model.forward_on_numpy(self.np.asarray(small).copy())
            text_features = self.model.forward_on_text([text])
            maps, _ = self.localize_query(text_features.float(), spatial.float(),
                                          (image.height, image.width),
                                          self.config["pixel_level_seg"], self.threshold)
        dense = self.np.asarray(maps[0, 0], dtype=self.np.float32)
        _, kept, regions = self.query.postprocess(dense, self.threshold, self.min_area)
        self.np.save(out / "cosine.npy", dense, allow_pickle=False)
        Image.fromarray((kept * 255).astype(self.np.uint8)).save(out / "region_mask.png")
        trav_np = trav[0, 0].float().cpu().numpy() if trav is not None else None
        front_np = front[0, 0].float().cpu().numpy() if front is not None else None
        if trav_np is not None:
            self.np.save(out / "traversability.npy", trav_np, allow_pickle=False)
        if front_np is not None:
            self.np.save(out / "frontier.npy", front_np, allow_pickle=False)
        response = {"status": "ok", "query": text, "image_size": list(image.size),
                    "regions": regions,
                    "mask_pixels": int(kept.sum()),
                    "traversability_range": None if trav_np is None else [float(trav_np.min()), float(trav_np.max())],
                    "frontier_range": None if front_np is None else [float(front_np.min()), float(front_np.max())]}
        (out / "worker_result.json").write_text(json.dumps(response, indent=2), encoding="utf-8")
        return response


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint-root", required=True)
    p.add_argument("--source-root", required=True)
    p.add_argument("--image")
    p.add_argument("--query")
    p.add_argument("--out")
    a = p.parse_args(argv)
    worker = Worker(a.checkpoint_root, a.source_root)
    print(json.dumps({"status": "ready"}), flush=True)
    if a.image:
        try:
            print(json.dumps(worker.infer(a.image, a.query, a.out)), flush=True)
            return 0
        except Exception as exc:
            print(json.dumps({"status": "error", "error": str(exc), "traceback": traceback.format_exc()}), flush=True)
            return 1
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        if line == "quit":
            break
        try:
            req = json.loads(line)
            print(json.dumps(worker.infer(req["image"], req["query"], req["out"])), flush=True)
        except Exception as exc:
            print(json.dumps({"status": "error", "error": str(exc), "traceback": traceback.format_exc()}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
