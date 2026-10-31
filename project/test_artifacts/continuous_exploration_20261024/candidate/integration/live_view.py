"""Responsive live viewer for the mounted rig on display :6.

Shows a 2x2 panel while staying interactive:
    [ camera ]            [ camera + LiDAR depth points ]
    [ traversability ]    [ visual frontier ]

ExploRFM heads are heavy, so they run in a background worker (subprocess into the
vision-cu128 interpreter) while the UI keeps refreshing; a panel shows
"computing heads..." until the maps arrive. Heads output must live under
vocabulary_query, so they go to vocabulary_query/live_heads/<run>_<frame>.

    python -B integration/live_view.py --watch ~/capture_out \
        --checkpoint-root ~/ckpts --source-root .../nebula2-wildos \
        --python ~/vision-cu128/bin/python
"""
import argparse
import json
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

W, H = 960, 640
FOCAL = H / (2.0 * np.tan(np.radians(60.0) / 2.0))
K = np.array([[FOCAL, 0.0, W / 2.0], [0.0, FOCAL, H / 2.0], [0.0, 0.0, 1.0]])
MAX_POINTS = 25000
DISPLAY_SCALE = 0.62


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def project(cloud, rotation, translation):
    camera = cloud @ np.asarray(rotation, float).T + np.asarray(translation, float)
    depth = camera[:, 2]
    keep = depth > 0.1
    camera, depth = camera[keep], depth[keep]
    u = K[0, 0] * camera[:, 0] / depth + K[0, 2]
    v = K[1, 1] * camera[:, 1] / depth + K[1, 2]
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    return u[inside].astype(int), v[inside].astype(int), depth[inside]


def draw_lidar(image, cloud, rotation, translation):
    vis = image.copy()
    u, v, depth = project(cloud, rotation, translation)
    if len(u) == 0:
        return vis
    if len(u) > MAX_POINTS:
        stride = max(1, len(u) // MAX_POINTS)
        u, v, depth = u[::stride], v[::stride], depth[::stride]
    norm = np.clip((depth - depth.min()) / max(depth.max() - depth.min(), 1e-6), 0, 1)
    colors = (np.array([255, 0, 0]) * (1 - norm)[:, None] + np.array([0, 0, 255]) * norm[:, None])
    vis[v, u] = colors.astype(np.uint8)
    return vis


def heat_overlay(image, heat):
    heat = np.clip(np.asarray(heat, np.float32), 0.0, 1.0)
    heat = cv2.resize(heat, (W, H), interpolation=cv2.INTER_LINEAR)
    colored = cv2.applyColorMap((heat * 255).astype(np.uint8), cv2.COLORMAP_JET)
    return cv2.addWeighted(image, 0.55, colored, 0.45, 0)


class HeadWorker(threading.Thread):
    def __init__(self, py, script, ckpt, src):
        super().__init__(daemon=True)
        self.py, self.script, self.ckpt, self.src = py, Path(script), ckpt, src
        self.root = self.script.resolve().parent
        self.jobs = queue.Queue()
        self.results = {}
        self.lock = threading.Lock()

    def submit(self, frame_dir):
        self.jobs.put(Path(frame_dir))

    def get(self, frame_dir):
        with self.lock:
            return self.results.get(str(frame_dir))

    def run(self):
        while True:
            frame_dir = self.jobs.get()
            if frame_dir is None:
                return
            key = "%s_%s" % (frame_dir.parent.parent.name, frame_dir.name)
            out_dir = self.root / "live_heads" / key
            trav = out_dir / "traversability.npy"
            front = out_dir / "frontier.npy"
            if not (trav.is_file() and front.is_file()):
                out_dir.mkdir(parents=True, exist_ok=True)
                subprocess.run([self.py, "-B", str(self.script), "--image", str(frame_dir / "image.png"),
                                "--checkpoint-root", self.ckpt, "--source-root", self.src,
                                "--output", str(out_dir), "--heatmap", "relative", "--smooth"], check=False)
            maps = (np.load(trav) if trav.is_file() else None, np.load(front) if front.is_file() else None)
            with self.lock:
                self.results[str(frame_dir)] = maps


def load_frame(frame_dir):
    record = read_json(frame_dir / "capture.json")
    image = cv2.imread(str(frame_dir / "image.png"), cv2.IMREAD_COLOR)
    cloud = np.load(frame_dir / "cloud.npy", allow_pickle=False)
    return {"dir": frame_dir, "image": image, "cloud": cloud,
            "R": record["pose"]["R"], "t": record["pose"]["translation_m"]}


def compose(frame, heads):
    image = frame["image"]
    panel = np.zeros((2 * H, 2 * W, 3), np.uint8)
    panel[0:H, 0:W] = image
    panel[0:H, W:2 * W] = draw_lidar(image, frame["cloud"], frame["R"], frame["t"])
    trav, front = (heads if heads else (None, None))
    panel[H:2 * H, 0:W] = heat_overlay(image, trav) if trav is not None else image
    panel[H:2 * H, W:2 * W] = heat_overlay(image, front) if front is not None else image
    labels = [("camera", 0, 0), ("camera+LiDAR", W, 0),
              ("traversability", 0, H), ("frontier", W, H)]
    for text, x, y in labels:
        cv2.putText(panel, text, (x + 12, y + 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    if heads is None:
        cv2.putText(panel, "computing heads...", (24, 2 * H - 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 255, 255), 2, cv2.LINE_AA)
    return panel


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--watch", type=Path, required=True)
    p.add_argument("--checkpoint-root", required=True)
    p.add_argument("--source-root", required=True)
    p.add_argument("--python", default=sys.executable)
    p.add_argument("--heads-script", default=str(Path(__file__).resolve().parent.parent /
                                                "vocabulary_query" / "explorfm_heads.py"))
    p.add_argument("--idle-exit", type=float, default=25.0)
    p.add_argument("--max-wait", type=float, default=600.0)
    a = p.parse_args(argv)

    worker = HeadWorker(a.python, a.heads_script, a.checkpoint_root, a.source_root)
    worker.start()
    started = time.time() - 2.0
    frames, seen, saved = [], set(), set()
    last_new = time.time()
    print("viewer watching %s" % a.watch, flush=True)
    try:
        while True:
            for capture in sorted(a.watch.glob("*/frames/frame_*/capture.json")):
                frame_dir = capture.parent
                if str(frame_dir) in seen or capture.stat().st_mtime < started:
                    continue
                seen.add(str(frame_dir))
                frames.append(load_frame(frame_dir))
                worker.submit(frame_dir)
                last_new = time.time()
                print("queued %s" % frame_dir.name, flush=True)
            if frames:
                frame = frames[-1]
                heads = worker.get(frame["dir"])
                panel = compose(frame, heads)
                view = cv2.resize(panel, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_AREA)
                cv2.imshow("wildos-live", view)
                if heads is not None and str(frame["dir"]) not in saved:
                    saved.add(str(frame["dir"]))
                    cv2.imwrite(str(frame["dir"] / "live_panel.png"), panel)
                cv2.waitKey(30)
                done = all(worker.get(f["dir"]) is not None for f in frames)
                if done and time.time() - last_new > 6:
                    break
                if time.time() - last_new > a.max_wait:
                    break
            else:
                cv2.waitKey(100)
                if time.time() - last_new > a.max_wait:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        worker.jobs.put(None)
        cv2.destroyAllWindows()
    print("viewer done, frames=%d" % len(frames), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
