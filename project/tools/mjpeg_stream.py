"""Minimal MJPEG web viewer for official WildOS outputs.

Streams the BeamNG front camera and the WildOS frontier/traversability
visualization over HTTP as multipart/x-mixed-replace. Read-only: it never
publishes or commands anything. Bind to 127.0.0.1 and reach it through an SSH
local port forward.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

LATEST = {"cam": None, "model": None}
LOCK = threading.Lock()
PERIOD = 0.08

TOPICS = {
    "cam": "/spot1/realsense/front/color/image_raw",
    "model": "/spot1/model_visualization",
}

INDEX = b"""<!doctype html><html><head><meta charset="utf-8">
<title>WildOS frontier/traversability (nizhen_countryside)</title>
<style>
body{background:#202124;color:#e8eaed;font-family:Segoe UI,Arial,sans-serif;margin:0;padding:12px}
h1{font-size:16px;font-weight:600;margin:0 0 8px}
.row{display:flex;gap:12px;flex-wrap:wrap}
.card{background:#2b2c2f;border:1px solid #3c4043;border-radius:8px;padding:8px}
.card h2{font-size:13px;font-weight:500;margin:0 0 6px;color:#bdc1c6}
img{display:block;max-width:100%;border-radius:4px}
.small{color:#9aa0a6;font-size:12px;margin-top:8px}
</style></head><body>
<h1>WildOS &mdash; nizhen_countryside (headless)</h1>
<div class="row">
<div class="card"><h2>WildOS model viz: image / frontier conf. / traversability conf. / frontier nodes</h2>
<img src="/stream/model.mjpg" style="width:760px"></div>
<div class="card"><h2>Front camera</h2>
<img src="/stream/cam.mjpg" style="width:520px"></div>
</div>
<div class="small">Streams update live. Recorder PNGs: /home/ly/nizhen_viz_capture</div>
</body></html>"""


def image_to_bgr(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    if msg.encoding in ("rgb8", "bgr8"):
        row = arr.reshape(msg.height, msg.step)
        im = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else im
    return None


class Streamer(Node):
    def __init__(self):
        super().__init__("wildos_mjpeg_stream")
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, topic in TOPICS.items():
            self.create_subscription(Image, topic, self._make_cb(key), qos)

    def _make_cb(self, key):
        def cb(msg):
            try:
                bgr = image_to_bgr(msg)
            except Exception:
                return
            if bgr is None:
                return
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with LOCK:
                    LATEST[key] = buf.tobytes()
        return cb


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX)))
            self.end_headers()
            self.wfile.write(INDEX)
            return
        if self.path.startswith("/stream/"):
            key = self.path.rsplit("/", 1)[-1].split(".")[0]
            if key not in TOPICS:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        data = LATEST.get(key)
                    if data:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n")
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                    time.sleep(PERIOD)
            except (BrokenPipeError, ConnectionResetError):
                return
            return
        self.send_error(404)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    rclpy.init()
    node = Streamer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"mjpeg viewer on http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
