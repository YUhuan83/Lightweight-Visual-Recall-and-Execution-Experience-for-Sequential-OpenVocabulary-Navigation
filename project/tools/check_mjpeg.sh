#!/usr/bin/env bash
set +u
echo "== stream bytes in 6s (cam) =="
timeout 6 curl -s http://127.0.0.1:8137/stream/cam.mjpg | wc -c
echo "== stream bytes in 6s (model) =="
timeout 6 curl -s http://127.0.0.1:8137/stream/model.mjpg | wc -c
echo "== mjpeg log =="
tail -n 10 /tmp/mjpeg_stream.log 2>/dev/null || echo no_log
