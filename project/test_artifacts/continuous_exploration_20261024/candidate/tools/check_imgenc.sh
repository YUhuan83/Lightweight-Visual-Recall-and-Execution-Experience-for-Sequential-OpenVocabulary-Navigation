#!/usr/bin/env bash
set +u
P=/home/ly/vision-cu128/bin/python
$P - <<'PY'
mods = ["cv2", "PIL", "simplejpeg", "av"]
for m in mods:
    try:
        mod = __import__(m)
        print("OK  ", m, getattr(mod, "__version__", ""))
    except Exception as e:
        print("MISS", m, type(e).__name__)
PY
