"""Print missing modules required by upstream WildOS requirements.txt."""
import importlib.util

MODULES = {
    "timm": "timm==1.0.15",
    "open_clip": "open-clip-torch==2.32.0",
    "transformers": "transformers==4.51.3",
    "einops": "einops==0.8.1",
    "fairscale": "fairscale==0.4.4",
    "skimage": "scikit-image",
    "onnx": "onnx",
    "cv2": "opencv-python",
    "matplotlib": "matplotlib",
    "ruamel.yaml": "ruamel.yaml==0.18.11",
    "omegaconf": "omegaconf",
    "folium": "folium",
    "selenium": "selenium",
    "scipy": "scipy",
}

missing = []
for module, package in MODULES.items():
    try:
        present = importlib.util.find_spec(module) is not None
    except ModuleNotFoundError:
        present = False
    if not present:
        missing.append(package)
print("MISSING", " ".join(missing))
