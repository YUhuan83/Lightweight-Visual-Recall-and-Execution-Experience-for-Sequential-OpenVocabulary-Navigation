"""Server-side import preflight for the official WildOS vision interpreter."""
from pathlib import Path


status = Path("/tmp/vision_import_status.txt")
status.write_text("START\n", encoding="ascii")
import rclpy  # noqa: E402,F401
status.write_text(status.read_text() + "RCLPY\n", encoding="ascii")
import torch  # noqa: E402
status.write_text(status.read_text() + "TORCH\n", encoding="ascii")
import visual_navigation.wildos.nav  # noqa: E402,F401
status.write_text(status.read_text() + "WILDOS\n", encoding="ascii")
import visual_navigation.explorfm_triangulation.obj_mask_triangulation  # noqa: E402,F401
status.write_text(status.read_text() + "TRIANGULATION\nOK\n", encoding="ascii")
print("VISION_IMPORT_OK", torch.__version__, torch.cuda.is_available(), torch.cuda.device_count())
