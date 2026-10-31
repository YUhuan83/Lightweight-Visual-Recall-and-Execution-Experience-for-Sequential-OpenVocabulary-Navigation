"""Create an isolated server candidate from frozen v10 plus one recovery fix."""
from pathlib import Path
import hashlib
import json
import shutil

BASE = Path("/home/ly/execution_continuity_review_20261022")
OLD = BASE / "v10_point_support"
NEW = BASE / "v11_path_recovery"
OLD_TEXT = '''            self.block_current_route()
            self.status("path_changed_before_execution", reason=reason)
'''
NEW_TEXT = '''            # The candidate path failed current-map validation before execution.
            # Request a replacement exploration route immediately; keep only a
            # short bounded retry for explicit or remembered destinations.
            retry_delay = 2.0
            self.block_current_route(backoff=retry_delay)
            route_rejected = self.reject_exploration_route()
            self.status("path_changed_before_execution", reason=reason,
                        retry_after_sec=retry_delay,
                        route_rejected=route_rejected)
'''

assert not NEW.exists(), str(NEW)
old_manifest = json.loads((OLD / "full/frozen_source_manifest.json").read_text())
for name, digest in old_manifest["members"].items():
    assert hashlib.sha256((OLD / "candidate" / name).read_bytes()).hexdigest() == digest

shutil.copytree(OLD / "candidate", NEW / "candidate", symlinks=True)
bridge = NEW / "candidate/nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"
source = bridge.read_text()
assert source.count(OLD_TEXT) == 1, "expected one unchanged pre-execution rejection branch"
bridge.write_text(source.replace(OLD_TEXT, NEW_TEXT, 1))
old_manifest["members"]["nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"] = \
    hashlib.sha256(bridge.read_bytes()).hexdigest()

full = NEW / "full"
full.mkdir()
for source_file in (OLD / "full").glob("*.py"):
    shutil.copy2(source_file, full / source_file.name)
(full / "candidate").symlink_to(NEW / "candidate", target_is_directory=True)
(full / "frozen_source_manifest.json").write_text(json.dumps(old_manifest, indent=2))

config = json.loads((OLD / "full/config.json").read_text())
config["setups"].append(str(NEW / "install/local_setup.bash"))
config["output_root"] = str(full / "sessions")
(full / "config.json").write_text(json.dumps(config, indent=2) + "\n")

for name in ("run_full.sh", "build.sh", "launch_trial.py"):
    text = (OLD / name).read_text().replace(str(OLD), str(NEW))
    (NEW / name).write_text(text)
(NEW / "real_ros_review.py").write_text((OLD / "real_ros_review.py").read_text())

delta = {
    "candidate": str(NEW),
    "base": str(OLD),
    "changed_runtime_files": ["nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"],
    "change": "path validation failure uses 2 s retry and rejects exploration route immediately",
    "members": len(old_manifest["members"]),
    "graph_path_bridge_sha256": old_manifest["members"][
        "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"],
}
(NEW / "candidate_delta.json").write_text(json.dumps(delta, indent=2) + "\n")
print(json.dumps(delta, indent=2))
