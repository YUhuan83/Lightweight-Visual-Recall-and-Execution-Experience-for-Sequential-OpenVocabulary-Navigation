"""Create an isolated v12 field candidate from the frozen v11 trial."""
from pathlib import Path
import hashlib
import json
import shutil

BASE = Path("/home/ly/execution_continuity_review_20261022")
OLD = BASE / "v11_path_recovery"
NEW = BASE / "v12_fresh_command_recovery"

assert not NEW.exists(), str(NEW)
manifest = json.loads((OLD / "full/frozen_source_manifest.json").read_text())
for name, digest in manifest["members"].items():
    actual = hashlib.sha256((OLD / "candidate" / name).read_bytes()).hexdigest()
    assert actual == digest, f"v11 frozen source changed: {name}"

shutil.copytree(OLD / "candidate", NEW / "candidate", symlinks=True)
full = NEW / "full"
full.mkdir(parents=True)
for item in (OLD / "full").iterdir():
    if item.is_file() and (item.suffix in (".py", ".json")):
        shutil.copy2(item, full / item.name)
(full / "candidate").symlink_to(NEW / "candidate", target_is_directory=True)

for name in ("build.sh", "launch_trial.py", "run_full.sh", "real_ros_review.py"):
    text = (OLD / name).read_text()
    (NEW / name).write_text(text.replace(str(OLD), str(NEW)))

config = json.loads((full / "config.json").read_text())
config["setups"] = [p.replace(str(OLD), str(NEW)) for p in config["setups"]]
config["output_root"] = str(full / "sessions")
(full / "config.json").write_text(json.dumps(config, indent=2) + "\n")

delta = {
    "candidate": str(NEW),
    "base": str(OLD),
    "changed_runtime_files": ["nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"],
    "change": "After a repeated local-footprint collision on a bearing-only visible-target approach, release that blocked hint after one refreshed-map retry; preserve memory, confirmed-target, frontier, and side-lookback handling.",
    "members": len(manifest["members"]),
}
(NEW / "candidate_delta.json").write_text(json.dumps(delta, indent=2) + "\n")
print(json.dumps({"candidate": str(NEW), "members": len(manifest["members"]),
                  "config_output": config["output_root"]}, indent=2))
