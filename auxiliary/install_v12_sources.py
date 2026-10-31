"""Install the reviewed v12 source/test files into the isolated candidate."""
from pathlib import Path
import hashlib
import json
import shutil

base = Path("/home/ly/execution_continuity_review_20261022")
incoming = base / "incoming"
root = base / "v12_fresh_command_recovery"
files = {
    "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py":
        incoming / "graph_path_bridge.py",
    "nav2_adapter/test/test_graph_path_bridge_routing.py":
        incoming / "test_graph_path_bridge_routing.py",
}
manifest_path = root / "full/frozen_source_manifest.json"
manifest = json.loads(manifest_path.read_text())
for relative, staged in files.items():
    destination = root / "candidate" / relative
    assert relative in manifest["members"], relative
    assert staged.is_file(), str(staged)
    shutil.copy2(staged, destination)
    manifest["members"][relative] = hashlib.sha256(destination.read_bytes()).hexdigest()
manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

delta_path = root / "candidate_delta.json"
delta = json.loads(delta_path.read_text())
delta["changed_runtime_files"] = ["nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"]
delta["changed_test_files"] = ["nav2_adapter/test/test_graph_path_bridge_routing.py"]
delta["graph_path_bridge_sha256"] = manifest["members"][
    "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"]
delta_path.write_text(json.dumps(delta, indent=2) + "\n")

for relative, digest in manifest["members"].items():
    actual = hashlib.sha256((root / "candidate" / relative).read_bytes()).hexdigest()
    assert actual == digest, f"freeze verification failed: {relative}"
print(json.dumps({"updated": list(files), "verified_members": len(manifest["members"]),
                  "graph_path_bridge_sha256": delta["graph_path_bridge_sha256"]}, indent=2))
