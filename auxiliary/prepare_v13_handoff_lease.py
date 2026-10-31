"""Create an isolated v13 from v12 with a bounded exploration handoff lease."""
from pathlib import Path
import hashlib
import json
import shutil

base = Path("/home/ly/execution_continuity_review_20261022")
old = base / "v12_fresh_command_recovery"
new = base / "v13_handoff_lease"
assert not new.exists(), str(new)

manifest = json.loads((old / "full/frozen_source_manifest.json").read_text())
for relative, digest in manifest["members"].items():
    actual = hashlib.sha256((old / "candidate" / relative).read_bytes()).hexdigest()
    assert actual == digest, "v12 frozen source changed: " + relative

shutil.copytree(old / "candidate", new / "candidate", symlinks=True)
full = new / "full"
full.mkdir()
for item in (old / "full").iterdir():
    if item.is_file() and item.suffix in (".py", ".json"):
        shutil.copy2(item, full / item.name)
(full / "candidate").symlink_to(new / "candidate", target_is_directory=True)
for name in ("build.sh", "launch_trial.py", "run_full.sh", "real_ros_review.py"):
    (new / name).write_text((old / name).read_text().replace(str(old), str(new)))
config = json.loads((full / "config.json").read_text())
config["setups"] = [path.replace(str(old), str(new)) for path in config["setups"]]
config["output_root"] = str(full / "sessions")
(full / "config.json").write_text(json.dumps(config, indent=2) + "\n")

continuous_rel = "nav2_adapter/wildos_nav2_adapter/continuous_path.py"
continuous_path = new / "candidate" / continuous_rel
source = continuous_path.read_text()
source = source.replace(
    "CONTINUOUS_HANDOFF_DISTANCE_M = 8.\n",
    "CONTINUOUS_HANDOFF_DISTANCE_M = 8.\n"
    "# Keep the generic action handshake short; exploration replanning can take longer.\n"
    "CONTINUOUS_HANDOFF_LEASE_DISTANCE_M = 16.\n"
    "CONTINUOUS_HANDOFF_LEASE_SEC = 15.\n", 1)
source = source.replace(
    "deadline=time.monotonic()+float(self.get_parameter('follow_response_timeout_sec').value),",
    "deadline=time.monotonic()+CONTINUOUS_HANDOFF_LEASE_SEC,", 1)
source = source.replace(
    "motion['travelled'] >= CONTINUOUS_HANDOFF_DISTANCE_M",
    "motion['travelled'] >= CONTINUOUS_HANDOFF_LEASE_DISTANCE_M", 1)
assert "CONTINUOUS_HANDOFF_LEASE_SEC" in source
assert "CONTINUOUS_HANDOFF_LEASE_DISTANCE_M" in source
continuous_path.write_text(source)

test_rel = "nav2_adapter/test/test_motion_intent_handoff_ros.py"
test_path = new / "candidate" / test_rel
test = test_path.read_text()
anchor = "    def test_resolved_frontier_retires_pending_extension_not_accepted_motion(self):\n"
addition = "\n".join([
    "    def test_exploration_handoff_lease_is_independent_and_bounded(self):",
    "        n = self.node",
    "        self.resolve()",
    "        remaining = n.retired_motion['deadline'] - time.monotonic()",
    "        self.assertGreater(remaining, 14.8)",
    "        self.assertLess(remaining, 15.1)",
    "        self.assertEqual(n.get_parameter('follow_response_timeout_sec').value, 5.)",
    "",
])
assert test.count(anchor) == 1
test = test.replace(anchor, addition + anchor, 1)
old_distance_test = "\n".join([
    "    def test_distance_budget_is_checked_while_background_planning(self):",
    "        n = self.node",
    "        self.resolve()",
    "        n.pending = n.extending_path = True",
    "        n.odom.pose.pose.position.x = 8.1",
    "        n.maybe_extend_path()",
    "        self.handle.cancel_goal_async.assert_called_once()",
    "        self.assertTrue(n.follow_hold)",
    "",
])
new_distance_test = "\n".join([
    "    def test_distance_budget_is_checked_while_background_planning(self):",
    "        n = self.node",
    "        self.resolve()",
    "        n.pending = n.extending_path = True",
    "        n.odom.pose.pose.position.x = 8.1",
    "        n.maybe_extend_path()",
    "        self.handle.cancel_goal_async.assert_not_called()",
    "        self.assertIsNotNone(n.retired_motion)",
    "        n.odom.pose.pose.position.x = 16.1",
    "        n.maybe_extend_path()",
    "        self.handle.cancel_goal_async.assert_called_once()",
    "        self.assertTrue(n.follow_hold)",
    "",
])
assert test.count(old_distance_test) == 1, "v12 distance test changed"
test_path.write_text(test.replace(old_distance_test, new_distance_test, 1))

for relative in (continuous_rel, test_rel):
    manifest["members"][relative] = hashlib.sha256(
        (new / "candidate" / relative).read_bytes()).hexdigest()
(full / "frozen_source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
for relative, digest in manifest["members"].items():
    actual = hashlib.sha256((new / "candidate" / relative).read_bytes()).hexdigest()
    assert actual == digest, "v13 freeze verification failed: " + relative

delta = json.loads((old / "candidate_delta.json").read_text())
delta["candidate"] = str(new)
delta["base"] = str(old)
delta["changed_runtime_files"] = [
    "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py", continuous_rel]
delta["changed_test_files"] = [
    "nav2_adapter/test/test_graph_path_bridge_routing.py", test_rel]
delta["handoff_lease"] = {"distance_m": 16, "timeout_sec": 15,
                           "generic_action_timeout_sec": 5}
(new / "candidate_delta.json").write_text(json.dumps(delta, indent=2) + "\n")
print(json.dumps({"candidate": str(new), "members": len(manifest["members"]),
                  "changed_runtime_files": delta["changed_runtime_files"],
                  "lease": delta["handoff_lease"]}, indent=2))
