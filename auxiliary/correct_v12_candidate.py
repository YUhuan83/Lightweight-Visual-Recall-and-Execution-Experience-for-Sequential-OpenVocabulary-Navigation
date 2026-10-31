"""Reset the isolated candidate to frozen v11 and apply one focused change."""
from pathlib import Path
import hashlib
import json
import os
import shutil

base = Path("/home/ly/execution_continuity_review_20261022")
old = base / "v11_path_recovery"
new = base / "v12_fresh_command_recovery"
bridge_rel = "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"
test_rel = "nav2_adapter/test/test_graph_path_bridge_routing.py"

bridge = (old / "candidate" / bridge_rel).read_text()
anchor = "\n".join([
    "        repeated_local_collision = (reason == 'motion_section_invalid'",
    "                                    and details.get('detail') == 'local_footprint_collision'",
    "                                    and self.recovery_attempts >= 2)",
]) + "\n"
addition = anchor + "\n".join([
    "        repeated_visible_collision = (",
    "            reason == 'motion_section_invalid'",
    "            and details.get('detail') == 'local_footprint_collision'",
    "            and self.recovery_attempts >= 1",
    "            and self.visible_approach_active",
    "            and self.expected is not None",
    "            and self.memory_goal is None",
    "            and self.memory_active is None",
    "            and self.confirmed_route_goal is None",
    "            and not getattr(self, 'lookback_active', False))",
    "        if repeated_visible_collision:",
    "            # A bearing-only goal has no confirmed object pose to justify",
    "            # retrying the same blocked approach for the full 30 s deadline.",
    "            # After the first map refresh/replan also hits the local body",
    "            # footprint, release this observation point and resume exploration.",
    "            self.release_visible_approach('visible_target_path_blocked')",
    "            return",
]) + "\n"
assert bridge.count(anchor) == 1, "v11 recovery insertion point changed"
bridge = bridge.replace(anchor, addition, 1)

test = (old / "candidate" / test_rel).read_text()
test_anchor = "    def test_local_collision_exhaustion_preserves_destination_and_query_guards(self):\n"
test_addition = "\n".join([
    "    def test_repeated_visible_target_collision_releases_blocked_hint_after_map_retry(self):",
    "        self.node.follow_enabled = True",
    "        self.node.route = route((0., 0.), (8., 0.))",
    "        self.node.route_endpoint = (8., 0.)",
    "        self.node.expected = (8., 0.)",
    "        self.node.visible_approach_active = True",
    "        self.node.release_visible_approach = Mock()",
    "        self.node.block_current_route = Mock()",
    "        self.node.recovery_attempts = 0",
    "        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')",
    "        self.node.release_visible_approach.assert_not_called()",
    "        self.assertEqual(self.node.last_status['retry_after_sec'], 2.)",
    "        self.node.recovery_attempts = 1",
    "        self.node.block_current_route.reset_mock()",
    "        self.node.follow_failed('motion_section_invalid', detail='local_footprint_collision')",
    "        self.node.release_visible_approach.assert_called_once_with('visible_target_path_blocked')",
    "        self.node.block_current_route.assert_not_called()",
    "",
    "    def test_repeated_collision_does_not_release_memory_or_side_lookback_goal(self):",
    "        self.node.follow_enabled = True",
    "        self.node.route = route((0., 0.), (8., 0.))",
    "        self.node.route_endpoint = (8., 0.)",
    "        self.node.expected = (8., 0.)",
    "        self.node.visible_approach_active = True",
    "        self.node.recovery_attempts = 1",
    "        self.node.release_visible_approach = Mock()",
    "        for guard in ({'memory_goal': (8., 0.)},",
    "                      {'memory_active': {'position': (8., 0.)}},",
    "                      {'lookback_active': True}):",
    "            with self.subTest(guard=guard):",
    "                with patch.multiple(self.node, **guard):",
    "                    self.node.follow_failed('motion_section_invalid',",
    "                                            detail='local_footprint_collision')",
    "                self.node.release_visible_approach.assert_not_called()",
    "",
])
assert test.count(test_anchor) == 1, "v11 test insertion point changed"
test = test.replace(test_anchor, test_addition + test_anchor, 1)

for name in ("build", "install", "build_log"):
    current = new / name
    if current.exists():
        archived = new / (name + "_mixed_local_attempt1")
        assert not archived.exists(), str(archived)
        os.replace(current, archived)
for name in ("build.log", "real_ros_review.log", "real_ros_report.json"):
    current = new / name
    if current.exists():
        archived = new / (name.replace(".log", "").replace(".json", "")
                          + "_mixed_local_attempt1" + current.suffix)
        assert not archived.exists(), str(archived)
        os.replace(current, archived)

(new / "candidate" / bridge_rel).write_text(bridge)
(new / "candidate" / test_rel).write_text(test)
manifest = json.loads((old / "full/frozen_source_manifest.json").read_text())
for relative, digest in manifest["members"].items():
    actual = hashlib.sha256((old / "candidate" / relative).read_bytes()).hexdigest()
    assert actual == digest, "v11 frozen source changed: " + relative
for relative in (bridge_rel, test_rel):
    manifest["members"][relative] = hashlib.sha256(
        (new / "candidate" / relative).read_bytes()).hexdigest()
(new / "full/frozen_source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
for relative, digest in manifest["members"].items():
    actual = hashlib.sha256((new / "candidate" / relative).read_bytes()).hexdigest()
    assert actual == digest, "v12 freeze verification failed: " + relative

delta_path = new / "candidate_delta.json"
delta = json.loads(delta_path.read_text())
delta["changed_runtime_files"] = [bridge_rel]
delta["changed_test_files"] = [test_rel]
delta["source_base"] = str(old / "candidate")
delta["graph_path_bridge_sha256"] = manifest["members"][bridge_rel]
delta_path.write_text(json.dumps(delta, indent=2) + "\n")
print(json.dumps({"runtime_file": bridge_rel, "test_file": test_rel,
                  "verified_members": len(manifest["members"]),
                  "graph_path_bridge_sha256": delta["graph_path_bridge_sha256"]}, indent=2))
