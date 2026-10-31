"""Recorded evidence checks; passing does not convert pilot5 into a success."""
import hashlib
import json
from pathlib import Path
import unittest
import zipfile


ARCHIVE = Path(__file__).resolve().parents[1] / "test_artifacts/server_validation_20261011/pilot5_closed_evidence.zip"


class Pilot5EvidenceTests(unittest.TestCase):
    def document(self, member):
        with zipfile.ZipFile(ARCHIVE) as archive:
            return json.loads(archive.read(member))

    def test_closed_archive_matches_every_recorded_member(self):
        self.assertEqual(ARCHIVE.stat().st_size, 8020554)
        self.assertEqual(hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                         "2de4b391160ddf86ee874eb5fc56d3d92b0a11979310861320b0e4b72beeda4c")
        with zipfile.ZipFile(ARCHIVE) as archive:
            manifest = json.loads(archive.read("manifest.json"))
            self.assertEqual(len(manifest), 92)
            for name, expected in manifest.items():
                data = archive.read(name)
                self.assertEqual(len(data), expected["bytes"], name)
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected["sha256"], name)
            self.assertFalse(any(n.lower().endswith(".mp4") for n in archive.namelist()))

    def test_required_preflight_pass_does_not_imply_search_success(self):
        pre = self.document("physical_green_house_pilot5/server_preflight.json")
        result = self.document("physical_green_house_pilot5/full_test.json")
        self.assertTrue(pre["passed"])
        self.assertFalse(result["passed"])
        self.assertFalse(result["automatic_arrival"])
        self.assertIn("Native actuator unhealthy", result["failure_reason"])
        self.assertGreater(result["search_elapsed_s"], 168)
        self.assertLess(result["search_elapsed_s"], 170)
        self.assertEqual(result["queries_sent"], ["green residential house"])
        self.assertEqual(result["map_reload_count"], 0)
        self.assertEqual(result["vehicle_teleports_during_sequence"], 0)

    def test_receive_gap_is_upstream_of_ros_control_publication(self):
        audit = self.document("physical_green_house_pilot5/interface_gap_audit.json")
        gaps = audit["topic_gaps"]
        for topic in ("/spot1/odom", "/vehicles/ego/sensors/state"):
            worst = gaps[topic][0]
            self.assertGreater(worst["gap_s"], 4.6)
            self.assertLess(worst["start_elapsed_s"], 168)
            self.assertGreater(worst["end_elapsed_s"], 172)
        for topic in ("/wildos/control_status", "/wildos/native_control"):
            self.assertLess(gaps[topic][0]["gap_s"], .1)
        self.assertGreater(gaps["/wildos/native_control_status"][0]["gap_s"], 2)
        self.assertEqual(audit["mask_count"], 0)
        self.assertEqual(audit["confirmation_count"], 0)

    def test_failure_cleanup_retains_real_physics_and_production(self):
        audit = self.document("physical_green_house_pilot5/interface_gap_audit.json")
        result = self.document("physical_green_house_pilot5/full_test.json")
        cleanup = self.document("pilot5_session/cleanup.json")
        self.assertEqual(cleanup["remaining"], [])
        self.assertEqual(audit["production_status"], "")
        self.assertTrue(result["simulation_paused"])
        for key in ("physics_before", "physics_after"):
            state = result[key]
            self.assertLess(state["speed"], .05)
            self.assertEqual(state["fixedNodeCount"], 0)
            self.assertEqual(state["bodyCollisionNodeCount"], 467)
            self.assertEqual([w["collisionNodeCount"] for w in state["wheels"]], [32] * 4)
        self.assertFalse(result["failure_image"]["is_arrival_evidence"])


if __name__ == "__main__":
    unittest.main()
