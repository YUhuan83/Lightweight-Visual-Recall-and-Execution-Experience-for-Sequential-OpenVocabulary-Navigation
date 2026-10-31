"""All generated episodes are SYNTHETIC FIXTURES, never BeamNG results."""

import contextlib
import copy
import io
import json
import unittest
from pathlib import Path

import evaluator as ev


def fixture():
    """Artificial straight line: 3 metres, then a two-second sampled dwell."""
    metadata = {key: "fixture-only" for key in (
        "run_id episode_id trial_id simulator_version map_id map_revision layout_revision scenario_id "
        "protocol_revision vehicle_profile sensor_profile environment_profile algorithm baseline_name "
        "algorithm_revision frame_id frame_epoch collision_source intervention_source").split()}
    metadata.update(seed=1, provenance="synthetic_fixture", input_contract="sensors_only",
                    localization_mode="estimated_pose", prior_revision="none", position_source="scorer_world_pose")
    frames = []
    for index, x in enumerate((0, 1, 2, 3, 3, 3)):
        frames.append({"seq": index, "sim_time_s": index, "frame_id": metadata["frame_id"],
                       "frame_epoch": metadata["frame_epoch"], "position_xyz": [x, 0, 0],
                       "goal_distance_m": 3 - x,
                       "target_evidence": {"identity": "fixture barrel", "observed_sim_time_s": index,
                                           "sensor_ref": "synthetic://frame/" + str(index), "scorer_verified": True},
                       "collisions": [], "interventions": []})
    return {"schema_version": "wildos-episode/2", "metadata": metadata,
            "protocol": {"family": "long_range_search", "task_mode": "object_search", "budget_s": 20,
                         "success_radius_m": 0.5, "dwell_s": 2, "max_sample_gap_s": 1,
                         "distance_tolerance_m": 0.001, "collision_policy": "report_only",
                         "intervention_policy": "disqualify"},
            "scoring_target": {"identity": "fixture barrel", "reachable_xyz": [3, 0, 0],
                               "reachable_verified": True, "verification_ref": "synthetic-only"},
            "complete": True, "end": {"sim_time_s": 5, "outcome": "success",
                                       "reason": "synthetic dwell", "reported_success": True}, "frames": frames}


class EvaluatorTests(unittest.TestCase):
    def test_localization_modes_and_grouping(self):
        for contract in ("sensors_only", "sensors_approximate_prior", "sensors_exact_point_goal"):
            with self.subTest(input_contract=contract):
                results = []
                for mode in ("oracle_pose", "estimated_pose"):
                    doc = fixture()
                    doc["metadata"].update(localization_mode=mode, episode_id=mode, input_contract=contract)
                    if contract == "sensors_approximate_prior":
                        doc["metadata"]["prior_revision"] = "synthetic-prior"
                    elif contract == "sensors_exact_point_goal":
                        doc["protocol"]["task_mode"] = "point_goal"
                    result = ev.evaluate(doc)
                    self.assertEqual(result["comparison_contract"]["metadata"]["localization_mode"], mode)
                    self.assertEqual(result["metadata"]["position_source"], "scorer_world_pose")
                    self.assertTrue(result["success"])
                    self.assertEqual((result["distance_m"], result["elapsed_sim_s"]), (3, 5))
                    results.append(result)
                self.assertNotEqual(results[0]["comparison_group"], results[1]["comparison_group"])
                summaries = ev.summarize(results)
                self.assertEqual(len(summaries), 2)
                self.assertEqual({row["localization_mode"] for row in summaries}, {"oracle_pose", "estimated_pose"})
                self.assertTrue(all(row["episodes"] == 1 for row in summaries))
                self.assertEqual(summaries[0]["trial_seed_pairs"], summaries[1]["trial_seed_pairs"])

    def test_invalid_localization_modes(self):
        for mode in ("oracle", "estimated", "truth", "", None, True, 1, [], {}):
            with self.subTest(mode=mode):
                doc = fixture()
                doc["metadata"]["localization_mode"] = mode
                with self.assertRaises(ev.Invalid):
                    ev.evaluate(doc)
        doc = fixture()
        del doc["metadata"]["localization_mode"]
        with self.assertRaises(ev.Invalid):
            ev.evaluate(doc)

    def test_old_episode_schema_rejected(self):
        for include_mode in (True, False):
            doc = fixture()
            doc["schema_version"] = "wildos-episode/1"
            if not include_mode:
                del doc["metadata"]["localization_mode"]
            with self.assertRaisesRegex(ev.Invalid, "unsupported episode schema"):
                ev.evaluate(doc)

    def test_known_metrics(self):
        result = ev.evaluate(fixture())
        self.assertTrue(result["success"])
        self.assertEqual((result["distance_m"], result["elapsed_sim_s"], result["qualified_at_sim_s"]), (3, 5, 5))
        self.assertEqual(result["max_observed_dwell_s"], 2)
        self.assertEqual((result["unique_collisions"], result["interventions"]), (0, 0))

    def test_unique_events_and_disqualification(self):
        doc = fixture()
        event = {"event_id": "contact-1", "kind": "fence", "started_sim_time_s": 1}
        for frame in doc["frames"][1:]:
            frame["collisions"] = [copy.deepcopy(event)]
        doc["frames"][4]["collisions"].append({"event_id": "contact-2", "kind": "fence", "started_sim_time_s": 4})
        result = ev.evaluate(doc)
        self.assertEqual(result["unique_collisions"], 2)
        self.assertTrue(result["success"])
        doc["protocol"]["collision_policy"] = "disqualify"
        self.assertFalse(ev.evaluate(doc)["success"])
        doc["protocol"]["collision_policy"] = "report_only"
        doc["frames"][2]["interventions"] = [{"event_id": "override-1", "kind": "manual_control", "started_sim_time_s": 2}]
        result = ev.evaluate(doc)
        self.assertEqual(result["interventions"], 1)
        self.assertTrue(result["disqualified"])

    def test_absent_wrong_unverified_or_interrupted_evidence(self):
        for change in ("absent", "wrong", "unverified", "interrupted"):
            with self.subTest(change=change):
                doc = fixture()
                for frame in doc["frames"][3:]:
                    if change == "absent":
                        frame["target_evidence"] = None
                    elif change == "wrong":
                        frame["target_evidence"]["identity"] = "distractor"
                    elif change == "unverified":
                        frame["target_evidence"]["scorer_verified"] = False
                if change == "interrupted":
                    doc["frames"][4]["target_evidence"] = None
                result = ev.evaluate(doc)
                self.assertFalse(result["success"])
                self.assertTrue(result["success_claim_rejected"])

    def test_short_dwell_and_outside_radius(self):
        doc = fixture()
        doc["protocol"]["dwell_s"] = 3
        self.assertFalse(ev.evaluate(doc)["success"])
        doc = fixture()
        doc["scoring_target"]["reachable_xyz"] = [30, 0, 0]
        for frame in doc["frames"]:
            frame["goal_distance_m"] = 30 - frame["position_xyz"][0]
        self.assertFalse(ev.evaluate(doc)["success"])

    def test_unknown_and_failure_never_success(self):
        for outcome, reported in (("unknown", None), ("failure", False)):
            doc = fixture()
            doc["end"].update(outcome=outcome, reported_success=reported)
            result = ev.evaluate(doc)
            self.assertFalse(result["success"])
            summary = ev.summarize([result])[0]
            self.assertEqual(summary["success_rate_all_valid_episodes"], 0)
            self.assertEqual(summary["unknown_outcomes"], int(outcome == "unknown"))

    def test_point_goal_is_not_object_search(self):
        doc = fixture()
        doc["protocol"]["task_mode"] = "point_goal"
        doc["metadata"]["input_contract"] = "sensors_exact_point_goal"
        for frame in doc["frames"]:
            frame["target_evidence"] = None
        result = ev.evaluate(doc)
        self.assertTrue(result["success"])
        self.assertNotEqual(result["comparison_group"], ev.evaluate(fixture())["comparison_group"])
        doc["protocol"]["task_mode"] = "object_search"
        with self.assertRaises(ev.Invalid):
            ev.evaluate(doc)

    def test_invalid_records(self):
        changes = [
            ("nan", lambda d: d["frames"][1].update(sim_time_s=float("nan"))),
            ("inf", lambda d: d["frames"][1].update(position_xyz=[float("inf"), 0, 0])),
            ("bool time", lambda d: d["frames"][1].update(sim_time_s=True)),
            ("huge", lambda d: d["frames"][1].update(goal_distance_m=10 ** 400)),
            ("reverse", lambda d: d["frames"][2].update(sim_time_s=0.5)),
            ("duplicate time", lambda d: d["frames"][2].update(sim_time_s=1)),
            ("gap", lambda d: d["frames"][2].update(sim_time_s=2.5)),
            ("sequence reset", lambda d: d["frames"][2].update(seq=0)),
            ("frame reset", lambda d: d["frames"][2].update(frame_id="new")),
            ("epoch reset", lambda d: d["frames"][2].update(frame_epoch="new")),
            ("partial", lambda d: d.update(complete=False)),
            ("missing", lambda d: d["frames"][2].pop("collisions")),
            ("null event list", lambda d: d["frames"][2].update(interventions=None)),
            ("end mismatch", lambda d: d["end"].update(sim_time_s=6)),
            ("report mismatch", lambda d: d["end"].update(reported_success=None)),
            ("unknown outcome", lambda d: d["end"].update(outcome="timeout")),
            ("false distance", lambda d: d["frames"][2].update(goal_distance_m=0)),
            ("unreachable", lambda d: d["scoring_target"].update(reachable_verified=False)),
            ("stale evidence", lambda d: d["frames"][2]["target_evidence"].update(observed_sim_time_s=1)),
            ("future evidence", lambda d: d["frames"][2]["target_evidence"].update(observed_sim_time_s=3)),
            ("budget", lambda d: d["protocol"].update(budget_s=4)),
            ("zero dwell", lambda d: d["protocol"].update(dwell_s=0)),
            ("loose tolerance", lambda d: d["protocol"].update(distance_tolerance_m=100)),
            ("not zero start", lambda d: d["frames"][0].update(sim_time_s=0.1)),
            ("unknown field", lambda d: d.update(teleport=True)),
            ("odometry", lambda d: d["metadata"].update(position_source="resetting_odometry")),
            ("prior missing", lambda d: d["metadata"].update(input_contract="sensors_approximate_prior")),
        ]
        for label, change in changes:
            with self.subTest(label=label):
                doc = fixture()
                change(doc)
                with self.assertRaises(ev.Invalid):
                    ev.evaluate(doc)

    def test_bad_events(self):
        for kind in ("future", "late", "reused", "duplicate"):
            with self.subTest(kind=kind):
                doc = fixture()
                event = {"event_id": "id", "kind": "contact", "started_sim_time_s": 1}
                doc["frames"][1]["collisions"] = [event]
                if kind == "future":
                    event["started_sim_time_s"] = 2
                elif kind == "late":
                    event["started_sim_time_s"] = 0
                elif kind == "reused":
                    doc["frames"][2]["collisions"] = [dict(event, kind="different")]
                else:
                    doc["frames"][1]["collisions"].append(copy.deepcopy(event))
                with self.assertRaises(ev.Invalid):
                    ev.evaluate(doc)

    def test_grouping_and_duplicates(self):
        first = ev.evaluate(fixture())
        for section, key, value in (("metadata", "map_revision", "different"),
                                    ("metadata", "sensor_profile", "different"),
                                    ("metadata", "protocol_revision", "different"),
                                    ("metadata", "provenance", "beamng_recording"),
                                    ("protocol", "dwell_s", 3),
                                    ("protocol", "family", "dead_end")):
            doc = fixture()
            doc[section][key] = value
            self.assertNotEqual(first["comparison_group"], ev.evaluate(doc)["comparison_group"])
        doc = fixture()
        doc["metadata"].update(algorithm="baseline", baseline_name="frontier", episode_id="other")
        other = ev.evaluate(doc)
        self.assertEqual(first["comparison_group"], other["comparison_group"])
        self.assertEqual(len(ev.summarize([first, other])), 2)
        with self.assertRaises(ev.Invalid):
            ev.summarize([first, first])
        doc = fixture()
        doc["metadata"]["episode_id"] = "renamed"
        with self.assertRaises(ev.Invalid):
            ev.summarize([first, ev.evaluate(doc)])

    def test_draft_not_ready(self):
        doc = ev.load(Path(__file__).with_name("scenario_template.json"))
        result = ev.validate_scenario(doc)
        self.assertFalse(result["ready"])
        self.assertEqual(len(result["unresolved"]), 24)
        doc["status"] = "ready"
        with self.assertRaises(ev.Invalid):
            ev.validate_scenario(doc)

    def test_json_parser(self):
        from unittest.mock import patch
        for data in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', '{broken'):
            with self.subTest(data=data), patch.object(Path, "open", return_value=io.StringIO(data)):
                with self.assertRaises(ValueError):
                    ev.load("unused.json")

    def test_cli_success_and_atomic_error(self):
        from unittest.mock import patch
        out, err = io.StringIO(), io.StringIO()
        with patch.object(ev, "load", return_value=fixture()), contextlib.redirect_stdout(out):
            self.assertEqual(ev.main(["score", "synthetic-only.json"]), 0)
        report = json.loads(out.getvalue())
        self.assertEqual(report["evaluator_version"], "2")
        self.assertEqual(report["summaries"][0]["successes"], 1)
        self.assertEqual(report["summaries"][0]["localization_mode"], "estimated_pose")
        self.assertIn("physical input sources", report["validation_warning"])
        self.assertIn("oracle_pose with estimated_pose", report["comparison_warning"])
        out = io.StringIO()
        with patch.object(ev, "load", side_effect=[fixture(), ev.Invalid("bad fixture")]), contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            self.assertEqual(ev.main(["score", "one", "two"]), 2)
        self.assertEqual(out.getvalue(), "")
        self.assertIn("two: bad fixture", err.getvalue())


if __name__ == "__main__":
    unittest.main()
