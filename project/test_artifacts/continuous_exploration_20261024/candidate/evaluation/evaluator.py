"""Offline scoring only. No simulator, algorithm inputs, or third-party packages."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


class Invalid(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise Invalid(message)


def fields(value, names, where):
    require(isinstance(value, dict), where + ": expected object")
    require(set(value) == set(names.split()), where + ": missing or unknown fields")


def text(value, where):
    require(isinstance(value, str) and bool(value.strip()), where + ": expected nonempty string")


def number(value, where, minimum=0):
    require(type(value) in (int, float), where + ": expected number, not bool/null")
    try:
        valid = math.isfinite(value) and value >= minimum
    except OverflowError:
        valid = False
    require(valid, where + ": nonfinite or out of range")
    return value


def point(value, where):
    require(isinstance(value, list) and len(value) == 3, where + ": expected xyz metres")
    for coordinate in value:
        number(coordinate, where, -math.inf)


def boolean(value, where):
    require(type(value) is bool, where + ": expected boolean")


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def load(path):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key: " + key)
            result[key] = value
        return result

    def bad_constant(value):
        raise Invalid("nonfinite JSON constant: " + value)

    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream, object_pairs_hook=pairs, parse_constant=bad_constant)


def validate_scenario(doc):
    fields(doc, "schema_version status map_id map_revision frame_id vehicle_profile layout_revision acceptance_ref tasks", "scenario")
    require(doc["schema_version"] == "wildos-scenarios/1", "unsupported scenario schema")
    require(doc["status"] in ("draft", "ready"), "invalid scenario status")
    unresolved = []
    for key in ("map_id", "map_revision", "frame_id", "vehicle_profile", "layout_revision", "acceptance_ref"):
        if doc[key] is None:
            unresolved.append(key)
        else:
            text(doc[key], key)
    require(isinstance(doc["tasks"], list) and len(doc["tasks"]) == 3, "three task families required")
    families = set()
    for task in doc["tasks"]:
        fields(task, "family task_id start_xyz target_identity reachable_target_xyz target_object_xyz approximate_prior protocol_revision", "task")
        require(task["family"] in ("long_range_search", "fence_approach", "dead_end"), "unknown family")
        require(task["family"] not in families, "duplicate family")
        families.add(task["family"])
        for key in ("task_id", "target_identity", "protocol_revision"):
            text(task[key], key)
        for key in ("start_xyz", "reachable_target_xyz", "target_object_xyz"):
            if task[key] is None:
                unresolved.append(task["family"] + "." + key)
            else:
                point(task[key], key)
        prior = task["approximate_prior"]
        fields(prior, "description center_xyz radius_m source_ref", "prior")
        text(prior["description"], "prior.description")
        for key in ("center_xyz", "radius_m", "source_ref"):
            if prior[key] is None:
                unresolved.append(task["family"] + ".prior." + key)
            elif key == "center_xyz":
                point(prior[key], key)
            elif key == "radius_m":
                require(number(prior[key], key) > 0, "prior radius must be positive")
            else:
                text(prior[key], key)
    require(not (doc["status"] == "ready" and unresolved), "ready rejected: unresolved " + ", ".join(unresolved))
    return {"status": doc["status"], "ready": doc["status"] == "ready", "unresolved": unresolved,
            "note": "Structural validation only; acceptance_ref requires independent field verification."}


def evaluate(doc):
    fields(doc, "schema_version metadata protocol scoring_target complete end frames", "episode")
    require(doc["schema_version"] == "wildos-episode/2", "unsupported episode schema")
    require(doc["complete"] is True, "partial episode rejected")
    meta = doc["metadata"]
    fields(meta, "run_id episode_id trial_id seed provenance simulator_version map_id map_revision layout_revision scenario_id protocol_revision vehicle_profile sensor_profile environment_profile algorithm baseline_name algorithm_revision input_contract localization_mode prior_revision frame_id frame_epoch position_source collision_source intervention_source", "metadata")
    for key, value in meta.items():
        if key == "seed":
            require(type(value) is int and value >= 0, "seed must be nonnegative integer")
        else:
            text(value, "metadata." + key)
    require(meta["provenance"] in ("synthetic_fixture", "beamng_recording"), "invalid provenance")
    require(meta["localization_mode"] in ("oracle_pose", "estimated_pose"), "invalid localization_mode")
    require(meta["position_source"] == "scorer_world_pose", "distance requires continuous scorer world pose")
    protocol = doc["protocol"]
    fields(protocol, "family task_mode budget_s success_radius_m dwell_s max_sample_gap_s distance_tolerance_m collision_policy intervention_policy", "protocol")
    require(protocol["family"] in ("long_range_search", "fence_approach", "dead_end"), "unknown family")
    mode = protocol["task_mode"]
    require(mode in ("object_search", "point_goal"), "unknown task mode")
    allowed = {"object_search": {"sensors_only", "sensors_approximate_prior"}, "point_goal": {"sensors_exact_point_goal"}}
    require(meta["input_contract"] in allowed[mode], "task/input contract mismatch; target GT forbidden for object search")
    if meta["input_contract"] == "sensors_approximate_prior":
        require(meta["prior_revision"] != "none", "prior revision required")
    else:
        require(meta["prior_revision"] == "none", "undeclared prior")
    for key in ("budget_s", "success_radius_m", "dwell_s", "max_sample_gap_s", "distance_tolerance_m"):
        require(number(protocol[key], key) > 0, key + " must be positive")
    require(protocol["distance_tolerance_m"] <= 0.001, "distance tolerance cannot exceed 1 mm")
    require(protocol["dwell_s"] <= protocol["budget_s"], "dwell exceeds budget")
    require(protocol["max_sample_gap_s"] <= protocol["dwell_s"], "sample gap exceeds dwell")
    require(protocol["collision_policy"] in ("report_only", "disqualify"), "invalid collision policy")
    require(protocol["intervention_policy"] in ("report_only", "disqualify"), "invalid intervention policy")
    target = doc["scoring_target"]
    fields(target, "identity reachable_xyz reachable_verified verification_ref", "scoring_target")
    text(target["identity"], "target identity")
    point(target["reachable_xyz"], "reachable target")
    require(target["reachable_verified"] is True, "reachable target not verified")
    text(target["verification_ref"], "reachability verification")
    end = doc["end"]
    fields(end, "sim_time_s outcome reason reported_success", "end")
    number(end["sim_time_s"], "end time")
    require(end["outcome"] in ("success", "failure", "unknown"), "invalid outcome")
    text(end["reason"], "end reason")
    expected_report = {"success": True, "failure": False, "unknown": None}[end["outcome"]]
    require(end["reported_success"] is expected_report, "outcome/reported_success mismatch")
    frames = doc["frames"]
    require(isinstance(frames, list) and len(frames) >= 2, "at least two complete frames required")
    distance = 0.0
    previous = None
    dwell_start = None
    qualified_at = None
    best_dwell = 0.0
    events = {"collisions": {}, "interventions": {}}
    for index, frame in enumerate(frames):
        fields(frame, "seq sim_time_s frame_id frame_epoch position_xyz goal_distance_m target_evidence collisions interventions", "frame")
        require(type(frame["seq"]) is int and frame["seq"] == index, "missing/duplicate/reset frame sequence")
        require(frame["frame_id"] == meta["frame_id"] and frame["frame_epoch"] == meta["frame_epoch"], "coordinate frame/epoch reset")
        t = number(frame["sim_time_s"], "sim time")
        point(frame["position_xyz"], "position")
        number(frame["goal_distance_m"], "goal distance")
        actual = math.dist(frame["position_xyz"], target["reachable_xyz"])
        number(actual, "computed goal distance")
        require(abs(actual - frame["goal_distance_m"]) <= protocol["distance_tolerance_m"], "goal distance inconsistent with reachable target")
        if previous is None:
            require(t == 0, "first frame must be episode-relative time zero")
        else:
            delta = t - previous["sim_time_s"]
            require(0 < delta <= protocol["max_sample_gap_s"], "duplicate/reversed time or missing sample gap")
            distance += math.dist(previous["position_xyz"], frame["position_xyz"])
            number(distance, "derived path distance")
        require(t <= protocol["budget_s"], "episode exceeds declared budget")
        evidence = frame["target_evidence"]
        matched = False
        if evidence is not None:
            fields(evidence, "identity observed_sim_time_s sensor_ref scorer_verified", "target_evidence")
            text(evidence["identity"], "evidence identity")
            text(evidence["sensor_ref"], "sensor reference")
            number(evidence["observed_sim_time_s"], "evidence time")
            require(evidence["observed_sim_time_s"] == t, "stale/future evidence")
            boolean(evidence["scorer_verified"], "evidence verification")
            matched = evidence["scorer_verified"] and evidence["identity"] == target["identity"]
        for category, registry in events.items():
            require(isinstance(frame[category], list), category + ": expected list; null is not zero")
            seen = set()
            for event in frame[category]:
                fields(event, "event_id kind started_sim_time_s", category)
                text(event["event_id"], "event id")
                text(event["kind"], "event kind")
                started = number(event["started_sim_time_s"], "event start")
                identifier = event["event_id"]
                require(identifier not in seen, "duplicate event within frame")
                seen.add(identifier)
                require(started <= t, "future event")
                if identifier in registry:
                    require(registry[identifier] == event, "event ID reused with different attributes")
                else:
                    require((previous is None and started == 0) or
                            (previous is not None and previous["sim_time_s"] < started <= t), "late event or reused event ID")
                    registry[identifier] = event
        qualifies = actual <= protocol["success_radius_m"] and (mode == "point_goal" or matched)
        if qualifies:
            if dwell_start is None:
                dwell_start = t
            best_dwell = max(best_dwell, t - dwell_start)
            if qualified_at is None and t - dwell_start >= protocol["dwell_s"]:
                qualified_at = t
        else:
            dwell_start = None
        previous = frame
    require(end["sim_time_s"] == frames[-1]["sim_time_s"], "end time differs from final frame")
    disqualified = (protocol["collision_policy"] == "disqualify" and bool(events["collisions"])) or (protocol["intervention_policy"] == "disqualify" and bool(events["interventions"]))
    success = end["outcome"] == "success" and qualified_at is not None and not disqualified
    # Includes declared localization mode, not proof of physical pose provenance.
    excluded = {"run_id", "episode_id", "trial_id", "seed", "algorithm", "baseline_name", "algorithm_revision"}
    comparison = {"metadata": {key: value for key, value in meta.items() if key not in excluded},
                  "protocol": protocol, "scoring_target": target}
    group_id = hashlib.sha256(canonical(comparison).encode()).hexdigest()
    return {"metadata": meta, "comparison_group": group_id, "comparison_contract": comparison,
            "distance_m": distance, "elapsed_sim_s": end["sim_time_s"],
            "unique_collisions": len(events["collisions"]), "interventions": len(events["interventions"]),
            "max_observed_dwell_s": best_dwell, "qualified_at_sim_s": qualified_at,
            "reported_outcome": end["outcome"], "success": success,
            "unknown_outcome": end["outcome"] == "unknown", "disqualified": bool(disqualified),
            "success_claim_rejected": end["outcome"] == "success" and not success}


def summarize(results):
    groups = {}
    identities = set()
    trials = set()
    for result in results:
        meta = result["metadata"]
        identity = (meta["run_id"], meta["episode_id"])
        require(identity not in identities, "duplicate run/episode")
        identities.add(identity)
        method = (meta["algorithm"], meta["baseline_name"], meta["algorithm_revision"])
        key = (result["comparison_group"], method)
        trial = (key, meta["trial_id"], meta["seed"])
        require(trial not in trials, "duplicate method/trial/seed within comparison group")
        trials.add(trial)
        groups.setdefault(key, []).append(result)
    output = []
    for (group_id, method), rows in sorted(groups.items()):
        n = len(rows)
        output.append({"comparison_group": group_id, "algorithm": method[0], "baseline_name": method[1],
                       "localization_mode": rows[0]["metadata"]["localization_mode"],
                       "algorithm_revision": method[2], "episodes": n,
                       "successes": sum(row["success"] for row in rows),
                       "unknown_outcomes": sum(row["unknown_outcome"] for row in rows),
                       "success_claims_rejected": sum(row["success_claim_rejected"] for row in rows),
                       "success_rate_all_valid_episodes": sum(row["success"] for row in rows) / n,
                       "mean_distance_m": sum(row["distance_m"] / n for row in rows),
                       "mean_elapsed_sim_s": sum(row["elapsed_sim_s"] / n for row in rows),
                       "total_unique_collisions": sum(row["unique_collisions"] for row in rows),
                       "total_interventions": sum(row["interventions"] for row in rows),
                       "trial_seed_pairs": sorted([[row["metadata"]["trial_id"], row["metadata"]["seed"]] for row in rows])})
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser("validate-scenario", help="validate draft/ready scenario structure")
    validate.add_argument("path")
    score = commands.add_parser("score", help="validate and score wildos-episode/2 files; separate localization modes")
    score.add_argument("paths", nargs="+")
    args = parser.parse_args(argv)
    try:
        if args.command == "validate-scenario":
            report = validate_scenario(load(args.path))
        else:
            results = []
            for path in args.paths:
                try:
                    results.append(evaluate(load(path)))
                except (Invalid, ValueError, OSError) as exc:
                    raise Invalid(str(path) + ": " + str(exc)) from exc
            report = {"evaluator_version": "2", "episodes": results, "summaries": summarize(results),
                      "comparison_warning": "Compare only equal group IDs AND equal trial_seed_pairs. No automatic rankings or cross-group pooling, including oracle_pose with estimated_pose. Oracle pose is not full reproduction. Synthetic fixtures are not BeamNG results.",
                      "validation_warning": "Metadata/provenance declarations and internal consistency only; physical input sources and truthful information boundaries are not verified."}
        print(json.dumps(report, ensure_ascii=True, indent=2, allow_nan=False))
        return 0
    except (Invalid, ValueError, OSError, OverflowError) as exc:
        print("validation error: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
