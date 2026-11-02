"""Independent field-trial acceptance and odometry metrics."""

import math


def query_consumers_ready(node_names):
    """The vision subscription is created after its models initialize.

    Recorders and diagnostic subscribers cannot substitute for a consumer.
    """
    return {"wildos", "obj_mask_triangulator", "wildos_graph_path_bridge"}.issubset(
        node_names
    )


def normalize_query(query):
    return " ".join(str(query).casefold().split())


def independent_arrival_audit(
    odom_xy,
    odom_stamp_ns,
    target,
    target_stamp_ns,
    evidence,
    query,
    query_started_ns,
    now_ns,
    evidence_received_ns,
):
    if odom_xy is None or target is None or not evidence:
        return {"verified": False, "reason": "missing_pose_or_visual_evidence"}
    if not (
        query_started_ns <= odom_stamp_ns <= now_ns
        and query_started_ns <= target_stamp_ns <= now_ns
        and normalize_query(evidence.get("query", "")) == normalize_query(query)
        and evidence.get("frame") == "spot1/odom"
        and evidence.get("search_confirmed") is True
    ):
        return {"verified": False, "reason": "query_or_frame_mismatch"}
    try:
        observation = tuple(map(float, evidence["object_position"]))
        target_xyz = tuple(map(float, target))
        vehicle = tuple(map(float, odom_xy))
        observation_stamp = int(evidence["observation_stamp_ns"])
    except (KeyError, TypeError, ValueError):
        return {"verified": False, "reason": "invalid_evidence_fields"}
    if len(observation) != 3 or len(target_xyz) != 3 or len(vehicle) != 2:
        return {"verified": False, "reason": "invalid_position_dimensions"}
    if not all(math.isfinite(value) for value in (*observation, *target_xyz, *vehicle)):
        return {"verified": False, "reason": "non_finite_position"}
    age_ns = now_ns - observation_stamp
    if not (
        isinstance(evidence_received_ns, int)
        and query_started_ns <= observation_stamp <= evidence_received_ns <= now_ns
        and evidence_received_ns - observation_stamp <= 5_000_000_000
    ):
        return {"verified": False, "reason": "visual_evidence_not_fresh_when_received"}
    distances = {
        "vehicle_to_target_xy_m": math.hypot(
            vehicle[0] - target_xyz[0], vehicle[1] - target_xyz[1]
        ),
        "target_to_observation_xy_m": math.hypot(
            target_xyz[0] - observation[0], target_xyz[1] - observation[1]
        ),
        "vehicle_to_observation_xy_m": math.hypot(
            vehicle[0] - observation[0], vehicle[1] - observation[1]
        ),
    }
    verified = all(value < 3.0 for value in distances.values())
    return {
        "verified": verified,
        "reason": "within_xy_3m" if verified else "outside_xy_3m",
        "age_s": age_ns / 1_000_000_000,
        "receipt_age_s": (evidence_received_ns - observation_stamp) / 1_000_000_000,
        "evidence_received_ns": evidence_received_ns,
        "evidence_basis": "current_task_confirmed_object",
        **distances,
        "vehicle_xy": vehicle,
        "target_xyz": target_xyz,
        "observation_xyz": observation,
    }


def motion_metrics(samples, horizon_s, stop_threshold=0.05):
    """Summarize sampled motion up to ``horizon_s`` seconds.

    Position between adjacent odometry samples is interpolated linearly when
    the requested horizon clips the final interval. Gaps over one second are
    reported but excluded from measured-time stop fraction.
    """
    horizon_s = max(0.0, float(horizon_s))
    observed_s = near_stop_s = distance_m = gap_s = 0.0
    longest_near_stop_s = current_near_stop_s = 0.0
    gaps = 0
    first_xy = None
    end_xy = None
    last_time = 0.0

    for current, following in zip(samples, samples[1:]):
        start, vx, vy, x, y = current
        end, _, _, next_x, next_y = following
        if start >= horizon_s:
            break
        interval = min(end, horizon_s) - start
        if interval <= 0:
            continue
        if first_xy is None:
            first_xy = (x, y)
        ratio = interval / (end - start)
        clipped_xy = (x + (next_x - x) * ratio, y + (next_y - y) * ratio)
        if end - start > 1.0:
            gaps += 1
            gap_s += interval
            current_near_stop_s = 0.0
        else:
            observed_s += interval
            distance_m += math.hypot(next_x - x, next_y - y) * ratio
            if math.hypot(vx, vy) < stop_threshold:
                near_stop_s += interval
                current_near_stop_s += interval
                longest_near_stop_s = max(longest_near_stop_s, current_near_stop_s)
            else:
                current_near_stop_s = 0.0
        end_xy = clipped_xy
        last_time = min(end, horizon_s)

    if first_xy is None and samples and samples[0][0] <= horizon_s:
        first_xy = (samples[0][3], samples[0][4])
        end_xy = first_xy
    net_displacement = (
        math.hypot(end_xy[0] - first_xy[0], end_xy[1] - first_xy[1])
        if first_xy is not None and end_xy is not None
        else 0.0
    )
    return {
        "horizon_s": horizon_s,
        "observed_s": observed_s,
        "gap_s": gap_s,
        "coverage_fraction": observed_s / horizon_s if horizon_s else None,
        "near_stop_s": near_stop_s,
        "near_stop_fraction": near_stop_s / observed_s if observed_s else None,
        "longest_near_stop_s": longest_near_stop_s,
        "distance_m": distance_m,
        "net_displacement_m": net_displacement,
        "odometry_gaps_over_1s": gaps,
        "last_sampled_time_s": last_time,
        "stop_threshold_mps": stop_threshold,
    }
