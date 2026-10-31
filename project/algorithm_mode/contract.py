"""Strict, stdlib-only configuration and synchronized observation contract."""

import copy
import json
import math


class Invalid(ValueError):
    pass


def require(ok, message):
    if not ok:
        raise Invalid(message)


def fields(value, names):
    require(type(value) is dict and set(value) == set(names.split()),
            "Missing or unknown fields: " + names)


def text(value):
    require(type(value) is str and bool(value.strip()), "Expected nonempty text")


def number(value):
    require(type(value) in (int, float), "Expected number, not bool/null")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    require(finite, "Nonfinite number")
    return value


def vector(value, size=3):
    require(type(value) is list and len(value) == size, "Invalid vector size")
    for item in value:
        number(item)


def transpose(a):
    return [list(row) for row in zip(*a)]


def mv(a, v):
    return [sum(x * y for x, y in zip(row, v)) for row in a]


def mm(a, b):
    return [mv(transpose(b), row) for row in a]


def rotation(value):
    require(type(value) is list and len(value) == 3, "Expected 3x3 rotation")
    for row in value:
        vector(row)
    product = mm(transpose(value), value)
    require(all(abs(product[i][j] - (i == j)) < 1e-9
                for i in range(3) for j in range(3)), "Rotation not orthonormal")
    a, b, c = value
    det = (a[0] * (b[1]*c[2] - b[2]*c[1])
           - a[1] * (b[0]*c[2] - b[2]*c[0])
           + a[2] * (b[0]*c[1] - b[1]*c[0]))
    require(abs(det - 1) < 1e-9, "Rotation must be right handed")


PROFILES = {
    "oracle_pose": {"pose_source": "ego_truth", "claim": "experimental_upper_bound_not_full_reproduction"},
    "estimated_pose": {"pose_source": "lio", "claim": "estimated_localization_phase"},
}


def load_config(path):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "Duplicate JSON key")
            result[key] = value
        return result

    def bad_constant(value):
        raise Invalid("Nonfinite JSON constant: " + value)

    with open(path, encoding="utf-8") as stream:
        config = json.load(stream, object_pairs_hook=pairs, parse_constant=bad_constant)
    validate_config(config)
    return config

def validate_config(config, ready=False):
    fields(config, "schema_version profiles forbidden_inputs calibration")
    require(config["schema_version"] == "wildos-algorithm-mode/1", "Wrong schema")
    require(config["profiles"] == PROFILES, "Profiles are fixed")
    require(config["forbidden_inputs"] == ["target_ground_truth", "full_map", "hidden_semantics"],
            "Information boundary cannot be relaxed")
    cal = config["calibration"]
    fields(cal, "verified evidence_ref source_world_axes source_body_axes nav_body_axes units reference_points_coincident R_source_nav velocity_frame camera_frame lidar_frame sensor_evidence_ref")
    require(type(cal["verified"]) is bool, "verified must be boolean")
    require(cal["nav_body_axes"] == "x_forward_y_left_z_up", "Unsupported nav axes")
    require(cal["units"] == "m_s_rad", "Unsupported units")
    pending = []
    for key in ("evidence_ref", "source_world_axes", "source_body_axes", "R_source_nav",
                "velocity_frame", "camera_frame", "lidar_frame", "sensor_evidence_ref",
                "reference_points_coincident"):
        value = cal[key]
        if value is None:
            pending.append(key)
        elif key == "R_source_nav":
            rotation(value)
        elif key == "velocity_frame":
            require(value in ("source_world", "source_body"), "Unknown velocity frame")
        elif key == "reference_points_coincident":
            require(value is True, "Lever arms are not supported; reference points must coincide")
        else:
            text(value)
    require(not cal["verified"] or not pending, "Verified calibration is incomplete")
    require(not ready or (cal["verified"] and not pending), "Verified calibration required")
    return {"ready": cal["verified"] and not pending, "unresolved": pending}


def validate_observation(obs, mode):
    require(mode in PROFILES, "Unknown localization mode")
    fields(obs, "schema_version localization_mode acquisition_sim_time_s epoch odom camera lidar")
    require(obs["schema_version"] == "wildos-observation/1", "Wrong observation schema")
    require(obs["localization_mode"] == mode, "Localization mode mismatch")
    stamp = number(obs["acquisition_sim_time_s"])
    require(stamp >= 0, "Negative acquisition time")
    text(obs["epoch"])
    odom = obs["odom"]
    fields(odom, "frame_id child_frame_id position_m R_odom_nav linear_velocity_m_s angular_velocity_rad_s velocity_frame covariance provenance")
    require(odom["frame_id"] == "odom" and odom["child_frame_id"] == "base_link",
            "Wrong odometry frames")
    vector(odom["position_m"])
    rotation(odom["R_odom_nav"])
    for key in ("linear_velocity_m_s", "angular_velocity_rad_s"):
        if odom[key] is not None:
            vector(odom[key])
    require(odom["velocity_frame"] == "base_link", "Twist must be in nav body frame")
    require(odom["covariance"] is None, "v1 covariance must explicitly be unknown/null")
    expected = "ego_truth_covariance_unknown" if mode == "oracle_pose" else "lio_covariance_unknown"
    require(odom["provenance"] == expected, "Truth odometry forbidden in estimated mode")
    for kind in ("camera", "lidar"):
        sensor = obs[kind]
        payload = "width height rgb" if kind == "camera" else "points_xyz_m"
        fields(sensor, "acquisition_sim_time_s epoch frame_id " + payload)
        number(sensor["acquisition_sim_time_s"])
        require(sensor["acquisition_sim_time_s"] == stamp and sensor["epoch"] == obs["epoch"],
                "Unsynchronized sensor time/epoch")
        text(sensor["frame_id"])
        if kind == "camera":
            require(all(type(sensor[k]) is int and sensor[k] > 0 for k in ("width", "height")),
                    "Invalid image dimensions")
            require(type(sensor["rgb"]) is bytes and
                    len(sensor["rgb"]) == sensor["width"] * sensor["height"] * 3,
                    "Expected packed RGB8 bytes")
        else:
            require(type(sensor["points_xyz_m"]) is list, "Expected lidar points")
            for point in sensor["points_xyz_m"]:
                vector(point)
    return obs


class OraclePoseAdapter:
    """Offline ego-only projection. Matrices use column vectors, not Euler guesses."""

    def __init__(self, config, origin, mode="oracle_pose"):
        validate_config(config, ready=True)
        require(mode == "oracle_pose", "Oracle adapter cannot serve estimated mode")
        fields(origin, "epoch initial_sim_time_s position_world_m R_world_odom")
        text(origin["epoch"])
        require(number(origin["initial_sim_time_s"]) >= 0, "Negative initial time")
        vector(origin["position_world_m"])
        rotation(origin["R_world_odom"])
        self._cal = copy.deepcopy(config["calibration"])
        self._origin = copy.deepcopy(origin)
        self._last_stamp = None

    def adapt(self, ego, camera, lidar):
        # Raw ego may contain scorer-only keys. Never copy or recursively forward it.
        require(type(ego) is dict, "Expected ego object")
        required = "acquisition_sim_time_s epoch position_world_m R_world_source linear_velocity angular_velocity".split()
        require(all(key in ego for key in required), "Missing ego fields")
        stamp = number(ego["acquisition_sim_time_s"])
        require(ego["epoch"] == self._origin["epoch"], "Epoch reset requires new adapter/episode")
        require(stamp >= self._origin["initial_sim_time_s"], "Time before origin")
        require(self._last_stamp is None or stamp > self._last_stamp, "Duplicate/reversed acquisition time")
        if self._last_stamp is None:
            require(stamp == self._origin["initial_sim_time_s"], "First sample must match origin time")
        vector(ego["position_world_m"])
        rotation(ego["R_world_source"])
        r_world_nav = mm(ego["R_world_source"], self._cal["R_source_nav"])
        r_odom_world = transpose(self._origin["R_world_odom"])
        position = mv(r_odom_world, [a-b for a, b in zip(ego["position_world_m"], self._origin["position_world_m"])])
        velocities = []
        for key in ("linear_velocity", "angular_velocity"):
            value = ego[key]
            if value is not None:
                vector(value)
                r = transpose(r_world_nav) if self._cal["velocity_frame"] == "source_world" else transpose(self._cal["R_source_nav"])
                value = mv(r, value)
            velocities.append(value)
        obs = {
            "schema_version": "wildos-observation/1", "localization_mode": "oracle_pose",
            "acquisition_sim_time_s": stamp, "epoch": self._origin["epoch"],
            "odom": {"frame_id": "odom", "child_frame_id": "base_link", "position_m": position,
                     "R_odom_nav": mm(r_odom_world, r_world_nav),
                     "linear_velocity_m_s": velocities[0], "angular_velocity_rad_s": velocities[1],
                     "velocity_frame": "base_link", "covariance": None,
                     "provenance": "ego_truth_covariance_unknown"},
            "camera": copy.deepcopy(camera), "lidar": copy.deepcopy(lidar),
        }
        # Sensor objects are exact-schema, including nested numeric point vectors.
        validate_observation(obs, "oracle_pose")
        require(obs["camera"]["frame_id"] == self._cal["camera_frame"] and
                obs["lidar"]["frame_id"] == self._cal["lidar_frame"], "Uncalibrated sensor frame")
        self._last_stamp = stamp
        return obs
