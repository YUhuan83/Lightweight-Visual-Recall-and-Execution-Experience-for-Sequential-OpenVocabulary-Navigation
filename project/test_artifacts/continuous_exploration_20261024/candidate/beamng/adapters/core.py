"""Validated normalized inputs only. Transform convention: p_target = R p_source + t."""

import copy
import json
import math
from decimal import Decimal, ROUND_HALF_EVEN, localcontext


class ValidationError(ValueError):
    pass


class BackwardTimeError(ValidationError):
    pass


def require(condition, message):
    if not condition:
        raise ValidationError(message)


def text(value):
    require(isinstance(value, str) and bool(value.strip()), "nonempty string required")
    return value


def number(value):
    require(type(value) in (int, float), "finite numeric value required (not bool/null)")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValidationError("numeric overflow") from exc
    require(math.isfinite(result), "nonfinite value")
    return result


def vector(value, size):
    require(isinstance(value, (list, tuple)) and len(value) == size, "wrong vector size")
    return tuple(number(x) for x in value)


def evidence(value):
    require(isinstance(value, dict), "evidence object required")
    require(value.get("accepted") is True, "calibration evidence not accepted")
    text(value.get("reference"))


def seconds_to_ns(seconds):
    """Nonnegative int/float/Decimal seconds, decimal half-even rounding, int64 ns.

    Floats use their shortest decimal representation, not binary multiplication.
    Decimal is useful for exact replay; Float64 cannot recover lost precision.
    """
    require(type(seconds) in (int, float, Decimal), "numeric acquisition seconds required")
    with localcontext() as context:
        context.prec = 50
        value = Decimal(str(seconds))
        require(value.is_finite() and value >= 0, "invalid simulation seconds")
        require(value <= Decimal(2**63 - 1) / 1000000000, "time exceeds int64 nanoseconds")
        return int((value * 1000000000).to_integral_value(rounding=ROUND_HALF_EVEN))


class AcquisitionClock:
    """Single ordered source/stream. Duplicates drop; any backward value latches.

    Epochs do not offset time or invent stamps. A new unique epoch explicitly
    permits restarting simulation seconds, and requires downstream state flush.
    """

    def __init__(self, source, epoch):
        self.source = text(source)
        self._epochs = set()
        self.begin_epoch(epoch)

    def begin_epoch(self, epoch):
        text(epoch)
        require(epoch not in self._epochs, "epoch must be new")
        self._epochs.add(epoch)
        self.epoch = epoch
        self.last_seconds = None
        self.last_ns = None
        self.latched = False

    def accept(self, seconds, *, source, epoch, time_kind):
        require(not self.latched, "clock latched; explicit begin_epoch required")
        require(source == self.source and epoch == self.epoch, "source/epoch mismatch")
        require(time_kind == "acquisition", "polling/header/wall time is not acquisition time")
        ns = seconds_to_ns(seconds)
        raw = Decimal(str(seconds))
        if self.last_seconds is not None and raw < self.last_seconds:
            self.latched = True
            raise BackwardTimeError("simulation time moved backward; clock latched")
        self.last_seconds = raw
        if ns == self.last_ns:
            return None
        self.last_ns = ns
        return {"sec": ns // 1000000000, "nanosec": ns % 1000000000,
                "ns": ns, "epoch": epoch, "source": source, "time_kind": time_kind}


class RigidTransform:
    def __init__(self, source_frame, target_frame, translation, quaternion_xyzw):
        self.source_frame = text(source_frame)
        self.target_frame = text(target_frame)
        self.translation = vector(translation, 3)
        self.quaternion = vector(quaternion_xyzw, 4)
        x, y, z, w = self.quaternion
        require(abs(math.hypot(x, y, z, w) - 1.0) <= 1e-9, "quaternion must be unit; not auto-normalized")
        self.rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                         (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                         (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))

    def rotate(self, value):
        v = vector(value, 3)
        return vector([sum(row[j]*v[j] for j in range(3)) for row in self.rotation], 3)

    def point(self, value):
        return vector([a+b for a, b in zip(self.rotate(value), self.translation)], 3)

    def inverse(self):
        x, y, z, w = self.quaternion
        inverse = RigidTransform(self.target_frame, self.source_frame, (0, 0, 0), (-x, -y, -z, w))
        return RigidTransform(inverse.source_frame, inverse.target_frame,
                              inverse.rotate(tuple(-x for x in self.translation)), inverse.quaternion)

    def then(self, other):
        require(self.target_frame == other.source_frame, "composition frame mismatch")
        ax, ay, az, aw = other.quaternion
        bx, by, bz, bw = self.quaternion
        q = (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
             aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
        return RigidTransform(self.source_frame, other.target_frame, other.point(self.translation), q)


def transform_from_dict(value):
    require(isinstance(value, dict), "transform required")
    return RigidTransform(value.get("source_frame"), value.get("target_frame"),
                          value.get("translation_m"), value.get("quaternion_xyzw"))


def rotate_covariance(value, transform):
    """ROS Imu: first=-1 unavailable; all zeros unknown. Preserve markers exactly."""
    c = vector(value, 9)
    if c[0] == -1 or all(x == 0 for x in c):
        return c
    require(all(c[i*3+i] >= 0 for i in range(3)), "negative variance")
    scale = max(abs(x) for x in c)
    a = tuple(x/scale for x in c)
    require(all(abs(a[i*3+j]-a[j*3+i]) <= 1e-12 for i in range(3) for j in range(3)),
            "covariance must be symmetric")
    # All principal minors characterize positive semidefiniteness for symmetric 3x3.
    require(all(a[i*3+i]*a[j*3+j]-a[i*3+j]**2 >= -1e-12
                for i in range(3) for j in range(i+1, 3)), "covariance not PSD")
    determinant = (a[0]*(a[4]*a[8]-a[5]*a[7]) - a[1]*(a[3]*a[8]-a[5]*a[6])
                   + a[2]*(a[3]*a[7]-a[4]*a[6]))
    require(determinant >= -1e-12, "covariance not PSD")
    r = transform.rotation
    return vector([sum(r[i][k]*c[k*3+l]*r[j][l] for k in range(3) for l in range(3))
                   for i in range(3) for j in range(3)], 9)


def camera_intrinsics(camera):
    require(isinstance(camera, dict), "camera calibration required")
    evidence(camera.get("evidence"))
    width, height = camera.get("width"), camera.get("height")
    require(type(width) is int and width > 0 and type(height) is int and height > 0, "invalid image size")
    require(camera.get("projection") == "rectilinear_pinhole", "unsupported projection")
    require(camera.get("pixel_center") == "integer_centers_half_extent", "unvalidated pixel convention")
    require(camera.get("axes") == "x_right_y_down_z_forward", "optical axes required")
    require(camera.get("pixels_in_target_optical_frame") is True, "cannot rotate image by renaming frame")
    hfov, vfov = number(camera.get("hfov_deg")), number(camera.get("vfov_deg"))
    require(0 < hfov < 180 and 0 < vfov < 180, "FOV must be in (0,180)")
    distortion = vector(camera.get("D"), 5)
    require(all(x == 0 for x in distortion), "only explicitly validated undistorted pinhole supported")
    fx = number(width/(2*math.tan(math.radians(hfov)/2)))
    fy = number(height/(2*math.tan(math.radians(vfov)/2)))
    cx, cy = (width-1)/2, (height-1)/2
    return {"width": width, "height": height, "distortion_model": "plumb_bob", "D": distortion,
            "K": (fx, 0, cx, 0, fy, cy, 0, 0, 1), "R": (1, 0, 0, 0, 1, 0, 0, 0, 1),
            "P": (fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0)}


def validate_calibration(value):
    """Operational validation (including numeric constraints JSON Schema cannot express)."""
    require(isinstance(value, dict) and value.get("version") == 1 and type(value.get("version")) is int,
            "calibration version 1 required")
    c = copy.deepcopy(value)
    evidence(c.get("evidence"))
    kind = c.get("kind")
    require(kind in ("camera", "lidar", "imu"), "unsupported sensor kind")
    for key in ("sensor_id", "input_frame", "output_frame", "time_source"):
        text(c.get(key))
    require(c.get("input_provenance") == "sensor_measurement", "truth/semantic input forbidden")
    require(c.get("time_kind") == "acquisition", "acquisition time required")
    if kind == "lidar":
        require(c.get("coordinates") in ("world", "sensor_local"), "explicit coordinate semantics required")
        require(c.get("transform_policy") == "matched_external", "independently supplied transform required")
    else:
        evidence(c.get("extrinsic_evidence"))
        t = transform_from_dict(c.get("transform"))
        require((t.source_frame, t.target_frame) == (c["input_frame"], c["output_frame"]), "extrinsic frame mismatch")
        if kind == "camera":
            camera_intrinsics(c.get("camera"))
        else:
            require(all(x == 0 for x in t.translation), "IMU origins must be colocated; lever arm unsupported")
            require(c.get("colocated_origin_validated") is True, "IMU origin validation required")
            require(c.get("angular_unit") == "rad/s" and c.get("acceleration_unit") == "m/s^2", "IMU unit mismatch")
            require(c.get("gravity") in ("included", "excluded"), "explicit gravity semantics required")
            text(c.get("smoothing"))
    return c


def load_calibration(path):
    with open(path, encoding="utf-8") as stream:
        return validate_calibration(json.load(stream))


class SensorAdapter:
    """One calibrated sensor, one epoch-ordered acquisition stream. No GT reader.

    Call begin_epoch on ALL adapters and the clock under an external epoch owner.
    Validation is transactional: invalid payloads never consume an acquisition stamp.
    """

    def __init__(self, calibration, epoch, *, purpose="navigation"):
        self._calibration = validate_calibration(calibration)
        require(purpose in ("navigation", "debug"), "purpose must be navigation or debug")
        self.purpose = purpose
        self.clock = AcquisitionClock(self._calibration["time_source"], epoch)

    def begin_epoch(self, epoch):
        self.clock.begin_epoch(epoch)

    def adapt(self, sample, *, matched_transform=None):
        require(isinstance(sample, dict), "sample object required")
        c = self._calibration
        require(sample.get("sensor_id") == c["sensor_id"], "sensor identity mismatch")
        require(sample.get("frame") == c["input_frame"], "input frame mismatch; no aliases")
        require(sample.get("provenance") == "sensor_measurement", "truth/semantic input forbidden")
        timing = sample.get("timing")
        require(isinstance(timing, dict), "acquisition timing required")
        seconds = timing.get("seconds")
        ns = seconds_to_ns(seconds)
        require(timing.get("source") == c["time_source"] and timing.get("epoch") == self.clock.epoch
                and timing.get("time_kind") == "acquisition", "invalid acquisition source/epoch/kind")
        output = {"frame": c["output_frame"], "sensor_id": c["sensor_id"], "kind": c["kind"],
                  "calibration_evidence": copy.deepcopy(c["evidence"]),
                  "input_provenance": "sensor_measurement", "lio_validated": False}
        if c["kind"] == "lidar":
            m = matched_transform
            require(isinstance(m, dict), "matched external transform required; no frame-only rename")
            evidence(m.get("evidence"))
            require(m.get("sensor_id") == c["sensor_id"] and m.get("epoch") == self.clock.epoch
                    and m.get("time_source") == c["time_source"]
                    and type(m.get("stamp_ns")) is int and m["stamp_ns"] == ns, "transform acquisition mismatch")
            provenance = m.get("provenance")
            require(provenance in ("static_calibration", "independent_estimate", "ground_truth"), "transform provenance required")
            require(self.purpose == "debug" or provenance != "ground_truth", "GT transform forbidden in navigation")
            require(c["coordinates"] != "world" or provenance != "static_calibration", "world-to-moving-lidar needs acquired pose, not static extrinsic")
            t = transform_from_dict(m.get("transform"))
            require((t.source_frame, t.target_frame) == (c["input_frame"], c["output_frame"]), "transform frame mismatch")
            points = sample.get("xyz")
            require(isinstance(points, (list, tuple)), "xyz array required")
            output["xyz"] = [t.point(p) for p in points]
            if "normals" in sample:
                normals = sample["normals"]
                require(isinstance(normals, (list, tuple)) and len(normals) == len(points), "normals count mismatch")
                require(all(abs(math.hypot(*vector(n, 3))-1) <= 1e-9 for n in normals), "unit normals required")
                output["normals"] = [t.rotate(n) for n in normals]
            if "rgba" in sample:
                rgba = sample["rgba"]
                require(isinstance(rgba, (list, tuple)) and len(rgba) == len(points)
                        and all(type(x) is int and 0 <= x <= 0xffffffff for x in rgba), "invalid rgba")
                output["rgba"] = list(rgba)
            output.update(transform_provenance=provenance, transform_evidence=copy.deepcopy(m["evidence"]),
                          navigation_allowed=provenance != "ground_truth", deskew_supported=False)
        elif c["kind"] == "imu":
            t = transform_from_dict(c["transform"])
            require(sample.get("gravity") == c["gravity"] and sample.get("smoothing") == c["smoothing"]
                    and sample.get("angular_unit") == c["angular_unit"]
                    and sample.get("acceleration_unit") == c["acceleration_unit"], "IMU semantics mismatch")
            for field in ("angular_velocity", "linear_acceleration"):
                output[field] = t.rotate(sample.get(field))
                output[field+"_covariance"] = rotate_covariance(sample.get(field+"_covariance"), t)
            orientation_covariance = vector(sample.get("orientation_covariance"), 9)
            require(orientation_covariance[0] == -1, "measured attitude not supported by this adapter")
            output.update(orientation=None, orientation_covariance=orientation_covariance,
                          gravity=c["gravity"], smoothing=c["smoothing"],
                          angular_unit=c["angular_unit"], acceleration_unit=c["acceleration_unit"],
                          navigation_allowed=True, transform_provenance="static_calibration")
        else:
            info = camera_intrinsics(c["camera"])
            require(sample.get("width") == info["width"] and sample.get("height") == info["height"], "image size mismatch")
            require(sample.get("encoding") == "rgb8", "only colour rgb8 supported, not depth/labels")
            data = sample.get("data")
            require(isinstance(data, (bytes, list, tuple)) and len(data) == info["width"]*info["height"]*3,
                    "packed RGB data required (no row padding)")
            require(all(type(x) is int and 0 <= x <= 255 for x in data), "invalid RGB byte")
            output.update(data=list(data), encoding="rgb8", camera_info=info, navigation_allowed=True,
                          transform_provenance="static_calibration", optical_extrinsic=copy.deepcopy(c["transform"]))
        stamp = self.clock.accept(seconds, source=timing["source"], epoch=timing["epoch"], time_kind=timing["time_kind"])
        if stamp is None:
            return None
        output["stamp"] = stamp
        if c["kind"] == "camera":
            output["camera_info"].update(stamp=copy.deepcopy(stamp), frame=c["output_frame"])
        return output
