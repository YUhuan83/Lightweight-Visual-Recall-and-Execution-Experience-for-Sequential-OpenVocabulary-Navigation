"""Local 2D closed-loop demo of the WildOS-lite policy (no server, no weights).

A kinematic bicycle ego, rectangular obstacles, a ray-cast 360 LiDAR (raised hits +
sparse ground), and a field-of-view/line-of-sight target detector feed the real
wildos_lite policy each step. The run is plotted (trajectory coloured by phase)
and summarised as JSON.

    python tools/sim2d_demo.py --steps 800
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from wildos_lite import Config, Observation, Policy  # noqa: E402

OUT = ROOT / "wildos_lite" / "sim2d_demo.png"
OUT_JSON = ROOT / "wildos_lite" / "sim2d_demo.json"

OBSTACLES = [  # centre_x, centre_y, width, height (flanking the straight lane)
    (24.0, 6.5, 7.0, 7.0),
    (34.0, -7.5, 7.0, 7.0),
    (13.0, -11.0, 5.0, 5.0),
]
TARGET = (48.0, 0.0)
START = (0.0, 0.0, 0.0)  # x, y, yaw(rad) forward=(cos,sin)
DETECT_RANGE = 40.0
WHEELBASE = 3.0
MAX_STEER = math.radians(30.0)
MAX_SPEED = 8.0
ACCEL, DECEL = 3.0, 7.0
MAX_THROTTLE = 0.45


def rect_segments(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def all_segments():
    segments = []
    for cx, cy, w, h in OBSTACLES:
        segments.extend(rect_segments(cx, cy, w, h))
    return segments


def ray_segment(origin, direction, seg):
    (x1, y1), (x2, y2) = seg
    dx, dy = direction
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None
    ox, oy = origin
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def cast_lidar(position_xy, segments, n_rays=180, max_range=60.0):
    points = []
    for i in range(n_rays):
        angle = 2.0 * math.pi * i / n_rays
        direction = (math.cos(angle), math.sin(angle))
        hits = [ray_segment(position_xy, direction, seg) for seg in segments]
        hits = [h for h in hits if h is not None]
        hit = min(hits) if hits else None
        if hit is not None and hit <= max_range:
            points.append([position_xy[0] + hit * direction[0],
                           position_xy[1] + hit * direction[1], 1.5])
        if i % 12 == 0:  # sparse ground ring keeps the frontier proxy about obstacles
            ground = min(hit, 30.0) if hit is not None else 30.0
            points.append([position_xy[0] + ground * direction[0],
                           position_xy[1] + ground * direction[1], 0.0])
    return np.asarray(points, float)


def line_of_sight(a, b, segments):
    for seg in segments:
        t = ray_segment(a, (b[0] - a[0], b[1] - a[1]), seg)
        if t is not None and 0.02 < t < 0.98:
            return False
    return True


def detect_target(position_xy, forward, target, segments, fov_deg=60.0, max_range=DETECT_RANGE, rng=None):
    d = np.asarray(target, float) - np.asarray(position_xy, float)
    dist = float(np.linalg.norm(d))
    if dist > max_range:
        return None
    cosang = float(d @ np.asarray(forward, float)) / max(dist, 1e-9)
    if cosang < math.cos(math.radians(fov_deg / 2.0)):
        return None
    if not line_of_sight(position_xy, target, segments):
        return None
    noise = rng.normal(0.0, 0.15, size=2) if rng is not None else np.zeros(2)
    return (float(target[0] + noise[0]), float(target[1] + noise[1]))


def step_physics(x, y, yaw, speed, command, dt):
    steer = float(np.clip(command["steering"], -1.0, 1.0)) * MAX_STEER
    throttle = float(np.clip(command["throttle"], 0.0, 1.0))
    brake = float(np.clip(command["brake"], 0.0, 1.0))
    target_speed = (throttle / MAX_THROTTLE) * MAX_SPEED
    if brake > 0.0:
        speed = max(0.0, speed - DECEL * brake * dt)
    elif speed < target_speed:
        speed = min(target_speed, speed + ACCEL * dt)
    else:
        speed = max(target_speed, speed - ACCEL * dt)
    yaw = yaw + (speed / WHEELBASE) * math.tan(steer) * dt
    x = x + speed * math.cos(yaw) * dt
    y = y + speed * math.sin(yaw) * dt
    return x, y, yaw, speed


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--steps", type=int, default=800)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--success-radius", type=float, default=6.0)
    a = p.parse_args(argv)

    segments = all_segments()
    rng = np.random.default_rng(a.seed)
    policy = Policy(Config(success_radius_m=a.success_radius))
    x, y, yaw = START
    speed = 0.0
    trace = []
    for step in range(a.steps):
        forward = (math.cos(yaw), math.sin(yaw))
        cloud = cast_lidar((x, y), segments)
        target_xy = detect_target((x, y), forward, TARGET, segments, rng=rng)
        decision = policy.decide(Observation(position_xy=(x, y), forward_xy=forward,
                                             lidar_xyz=cloud, target_xy=target_xy,
                                             target_seen=target_xy is not None))
        distance = math.hypot(TARGET[0] - x, TARGET[1] - y)
        trace.append({"step": step, "x": x, "y": y, "yaw": yaw, "phase": decision.phase,
                      "goal_xy": list(map(float, decision.goal_xy)),
                      "target_seen": target_xy is not None, "distance_m": distance})
        if decision.phase == "FOUND":
            break
        x, y, yaw, speed = step_physics(x, y, yaw, speed, decision.command, a.dt)

    phases = [t["phase"] for t in trace]
    summary = {"schema": "wildos-lite-sim2d/1", "steps": len(trace), "found": phases[-1] == "FOUND",
               "final_distance_m": trace[-1]["distance_m"],
               "phase_sequence": [ph for i, ph in enumerate(phases) if i == 0 or phases[i - 1] != ph],
               "target": list(TARGET), "start": list(START[:2]), "obstacles": OBSTACLES,
               "trace": trace}
    OUT_JSON.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("steps", "found", "final_distance_m", "phase_sequence")}, indent=2))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, ax = plt.subplots(figsize=(9, 7))
        for cx, cy, w, h in OBSTACLES:
            ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, color="0.6"))
        colors = {"SEARCHING": "#1f77b4", "APPROACHING": "#ff7f0e", "FOUND": "#2ca02c"}
        xs = [t["x"] for t in trace]
        ys = [t["y"] for t in trace]
        for i in range(1, len(trace)):
            ax.plot(xs[i - 1:i + 1], ys[i - 1:i + 1], color=colors[trace[i]["phase"]], lw=2)
        ax.plot(xs[0], ys[0], "ko", ms=8, label="start")
        ax.plot(TARGET[0], TARGET[1], "r*", ms=16, label="target")
        ax.plot(xs[-1], ys[-1], "g^", ms=10, label="end")
        ax.add_patch(plt.Circle(TARGET, a.success_radius, color="r", fill=False, ls="--", alpha=0.6))
        handles = [plt.Line2D([], [], color=c, lw=3, label=p) for p, c in colors.items()]
        ax.legend(handles=handles + [plt.Line2D([], [], color="k", marker="o", ls="", label="start"),
                                     plt.Line2D([], [], color="r", marker="*", ls="", label="target"),
                                     plt.Line2D([], [], color="g", marker="^", ls="", label="end")], loc="upper left")
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("WildOS-lite 2D demo: %s" % " -> ".join(summary["phase_sequence"]))
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(OUT, dpi=120)
        print("figure:", OUT)
    except Exception as exc:  # plotting is optional
        print("plot skipped:", exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
