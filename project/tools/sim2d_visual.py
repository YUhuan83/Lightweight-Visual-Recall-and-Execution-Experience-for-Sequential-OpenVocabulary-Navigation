"""Animated visualisation of the local WildOS-lite 2D closed loop.

Top-down animation: obstacles, LiDAR returns, candidate nodes coloured by score,
selected goal, FOV wedge, trajectory coloured by phase, target + success circle,
plus a HUD. Saves a GIF (Pillow, no ffmpeg) and a storyboard PNG; --show opens an
interactive window.

    python tools/sim2d_visual.py --steps 1500
    python tools/sim2d_visual.py --show
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(TOOLS))

import sim2d_demo as sim  # noqa: E402
from wildos_lite import Config, Observation, Policy  # noqa: E402

PHASE_COLORS = {"SEARCHING": "#1f77b4", "APPROACHING": "#ff7f0e", "FOUND": "#2ca02c"}
GIF = ROOT / "wildos_lite" / "sim2d_demo.gif"
STORY = ROOT / "wildos_lite" / "sim2d_storyboard.png"


def run_episode(steps, dt, seed, success_radius):
    segments = sim.all_segments()
    rng = np.random.default_rng(seed)
    policy = Policy(Config(success_radius_m=success_radius))
    x, y, yaw = sim.START
    speed = 0.0
    frames = []
    for _ in range(steps):
        forward = (math.cos(yaw), math.sin(yaw))
        cloud = sim.cast_lidar((x, y), segments)
        target_xy = sim.detect_target((x, y), forward, sim.TARGET, segments, rng=rng)
        decision = policy.decide(Observation(position_xy=(x, y), forward_xy=forward,
                                             lidar_xyz=cloud, target_xy=target_xy,
                                             target_seen=target_xy is not None))
        frames.append({"x": x, "y": y, "yaw": yaw, "phase": decision.phase,
                       "goal_xy": [float(v) for v in decision.goal_xy],
                       "nodes": [{"xy": list(n["xy"]), "score": float(n["score"]),
                                  "rconf": float(n["rconf"])} for n in decision.nodes],
                       "cloud": cloud, "target_seen": target_xy is not None,
                       "distance_m": math.hypot(sim.TARGET[0] - x, sim.TARGET[1] - y)})
        if decision.phase == "FOUND":
            break
        x, y, yaw, speed = sim.step_physics(x, y, yaw, speed, decision.command, dt)
    return frames


def draw_world(ax, frame, subtrace, all_scores, cmap, success_radius, show_nodes=True):
    from matplotlib.patches import Rectangle, Wedge, Circle
    for cx, cy, w, h in sim.OBSTACLES:
        ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, color="0.55", zorder=1))
    cloud = frame["cloud"]
    if len(cloud):
        ground = cloud[np.abs(cloud[:, 2]) < 0.5]
        raised = cloud[np.abs(cloud[:, 2]) >= 0.5]
        if len(ground):
            ax.scatter(ground[:, 0], ground[:, 1], s=2, c="0.82", zorder=2)
        if len(raised):
            ax.scatter(raised[:, 0], raised[:, 1], s=7, c="#8b0000", zorder=2)

    for i in range(1, len(subtrace)):
        ax.plot(subtrace[i - 1:i + 1][0]["x":i + 1] if False else [subtrace[i - 1]["x"], subtrace[i]["x"]],
                [subtrace[i - 1]["y"], subtrace[i]["y"]],
                color=PHASE_COLORS[subtrace[i]["phase"]], lw=2.5, zorder=4)

    if show_nodes and frame["nodes"]:
        lo, hi = all_scores
        for n in frame["nodes"]:
            norm = (n["score"] - lo) / max(hi - lo, 1e-6)
            ax.scatter(n["xy"][0], n["xy"][1], s=26, c=[cmap(norm)],
                       edgecolors="k" if n["rconf"] > 0 else "none", linewidths=0.4,
                       zorder=3, alpha=0.9)
    ax.scatter(frame["goal_xy"][0], frame["goal_xy"][1], marker="*", s=260, c="#ffd400",
               edgecolors="k", zorder=6)

    ax.add_patch(Wedge((frame["x"], frame["y"]), 40.0,
                       math.degrees(frame["yaw"]) - 30, math.degrees(frame["yaw"]) + 30,
                       color="#ffee88", alpha=0.18, zorder=0))
    ax.add_patch(Circle((sim.TARGET[0], sim.TARGET[1]), success_radius,
                        color="r", fill=False, ls="--", alpha=0.6, zorder=2))
    ax.scatter(sim.TARGET[0], sim.TARGET[1], marker="*", s=420, c="red", zorder=7)
    ax.arrow(frame["x"], frame["y"], 1.6 * math.cos(frame["yaw"]), 1.6 * math.sin(frame["yaw"]),
             head_width=1.0, head_length=1.2, fc=PHASE_COLORS[frame["phase"]], ec="k", zorder=8)
    ax.set_aspect("equal")
    ax.set_xlim(-8, 56)
    ax.set_ylim(-16, 16)
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--steps", type=int, default=1500)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--success-radius", type=float, default=6.0)
    p.add_argument("--interval", type=int, default=70)
    p.add_argument("--no-nodes", action="store_true")
    p.add_argument("--show", action="store_true")
    a = p.parse_args(argv)

    import matplotlib
    if not a.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    frames = run_episode(a.steps, a.dt, a.seed, a.success_radius)
    all_scores = (min(n["score"] for f in frames for n in f["nodes"]),
                  max(n["score"] for f in frames for n in f["nodes"]))
    cmap = plt.get_cmap("viridis")
    show_nodes = not a.no_nodes

    # storyboard: pick key indices (start, detection, mid-approach, found, plus spread)
    phases = [f["phase"] for f in frames]
    first_seen = next((i for i, f in enumerate(frames) if f["target_seen"]), len(frames) - 1)
    found = len(frames) - 1
    key = sorted({0, max(0, first_seen - 1), first_seen, (first_seen + found) // 2,
                  max(0, found - 1), found})
    key = [k for k in key if 0 <= k < len(frames)][:6]
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    for ax_, idx in zip(axes.ravel(), key):
        draw_world(ax_, frames[idx], frames[:idx + 1], all_scores, cmap, a.success_radius, show_nodes)
        ax_.set_title("step %d  %s  dist=%.1fm" % (idx, frames[idx]["phase"], frames[idx]["distance_m"]))
    for ax_ in axes.ravel()[len(key):]:
        ax_.axis("off")
    fig.suptitle("WildOS-lite 2D demo storyboard: " + " -> ".join(
        [phases[i] for i in range(len(phases)) if i == 0 or phases[i - 1] != phases[i]]))
    fig.tight_layout()
    fig.savefig(STORY, dpi=110)
    print("storyboard:", STORY)
    print("phase sequence:", [phases[i] for i in range(len(phases)) if i == 0 or phases[i - 1] != phases[i]])

    if a.show:
        fig2, ax2 = plt.subplots(figsize=(11, 7))

        def update(i):
            ax2.clear()
            draw_world(ax2, frames[i], frames[:i + 1], all_scores, cmap, a.success_radius, show_nodes)
            ax2.set_title("step %d  %s  dist=%.1fm  seen=%s" % (
                i, frames[i]["phase"], frames[i]["distance_m"], frames[i]["target_seen"]))
            return []

        _ = FuncAnimation(fig2, update, frames=len(frames), interval=a.interval, blit=False)
        plt.show()
    else:
        fig3, ax3 = plt.subplots(figsize=(11, 7))

        def update(i):
            ax3.clear()
            draw_world(ax3, frames[i], frames[:i + 1], all_scores, cmap, a.success_radius, show_nodes)
            ax3.set_title("step %d  %s  dist=%.1fm  seen=%s" % (
                i, frames[i]["phase"], frames[i]["distance_m"], frames[i]["target_seen"]))
            return []

        anim = FuncAnimation(fig3, update, frames=len(frames), interval=a.interval, blit=False)
        anim.save(GIF, writer=PillowWriter(fps=max(1, 1000 // a.interval)))
        print("gif:", GIF)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
