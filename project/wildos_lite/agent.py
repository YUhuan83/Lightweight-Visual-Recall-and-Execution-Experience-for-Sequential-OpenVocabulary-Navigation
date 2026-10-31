"""WildOS-lite closed-loop policy: text goal -> node graph -> plan -> motion command."""
from dataclasses import dataclass, field

import numpy as np

from . import controller, graph, planner, scoring


@dataclass
class Config:
    success_radius_m: float = 5.0
    obstacle_radius: float = 1.2
    obstacle_min_height_m: float = 0.4
    obstacle_max_height_m: float = 2.5
    w_goal: float = 1.0
    w_frontier: float = 0.3
    steer_sign: float = 1.0
    radii: tuple = graph.DEFAULT_RADII
    headings_deg: tuple = graph.DEFAULT_HEADINGS_DEG


@dataclass
class Observation:
    position_xy: tuple
    forward_xy: tuple
    lidar_xyz: object = None
    target_xy: object = None
    target_seen: bool = False


@dataclass
class Decision:
    phase: str
    goal_xy: object
    command: dict
    nodes: list
    target_distance_m: object = None


def split_cloud(lidar_xyz, min_height, max_height):
    """Return (all_xy, obstacle_xy) separating raised obstacles from the ground."""
    if lidar_xyz is None or len(lidar_xyz) == 0:
        return np.zeros((0, 2)), np.zeros((0, 2))
    pts = np.asarray(lidar_xyz, float)
    all_xy = pts[:, :2]
    ground = float(np.percentile(pts[:, 2], 5))
    height = pts[:, 2] - ground
    mask = (height > min_height) & (height < max_height)
    return all_xy, pts[mask][:, :2]


class Policy:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.phase = "SEARCHING"

    def decide(self, obs):
        cfg = self.cfg
        all_xy, obstacle_xy = split_cloud(obs.lidar_xyz, cfg.obstacle_min_height_m,
                                          cfg.obstacle_max_height_m)
        nodes = graph.build_candidates(obs.position_xy, obs.forward_xy, cfg.radii, cfg.headings_deg)
        graph.reachability(obs.position_xy, nodes, obstacle_xy, cfg.obstacle_radius)
        target_xy = obs.target_xy if obs.target_seen else None
        scoring.score_nodes(nodes, obs.position_xy, obs.forward_xy, target_xy, all_xy,
                            cfg.w_goal, cfg.w_frontier)
        goal = planner.select_goal(nodes)

        target_distance = None
        if obs.target_seen and obs.target_xy is not None:
            target_distance = float(np.linalg.norm(np.asarray(obs.target_xy, float)
                                                   - np.asarray(obs.position_xy, float)))
            self.phase = "FOUND" if target_distance <= cfg.success_radius_m else "APPROACHING"
        else:
            self.phase = "SEARCHING"

        if self.phase == "FOUND":
            command = {"steering": 0.0, "throttle": 0.0, "brake": 1.0,
                       "distance_m": target_distance, "heading_error_rad": 0.0}
        else:
            command = controller.pure_pursuit(obs.position_xy, obs.forward_xy, goal["xy"],
                                              steer_sign=cfg.steer_sign)
        return Decision(phase=self.phase, goal_xy=goal["xy"], command=command,
                        nodes=nodes, target_distance_m=target_distance)
