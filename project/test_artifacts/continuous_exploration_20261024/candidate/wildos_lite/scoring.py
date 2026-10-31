"""Node scoring: Gconf (goal alignment) x Rconf (reachability) x Fconf (frontier)."""
import math

import numpy as np

from . import graph


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def goal_conf(node, position_xy, forward_xy, target_xy):
    """Alignment x proximity of the node to the target, in [0,1]."""
    if target_xy is None:
        return 0.0
    pos = np.asarray(position_xy, float)
    target = np.asarray(target_xy, float)
    node_bearing = graph.bearing_deg(node["xy"], position_xy, forward_xy)
    target_bearing = graph.bearing_deg(target_xy, position_xy, forward_xy)
    diff = abs(wrap180(node_bearing - target_bearing))
    align = max(0.0, math.cos(math.radians(min(diff, 90.0))))
    dist_target = float(np.linalg.norm(target - pos))
    dist_node = float(np.linalg.norm(np.asarray(node["xy"], float) - pos))
    proximity = float(np.clip(1.0 - abs(dist_node - dist_target) / max(dist_target, 1e-6), 0.0, 1.0))
    return float(align * proximity)


def score_nodes(nodes, position_xy, forward_xy, target_xy=None, all_xy=None,
                w_goal=1.0, w_frontier=0.3, w_progress=0.5, w_straight=0.5):
    """Set 'gconf', 'fconf' and 'score' on each node and return the list.

    With a target, score = rconf x (goal x frontier) (exploit).
    Without a target, score = rconf x (frontier + progress + straight) so the
    greedy search makes forward progress instead of stalling on a near tie.
    """
    max_radius = max((n.get("radius_m", 0.0) for n in nodes), default=1.0) or 1.0
    for node in nodes:
        node["fconf"] = graph.frontier_conf(node["xy"], all_xy)
        if target_xy is not None:
            node["gconf"] = goal_conf(node, position_xy, forward_xy, target_xy)
            exploit = w_goal * node["gconf"] + w_frontier * node["fconf"]
        else:
            node["gconf"] = 0.0
            progress = node.get("radius_m", 0.0) / max_radius
            straight = max(0.0, math.cos(math.radians(node.get("heading_deg", 0.0))))
            exploit = w_frontier * node["fconf"] + w_progress * progress + w_straight * straight
        node["score"] = float(node.get("rconf", 0.0)) * exploit
    return nodes
