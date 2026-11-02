"""Resolve simple relational navigation queries against persistent objects.

The resolver deliberately stays independent of the model and ROS. A caller
provides object memories in the world frame, the robot pose, and the current
road tangent. This makes language/relationship behavior unit-testable.
"""

from dataclasses import dataclass
import re
from typing import Iterable, Optional, Sequence

import numpy as np


ORDINALS = {
    "first": 1,
    "1st": 1,
    "one": 1,
    "second": 2,
    "2nd": 2,
    "two": 2,
    "third": 3,
    "3rd": 3,
    "three": 3,
    "fourth": 4,
    "4th": 4,
    "four": 4,
    "第一": 1,
    "第二": 2,
    "第三": 3,
    "第四": 4,
}


@dataclass(frozen=True)
class SemanticObject:
    object_id: str
    label: str
    position: Sequence[float]
    confidence: float = 1.0


def _ordinal(text: str) -> Optional[int]:
    for token, value in ORDINALS.items():
        if token in text.lower():
            return value
    match = re.search(r"(?:第)?(\d+)(?:个|座|th|st|nd|rd)?", text.lower())
    return int(match.group(1)) if match else None


def _label(text: str) -> str:
    lowered = text.lower()
    for value, aliases in {
        "house": ("house", "building", "房子", "房屋", "建筑"),
        "door": ("door", "门"),
        "tree": ("tree", "树"),
    }.items():
        if any(alias in lowered for alias in aliases):
            return value
    return ""


def resolve_relation(
    query: str,
    objects: Iterable[SemanticObject],
    robot_position: Sequence[float],
    road_tangent: Sequence[float],
) -> Optional[SemanticObject]:
    """Resolve a left/right ordinal object relative to the road direction."""
    text = query.lower()
    side = (
        "left"
        if ("left" in text or "左" in text)
        else "right"
        if ("right" in text or "右" in text)
        else None
    )
    number = _ordinal(text)
    label = _label(text)
    if side is None or number is None or not label:
        return None

    origin = np.asarray(robot_position, dtype=float)[:2]
    tangent = np.asarray(road_tangent, dtype=float)[:2]
    norm = np.linalg.norm(tangent)
    if norm <= 1e-9:
        return None
    tangent = tangent / norm
    candidates = []
    for obj in objects:
        if obj.label.lower() != label or obj.confidence <= 0.0:
            continue
        relative = np.asarray(obj.position, dtype=float)[:2] - origin
        longitudinal = float(np.dot(relative, tangent))
        lateral = float(tangent[0] * relative[1] - tangent[1] * relative[0])
        if longitudinal <= 0.0:
            continue
        if side == "left" and lateral <= 0.0:
            continue
        if side == "right" and lateral >= 0.0:
            continue
        candidates.append((longitudinal, -float(obj.confidence), obj))
    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[number - 1][2] if len(candidates) >= number else None
