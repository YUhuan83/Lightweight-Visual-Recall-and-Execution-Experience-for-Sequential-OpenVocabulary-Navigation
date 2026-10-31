"""Goal-node selection and A* path planning on a local 2D occupancy grid."""
import math

import numpy as np


def select_goal(nodes):
    """Pick the reachable node with the highest score; tie-break to the nearest."""
    reachable = [n for n in nodes if n.get("rconf", 0.0) > 0.0]
    pool = reachable if reachable else nodes
    return max(pool, key=lambda n: (n.get("score", 0.0), -n.get("radius_m", 0.0)))


def straight_segment(start_xy, goal_xy):
    """Trivial planner: the demo drives a straight waypoint path (occupancy A* is next)."""
    return [np.asarray(goal_xy, float)]


class Grid:
    """Axis-aligned 2D occupancy grid in world coordinates."""

    def __init__(self, bounds, resolution):
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.res = float(resolution)
        self.nx = int(np.ceil((self.x1 - self.x0) / self.res)) + 1
        self.ny = int(np.ceil((self.y1 - self.y0) / self.res)) + 1
        self.occ = np.zeros((self.nx, self.ny), dtype=bool)

    def to_index(self, xy):
        ix = int(round((float(xy[0]) - self.x0) / self.res))
        iy = int(round((float(xy[1]) - self.y0) / self.res))
        return (ix, iy)

    def to_world(self, ij):
        return (self.x0 + ij[0] * self.res, self.y0 + ij[1] * self.res)

    def in_bounds(self, ij):
        return 0 <= ij[0] < self.nx and 0 <= ij[1] < self.ny

    def free(self, ij):
        return self.in_bounds(ij) and not self.occ[ij]


def build_grid(obstacle_xy, start_xy, goal_xy, half_extent=24.0, resolution=0.5, inflation=1.4):
    """Grid covering start..goal with margin; mark cells within inflation of obstacles."""
    pts = np.asarray(obstacle_xy, float) if obstacle_xy is not None and len(obstacle_xy) else np.zeros((0, 2))
    lo = np.minimum(np.asarray(start_xy, float), np.asarray(goal_xy, float)) - half_extent
    hi = np.maximum(np.asarray(start_xy, float), np.asarray(goal_xy, float)) + half_extent
    grid = Grid((lo[0], lo[1], hi[0], hi[1]), resolution)
    if len(pts):
        for cx, cy in pts:
            ij = grid.to_index((cx, cy))
            r = int(np.ceil(inflation / grid.res))
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r and grid.in_bounds((ij[0] + dx, ij[1] + dy)):
                        grid.occ[ij[0] + dx, ij[1] + dy] = True
    return grid


def nearest_free(grid, ij, max_radius=25):
    """Ring search for the closest free cell to ij (used when the goal is blocked)."""
    if grid.free(ij):
        return ij
    for r in range(1, max_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                candidate = (ij[0] + dx, ij[1] + dy)
                if grid.free(candidate):
                    return candidate
    return None


def astar(grid, start_xy, goal_xy):
    """8-connected A*; returns a list of world XY waypoints (start..goal) or []."""
    import heapq
    start, goal = grid.to_index(start_xy), grid.to_index(goal_xy)
    if not grid.in_bounds(start):
        return []
    goal = nearest_free(grid, goal)
    if goal is None:
        return []
    if goal == start:
        return [grid.to_world(goal)]
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def heuristic(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1]) * grid.res

    open_heap = [(heuristic(start), 0.0, start)]
    came, cost = {}, {start: 0.0}
    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while path[-1] in came:
                path.append(came[path[-1]])
            path.reverse()
            return [grid.to_world(ij) for ij in path]
        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.free(nxt):
                continue
            step = math.hypot(dx, dy)
            ng = g + step
            if ng < cost.get(nxt, float("inf")):
                cost[nxt] = ng
                came[nxt] = current
                heapq.heappush(open_heap, (ng + heuristic(nxt), ng, nxt))
    return []


def simplify(path, obstacle_xy, obstacle_radius=1.4):
    """Greedy line-of-sight smoothing of a waypoint list."""
    from . import graph
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not graph.segment_clear(path[i], path[j], obstacle_xy, obstacle_radius):
            j -= 1
        out.append(path[j])
        i = j
    return out
