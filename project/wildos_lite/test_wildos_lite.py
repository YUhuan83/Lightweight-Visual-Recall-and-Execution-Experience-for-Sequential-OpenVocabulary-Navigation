"""Unit tests for the simulator-agnostic WildOS-lite core."""
import math
import unittest

import numpy as np

from wildos_lite import Config, Observation, Policy
from wildos_lite import controller, graph, planner, scoring

FORWARD = (0.0, -1.0)   # -Y forward (fixture/American convention used here)
POS = (0.0, 0.0)


class GraphTests(unittest.TestCase):
    def test_candidate_count_and_bearing(self):
        nodes = graph.build_candidates(POS, FORWARD, radii=(3.0, 6.0), headings_deg=(-90, 0, 90))
        self.assertEqual(len(nodes), 6)
        ahead = next(n for n in nodes if n["radius_m"] == 6.0 and n["heading_deg"] == 0.0)
        self.assertAlmostEqual(graph.bearing_deg(ahead["xy"], POS, FORWARD), 0.0, places=6)
        left = next(n for n in nodes if n["radius_m"] == 6.0 and n["heading_deg"] == 90.0)
        self.assertAlmostEqual(left["xy"][0], 6.0, places=6)
        self.assertAlmostEqual(graph.bearing_deg(left["xy"], POS, FORWARD), 90.0, places=6)

    def test_segment_clear(self):
        obstacle = [[5.0, 0.0]]
        self.assertFalse(graph.segment_clear((0, 0), (10, 0), obstacle, obstacle_radius=1.2))
        self.assertTrue(graph.segment_clear((0, 0), (3, 0), obstacle, obstacle_radius=1.2))
        self.assertTrue(graph.segment_clear((0, 0), (10, 0), np.zeros((0, 2))))

    def test_reachability_flags_blocked_nodes(self):
        nodes = graph.build_candidates(POS, FORWARD, radii=(3.0, 9.0), headings_deg=(90,))
        graph.reachability(POS, nodes, [[5.0, 0.0]], obstacle_radius=1.2)
        near = next(n for n in nodes if n["radius_m"] == 3.0)
        far = next(n for n in nodes if n["radius_m"] == 9.0)
        self.assertEqual(near["rconf"], 1.0)
        self.assertEqual(far["rconf"], 0.0)


class ScoringTests(unittest.TestCase):
    def test_goal_alignment_prefers_target_direction(self):
        nodes = graph.build_candidates(POS, FORWARD, radii=(3.0, 6.0, 9.0), headings_deg=(-90, 0, 90))
        graph.reachability(POS, nodes, None)
        scoring.score_nodes(nodes, POS, FORWARD, target_xy=(10.0, 0.0), all_xy=np.zeros((0, 2)),
                            w_goal=1.0, w_frontier=0.0)
        goal = planner.select_goal(nodes)
        self.assertEqual(goal["heading_deg"], 90.0)
        self.assertEqual(goal["radius_m"], 9.0)

    def test_frontier_decays_with_hits(self):
        dense = np.zeros((40, 2))
        self.assertLess(graph.frontier_conf((0, 0), dense), graph.frontier_conf((0, 0), np.zeros((0, 2))))


class ControllerTests(unittest.TestCase):
    def test_signed_steering(self):
        left = controller.pure_pursuit(POS, FORWARD, (10.0, 0.0), steer_sign=1.0)
        self.assertGreater(left["steering"], 0.0)
        self.assertLess(abs(left["heading_error_rad"]), math.pi)
        ahead = controller.pure_pursuit(POS, FORWARD, (0.0, -10.0), steer_sign=1.0)
        self.assertAlmostEqual(ahead["steering"], 0.0, places=6)
        self.assertGreater(ahead["throttle"], 0.0)

    def test_brakes_when_close(self):
        close = controller.pure_pursuit(POS, FORWARD, (0.0, -0.5), steer_sign=1.0)
        self.assertGreater(close["brake"], 0.0)
        self.assertEqual(close["throttle"], 0.0)


class PolicyTests(unittest.TestCase):
    def test_phase_transitions(self):
        policy = Policy(Config(success_radius_m=5.0))
        searching = policy.decide(Observation(position_xy=POS, forward_xy=FORWARD, lidar_xyz=None))
        self.assertEqual(searching.phase, "SEARCHING")
        self.assertIsNotNone(searching.goal_xy)

        approaching = policy.decide(Observation(position_xy=POS, forward_xy=FORWARD, lidar_xyz=None,
                                                target_xy=(10.0, 0.0), target_seen=True))
        self.assertEqual(approaching.phase, "APPROACHING")
        self.assertAlmostEqual(approaching.target_distance_m, 10.0, places=6)
        goal_node = max(approaching.nodes, key=lambda n: n["score"])
        self.assertEqual(goal_node["heading_deg"], 90.0)

        found = policy.decide(Observation(position_xy=POS, forward_xy=FORWARD, lidar_xyz=None,
                                          target_xy=(3.0, 0.0), target_seen=True))
        self.assertEqual(found.phase, "FOUND")
        self.assertEqual(found.command["throttle"], 0.0)
        self.assertGreater(found.command["brake"], 0.0)

    def test_obstacle_blocks_straight_path(self):
        ground = np.zeros((60, 3))
        ground[:, 0] = np.linspace(-5.0, 20.0, 60)
        ground[:, 1] = np.linspace(-8.0, 8.0, 60)
        ground[:, 2] = 0.0
        wall = np.zeros((30, 3))
        wall[:, 0] = np.linspace(4.0, 6.0, 30)
        wall[:, 1] = 0.0
        wall[:, 2] = 1.5  # raised wall in front-left
        cloud = np.vstack([ground, wall])
        policy = Policy(Config(obstacle_radius=1.0))
        decision = policy.decide(Observation(position_xy=POS, forward_xy=FORWARD, lidar_xyz=cloud))
        blocked_left = [n for n in decision.nodes if n["heading_deg"] == 90.0 and n["radius_m"] >= 6.0]
        self.assertTrue(blocked_left)
        self.assertTrue(all(n["rconf"] == 0.0 for n in blocked_left))


if __name__ == "__main__":
    unittest.main()
