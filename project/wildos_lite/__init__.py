"""WildOS-lite: text goal -> candidate-node graph -> scoring -> plan -> motion.

Pure-logic, simulator-agnostic core. The BeamNG runner (run_demo.py) feeds it
observations and applies the emitted control commands.
"""
from .agent import Config, Decision, Observation, Policy, split_cloud

__all__ = ["Config", "Decision", "Observation", "Policy", "split_cloud"]
