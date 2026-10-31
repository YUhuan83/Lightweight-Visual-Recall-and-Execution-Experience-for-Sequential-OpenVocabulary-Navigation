"""Synthetic same-process integration, NOT a Nav2/BeamNG acceptance test.

Production memory methods, GoalStandoff, Bridge and action callbacks are used.
Perception features/localizations, graph routes, planner responses and perfect
motion are supplied. No model encoder, C++ GraphNav/Nav2, DDS or physics runs.
"""
from concurrent.futures import Future
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'),
               str(ROOT/'official_adapter'),
               str(ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from std_msgs.msg import String
from test_multi_goal_session_ros import MultiGoalSessionTests
from test_graph_path_bridge_routing import route
from visual_navigation.wildos.semantic_landmark_memory import SemanticLandmarkMemory


class Features:
    def __init__(self, values):
        self.value = np.asarray(values, dtype=np.float32)
        self.shape = self.value.shape
    def detach(self): return self
    def float(self): return self
    def cpu(self): return self
    def numpy(self): return self.value


class Handle:
    accepted = True
    def __init__(self):
        self.result = Future()
        self.cancelled = False
    def get_result_async(self): return self.result
    def cancel_goal_async(self):
        self.cancelled = True
        value = Future(); value.set_result(S())
        return value


class Session(MultiGoalSessionTests):
    def setup_chain(self, profile):
        self.setUp()
        self.profile = profile
        self.wall = 1000.
        self.trace, self.actions, self.plans = [], [], []
        self.heading = 0.
        self.bridge.carrier_model = profile
        self.bridge.minimum_turning_radius = 8. if profile == 'ackermann' else 0.
        self.bridge.planner_id = 'CarLike' if profile == 'ackermann' else 'GridBased'
        self.bridge.status_pub.publish.side_effect = lambda m: self.trace.append(json.loads(m.data))
        self.bridge.plan_client = Mock()
        self.bridge.plan_client.wait_for_server.return_value = True
        self.bridge.plan_client.send_goal_async.side_effect = self.plan_response
        self.bridge.follow_client.send_goal_async.side_effect = self.follow_response
        self.initial_identity = tuple(map(id, (self.vision, self.db, self.vision.object_goal_memory)))
        self.refresh(0.)

    def refresh(self, x, heading=None):
        self.wall += .1
        self.update_inputs(x)
        if heading is not None:
            self.heading = heading
        for node in (self.bridge, self.confirm):
            node.odom.pose.pose.orientation.z = math.sin(self.heading/2)
            node.odom.pose.pose.orientation.w = math.cos(self.heading/2)
        self.vision.latest_odom_msg = self.bridge.odom
        self.bridge.on_native_status(String(data=json.dumps(dict(wall_time=time.time(),
            speed=0., telemetry_ok=True, actuator_ok=True, control_acknowledged=True,
            telemetry_age_sec=0., actual={}, wheel_contact={}, sent={'brake': 1.}))))

    def begin(self, label):
        self.query(label)
        # Let the production query grace expire using the controlled wall clock.
        self.wall += 2.1
        self.refresh(self.bridge.odom.pose.pose.position.x)

    def observation(self, label, object_x, count=5):
        for _ in range(count):
            self.refresh(self.bridge.odom.pose.pose.position.x)
            self.confirm.on_localization(String(data=json.dumps(dict(query=label,
                frame='spot1/odom', observation_stamp_ns=self.now,
                position=[object_x, 0., 2.],
                observer_position=[self.bridge.odom.pose.pose.position.x, 0., 0.],
                observer_orientation=[0., 0., math.sin(self.heading/2), math.cos(self.heading/2)],
                source='lidar', views=0, localization_valid=True,
                quality={'confidence': .9, 'points': 25, 'horizontal_fov': 1.4,
                         'association_validated': True}))))

    def graph_route(self, endpoint):
        self.refresh(self.bridge.odom.pose.pose.position.x)
        path = route((self.bridge.odom.pose.pose.position.x, 0.), endpoint)
        path.header.stamp = self.bridge.get_clock().now().to_msg()
        stamp = self.now
        self.bridge.on_route_status(String(data=json.dumps(dict(stamp_ns=stamp,
            complete=True, goal=list(self.bridge.expected or endpoint), mode='offline_supplied_route'))))
        self.bridge.on_route(path)

    def plan_response(self, goal):
        target = goal.goal.pose.position
        q = goal.goal.pose.orientation
        desired_heading = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        start = self.bridge.odom.pose.pose.position.x
        path = route((start, 0.), (target.x, target.y))
        path.header.stamp = self.bridge.get_clock().now().to_msg()
        # This supplied planner supports only straight physical motion with
        # unchanged body yaw. Unsupported headings fail, rather than inventing
        # a curvature-violating physical route or relaxing production checks.
        possible = (abs(target.y) < 1e-8 and (self.profile == 'ideal_point' or
                    abs(math.remainder(desired_heading-self.heading, math.tau)) < .01))
        for p in path.poses:
            p.pose.orientation.z = math.sin(desired_heading/2)
            p.pose.orientation.w = math.cos(desired_heading/2)
        self.plans.append(dict(start=start, target=[target.x, target.y],
            heading=desired_heading, status=4 if possible else 6, planner_id=goal.planner_id))
        handle = Handle()
        handle.result.set_result(S(status=4 if possible else 6,
                                   result=S(path=path if possible else route())))
        response = Future(); response.set_result(handle)
        return response

    def follow_response(self, goal):
        handle = Handle()
        self.actions.append((deepcopy(goal), handle))
        response = Future(); response.set_result(handle)
        return response

    def complete_actions(self, limit=32):
        consumed = 0
        while True:
            pending = [(g, h) for g, h in self.actions if not h.result.done() and not h.cancelled]
            if not pending:
                return consumed
            if consumed >= limit:
                raise AssertionError('offline action budget exhausted')
            goal, handle = pending[0]
            end = goal.path.poses[-1].pose
            start_x = self.bridge.odom.pose.pose.position.x
            target_heading = math.atan2(2*end.orientation.w*end.orientation.z,
                                        1-2*end.orientation.z**2)
            if self.profile == 'ideal_point':
                # Constructive point motion, not the actual RPP command law.
                travel_heading = 0. if end.position.x >= start_x else math.pi
                turn = math.remainder(travel_heading-self.heading, math.tau)
                for i in range(1, max(1, math.ceil(abs(turn)/.08))+1):
                    self.refresh(start_x, self.heading+turn/max(1, math.ceil(abs(turn)/.08)))
            count = max(1, math.ceil(abs(end.position.x-start_x)/.12))
            for i in range(1, count+1):
                self.refresh(start_x+(end.position.x-start_x)*i/count)
            # Point yaw can change at rest; physical supplied paths preserve it.
            self.refresh(end.position.x, target_heading)
            handle.result.set_result(S(status=4))
            consumed += 1

    def passive_station(self):
        v = self.vision
        v.text_queries = ['green residential house']
        v.search_target_cache = {'green residential house': Features([[1., 0.]]),
                                 'gas station': Features([[0., 1.]])}
        v.mask_threshold = .5
        v.semantic_landmarks = SemanticLandmarkMemory()
        features = np.zeros((1, 2, 2, 4), dtype=np.float32)
        features[:, 0] = 1.
        features[0, :, :, 1:3] = np.array([0., 1.])[:, None, None]
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        camera = dict(K=np.array([[100., 0., 60.], [0., 100., 40.], [0., 0., 1.]]),
            R_wc=rotation, t_wc=np.array([0., 0., 1.5]), width=120, height=80)
        camera_points = np.array([[x+dx, dy, 10.] for x in (-1.6, 1.2)
            for dx in np.linspace(-.18, .18, 4) for dy in np.linspace(.1, .46, 4)])
        points = camera_points @ rotation.T + camera['t_wc']
        captured = v.semantic_landmarks.observe(features, [camera], points, [0., 0., 0.],
            0., self.now, time.time(), 'spot1/odom')
        assert captured and not v.object_goal_memory.views

    def run_chain(self, passive, fault=None):
        stages = []
        if passive:
            self.passive_station()
        else:
            self.begin('gas station')
            self.observation('gas station', 10.)
            assert not self.bridge.object_found
            self.graph_route(self.bridge.expected)
            moved = self.complete_actions()
            self.observation('gas station', 10.)
            assert self.bridge.object_found
            stages.append(dict(query='gas station', found=True, actions=moved))
        self.begin('green residential house')
        self.observation('green residential house', 30.)
        assert not self.bridge.object_found
        self.graph_route(self.bridge.expected)
        if fault == 'extension_timeout':
            assert self.bridge.following and not self.bridge.object_found
            # Inject an outstanding extension request while the accepted
            # action is still running. Planner identity/time are the fault
            # input; all timeout/result handlers remain production callbacks.
            old_epoch = self.bridge.epoch
            self.bridge.extending_path = self.bridge.pending = True
            self.bridge.candidates = [(40., 0., 0., 40.)]
            self.bridge.candidate_index = 0
            self.bridge.normal_best = None
            self.bridge.plan_handle = Handle()
            self.bridge.plan_request_started_at = self.wall-self.bridge.compute_path_timeout-.1
            self.bridge.recover_timed_out_plan()
            moved = self.complete_actions()
            return dict(profile=self.profile, fault=fault, completed=False,
                failure_stage='green_house_action_handoff', reproduced_failure=(
                    self.bridge.epoch != old_epoch and self.bridge.following
                    and len(self.bridge.motion_queue) > 0),
                supplied_motion_actions=moved, final_status=deepcopy(self.bridge.last_status),
                following=self.bridge.following, object_found=self.bridge.object_found,
                trace=self.trace, supplied_plans=self.plans, field_acceptance=False)
        moved = self.complete_actions()
        self.observation('green residential house', 30.)
        assert self.bridge.object_found
        stages.append(dict(query='green residential house', found=True, actions=moved))
        self.begin('gas station')
        assert self.bridge.memory_selecting and not self.bridge.object_found
        evidence = self.bridge.memory_active.get('evidence', 'confirmed_viewpoint')
        for _ in range(16):
            if not self.bridge.memory_selecting:
                break
            self.graph_route(self.bridge.memory_goal)
        assert not self.bridge.memory_selecting and self.bridge.memory_goal is not None
        moved = self.complete_actions()
        assert not self.bridge.object_found  # Path/action success is insufficient.
        self.observation('gas station', 10.)
        # A historical view may require a fresh approach after reacquisition.
        if not self.bridge.object_found and self.bridge.expected is not None:
            self.graph_route(self.bridge.expected)
            moved += self.complete_actions()
            self.observation('gas station', 10.)
        assert self.bridge.object_found
        stages.append(dict(query='gas station', found=True, actions=moved, memory_evidence=evidence))
        assert self.initial_identity == tuple(map(id, (self.vision, self.db, self.vision.object_goal_memory)))
        return dict(profile=self.profile, passive_station_unqueried=passive,
            stages=stages, same_process_memory=True, trace=self.trace, supplied_plans=self.plans,
            action_count=len(self.actions), perception='synthetic supplied features/localization',
            planner='supplied straight paths, not Smac/NavFn',
            execution='perfect supplied motion, not RPP/BeamNG', field_acceptance=False)


def run(profile, passive, fault=None):
    session = Session('test_station_house_station_reuses_same_memory_and_can_finish_at_a_new_pose')
    with patch('time.monotonic', side_effect=lambda: getattr(session, 'wall', 1000.)):
        session.setup_chain(profile)
        try:
            return session.run_chain(passive, fault)
        finally:
            session.tearDown()


if __name__ == '__main__':
    output = ROOT/'test_artifacts/offline_system_chain_20261013'
    output.mkdir(exist_ok=True)
    report = dict(scope='Synthetic Python integration only; not offline full simulator success',
                  runs=[run('ackermann', False), run('ideal_point', True)],
                  fault_runs=[run('ackermann', False, 'extension_timeout'),
                              run('ideal_point', True, 'extension_timeout')])
    (output/'goal_chain.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:([{a:b for a,b in r.items() if a not in ('trace','supplied_plans')}
                         for r in v] if k in ('runs','fault_runs') else v)
                      for k,v in report.items()},ensure_ascii=False,indent=2))
