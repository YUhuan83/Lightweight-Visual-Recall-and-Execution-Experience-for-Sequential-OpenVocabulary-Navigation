"""Interface doubles for callback unit tests in an isolated offline process.

No DDS, sockets, timers, ROS serialization, TF or vehicle simulator is used.
Production callback code is imported unchanged. These doubles are deliberately
limited to the message fields and APIs used by the tested adapter classes.
"""
from concurrent.futures import Future
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock
import sys
import time


def message(name, **fields):
    def initialize(self, **values):
        unknown = set(values) - set(fields)
        if unknown:
            raise TypeError(f'{name}: unsupported fields {unknown}')
        for key, factory in fields.items():
            setattr(self, key, values[key] if key in values else factory())
    return type(name, (), {'__init__': initialize})


def expose(name, **symbols):
    parts = name.split('.')
    for index in range(1, len(parts) + 1):
        path = '.'.join(parts[:index])
        if path not in sys.modules:
            sys.modules[path] = ModuleType(path)
            sys.modules[path].__path__ = []
        if index > 1:
            setattr(sys.modules['.'.join(parts[:index - 1])], parts[index - 1], sys.modules[path])
    sys.modules[name].__dict__.update(symbols)


class ClockType:
    STEADY_TIME = 1


class Clock:
    def __init__(self, clock_type=None):
        self.steady = clock_type == ClockType.STEADY_TIME

    def now(self):
        value = time.monotonic_ns() if self.steady else time.time_ns()
        return SimpleNamespace(nanoseconds=value, to_msg=lambda:
            SimpleNamespace(sec=value // 10**9, nanosec=value % 10**9))


class Node:
    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.clock = Clock()
        self.logger = Mock()

    def declare_parameter(self, name, value):
        self.parameters[name] = SimpleNamespace(name=name, value=value)
        return self.parameters[name]

    def get_parameter(self, name):
        return self.parameters[name]

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def create_publisher(self, *_args, **_kwargs):
        return Mock()

    def create_subscription(self, *_args, **_kwargs):
        return SimpleNamespace()

    def create_timer(self, *_args, **_kwargs):
        return SimpleNamespace()  # Timers never fire automatically.

    def create_client(self, *_args, **_kwargs):
        return SimpleNamespace(service_is_ready=lambda: False, call_async=lambda _r: Future())

    def add_on_set_parameters_callback(self, _callback):
        pass

    def destroy_node(self):
        pass


class ActionClient:
    def __init__(self, *_args, **_kwargs):
        pass

    def wait_for_server(self, **_kwargs):
        return False

    def send_goal_async(self, _goal):
        return Future()


def install():
    if 'rclpy' in sys.modules:
        raise RuntimeError('Offline doubles must run in a fresh process before importing ROS')
    vector = message('Vector3', x=float, y=float, z=float)
    quaternion = message('Quaternion', x=float, y=float, z=float, w=float)
    stamp = message('Time', sec=int, nanosec=int)
    expose('builtin_interfaces.msg', Time=stamp)
    header = message('Header', stamp=stamp, frame_id=str)
    pose = message('Pose', position=vector, orientation=quaternion)
    pose_stamped = message('PoseStamped', header=header, pose=pose)
    twist = message('Twist', linear=vector, angular=vector)
    pose_cov = message('PoseWithCovariance', pose=pose, covariance=lambda: [0.] * 36)
    twist_cov = message('TwistWithCovariance', twist=twist, covariance=lambda: [0.] * 36)
    info = message('MapMetaData', width=int, height=int, resolution=float, origin=pose)
    expose('geometry_msgs.msg', PoseStamped=pose_stamped, Pose=pose, Twist=twist)
    uuid = message('UUID', id=lambda: [0] * 16)
    traversal = message('TraversalFeedback', header=header, from_node=uuid, to_node=uuid,
        traversability_class=str, robot_profile_id=str, start_pose=pose, end_pose=pose,
        commanded_heading=float, observed_heading=float, commanded_distance=float,
        observed_distance=float, duration=float, energy_cost=float, slip_ratio=float,
        outcome=int, retryable=bool, observations=list)
    traversal.OUTCOME_SUCCEEDED = 1
    traversal.OUTCOME_UNKNOWN = 0
    expose('graphnav_msgs.msg', NavigationGraph=message('NavigationGraph', header=header,
        nodes=list, edges=list, current_node_idx=int), TraversalFeedback=traversal,
        KeyValue=message('KeyValue', key=str, value=list), UUID=uuid)
    expose('nav_msgs.msg', Path=message('Path', header=header, poses=list),
           Odometry=message('Odometry', header=header, pose=pose_cov, twist=twist_cov),
           OccupancyGrid=message('OccupancyGrid', header=header, info=info, data=list))
    expose('std_msgs.msg', String=message('String', data=str))
    expose('beamng_msgs.msg', StateSensor=message('StateSensor', header=header,
           position=vector, velocity=vector, dir=vector, up=vector),
           VehicleControl=message('VehicleControl', throttle=float, brake=float, steering=float,
                                  parkingbrake=float, clutch=float, gear=int))
    expose('action_msgs.msg', GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4))
    expose('nav2_msgs.action',
           ComputePathToPose=SimpleNamespace(Goal=message('ComputeGoal', goal=pose_stamped,
               use_start=bool, planner_id=str)),
           FollowPath=SimpleNamespace(Goal=message('FollowGoal', path=lambda:
               sys.modules['nav_msgs.msg'].Path(), controller_id=str, goal_checker_id=str)))
    expose('std_srvs.srv', Trigger=SimpleNamespace(Request=message('TriggerRequest')))
    expose('rcl_interfaces.msg', SetParametersResult=message('SetParametersResult',
                                                           successful=bool, reason=str))
    expose('rclpy', init=lambda: None, shutdown=lambda: None, ok=lambda: True)
    expose('rclpy.node', Node=Node)
    expose('rclpy.clock', Clock=Clock, ClockType=ClockType)
    expose('rclpy.action', ActionClient=ActionClient)
    expose('rclpy.qos', qos_profile_sensor_data=SimpleNamespace())
    expose('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}))
