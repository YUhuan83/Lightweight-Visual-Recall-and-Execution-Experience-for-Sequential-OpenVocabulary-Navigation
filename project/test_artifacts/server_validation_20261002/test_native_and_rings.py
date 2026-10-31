"""Real ROS executor regression plus late-subscriber score-ring regression."""
import ast
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env(domain=53)).returncode)
sys.argv=[sys.argv[0]]
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from beamng_msgs.msg import VehicleControl
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from wildos_nav2_adapter.native_vehicle_control import NativeVehicleControl

class NativeTests(unittest.TestCase):
    def test_commands_arrive_during_blocking_poll_and_reverse_timeout_keeps_gear(self):
        rclpy.init();entered=threading.Event();received=threading.Event();finished=threading.Event()
        vehicle=Mock();vehicle.state={'vel':[0.,0.,0.],'pos':[0.,0.,0.],'time':0.}
        sensor=Mock();sensor.__getitem__=Mock(return_value={})
        def poll():
            entered.set();time.sleep(.35);finished.set()
        sensor.poll.side_effect=poll;vehicle.sensors=sensor
        bng=Mock();bng.vehicles.get_current.return_value={'ego':vehicle}
        with patch('wildos_nav2_adapter.native_vehicle_control.BeamNGpy',return_value=bng):
            node=NativeVehicleControl()
        sender=Node('native_regression_sender');pub=sender.create_publisher(VehicleControl,'/wildos/native_control',1)
        # Observe the actual receiver without replacing its subscription.
        original=node.on_control
        def receive(msg):original(msg);received.set()
        node.destroy_subscription(next(iter(node.subscriptions)))
        node.create_subscription(VehicleControl,'/wildos/native_control',receive,1,callback_group=node.control_group)
        msg=VehicleControl();msg.gear=-1;msg.throttle=.1
        sender.create_timer(.025,lambda:pub.publish(msg))
        executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);executor.add_node(sender)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            self.assertTrue(entered.wait(3),'IO timer did not run')
            self.assertTrue(received.wait(.25),'Command callback blocked behind synchronous polling')
            self.assertFalse(finished.is_set(),'Command arrived only after polling returned')
            time.sleep(.5)
        finally:
            executor.shutdown();thread.join();sender.destroy_node()
        try:
            self.assertLess(time.monotonic()-node.command_at,.5)
            self.assertTrue(any(c.kwargs['throttle']>0 for c in vehicle.control.call_args_list))
            with node.command_lock:node.command_at=time.monotonic()-2
            node.tick()
            self.assertEqual(vehicle.control.call_args.kwargs['gear'],-1)
            self.assertEqual(vehicle.control.call_args.kwargs['brake'],1.)
        finally:node.destroy_node();rclpy.shutdown()

class RingTests(unittest.TestCase):
    def test_removed_unpublished_and_late_existing_and_reintroduced_frontiers(self):
        source=WS/'src/nebula2-wildos/visual_navigation/visual_navigation/wildos/viz.py'
        tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='visualize_all_heading_scores')
        scope={'Marker':Marker,'MarkerArray':MarkerArray,'Point':Point,'np':np,
            'plt':SimpleNamespace(cm=SimpleNamespace(jet=lambda s:(s,0.,1.-s,1.)))}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        owner=SimpleNamespace(uuid_to_marker_id={},marker_id=0,num_bins=2,
            bin_starts=np.array([0.,np.pi]),discretization_angle=np.pi,ring_radius=1.)
        fn=lambda scores,removed,updated:scope[method.name](owner,scores,removed,updated,'map',rclpy.time.Time().to_msg())
        self.assertEqual(len(fn({},['never_published'],set()).markers),0)
        node=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.,z=3.)))
        scores={'late':([.2,.8],node)}
        first=fn(scores,[],set());self.assertEqual(len(first.markers),2)
        ids=[m.id for m in first.markers]
        deleted=fn({},['late'],{'late'});self.assertEqual([m.id for m in deleted.markers],ids)
        self.assertNotIn('late',owner.uuid_to_marker_id)
        again=fn(scores,[],set());self.assertEqual(len(again.markers),2)
        self.assertTrue(set(ids).isdisjoint(m.id for m in again.markers))

if __name__=='__main__':unittest.main(verbosity=2)
