"""Attach ROS sensors to the current vehicle without reloading the map."""
import json
from pathlib import Path
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, runtime_env
import os
os.environ.update(runtime_env())
for item in os.environ.get('PYTHONPATH','').split(':'):
    if item and item not in sys.path: sys.path.insert(0,item)
import rclpy
from rclpy.parameter import Parameter
from beamng_ros2.beamng import BeamNGBridge
from beamng_ros2.scenario_loader import decode_sensors
from beamngpy.connection import Connection

rclpy.init()
bridge=BeamNGBridge()
bridge.set_parameters([Parameter('launch', value=False),Parameter('host',value='127.0.0.1'),Parameter('port',value=25252)])
b=bridge.beamng()
spec=json.loads((ROOT/'east_coast_town_reconnect.json').read_text())
vehicle=b.vehicles.get_current()['ego']
profile=json.loads((ROOT/'body_collision_full/profile/application.json').read_text())
vehicle.connection=Connection('127.0.0.1',int(profile['known_vehicle_port']),timeout=8)
vehicle.connect(b)
sensors=decode_sensors(spec['vehicles'][0]['sensors'])
for sensor in sensors:
    sensor.pre_scenario_start(vehicle)
    sensor.post_scenario_start(vehicle)
bridge._spawn_vehicle_node(vehicle,dict(ros_sensors=sensors))
bridge._vehicles['ego'].health_check()
bridge.executor_thread.start()
bridge.running=True
b.control.resume()
print('Attached current Town vehicle; map reloads=0',flush=True)
try:
    rclpy.spin(bridge)
finally:
    bridge.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

