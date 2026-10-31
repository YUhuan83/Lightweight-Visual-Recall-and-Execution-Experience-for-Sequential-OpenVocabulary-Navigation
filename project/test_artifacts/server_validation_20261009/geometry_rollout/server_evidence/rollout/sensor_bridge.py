"""Attach ROS sensors to the current vehicle without reloading the map."""
import json
from pathlib import Path
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, runtime_env
import os
# Use the verified isolated overlay environment
for item in os.environ.get('PYTHONPATH','').split(':'):
    if item and item not in sys.path: sys.path.insert(0,item)
import rclpy
from rclpy.parameter import Parameter
from beamng_ros2.beamng import BeamNGBridge
from beamng_ros2.scenario_loader import decode_sensors
from beamngpy.connection import Connection

from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
install_fail_closed_reads(PrefixedLengthSocket)
rclpy.init()
bridge=BeamNGBridge()
bridge.set_parameters([Parameter('launch', value=False),Parameter('host',value='127.0.0.1'),Parameter('port',value=25252)])
b=bridge.beamng()
spec=json.loads((ROOT/'east_coast_town_reconnect.json').read_text())
vehicle=b.vehicles.get_current()['ego']
vehicle.connection=Connection('127.0.0.1',38197,timeout=6)
vehicle.connect(b)
sensors=decode_sensors(spec['vehicles'][0]['sensors'])
for sensor in sensors:
    sensor.pre_scenario_start(vehicle)
    sensor.post_scenario_start(vehicle)
bridge._spawn_vehicle_node(vehicle,dict(ros_sensors=sensors))
vehicle_node=bridge._vehicles['ego']
vehicle_node.set_parameters([Parameter('update_sec',value=0.1)])
vehicle_node.destroy_timer(vehicle_node.timer)
vehicle_node.reset_timer()
vehicle_node.health_check()
bridge.executor_thread.start()
bridge.running=True
b.control.resume()
print('Attached current Town vehicle; map reloads=0',flush=True)
try:
    rclpy.spin(bridge)
finally:
    bridge.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

