"""Attach the calibrated rig to the loaded map; own only these sensor nodes."""

import argparse
import json
from pathlib import Path
import signal
import threading

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from beamng_ros2.beamng import BeamNGBridge
from beamng_ros2.scenario_loader import decode_sensors
from beamngpy.connection import Connection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rig", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25252)
    args = parser.parse_args()
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    bridge = BeamNGBridge()
    bridge.set_parameters(
        [
            Parameter("launch", value=False),
            Parameter("host", value=args.host),
            Parameter("port", value=args.port),
        ]
    )
    sensors = []
    try:
        bng = bridge.beamng()
        vehicle = bng.vehicles.get_current()["ego"]
        response = bng.vehicles.start_connection(vehicle, vehicle.extensions)
        port = int(response["result"])
        vehicle.connection = Connection(args.host, port, timeout=3)
        vehicle.connect(bng)
        rig = json.loads(args.rig.read_text())["vehicles"][0]["sensors"]
        sensors = decode_sensors(rig)
        for sensor in sensors:
            sensor.pre_scenario_start(vehicle)
            sensor.post_scenario_start(vehicle)
        bridge._spawn_vehicle_node(vehicle, dict(ros_sensors=sensors))
        node = bridge._vehicles["ego"]
        node.set_parameters([Parameter("update_sec", value=0.1)])
        node.destroy_timer(node.timer)
        node.reset_timer()
        node.health_check()
        bridge.mt_executor.add_node(bridge)
        bridge.running = True
        bridge.executor_thread.start()
        bng.control.resume()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(dict(ready=True, vehicle_port=port)))
        while rclpy.ok() and not stopped.wait(0.2):
            pass
    finally:
        bridge.mt_executor.shutdown(timeout_sec=5)
        if bridge.executor_thread.is_alive():
            bridge.executor_thread.join(timeout=5)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
