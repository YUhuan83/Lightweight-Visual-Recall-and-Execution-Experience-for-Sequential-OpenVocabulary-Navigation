"""Independent actuator and telemetry sockets with recoverable I/O failures."""

import threading
import time

from beamngpy import BeamNGpy
from beamngpy.connection import Connection
from beamngpy.vehicle import Vehicle
from beamng_msgs.msg import StateSensor, VehicleControl
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy

from .direct_vehicle_control import install_direct_steering_control


class NativeVehicleControl(Node):
    def __init__(self):
        super().__init__("wildos_native_vehicle_control")
        for name, default in dict(host="127.0.0.1", port=25252, vehicle_port=0).items():
            self.declare_parameter(name, default)
        host = self.get_parameter("host").value
        self.bng = BeamNGpy(
            host, self.get_parameter("port").value, socket_timeout=2
        ).open(launch=False)
        self.vehicle = self.bng.vehicles.get_current()["ego"]
        port = self.get_parameter("vehicle_port").value
        if port:
            self.vehicle.connection = Connection(host, port, timeout=1)
        self.vehicle.connect(self.bng)
        if not port:
            port = self.vehicle.connection.port
        self.actuator = Vehicle(self.vehicle.vid, self.vehicle.model)
        self.actuator.connection = Connection(host, port, timeout=1)
        self.actuator.connect(self.bng)
        self.actuator.ai.set_mode("disabled")
        self.actuator.set_shift_mode("realistic_automatic")
        install_direct_steering_control(self.actuator)
        self.command = None
        self.lock = threading.Lock()
        self.retry_at = 0.0
        self.connected = True
        self.groups = [MutuallyExclusiveCallbackGroup() for _ in range(3)]
        self.publisher = self.create_publisher(StateSensor, "/wildos/native_state", 1)
        self.create_subscription(
            VehicleControl,
            "/wildos/native_control",
            self.on_control,
            1,
            callback_group=self.groups[0],
        )
        self.create_timer(0.05, self.control_tick, callback_group=self.groups[1])
        self.create_timer(0.1, self.poll_tick, callback_group=self.groups[2])

    def on_control(self, message):
        with self.lock:
            self.command = message, time.monotonic()

    def poll_tick(self):
        try:
            self.vehicle.sensors.poll()
            data = self.vehicle.state
            msg = StateSensor()
            msg.header.stamp = self.get_clock().now().to_msg()
            for name, key in [
                ("position", "pos"),
                ("velocity", "vel"),
                ("dir", "dir"),
                ("up", "up"),
            ]:
                vector = getattr(msg, name)
                vector.x, vector.y, vector.z = map(float, data[key])
            self.publisher.publish(msg)
        except (ConnectionError, OSError, TimeoutError) as error:
            self.get_logger().warning(
                f"Telemetry unavailable: {error}", throttle_duration_sec=5.0
            )

    def control_tick(self):
        now = time.monotonic()
        if not self.connected:
            if now < self.retry_at:
                return
            try:
                self.actuator.connect(self.bng, tries=1)
                install_direct_steering_control(self.actuator)
                self.connected = True
            except (ConnectionError, OSError, TimeoutError):
                self.retry_at = now + 1.0
                return
        with self.lock:
            value = self.command
        msg = (
            value[0]
            if value and now - value[1] < 0.5
            else VehicleControl(brake=1.0, gear=2)
        )
        try:
            self.actuator.control(
                **{
                    field: getattr(msg, field)
                    for field in (
                        "steering",
                        "throttle",
                        "brake",
                        "parkingbrake",
                        "clutch",
                        "gear",
                    )
                }
            )
        except (ConnectionError, OSError, TimeoutError) as error:
            self.get_logger().warning(
                f"Actuator reconnect: {error}", throttle_duration_sec=5.0
            )
            self.actuator.disconnect()
            self.connected = False
            self.retry_at = time.monotonic() + 1.0
            with self.lock:
                self.command = None

    def close(self):
        try:
            if self.connected:
                self.actuator.control(throttle=0.0, brake=1.0)
        finally:
            self.actuator.disconnect()
            self.vehicle.disconnect()
            self.bng.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = NativeVehicleControl()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.shutdown()
