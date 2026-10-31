"""Optional ROS 2 clock for an explicitly normalized acquisition Float64 stream.

NOT an adapter for raw upstream TimeSensor, vehicle headers or wall/poll times.
No timer, extrapolation, auto-discovery or in-process epoch reset.
"""

import argparse
import re

from .core import AcquisitionClock, ValidationError


class ClockInput:
    """ROS-independent callback logic, including fail-stop and ROS sec range."""

    def __init__(self, topic, source, epoch, *, normalized_acquisition_confirmed):
        if normalized_acquisition_confirmed is not True:
            raise ValidationError("explicit normalized acquisition confirmation required")
        if not isinstance(topic, str) or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*", topic):
            raise ValidationError("absolute explicit ROS topic required")
        if topic == "/clock" or topic.startswith("/vehicles/"):
            raise ValidationError("raw upstream endpoints and /clock feedback forbidden")
        self.topic = topic
        self.clock = AcquisitionClock(source, epoch)
        self.stopped = False

    def receive(self, seconds):
        if self.stopped:
            raise ValidationError("clock stopped; coordinated epoch restart required")
        try:
            stamp = self.clock.accept(seconds, source=self.clock.source, epoch=self.clock.epoch,
                                      time_kind="acquisition")
            if stamp is not None and stamp["sec"] > 2**31-1:
                raise ValidationError("ROS builtin_interfaces/Time sec exceeds int32")
            return stamp
        except ValidationError:
            self.stopped = True
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--source-id", required=True)
    parser.add_argument("--epoch", required=True)
    parser.add_argument("--confirm-normalized-acquisition", action="store_true", required=True)
    args = parser.parse_args(argv)
    gate = ClockInput(args.input_topic, args.source_id, args.epoch,
                     normalized_acquisition_confirmed=args.confirm_normalized_acquisition)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float64
    except ImportError as exc:
        parser.exit(2, f"ROS 2 Python dependencies unavailable: {exc}\n")

    rclpy.init(args=[])
    node = Node("normalized_acquisition_clock", use_global_arguments=False)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(Clock, "/clock", qos)

    def receive(message):
        if gate.stopped:
            return
        try:
            stamp = gate.receive(message.data)
            if stamp is None:
                return
            clock = Clock()
            clock.clock.sec = stamp["sec"]
            clock.clock.nanosec = stamp["nanosec"]
            publisher.publish(clock)
        except ValidationError as exc:
            node.get_logger().error(f"/clock publishing STOPPED: {exc}")

    node.create_subscription(Float64, gate.topic, receive, qos)
    node.get_logger().warning("Only normalized acquisition seconds accepted; NOT raw BeamNG TimeSensor. "
                              "One clock authority required. Restart all consumers on epoch change.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
