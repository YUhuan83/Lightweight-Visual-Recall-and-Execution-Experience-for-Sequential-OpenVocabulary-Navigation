"""Isolated ROS Humble metadata API probe; no vehicle or control topic."""
import json,time
import rclpy
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('isolated_subscription_metadata_probe')
pub=node.create_publisher(String,'/diagnostic_message_metadata',10)
sub=node.create_subscription(String,'/diagnostic_message_metadata',lambda _:None,10)
try:
    deadline=time.monotonic()+3
    while pub.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.05)
    assert pub.get_subscription_count()==1
    pub.publish(String(data='brakes-only metadata diagnostic'))
    result=None
    while result is None and time.monotonic()<deadline:
        with sub.handle:result=sub.handle.take_message(String,False)
        time.sleep(.01)
    assert result is not None
    print(repr(result));print(type(result[1]).__name__)
finally:node.destroy_node();rclpy.shutdown()
