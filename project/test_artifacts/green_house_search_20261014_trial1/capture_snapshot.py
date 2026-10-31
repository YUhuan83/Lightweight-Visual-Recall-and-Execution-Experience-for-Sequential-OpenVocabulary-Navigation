"""Read one fresh camera frame and Native status; never publish control."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError(args.output)
    rclpy.init(); node=rclpy.create_node('green_house_diagnostic_snapshot')
    data={}
    def camera(msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        if msg.encoding != 'rgb8' or not 0<=age<=2: return
        pixels=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)
        image=pixels[:,:msg.width*3].reshape(msg.height,msg.width,3)
        data['image']=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        data['camera']={'source_stamp_ns':stamp,'frame_id':msg.header.frame_id,
                        'age_ros_sec':age,'width':msg.width,'height':msg.height}
    def native(msg):
        value=json.loads(msg.data)
        if 0<=time.time()-value.get('wall_time',0)<=.5:
            data['native']={k:value.get(k) for k in ('wall_time','position','direction','speed',
                           'telemetry_ok','actuator_ok','control_acknowledged','actual')}
    node.create_subscription(Image,'/spot1/realsense/front/color/image_raw',camera,qos_profile_sensor_data)
    node.create_subscription(String,'/wildos/native_control_status',native,10)
    try:
        deadline=time.monotonic()+6
        while time.monotonic()<deadline and not ('image' in data and 'native' in data):
            rclpy.spin_once(node,timeout_sec=.1)
        if 'image' not in data or 'native' not in data: raise RuntimeError('Fresh inputs missing')
        if not cv2.imwrite(str(args.output),data.pop('image')): raise RuntimeError('Image save failed')
        data.update(query='green residential house',diagnostic_only=True,is_arrival_evidence=False)
        args.output.with_suffix('.json').write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps(data),flush=True)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
