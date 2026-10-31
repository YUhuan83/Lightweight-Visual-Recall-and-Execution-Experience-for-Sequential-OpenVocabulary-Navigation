"""Extract the earliest abort's real DDS pose/path evidence for offline replay."""
import json
import math
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/ly/town_memory_validation_20261014/green_house_trial1')


def stamp(header): return header.stamp.sec*10**9+header.stamp.nanosec
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(p): return [p.position.x,p.position.y,yaw(p.orientation)]


def main():
    events=[json.loads(x) for x in (ROOT/'events.jsonl').read_text().splitlines() if x]
    abort=next(x for x in events if x.get('kind')=='plan_status' and x.get('status')=='follow_failed')
    at=abort['stamp_ns']
    planned=[x for x in events if x.get('kind')=='plan_status' and x.get('status')=='planned'
             and x.get('stamp_ns',0)<at][-1]
    db=next((ROOT/'interfaces').glob('*.db3'))
    conn=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    topics={name:(tid,get_message(typ)) for tid,name,typ in conn.execute('SELECT id,name,type FROM topics')}
    result={'abort':abort,'last_plan_status':planned,'database':db.name,'topics':{}}
    for topic in ('/spot1/odom','/received_global_plan','/wildos/planned_path','/wildos/native_control_status'):
        tid,typ=topics[topic]
        rows=conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 5',
                          (tid,at)).fetchall()
        converted=[]
        for received,blob in reversed(rows):
            message=deserialize_message(blob,typ)
            item={'received_ns':received}
            if topic=='/spot1/odom':
                item.update(source_stamp_ns=stamp(message.header),frame_id=message.header.frame_id,
                    child_frame_id=message.child_frame_id,pose=pose(message.pose.pose),
                    linear=[message.twist.twist.linear.x,message.twist.twist.linear.y],
                    source_age_at_receipt_s=(received-stamp(message.header))/1e9)
            elif topic.endswith('native_control_status'):
                value=json.loads(message.data)
                item.update({k:value.get(k) for k in ('wall_time','position','direction','speed','actual',
                    'telemetry_age_sec','telemetry_ok','actuator_ok','control_acknowledged')})
            else:
                item.update(source_stamp_ns=stamp(message.header),frame_id=message.header.frame_id,
                    poses=[pose(x.pose) for x in message.poses])
            converted.append(item)
        result['topics'][topic]=converted
    odom=result['topics']['/spot1/odom'][-1]
    path=result['topics']['/wildos/planned_path'][-1]
    end=path['poses'][-1]
    current=odom['pose']
    summary={'controller_abort_stamp_ns':at,'odom_source_stamp_ns':odom['source_stamp_ns'],
        'odom_source_age_at_receipt_s':odom['source_age_at_receipt_s'],
        'odom_frame':odom['frame_id'],'odom_child_frame':odom['child_frame_id'],
        'current_pose':current,'section_endpoint_pose':end,
        'position_error_m':math.dist(current[:2],end[:2]),
        'yaw_error_rad':abs(math.atan2(math.sin(current[2]-end[2]),math.cos(current[2]-end[2]))),
        'goal_checker_id':planned['goal_checker_id'],
        'goal_position_tolerance_m':planned['goal_position_tolerance_m']}
    received=result['topics']['/received_global_plan'][-1]
    summary['controller_transformed_plan_frame']=received['frame_id']
    summary['controller_transformed_endpoint']=received['poses'][-1] if received['poses'] else None
    summary['controller_transformed_plan_receive_age_s']=(at-received['received_ns'])/1e9
    result['summary']=summary
    (ROOT/'first_abort_interface_fixture.json').write_text(json.dumps(result,indent=2)+'\n')
    conn.close()
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__': main()
