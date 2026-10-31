"""Read recorded Twist/Odometry/String CDR without ROS; never operate a vehicle.

Only standard little-endian CDR messages of these three fixed ROS types are
supported. Receipt time is kept separate from header time. Motion estimates
describe recorded data, not a rerun of Nav2 or BeamNG.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3
import statistics
import struct


class CDR:
    def __init__(self, data):
        if data[:4] != b'\x00\x01\x00\x00':
            raise ValueError('Only standard little-endian CDR is supported')
        self.data, self.offset = data, 4

    def unpack(self, fmt, alignment):
        self.offset = 4 + ((self.offset-4+alignment-1)//alignment)*alignment
        values = struct.unpack_from('<'+fmt, self.data, self.offset)
        self.offset += struct.calcsize('<'+fmt)
        return values

    def string(self):
        length, = self.unpack('I', 4)
        end = self.offset+length
        if length < 1 or end > len(self.data) or self.data[end-1] != 0:
            raise ValueError('Invalid CDR string')
        value = self.data[self.offset:end-1].decode('utf8')
        self.offset = end
        return value

    def finish(self):
        # ROS serialized strings may include up to three final zero padding
        # bytes. Reject any unparsed field rather than guessing its layout.
        tail = self.data[self.offset:]
        if len(tail) > 3 or any(tail):
            raise ValueError('Unexpected fields or trailing data')


def decode_odometry(data):
    reader = CDR(data)
    sec, nsec = reader.unpack('iI', 4)
    frame, child = reader.string(), reader.string()
    pose = reader.unpack('7d', 8)
    reader.unpack('36d', 8)
    twist = reader.unpack('6d', 8)
    reader.unpack('36d', 8)
    reader.finish()
    if not 0 <= nsec < 10**9 or not all(map(math.isfinite, (*pose, *twist))):
        raise ValueError('Invalid odometry measurement')
    x, y, z, qx, qy, qz, qw = pose
    if abs(qx*qx+qy*qy+qz*qz+qw*qw-1.) > .1:
        raise ValueError('Invalid odometry quaternion')
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return dict(stamp_ns=sec*10**9+nsec, frame=frame, child=child, x=x, y=y,
                z=z, yaw=yaw, vx=twist[0], vy=twist[1], wz=twist[5])


def stats(values):
    ordered = sorted(values)
    return dict(n=len(values), median=statistics.median(ordered),
                p95=ordered[int(.95*(len(ordered)-1))], max=max(ordered)) if values else dict(n=0)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def audit(database):
    odometry, controls, commands, plan = [], [], [], []
    with sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True) as connection:
        for name, kind, receipt, data in connection.execute(
                'SELECT t.name,t.type,m.timestamp,m.data FROM messages m '
                'JOIN topics t ON m.topic_id=t.id ORDER BY m.timestamp,m.id'):
            if name == '/spot1/odom' and kind == 'nav_msgs/msg/Odometry':
                odometry.append(dict(decode_odometry(data), receipt_ns=receipt))
            elif name == '/wildos/cmd_vel' and kind == 'geometry_msgs/msg/Twist':
                reader = CDR(data)
                values = reader.unpack('6d', 8)
                reader.finish()
                commands.append(dict(receipt_ns=receipt, v=values[0], w=values[5]))
            elif name in ('/wildos/control_status', '/wildos/plan_status') and kind == 'std_msgs/msg/String':
                reader = CDR(data)
                value = json.loads(reader.string())
                reader.finish()
                if not isinstance(value, dict):
                    raise ValueError('Status must be a JSON object')
                (controls if name.endswith('control_status') else plan).append(
                    dict(value, receipt_ns=receipt))
    if len(odometry) < 2:
        raise ValueError('Insufficient recorded odometry')
    if any(b['stamp_ns'] <= a['stamp_ns'] for a,b in zip(odometry,odometry[1:])):
        raise ValueError('Recorded odometry header is not strictly increasing')
    position_twist_error, angular_twist_error, stable, motion = [], [], [], []
    controls.sort(key=lambda item: item['stamp_ns'])
    control_stamps = [item['stamp_ns'] for item in controls]
    for a,b in zip(odometry,odometry[1:]):
        dt = (b['stamp_ns']-a['stamp_ns'])*1e-9
        if not .01 <= dt <= .3:
            continue
        w = wrap(b['yaw']-a['yaw'])/dt
        h = a['yaw']+wrap(b['yaw']-a['yaw'])/2
        v = ((b['x']-a['x'])*math.cos(h)+(b['y']-a['y'])*math.sin(h))/dt
        vy = (-(b['x']-a['x'])*math.sin(h)+(b['y']-a['y'])*math.cos(h))/dt
        motion.append(dict(stamp_ns=a['stamp_ns'],receipt_ns=a['receipt_ns'],v=v,vy=vy,w=w))
        position_twist_error.append(abs(v-(a['vx']+b['vx'])/2))
        angular_twist_error.append(abs(w-(a['wz']+b['wz'])/2))
        index = bisect.bisect_right(control_stamps,a['stamp_ns'])-1
        if index < 0:
            continue
        target = controls[index]
        recent = controls[max(0,bisect.bisect_left(control_stamps,a['stamp_ns']-300_000_000)):index+1]
        if (recent and a['stamp_ns']-recent[0]['stamp_ns'] >= 250_000_000
                and a['stamp_ns']-target['stamp_ns'] <= 100_000_000
                and all(item.get('actuating') and
                        abs(item['target_speed']-target['target_speed']) < .01 and
                        abs(item['target_angular']-target['target_angular']) < .01 for item in recent)):
            stable.append(dict(v=v,w=w,target_v=target['target_speed'],target_w=target['target_angular']))
    angular = [item for item in stable if abs(item['target_w']) > .05 and abs(item['w']) > .02]
    linear = [item for item in stable if abs(item['target_v']) > .1 and abs(item['v']) > .02]
    # The bridge republishes status heartbeats. Count transitions by original
    # status stamp, not by the number of bag deliveries of the same event.
    failures = list({item['stamp_ns']:item for item in plan
                     if item.get('status') == 'follow_failed'}.values())
    reasons = {reason:sum(item.get('reason')==reason for item in failures)
               for reason in sorted({item.get('reason','unknown') for item in failures})}
    lag_scan = []
    for domain in ('stamp_ns', 'receipt_ns'):
        for lag in (0., .2, .4, .6, .8, 1., 1.5, 2.):
            paired = []
            for item in motion:
                when = item[domain]-int(lag*10**9)
                index = bisect.bisect_right(control_stamps,when)-1
                if index < 0:
                    continue
                target = controls[index]
                recent = controls[max(0,bisect.bisect_left(control_stamps,when-1_000_000_000)):index+1]
                if (recent and when-recent[0]['stamp_ns'] >= 900_000_000
                        and when-target['stamp_ns'] <= 100_000_000 and abs(target['target_angular'])>.05
                        and abs(item['w'])>.02 and all(value.get('actuating') and
                        abs(value['target_angular']-target['target_angular'])<.01 for value in recent)):
                    paired.append((item['w'],target['target_angular'],item['vy']))
            lag_scan.append(dict(time_domain=domain,lag_sec=lag,samples=len(paired),
                same_sign=sum(a*b>0 for a,b,_ in paired)/len(paired) if paired else None,
                gain_median=statistics.median(a/b for a,b,_ in paired) if paired else None,
                lateral_speed_per_commanded_yaw_median=statistics.median(c/b for _,b,c in paired) if paired else None,
                positive_commands=sum(b>0 for _,b,_ in paired), negative_commands=sum(b<0 for _,b,_ in paired)))
    return dict(scope=__doc__, database=str(database.resolve()),
        odom_count=len(odometry), first=odometry[0], last=odometry[-1],
        travel_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(odometry,odometry[1:])),
        frames=sorted({(item['frame'],item['child']) for item in odometry}),
        odom_age_at_bag_receipt_sec=stats([(item['receipt_ns']-item['stamp_ns'])*1e-9 for item in odometry]),
        position_derivative_vs_reported_body_speed_error_mps=stats(position_twist_error),
        yaw_derivative_vs_reported_angular_speed_error_radps=stats(angular_twist_error),
        cmd_vel_count=len(commands), control_status_count=len(controls),
        stable_command_windows=len(stable),
        stable_angular_sign_agreement=(sum(item['target_w']*item['w']>0 for item in angular)/len(angular) if angular else None),
        stable_linear_sign_agreement=(sum(item['target_v']*item['v']>0 for item in linear)/len(linear) if linear else None),
        angular_sample_count=len(angular), linear_sample_count=len(linear),
        angular_lag_scan_one_second_constant_command=lag_scan,
        follow_failure_reasons=reasons,
        limitations=['Only the recorded run is analyzed; it predates the latest RPP parameter correction.',
            'Planar finite differences approximate motion and include sampling/clock remapping effects.',
            'Stable-sign agreement can exclude a persistent sign inversion, not establish correct gains or latency.',
            'Bag receipt is not the controller or bridge callback receipt.'])


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', type=Path, default=root/'test_artifacts/server_validation_20261006/green_house_station_arrival_20261006_idealpoint_syncfix1/rosbag/rosbag_0.db3')
    parser.add_argument('--output', type=Path, default=root/'test_artifacts/offline_navigation/interface_replay_20261006.json')
    args = parser.parse_args()
    result = audit(args.database)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
