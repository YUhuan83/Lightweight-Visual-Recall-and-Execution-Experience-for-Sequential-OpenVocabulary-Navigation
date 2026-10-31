"""Read-only audit of logged GraphNav goals against the logged global costmap.

Run from the project root with a plain Python (no rclpy needed):
    python test_artifacts/execution_continuity_repair_20261021/audit_goal_pose_filter.py

It decodes the CDR OccupancyGrid/route_status messages from the closed bags and
applies the same 1.0 m / 0.2 m vehicle-pose check added to the bridge. This is
an offline data audit, not a ROS/DDS or vehicle test.
"""
import json
import math
import sqlite3
import struct
import sys
from pathlib import Path
from types import SimpleNamespace as S

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nav2_adapter'))
from wildos_nav2_adapter.graph_route import footprint_blockers  # noqa: E402


def cdr_string(data):
    length = struct.unpack_from('<I', data, 4)[0]
    return data[8:8 + length - 1].decode('utf-8')


def cdr_grid(data):
    offset = 4

    def align(size):
        nonlocal offset
        offset = ((offset + 4) + size - 1) // size * size - 4

    def u32():
        nonlocal offset
        value = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        return value

    def i32():
        nonlocal offset
        value = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        return value

    def f32():
        nonlocal offset
        value = struct.unpack_from('<f', data, offset)[0]
        offset += 4
        return value

    def f64():
        nonlocal offset
        value = struct.unpack_from('<d', data, offset)[0]
        offset += 8
        return value

    align(4)
    i32()
    u32()
    align(4)
    length = u32()
    offset += length
    align(4)
    align(4)
    i32()
    u32()
    align(4)
    resolution = f32()
    align(4)
    width, height = u32(), u32()
    align(8)
    origin_x, origin_y, _ = f64(), f64(), f64()
    for _ in range(4):
        f64()
    length = u32()
    cells = list(struct.unpack_from('<%db' % length, data, offset))
    return S(info=S(resolution=resolution, width=width, height=height,
                    origin=S(position=S(x=origin_x, y=origin_y),
                             orientation=S(x=0., y=0., z=0., w=1.))),
             data=cells)


def read_topic(bag, name):
    con = sqlite3.connect('file:' + str(bag) + '?mode=ro', uri=True)
    topic = con.execute('select id from topics where name=?', (name,)).fetchone()
    if topic is None:
        return []
    return con.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                       (topic[0],)).fetchall()


def grid_before(bag, stamp_ns):
    rows = read_topic(bag, '/global_costmap/costmap')
    chosen = [row for row in rows if row[0] <= stamp_ns]
    return cdr_grid(chosen[-1][1]) if chosen else None


def clear_pose(grid, position, heading, tolerance=1.0, step=.2):
    if not footprint_blockers(grid, position, heading):
        return True, (0., 0.)
    limit = int(round(tolerance / step))
    for dx in range(-limit, limit + 1):
        for dy in range(-limit, limit + 1):
            offset = (dx * step, dy * step)
            if offset == (0., 0.) or math.hypot(*offset) > tolerance + 1e-9:
                continue
            if not footprint_blockers(
                    grid, (position[0] + offset[0], position[1] + offset[1]), heading):
                return True, offset
    return False, None


def audit(path, stamp_ns, goals):
    grid = grid_before(path, stamp_ns)
    result = []
    for position, heading, label in goals:
        feasible, offset = clear_pose(grid, position, heading)
        result.append(dict(label=label, position=list(position), heading=heading,
                           exact_blocked=bool(footprint_blockers(grid, position, heading)),
                           clear_within_1m=feasible, clear_offset=offset))
    return result


def main():
    base = ROOT / 'test_artifacts/execution_continuity_repair_20261021'
    trial_094550 = base / 'trial_20261021_094550/evidence_bundle/interfaces_0.db3'
    trial_093254 = base / ('v5/trial_20261021_093254/sessions/session_20261021_093254/'
                           'goal_001/interfaces/interfaces_0.db3')
    report = {'scope': 'closed bags, 1.0 m goal tolerance, actual SBR footprint',
              'real_ros_run': False, 'vehicle_simulation_run': False}
    report['094550_options'] = audit(trial_094550, 1792547211846000000, [
        ((732.009009969055, -22.37764837110526), -1.5707963148700164, 'option_1'),
        ((727.6213619726667, -22.684329991454042), -1.5707963148700164, 'option_2'),
        ((719.3069598018975, -4.794713031110428), -1.9634955565081997, 'option_3'),
        ((721.4559196206699, -15.897965154182383), -1.9634955565081997, 'option_4')])
    report['093254_options'] = audit(trial_093254, 1792546504397000000, [
        ((742.2360825971764, -30.829628827777164), -1.1780970732318325, 'option_1'),
        ((738.6971220818259, -31.85583145164346), -1.1780970732318325, 'option_2'),
        ((729.4943290701053, -30.65628668042873), -1.1780970732318325, 'option_3'),
        ((735.4990235931996, -33.908324345210715), -1.1780970732318325, 'option_4')])
    report['093254_route_samples'] = audit(trial_093254, 1792546511320000000, [
        ((742.24, -30.83), -1.434, 'sample_0'),
        ((742.06, -29.54), -1.434, 'sample_1'),
        ((741.79, -27.55), -1.553, 'sample_2'),
        ((741.53, -21.57), -1.338, 'sample_3'),
        ((740.61, -17.68), -1.338, 'sample_4'),
        ((740.15, -15.74), -1.606, 'sample_5')])
    out = base / 'goal_pose_filter_audit.json'
    out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
