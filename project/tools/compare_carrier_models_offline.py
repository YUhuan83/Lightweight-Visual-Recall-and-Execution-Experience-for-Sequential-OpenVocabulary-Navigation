"""Compare motion models on recorded cusp poses; never starts ROS or BeamNG.

Point-carrier commands below are a constructive reachability witness, not a
replay of NavFn/RPP. Bicycle results reuse the bounded endpoint-carrot audit.
The Lua arithmetic is read from production, but Lua/engine setters are not run.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from test_beamng_kinematic_math import source_expression
from offline_execution_contract_audit import endpoint_replay

LUA = ROOT / 'tools/beamng_extensions/wildosKinematic.lua'
FIXTURE = ROOT / 'nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json'


def point_step(position, heading, linear, angular, duration):
    """Navigation-origin XY equations selected from current Lua update()."""
    source = LUA.read_text(encoding='utf8')
    turn = angular * duration
    values = dict(linear=linear, angular=angular, heading=heading,
                  turn=turn, dtSim=duration, math=math)
    if abs(angular) > .000001:
        expressions = re.search(r'\n\s*dx = (.+)\n\s*dy = (.+)\n\s*else', source)
        dx, dy = [source_expression(x.strip(), values) for x in expressions.groups()]
    else:
        expressions = re.search(r'\n\s*dx, dy = (.+), (.+)\n\s*end', source)
        dx, dy = [source_expression(x.strip(), values) for x in expressions.groups()]
    return (position[0] + dx, position[1] + dy), heading + turn


def point_witness(position, heading, endpoint, endpoint_heading, *, direction=-1):
    """Rotate, drive a straight line, rotate: allowed only by ideal_point.

    No obstacles, actuator lag, deadlines, or actual RPP controller are modeled.
    Commands are refreshed conceptually and each dtSim remains below .25 s.
    """
    position = tuple(position)
    length = math.dist(position, endpoint)
    bearing = math.atan2(endpoint[1]-position[1], endpoint[0]-position[0])
    travel_heading = bearing + (math.pi if direction < 0 else 0.)
    first_turn = math.remainder(travel_heading-heading, 2*math.pi)
    schedule = [(0., math.copysign(.8, first_turn), abs(first_turn)/.8),
                (direction*1.2, 0., length/1.2)]
    last_turn = math.remainder(endpoint_heading-(heading+first_turn), 2*math.pi)
    schedule.append((0., math.copysign(.8, last_turn), abs(last_turn)/.8))
    initial = position
    for linear, angular, duration in schedule:
        if duration == 0:
            continue
        steps = math.ceil(duration/.02)
        for _ in range(steps):
            position, heading = point_step(position, heading, linear, angular, duration/steps)
    return dict(final_position=position, final_heading=heading,
                endpoint_distance_m=math.dist(position, endpoint),
                heading_error_rad=math.remainder(heading-endpoint_heading, 2*math.pi),
                translation_m=math.dist(initial, endpoint),
                commands=[dict(linear_m_s=v, angular_rad_s=w, duration_s=t)
                          for v, w, t in schedule])


def compare():
    case = json.loads(FIXTURE.read_text(encoding='utf8'))
    samples = []
    for sample in case['observations']:
        point = point_witness(sample['position'], sample['heading'],
                              case['points'][-1], case['headings'][-1])
        bicycle = endpoint_replay(sample['position'], sample['heading'],
                                  case['points'][-1], case['headings'][-1], direction=-1)
        samples.append(dict(start_position=sample['position'], start_heading=sample['heading'],
                            ideal_point_witness=point, bounded_bicycle_replay=bicycle))
    paths = [LUA, FIXTURE,
             ROOT/'nav2_adapter/config/nav2_official_control.yaml',
             ROOT/'nav2_adapter/config/nav2_ackermann_control.yaml',
             ROOT/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
             ROOT/'nav2_adapter/wildos_nav2_adapter/kinematic_vehicle_control.py',
             ROOT/'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py',
             ROOT/'nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py']
    return dict(scope='offline motion-model comparison; not full navigation acceptance',
                point='constructive command schedule using current Lua XY arithmetic',
                bicycle='endpoint-carrot feedback, instantaneous steering, radius 8 m',
                samples=samples,
                sources={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in paths})


if __name__ == '__main__':
    output = ROOT/'test_artifacts/offline_execution_audit_20261013/carrier_comparison.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    result = compare()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(dict(output=str(output), samples=[dict(
        point_error_m=s['ideal_point_witness']['endpoint_distance_m'],
        bicycle_reached=s['bounded_bicycle_replay']['reached'],
        bicycle_closest_m=s['bounded_bicycle_replay']['minimum_distance_m'])
        for s in result['samples']]), ensure_ascii=False, indent=2))
