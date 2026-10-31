"""Free-space bicycle-model replay of a saved path, without ROS or BeamNG.

The RPP geometry is reduced to pruning, lookahead, curvature and approach
scaling. Path headings are inferred from positions, not historical ROS poses.
No perception, obstacle layers, wheel physics, latency or speed loop is modeled.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav2_adapter'))
from wildos_nav2_adapter.drive_math import clamp, steering_from_angular
from wildos_nav2_adapter.graph_route import distance_to_route
from wildos_nav2_adapter.motion_path import motion_sections
from diagnose_town_follow_path import rpp_nearest_index


def model_parameters():
    """Use current source defaults and RPP configuration, without importing ROS."""
    import yaml
    tree = ast.parse((ROOT/'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py').read_text(encoding='utf8'))
    defaults = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) == 2):
            try:
                defaults[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
            except (ValueError, TypeError):
                pass
    config = yaml.safe_load((ROOT/'nav2_adapter/config/nav2_official_control.yaml').read_text(encoding='utf8'))
    follow = config['controller_server']['ros__parameters']['FollowPath']
    return dict(wheelbase=defaults['wheelbase'], max_steer_angle=defaults['max_steer_angle'],
        max_steering=defaults['max_steering'], max_angular=defaults['max_angular'],
        planner_radius=defaults['minimum_turning_radius'],
        lookahead_time=follow['lookahead_time'], min_lookahead=follow['min_lookahead_dist'],
        max_lookahead=follow['max_lookahead_dist'])


def tracking_error_limit_from_source():
    """Read the bridge's active default so replay cancellation matches runtime."""
    source = ROOT / 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
    tree = ast.parse(source.read_text(encoding='utf8'))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) == 2):
            try:
                name, value = map(ast.literal_eval, node.args)
            except (ValueError, TypeError):
                continue
            if name == 'tracking_error_limit':
                value = float(value)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('invalid GraphPathBridge tracking_error_limit')
                return value
    raise ValueError('GraphPathBridge tracking_error_limit default is missing')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inferred_headings(points, sections):
    result = [0.] * len(points)
    # Town's observed initial vehicle direction faces the opposite direction
    # from the first (reverse) path edge. Later signs alternate at the cusps.
    for section_index, (first, last) in enumerate(sections):
        reverse_offset = math.pi if section_index % 2 == 0 else 0.
        for index in range(first, last + 1):
            if last - first < 2:
                a, b = points[first], points[last]
                tangent = math.atan2(b[1] - a[1], b[0] - a[0])
            else:
                before = max(first, min(index - 1, last - 2))
                a, b, c = points[before:before + 3]
                ax, ay, bx, by = b[0] - a[0], b[1] - a[1], c[0] - a[0], c[1] - a[1]
                cross = ax * by - ay * bx
                if abs(cross) < 1e-9:
                    tangent = math.atan2(ay, ax)
                else:
                    length_a, length_b = ax * ax + ay * ay, bx * bx + by * by
                    center_x = a[0] + (length_a * by - length_b * ay) / (2 * cross)
                    center_y = a[1] + (ax * length_b - bx * length_a) / (2 * cross)
                    p = points[index]
                    tangent = math.atan2(p[1] - center_y, p[0] - center_x) + math.copysign(math.pi / 2., cross)
            result[index] = wrap(tangent + reverse_offset)
    return result


def carrot(local, lookahead):
    for index, b in enumerate(local):
        if math.hypot(*b) < lookahead:
            continue
        if index == 0:
            return b
        a = local[index - 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        quadratic = dx * dx + dy * dy
        if quadratic < 1e-12:
            return b
        linear = 2 * (a[0] * dx + a[1] * dy)
        constant = a[0] ** 2 + a[1] ** 2 - lookahead ** 2
        discriminant = max(0., linear * linear - 4 * quadratic * constant)
        roots = [(-linear + sign * math.sqrt(discriminant)) / (2 * quadratic) for sign in (-1, 1)]
        valid = [t for t in roots if 0. <= t <= 1.]
        if not valid:
            return b
        t = max(valid)
        return (a[0] + t * dx, a[1] + t * dy)
    return local[-1]


def simulate(points, headings, split, corrected, desired_speed=1.2, lateral_offset=0.,
             seconds=120., dt=.05, cusp_tolerance=.2, tracking_error_limit=1.0,
             parameters=None, actuator_radius=None):
    parameters = model_parameters() if parameters is None else parameters
    wheelbase, steer_angle, steer_limit = (parameters[k] for k in ('wheelbase', 'max_steer_angle', 'max_steering'))
    physical_radius = wheelbase / math.tan(steer_limit * steer_angle)
    radius_limit = physical_radius if actuator_radius is None else actuator_radius
    boundaries = motion_sections(points) if split else [(0, len(points) - 1)]
    section = 0
    first, last = boundaries[section]
    pruned = first
    x, y, heading = points[0][0], points[0][1] + lateral_offset, headings[0]
    speed = 0.
    max_error = 0.
    samples = []
    skipped = []
    transitions = []
    failure_reason = None
    original_cusps = [end for _, end in motion_sections(points)[:-1]]
    completed = False
    for tick in range(int(seconds / dt)):
        target = points[last]
        position_error = math.hypot(x - target[0], y - target[1])
        heading_error = abs(wrap(heading - headings[last]))
        intermediate = section < len(boundaries) - 1
        if position_error <= (cusp_tolerance if intermediate else 1.) and heading_error <= (.35 if intermediate else .7):
            transitions.append(dict(time=tick * dt, section=section, target_index=last,
                                    position_error=position_error, heading_error=heading_error))
            if not intermediate:
                completed = True
                break
            section += 1
            first, last = boundaries[section]
            pruned = first
            speed = 0.  # Instant stop/shift is an explicit ideal-model assumption.
        nearest, _ = rpp_nearest_index(points[:last + 1], (x, y), 10., pruned)
        for cusp in original_cusps:
            if pruned <= cusp < nearest:
                gap = math.hypot(x - points[cusp][0], y - points[cusp][1])
                cusp_heading_error = abs(wrap(heading - headings[cusp]))
                if gap > cusp_tolerance or cusp_heading_error > .35:
                    skipped.append(dict(time=tick * dt, cusp_index=cusp, nearest_index=nearest,
                                        gap=gap, heading_error=cusp_heading_error))
        pruned = nearest
        c, s = math.cos(heading), math.sin(heading)
        local = [(c * (p[0] - x) + s * (p[1] - y),
                  -s * (p[0] - x) + c * (p[1] - y)) for p in points[pruned:last + 1]]
        lookahead = clamp(abs(speed) * parameters['lookahead_time'],
                          parameters['min_lookahead'], parameters['max_lookahead'])
        for before, center, after in zip(local, local[1:], local[2:]):
            oa = (center[0] - before[0], center[1] - before[1])
            ab = (after[0] - center[0], after[1] - center[1])
            if oa[0] * ab[0] + oa[1] * ab[1] < 0.:
                lookahead = min(lookahead, math.hypot(*center))
                break
        cx, cy = carrot(local, lookahead)
        curvature = 2 * cy / (cx * cx + cy * cy) if cx * cx + cy * cy > .001 else 0.
        length_remaining = sum(math.dist(a, b) for a, b in zip(local, local[1:]))
        approach = math.hypot(*local[-1]) / 2. if length_remaining < 2. else 1.
        velocity = min(desired_speed, max(.2, desired_speed * approach)) * (1 if cx >= 0 else -1)
        angular = velocity * curvature
        limit = min(parameters['max_angular'], abs(velocity) / radius_limit)
        angular = clamp(angular, -limit, limit)
        if corrected:
            steering = steering_from_angular(angular, velocity, wheelbase, steer_angle)
        else:
            steering = clamp(math.atan(wheelbase * angular / math.copysign(max(abs(velocity), .5), velocity)) / steer_angle, -1., 1.)
        delta = clamp(steering, -steer_limit, steer_limit) * steer_angle
        yaw_rate = velocity * math.tan(delta) / wheelbase
        next_heading = heading + yaw_rate * dt
        if abs(yaw_rate) < 1e-9:
            x += velocity * dt * math.cos(heading)
            y += velocity * dt * math.sin(heading)
        else:
            x += velocity / yaw_rate * (math.sin(next_heading) - math.sin(heading))
            y += velocity / yaw_rate * (math.cos(heading) - math.cos(next_heading))
        heading = wrap(next_heading)
        speed = velocity
        error = distance_to_route((x, y), points[first:last + 1])
        max_error = max(max_error, error)
        if tick % 5 == 0:
            samples.append([tick * dt, x, y, heading, section, velocity, error])
        if error > tracking_error_limit:
            failure_reason = 'tracking_deviation'
            break
        if error > 5.:
            failure_reason = 'model_diverged'
            break
    return dict(split=split, corrected_steering=corrected, desired_speed=desired_speed,
                actuator_radius=radius_limit, planning_radius_cap_baseline=actuator_radius is not None,
                cusp_position_tolerance=cusp_tolerance,
                initial_lateral_offset=lateral_offset, completed=completed,
                elapsed=tick * dt, max_cross_track_error=max_error,
                tracking_error_limit=tracking_error_limit, failure_reason=failure_reason,
                cusp_skips=skipped, transitions=transitions, samples=samples,
                final_position=[x, y], final_heading=heading)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT / 'test_artifacts/server_validation_20260930/full_town_attempt6.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'test_artifacts/offline_navigation/kinematic_replay')
    args = parser.parse_args()
    saved = json.loads(args.input.read_text(encoding='utf8'))
    event = max((e for e in saved['events'] if e['kind'] == 'planned_path' and e['elapsed'] < 38.6),
                key=lambda e: e['elapsed'])
    points = event['points']
    sections = motion_sections(points)
    headings = inferred_headings(points, sections)
    import yaml
    config = yaml.safe_load((ROOT / 'nav2_adapter/config/nav2_official_control.yaml').read_text(encoding='utf8'))
    cusp_tolerance = config['controller_server']['ros__parameters']['maneuver_goal_checker']['xy_goal_tolerance']
    tracking_error_limit = tracking_error_limit_from_source()
    runs = []
    parameters = model_parameters()
    for speed in (.2, 1.2):
        for split, corrected in ((False, False), (False, True), (True, False), (True, True)):
            runs.append(simulate(points, headings, split, corrected, desired_speed=speed,
                                 cusp_tolerance=cusp_tolerance,
                                 tracking_error_limit=tracking_error_limit,
                                 parameters=parameters))
        runs.append(simulate(points, headings, True, True, desired_speed=speed,
            cusp_tolerance=cusp_tolerance, tracking_error_limit=tracking_error_limit,
            parameters=parameters,
            actuator_radius=parameters['planner_radius']))
    report = dict(scope=__doc__, model_parameters=parameters, assumptions=['ideal bicycle model using current configured wheel geometry; not wheel dynamics',
        'headings inferred from saved positions, initial direction reverse',
        'free space; no costmaps, collision prediction, ROS/DDS, sensor delay, steering lag or speed loop',
        'simplified Humble RPP geometry; not the actual controller plugin',
        'instant stop and shift at confirmed section boundaries'],
        recorded_path_at=event['elapsed'], input=str(args.input.resolve()),
        acceptance=dict(cusp_position_m=cusp_tolerance, cusp_heading_rad=.35,
                        tracking_error_m=tracking_error_limit,
                        endpoint_position_m=1.0, endpoint_heading_rad=.7),
        path_points=points, inferred_headings=headings, runs=runs)
    modified = [r for r in runs if r['split'] and r['corrected_steering']
                and not r['planning_radius_cap_baseline']]
    report['modified_model_cases_passed'] = all(
        r['completed'] and not r['cusp_skips']
        and r['max_cross_track_error'] <= tracking_error_limit for r in modified)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    labels = {(False, False): 'Full path / old steering', (False, True): 'Full path / corrected steering',
              (True, False): 'Split path / old steering', (True, True): 'Split path / corrected steering'}
    lines = ['# 离线车辆运动学回放', '',
             '输入为真实 Town 规划路径的 36 个位置点；车辆和控制器是简化模型，不是 BeamNG 或 ROS 插件。', '',
             '| 速度命令 m/s | 路径/转向 | 模型内完成 | 未满足换向容差越过点数 | 最大横向偏差 m |',
             '|---|---|---|---|---|']
    for run in runs:
        label = labels[(run['split'], run['corrected_steering'])]
        if run['planning_radius_cap_baseline']:
            label += ' / planner-radius cap baseline'
        lines.append(f"| {run['desired_speed']} | {label} | {run['completed']} | {len(run['cusp_skips'])} | {run['max_cross_track_error']:.3f} |")
    lines += ['', '## 结果范围', '',
              f"- 模型回归采用运行时阈值：换向点≤{cusp_tolerance}米/0.35弧度、路径跟踪偏差≤{tracking_error_limit}米；修正后的两个分段案例均完成且未越过不满足换向容差的点：{report['modified_model_cases_passed']}。",
              f'- 完成标准沿用配置：换向点位置误差≤{cusp_tolerance}米、朝向误差≤0.35弧度；终点位置误差≤1米、朝向误差≤0.7弧度。完成不代表精确到达或真实车辆成功。',
              '- 初始朝向和各点朝向由圆弧位置推导，采用理想单轨车辆、瞬时转向和换挡。',
              '- 自由空间，没有障碍、传感器延迟、真实轮胎/转向响应或速度环。这里只比较几何跟踪。',
              '- 模型内仍有横向偏差：规划的足迹碰撞检查不能保证实际跟踪时足迹始终处于规划范围内。',
              '- 原始失败点云缺失，无法离线判断车前占用栅格是真实障碍还是误报。']
    args.output.with_suffix('.md').write_text('\n'.join(lines) + '\n', encoding='utf8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin = points[0]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    for ax, desired in zip(axes, (.2, 1.2)):
        ax.plot([p[0] - origin[0] for p in points], [p[1] - origin[1] for p in points], 'k-', linewidth=2., label='Saved planned path')
        for run in runs:
            if run['desired_speed'] != desired:
                continue
            ax.plot([p[1] - origin[0] for p in run['samples']], [p[2] - origin[1] for p in run['samples']],
                    '--' if not run['corrected_steering'] else '-', alpha=.85,
                    label=labels[(run['split'], run['corrected_steering'])])
        ax.set_title(f'Command speed {desired} m/s')
        ax.set_xlabel('Relative world X (m)')
        ax.set_ylabel('Relative world Y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle('Offline ideal bicycle-model replay — not BeamNG validation')
    fig.tight_layout()
    fig.savefig(args.output.with_suffix('.png'), dpi=150)
    plt.close(fig)
    for run in runs:
        print(json.dumps({k: v for k, v in run.items() if k != 'samples'}, ensure_ascii=False))
    return 0 if report['modified_model_cases_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
