"""Reproduce path-pruning risk from a saved Town test, without ROS or motion."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav2_adapter'))
from wildos_nav2_adapter.drive_math import steering_from_angular
from wildos_nav2_adapter.graph_route import distance_to_route
from wildos_nav2_adapter.motion_path import motion_sections

SOURCE = ('https://github.com/ros-navigation/navigation2/blob/humble/'
          'nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp')


def rpp_nearest_index(points, position, search_distance, begin=0):
    """Match Humble's first_after_integrated_distance + exclusive min_by."""
    total = 0.
    end = len(points)
    for index in range(begin, len(points) - 1):
        total += math.dist(points[index], points[index + 1])
        if total > search_distance:
            end = index + 1
            break
    return min(range(begin, end), key=lambda i: math.dist(points[i], position)), end


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT / 'test_artifacts/'
                        'server_validation_20260930/full_town_attempt6.json')
    parser.add_argument('--sample-time', type=float, default=38.6167)
    parser.add_argument('--output', type=Path, default=ROOT / 'test_artifacts/'
                        'server_validation_20260930/follow_path_diagnosis')
    args = parser.parse_args()
    saved = json.loads(args.input.read_text(encoding='utf8'))
    samples = [e for e in saved['events'] if e['kind'] == 'search_progress']
    sample = min(samples, key=lambda e: abs(e['elapsed'] - args.sample_time))
    path_event = max((e for e in saved['events'] if e['kind'] == 'planned_path'
                      and e['elapsed'] <= sample['elapsed']), key=lambda e: e['elapsed'])
    points, position = path_event['points'], sample['position']
    sections = motion_sections(points)
    if len(sections) < 2:
        raise ValueError('Selected path has no forward/reverse cusp')
    cusp = sections[0][1]
    closest, end = rpp_nearest_index(points, position, 10.)
    prefix_closest, _ = rpp_nearest_index(points[:cusp + 1], position, 10.)
    later = max(samples, key=lambda e: e['elapsed'])
    later_path = max((e for e in saved['events'] if e['kind'] == 'planned_path'
                      and e['elapsed'] <= later['elapsed']), key=lambda e: e['elapsed'])
    statuses = [e for e in saved['events'] if e['kind'] == 'plan_status'
                and e.get('status') in ('follow_failed', 'start_footprint_blocked', 'memory_unreachable')]
    speed, curvature, wheelbase, max_angle = .2, 1./8., 3., .6
    angular = speed * curvature
    old_delta = math.atan(wheelbase * angular / .5)
    new_normalized = steering_from_angular(angular, speed, wheelbase, max_angle)
    data = dict(
        scope='offline reproduction; historical runtime pruning not recorded',
        input_file=str(args.input.resolve()),
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        nav2_source=SOURCE, indices_zero_based=True,
        planned_at=path_event['elapsed'], sampled_at=sample['elapsed'],
        vehicle_position=position, path_poses=len(points), path_length=path_event['length'],
        motion_sections=sections, cusp_index=cusp, cusp_position=points[cusp],
        search_distance=10., search_end_exclusive=end,
        original_nearest_index=closest, original_nearest_distance=math.dist(points[closest], position),
        section_nearest_index=prefix_closest,
        section_nearest_distance=math.dist(points[prefix_closest], position),
        cusp_skipped_by_offline_nearest=closest > cusp,
        prior_prune_sensitivity=[dict(begin=i, nearest=rpp_nearest_index(points, position, 10., i)[0])
                                for i in range(cusp + 1)],
        later_sample_at=later['elapsed'],
        later_path_deviation=distance_to_route(later['position'], later_path['points']),
        low_speed_example=dict(command_speed=speed, command_yaw_rate=angular,
            requested_radius=1./curvature, previous_steering_angle=old_delta,
            previous_modeled_radius=wheelbase / math.tan(old_delta),
            corrected_steering_angle=new_normalized * max_angle,
            corrected_modeled_radius=wheelbase / math.tan(new_normalized * max_angle)),
        historical_failures=statuses,
        missing_evidence=['native_samples.jsonl and controller log remain on server',
                          'historical received_global_plan and lookahead_point were not bagged',
                          'historical bag omitted raw LiDAR, local costmap and raw costmaps; new capture is required'],
        deployment_status='local code only; ROS and BeamNG field retest pending')
    offline_report = ROOT / 'test_artifacts/offline_navigation/report.json'
    if offline_report.is_file():
        checks = json.loads(offline_report.read_text(encoding='utf8'))['checks']
        data['offline_checks'] = dict(
            unit_tests=sum(c.get('detail', {}).get('tests', 0) for c in checks
                           if c['status'] == 'PASS' and isinstance(c.get('detail'), dict)),
            failed_checks=[c['name'] for c in checks if c['status'] == 'FAIL'],
            callback_tests=next((c['detail']['tests'] for c in checks if c['name'] == 'offline_callback_contracts'), 0),
            syntax_files=next((c['detail']['files'] for c in checks if c['name'] == 'python_syntax'), 0),
            ros_action_tests_run=False, field_retest_run=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf8')
    text = f'''# Town FollowPath 检查报告（2026-09-30）

## 结论和证据边界

已复现两个代码层面的问题，并修改本地代码。历史整轮测试仍然失败；本次没有重新驾驶车辆。
服务器 SSH 连接超时，本地没有 ROS 环境，分段 action 的 ROS 回归和现场效果均待验证。

## 1. 最近点剪枝可能跨过换向点

输入为 `full_town_attempt6.json`，采用 {path_event['elapsed']:.3f} 秒的真实规划路径，
以及 {sample['elapsed']:.3f} 秒的真实车辆位置 {position}。路径包含 {len(points)} 个点，
全长 {path_event['length']:.3f} 米；第 {cusp} 点是换向点（索引从 0 开始）。

按 [Nav2 Humble RPP 源码]({SOURCE}) 的最近点搜索复现，10 米搜索范围会选中
换向后的第 {closest} 点，距离车辆 {data['original_nearest_distance']:.3f} 米；
限定在第一段时选中第 {prefix_closest} 点，距离 {data['section_nearest_distance']:.3f} 米。
最近点剪枝发生在换向检测之前，因而后续检测可能看不到已被删除的换向点。
对已经剪去 0 至 {cusp} 个前缀点的情况分别复现，结果详见 JSON。

![保存路径的换向点与最近点复现]({args.output.with_suffix('.png').resolve().as_posix()})

这是实际记录上可复现的控制器兼容性风险。没有历史控制器内部路径和前视点记录，
因此不能据此断言当时确实发生了提前换挡，也不能断言它是唯一故障原因。

修改：完整路径继续经过足迹、曲率和图路线检查，并用于记忆候选成本比较；
执行时拆成 {sections}，每次 FollowPath 只收到当前同向路径。
换向点使用 0.1 米、0.35 弧度的独立 goal checker；桥接器还要求新鲜里程计，
位置误差不超过 0.2 米、朝向误差不超过 0.4 弧度，随后重新检查下一段的当前 costmap。
切换目标、关闭执行和传感器超时均丢弃剩余旧路径。

## 2. 低速时转向换算改变路线曲率

旧公式用 `max(abs(v), 0.5)` 作分母，但 RPP 的角速度是 `w = v * curvature`。
例如目标速度 0.2 m/s、半径 8 米，旧代码把模型转弯半径变成
{data['low_speed_example']['previous_modeled_radius']:.1f} 米。
新代码使用同一条命令的有符号 `w / v`，模型半径保持
{data['low_speed_example']['corrected_modeled_radius']:.1f} 米；零速命令仍停车。
本次修复的是转向几何，未调整速度环、目标速度或油门参数。
这是数学与离线测试确认的问题；实际车轮转角标定仍需测量。

## 3. 前部占用栅格还不能判定为误报

{later['elapsed']:.3f} 秒车辆偏离当时规划折线 {data['later_path_deviation']:.3f} 米；
49.921 秒 FollowPath 返回 aborted；50.476 秒开始报告车体前方 1.93–2.05 米的足迹占用。
换为加油站目标后，接受的记忆位置正确，但同一起始足迹仍被占用，无法发出新路径。
目前不能区分真实障碍、残留栅格或点云处理问题，不能通过清图、缩小足迹或关闭碰撞检查下结论。

原始 native telemetry、控制器日志和 rosbag 在服务器
`/home/ly/full_town_memory_20260930/attempt6/`。
历史 bag 录制了 global OccupancyGrid，但没有录制原始点云、local costmap 或原始代价格；
这些缺失证据需要新一轮采集，连接恢复本身不能补回未录制的数据。
本地其它轮次的 costmap 快照来自不同车辆位置，不是这次失败位置的证据。
本地已增加 native direction/up/steering_input 日志，并扩展下一轮 bag 的控制器前视点、
剪枝后路径、碰撞弧、原始与过滤点云、local/global costmap 采集。

## 验证范围

离线结果见同目录旁的 `../offline_navigation/report.md`。
本次离线测试 {data.get('offline_checks', {}).get('unit_tests', '待运行')} 项通过，其中包括 {data.get('offline_checks', {}).get('callback_tests', 0)} 项 ROS 接口替身回调测试。
{data.get('offline_checks', {}).get('syntax_files', '待运行')} 个 Python 文件语法检查通过，部署包与源文件一致。
低速前进/倒车等曲率、换向分段/到达判定及实际回调逻辑已在本地测试。真实 ROS/DDS action 尚未运行。
重新连接后需部署桥接器、转向适配器与新增 goal checker 配置并重启控制器，
先验证一次实际倒车→前进换向，再完成同进程加油站→垃圾桶→加油站测试。
不能把历史 `trash_found=true` 当成已到达垃圾桶；完整测试仍需返回并重新看到加油站。
'''
    args.output.with_suffix('.md').write_text(text, encoding='utf8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 7))
    origin = points[0]
    for section_index, (first, last) in enumerate(sections):
        segment = points[first:last + 1]
        ax.plot([p[0] - origin[0] for p in segment], [p[1] - origin[1] for p in segment],
                '-o', markersize=3, label=f'Motion section {section_index + 1}')
    ax.scatter(position[0] - origin[0], position[1] - origin[1], marker='x', s=90,
               color='black', label=f'Actual vehicle at {sample["elapsed"]:.2f} s')
    for i, label in ((cusp, 'Cusp'), (closest, 'Nearest with full path'),
                     (prefix_closest, 'Nearest with first section')):
        ax.annotate(f'{label} [{i}]', (points[i][0] - origin[0], points[i][1] - origin[1]),
                    xytext=(12, 12 if i != closest else 28), textcoords='offset points',
                    arrowprops={'arrowstyle': '->'}, fontsize=9)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('World X relative to planned start (m)')
    ax.set_ylabel('World Y relative to planned start (m)')
    ax.set_title('Saved Town path: nearest-point pruning can skip a cusp\nOffline reproduction; runtime pruning not recorded')
    ax.grid(alpha=.25)
    ax.legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(args.output.with_suffix('.png'), dpi=160)
    plt.close(fig)
    print(json.dumps({k: data[k] for k in ('cusp_index', 'original_nearest_index',
        'original_nearest_distance', 'section_nearest_index', 'section_nearest_distance',
        'later_path_deviation', 'low_speed_example')}, indent=2))


if __name__ == '__main__':
    main()
