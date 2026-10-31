"""Summarize closed trial receipts without touching bags or production code."""
import json
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024')
before=json.loads((root/'initial_ab_results.json').read_text())
after=json.loads((root/'r2_results.json').read_text())
assert len(after)==2
records=[]
for record in [r for r in before if r['version']=='v23_arc_heading']+after:
    assert record['result']['closed'] and not record['postcheck']['processes']
    assert not record['postcheck']['display148_lock']
    if record['version'].endswith('_r2'):
        assert 'continuous_exploration_v1' in record['observed_execution_modes']
    f=record['fluency'];a=record['attribution'];result=record['result']
    records.append(dict(version=record['version'],trial=Path(record['trial']).name,
        near_stop_pct=f['near_stop_pct'],longest_near_stop_s=f['longest_near_stop_s'],
        net_displacement_m=f['net_displacement_m'],sampled_distance_m=f['sampled_distance_m'],
        gear_change_near_stop_s=a['by_control_s'].get('gear_change',0),
        stale_command_near_stop_s=a['by_control_s'].get('stale_command',0),
        execution_hold_near_stop_s=a['by_control_s'].get('execution_hold',0),
        startup_s=result['startup_elapsed_sec'],search_s=result['search_elapsed_sec'],
        shutdown_s=result['shutdown_elapsed_sec'],complete_navigation_passed=result['complete_navigation_passed']))
payload=dict(records=records,scope='Two old rounds and two isolated continuous rounds. Native intervals <=.5s, speed <.1m/s; no videos. Different routes and other GPU jobs prevent a strict causal/stability claim.',
    source_preservation=json.loads((root/'preservation_verified.json').read_text()),
    complete_workflow_passed=all(r['complete_navigation_passed'] for r in records[2:]))
(root/'comparison.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False),encoding='utf8')
lines=['# 持续探索隔离试验结果','',
    '| 版本/轮次 | 近停占比 | 最长近停 | 净位移 | 采样路程 | 换向近停 | 无新鲜命令近停 | 完整到达 |',
    '|---|---:|---:|---:|---:|---:|---:|---|']
for r in records:
    label=('旧版' if r['version']=='v23_arc_heading' else '持续探索')+' '+r['trial'].split('_')[-1]
    lines.append(f"| {label} | {r['near_stop_pct']:.2f}% | {r['longest_near_stop_s']:.2f}s | {r['net_displacement_m']:.2f}m | {r['sampled_distance_m']:.2f}m | {r['gear_change_near_stop_s']:.2f}s | {r['stale_command_near_stop_s']:.2f}s | {'通过' if r['complete_navigation_passed'] else '未通过'} |")
lines+=['','搜索180秒，启动和收尾另计。所有轮次已关闭并核实无项目进程或X148锁；新版本标记已核实。',
    '仅两轮，不足以证明长期稳定性；路线和其他GPU负载未完全控制。未达到历史最好21.17%或建议15%近停目标；不推广、不commit。',
    '原主目录运行源码保留，隔离候选测试；初始新版本两次预检失败不混入行驶统计。']
(root/'comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print(json.dumps(records,indent=2))
