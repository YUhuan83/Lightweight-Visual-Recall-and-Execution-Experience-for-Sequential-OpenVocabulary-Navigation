from pathlib import Path
import collections,json,hashlib
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/confirmed_graph_handoff_20261027';field=root/'field'
r=json.loads((field/'field_result.json').read_text());arrival=json.loads((field/'closed_arrival_audit.json').read_text())
e=next((field/'closed_core').glob('sessions/*/goal_001/events.jsonl'));events=[json.loads(l) for l in e.read_text().splitlines()]
states=collections.Counter(x.get('status') for x in events if x.get('status'))
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
summary=dict(candidate='v54_confirmed_graph_handoff',field_passed=False,promoted=False,commit=None,
 component_tests=dict(related_tests_per_run=298,related_runs=2,offline_tests=828,real_ros_tests_per_run=875,real_ros_runs=2,cpp_preserved_tests_per_run=32,cpp_runs=2),
 field=r['result'],fluency=r['fluency'],arrival={k:arrival[k] for k in ('masks','valid_localizations','confirmed','min_body_gap_m')},statuses=dict(states),
 independent_metrics_equal=(field/'displacement_audit_1.json').read_bytes()==(field/'displacement_audit_2.json').read_bytes())
(root/'validation_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
text=f'''
## v54现场关闭：目标接近仍未完整通过

trial181924启动{r['result']['startup_elapsed_sec']:.3f}/搜索{r['result']['search_elapsed_sec']:.3f}/收尾{r['result']['shutdown_elapsed_sec']:.3f}秒。不是完整180秒搜寻；150.663秒goal_result以session_exit失败，原因是Native actuator unhealthy超过.4秒，车辆Lua报Vehicle control lease expired or closing。console1，完整到达false，所有项目进程与X148锁已清空。没有把控制租约异常归因到本次目标代码，也没有放宽/重置过期租约；确切喂租间隙或closing来源仍待查。

47mask/37有效LiDAR/14程序确认，确认帧最小车身距50.354m。Native149.590秒（含边界/非精确query裁切）近停27.585%/最长9.995秒、净81.757/累计106.124m；独立轨迹两遍一致。数据不能证明完整到达或无回退。本轮object_standoff_approach={states['object_standoff_approach']}，target_instance_released={states['target_instance_released']}，confirmed_target_partial_route等待分支触发={sum(x.get('reason')=='confirmed_target_partial_route' for x in events)}；未触发的分支仍只有组件验证，不伪称现场已覆盖。

远目标进入已有GraphNav的改动已实际触发，但此前planning/执行失败仍存在，不能把本轮所有停顿推给最后的Native异常。本候选不推广、不commit，主703/原ZIP保留。后续需要继续验证目标接近路径的可执行性与持续推进，并分别复核Native租约喂租/关闭来源，再完成180秒整轮；不通过延长倒计时、降低确认/到达门或关闭碰撞冒充修复。

证据test_artifacts/confirmed_graph_handoff_20261027/field与validation_summary.json。上述图像问题的直接解释仍是v52释放目标后恢复普通探索，不改写为识别目标没出现。
'''
(repo/'docs/CONFIRMED_GRAPH_HANDOFF_20261027.md').open('a',encoding='utf8').write(text)
(repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8').write('''

### 2026-10-27 v54关闭未通过（最新）

- trial181924启动50.983/search150.958/收尾20.837秒，不是完整180秒。150.663秒Native actuator unhealthy>.4s，车辆Lua报Vehicle control lease expired or closing，goal session_exit失败/console1；所有项目进程与X148锁空。准确feed间隙或closing来源未查明，不能直接归因本次目标逻辑，也未放宽/自动重置过期租约。
- 47mask/37LiDAR/14程序确认，确认帧最小车身50.354m。Native149.590秒近停27.585%/最长9.995、净81.757/累计106.124m，独立轨迹两遍一致。目标接近仍未通过，之前planning/执行失败不能全归于末尾Native异常。875真实ROS两遍、298相关两遍/828离线/32C++两遍是组件范围。主703/ZIP保留，未推广commit。
- 新机制保持远确认目标、复用GraphNav并消除远目标裸Pose双入口；partial前缀结束不进入最终观察释放。实际触发范围及未覆盖分支数见confirmed_graph_handoff_20261027/validation_summary.json。后续继续排查接近路径可执行性与推进，同时独立核对租约feed/closing后再180秒完整验收。详docs/CONFIRMED_GRAPH_HANDOFF_20261027.md末节。
''')
print(json.dumps(dict(statuses={k:states[k] for k in ('object_standoff_approach','target_instance_released','follow_failed')},partial_waits=sum(x.get('reason')=='confirmed_target_partial_route' for x in events),closed=r['result']['closed'],promoted=False)))
