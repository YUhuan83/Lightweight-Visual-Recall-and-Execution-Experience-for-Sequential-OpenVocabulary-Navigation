from pathlib import Path
import json,hashlib,math
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/range_candidate_20261027';field=root/'field'
r=json.loads((field/'field_result.json').read_text(encoding='utf8'));phase=json.loads((field/'field_phase_audit.json').read_text(encoding='utf8'));hint=json.loads((field/'range_hint_audit.json').read_text(encoding='utf8'))
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock'];assert (field/'displacement_audit_1.json').read_bytes()==(field/'displacement_audit_2.json').read_bytes()
assert phase['masks']==80 and phase['valid_localizations']==1 and phase['confirmed']==0 and hint['accepted_range_statuses']==0
v=json.loads((field/'retirement_boundary_audit.json').read_text(encoding='utf8'));assert v['retirement_messages']==20 and v['retained_old_future_branch_violations']==0
summary=dict(version='v48_range_candidate',changed_runtime_files=2,added_tests=7,related_tests_twice=61,offline=828,real_ros_final_twice=833,cpp_twice=32,initial_real_ros_segfault_unresolved=True,field_passed=False,field_new_branch_triggered=False,
 masks=80,lidar=0,multiview=1,confirmations=0,near_stop_pct=r['fluency']['near_stop_pct'],longest_stop_s=r['fluency']['longest_near_stop_s'],net_m=r['fluency']['net_displacement_m'],promoted=False,commit_created=False,
 scope='Current range can guide an unconfirmed observation in callback/map replay, but this field lacked eligible current ranges; full task and fluency not achieved. Static range probe pending.')
(root/'validation_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
p=repo/'docs/RANGE_CANDIDATE_20261027.md'
with p.open('a',encoding='utf8') as f:f.write('''
## v48现场关闭：未通过，不推广

trial_20261027_162318启动41.334、搜索180.012、收尾21.825秒，timeout（driver外层Field readiness/result deadline）、session errors=[]、console0、closed=true，项目进程/X148锁空。80mask，0LiDAR/1multiview，0确认；87条临时提示全部bearing_only，新range_candidate及其接收状态均0。因此没有新分支整车效果证据，不能说接近已修好。

全Native179.523秒口径近停26.668%/最长6.148秒、净60.353/累计109.047m；较v47近停21.844%更差，但两轮路线不同且本轮新增测距分支未触发，不能单凭两个总值判断新增分支造成回退，也不能称效果无回退。65次planning/38段实际路径，17.084秒stale中12.423秒在planning，换挡4.627秒；这些是归属统计，不把标签当成唯一根因。

原RPP/Smac/模型SHA匹配。20条退休限制后旧方向后缀越界0，命令曲率/原地旋转违约0，实际控制器弧碰撞0。Native/events两次复算相同；完整语义、定位、靠近、到达和流畅性仍未通过，未推广/commit。

下一诊断限定为输入侧：固定同一视点40/80米LiDAR对照，暂不更改运行量程或地图。候选仅通过组件和旧录包范围；静态量程结果待补。
''')
with (repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8') as f:f.write('''
### 2026-10-27 v48现场关闭，继续定位输入侧（最新）

- trial162318已关闭：启动41.334/search180.012/shutdown21.825秒超时，session errors=[]、console0、项目进程/X148锁空。80mask/0LiDAR/1multiview/0确认，87条提示全部bearing_only，新range_candidate未触发，无整车修复成功证据。
- 近停26.668%/最长6.148秒、净60.353/累计109.047m，比v47近停更差；路线不同且新测距分支未触发，不宣称因果或无回退，v48不推广/commit。65planning/38路径，stale17.084中planning12.423、gear4.627；不是单凭状态归属认定根因。
- Native/events两遍一致，20退休限制后旧方向越界0，命令曲率/原地旋转违约0、控制器弧碰撞0，实际库和模型SHA不变。此前61相关两遍/828离线/833真实ROS两遍/32C++两遍通过，仅组件与旧录包测距交接有增量。
- 当前继续固定视点40/80m静态量程对照，查mask零回波来源，未更改运行rig/量程/地图，结果待补。证据range_candidate_20261027/field；详docs/RANGE_CANDIDATE_20261027.md。
''')
print(json.dumps(summary,indent=2))
