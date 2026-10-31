from pathlib import Path
import json,hashlib
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/localization_range_20261027';field=root/'field'
r=json.loads((field/'field_result.json').read_text(encoding='utf8'));p=json.loads((field/'field_phase_audit.json').read_text(encoding='utf8'));t=json.loads((field/'range_transport_audit.json').read_text(encoding='utf8'));a=json.loads((field/'closed_arrival_audit.json').read_text(encoding='utf8'));b=json.loads((field/'retirement_boundary_audit.json').read_text(encoding='utf8'));h=json.loads((field/'range_hint_audit.json').read_text(encoding='utf8'))
assert (field/'displacement_audit_1.json').read_bytes()==(field/'displacement_audit_2.json').read_bytes()
assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
assert p['valid_localizations']==98 and p['confirmed']==70
assert all(x['range_violations']==0 and x['max_points']<=30000 for x in t['streams'].values())
summary=dict(version='v49_localization_range',runtime_changes=5,related_twice=61,offline=828,real_ros_twice=840,cpp_twice=32,physical_rig_tests=4,field_masks=117,lidar_geometric_localizations=98,confirmation_messages=70,complete_task_passed=False,promoted=False,commit_created=False,near_stop_pct=r['fluency']['near_stop_pct'],longest_stop_s=r['fluency']['longest_near_stop_s'],net_m=r['fluency']['net_displacement_m'],min_body_gap_m=a['min_body_gap_m'],dual_stream_stamp_matches=t['matched_measurement_stamps'],near_range_violations=0,far_range_violations=0,range_hints=h['evidence_counts'],retirement_messages=b['retirement_messages'],retired_future_violations=b['retained_old_future_branch_violations'])
(root/'validation_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
text=f'''
## v49现场关闭：三维输入恢复，完整接近仍未通过

trial_20261027_164340：startup40.389/search180.195/shutdown25.633秒超时，session errors=[]、console0、项目进程/X148锁空。117mask/98有效LiDAR几何定位，70条程序确认/可用接近消息；这些不是70次独立成功，也不证明语义真值。确认帧最小车身间距52.780m，未到2m，完整目标搜寻未通过。

实际80m定位流1052份，最多30k点、最大半径79.99996m，其中2490451份回波样本在40m之外；近场云1060份、最大半径39.99999m、越界0。1051个共同测量戳完全匹配，同一原虚拟frame；录包近场独有9/定位独有1不能据此说每帧均收录。定位流接收年龄P50 .272/P95 .392/最大.479秒，近场P95 .364/最大.445，局部map更新最大间隔.391秒。未发现范围串入地图或新定位DDS明显超龄；这不能代替所有地形语义/障碍正确性验证。

全Native185.741秒口径近停37.305%/最长11.813秒、净64.320/累计92.982m。包含收尾采样，非精确180秒裁切；相较之前整体仍不合格，不宣称无回退，候选不推广/commit。实际两个RPP弧碰撞发生在前期；原控制库217609ca.../Smac89e3005a...、模型f661897f...不变，{b['retirement_messages']}条退休限制后未来旧方向越界{b['retained_old_future_branch_violations']}。Native/events两遍复算一致。

剩余故障已经进入确认后的接近执行。记录126.663s切到仅2.327m远的中间点(736.300,-73.588)，129.2s有新点(737.303,-74.307)却因accepted_route_active暂存，129.701s原动作follow_aborted，之后仍重试旧短点；137.379s旧点结束后等观察，142.812s释放实例。另172.55–180.23s已生成约(736–737,-82)的前方接续点，却一直等待旧(737.219,-74.692)动作，至182.95s才交接（超过搜索时限）。目标几何一直在约(747–750,-130)，不是已经到达房屋。

源码复核：ConfirmedTargetApproach.route_confirmed_target对latched且confirmed_route_busy统一暂存直到动作结束；连续弱目标有advance_visible_goal，已确认目标没有相同的移动接续入口。这个策略能保护当前动作，但在短中间点上会串行停顿，需进一步用真实当前控制器路径/车姿/地图验证可保持的运动窗口，再设计连续交接；不能简单删掉busy保护，否则可能打断恢复、旧动作或未接受的规划。当前只确认这段串行交接实证，不把它写成所有停顿或两次FollowPath失败的唯一根因。未对该后续缺口追加未经回归的运行修改。

用户质疑整体修复效果后已明确承认仍未修好；三维几何恢复与完整执行成功分别记载。全部证据localization_range_20261027/field、validation_summary.json，保留v47/v48和主源码。当前服务器已清场，无录像。
'''
with (repo/'docs/LOCALIZATION_RANGE_20261027.md').open('a',encoding='utf8') as f:f.write(text)
with (repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8') as f:f.write(f'''
### 2026-10-27 v49关闭：定位恢复，确认后短点串行执行未解决（最新）

- trial164340启动40.389/search180.195/shutdown25.633秒超时，session errors=[]、console0、项目进程/X148锁空。117mask/98LiDAR几何定位/70程序确认和接近消息，最小确认帧车身间距52.780m，未到2m。不是70个目标成功，语义真值/完整流程仍未验收。
- 80m定位云1052份/近场云1060份，均≤30k，最大半径79.99996/39.99999m，越界0；1051共同测量戳一致，录包近场独有9/定位独有1不称全收录。定位接收P95 .392/最大.479秒，局部map最大更新间隔.391秒，新范围未串入地图；不能据此认证全部地形分类。
- 全Native185.741秒近停37.305%/最长11.813秒、净64.320/累计92.982m，含收尾非精确180裁切；整体不合格，不推广/commit。Native/events两遍一致，{b['retirement_messages']}退休限制旧后缀越界{b['retained_old_future_branch_violations']}，原RPP/Smac/模型保持。
- 接近后续实证：126.663s切换2.327m中间点，129.2s新点已存在因accepted_route_active延迟；129.701s follow_aborted后仍重试旧点，137.379s结束等观察，142.812s释放。172.55s生成约(736–737,-82)接续点却持续等旧(737.219,-74.692)动作，182.95s才交接，超过搜索时限。目标位置仍约(747–750,-130)。不是3D一直无效，也不能把所有停顿一概归于此门。
- 源码confirmed_route_busy统一等动作完；弱目标有连续advance，confirmed接近缺移动接续入口。下步须按真实控制器当前路径/几何可行窗口设计连续接续，保留异步、恢复和旧动作权限保护；不能直接删busy门。本轮未对此追加未验证改动。详docs/LOCALIZATION_RANGE_20261027.md关闭节，证据localization_range_20261027/field。
''')
print(json.dumps(summary,indent=2))
