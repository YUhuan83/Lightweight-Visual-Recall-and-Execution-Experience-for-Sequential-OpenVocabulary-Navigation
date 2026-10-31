from pathlib import Path
import json,hashlib,shutil
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/range_candidate_20261027';aux=Path(__file__).parent
for name in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py'):
 p=aux/name;s=p.read_text(encoding='utf8');old="'v47_depth_geometry')";new="'v47_depth_geometry','v48_range_candidate')"
 if old in s:s=s.replace(old,new);p.write_text(s,encoding='utf8')
 elif "'v48_range_candidate'" not in s:raise AssertionError(name)
for name in ('probe_static_lidar_coverage.py','probe_static_lidar_coverage.sh','replay_range_candidate.py','replay_range_candidate.sh','implement_range_candidate.py','audit_range_delivery.py'):
 shutil.copyfile(aux/name,root/'diagnostics'/name)
summary='''# 新鲜测距参与靠近观察（2026-10-27）

主源码/既有v47保留；隔离range_candidate_20261027/candidate（服务器v48_range_candidate）。不将本候选称为整体三维定位或目标到达修复完成。

## 已确认与排除

- v47关闭现场6条有效3D未构成4帧时序确认；确认门符合既有约定，不能降低门槛或把约21m的两个位置合并。
- 固定记录视点静态LiDAR对照：6视点×16/32/64线×两次，共36采样。通过原30k上限/40m范围/原深度方法，16/32/64分别6/2/3次定位（各12次），增加线数并未稳定改善且有原有效帧丢失。未改变运行rig、量程、密度、地图。静态传感器使用校准射线原点，地图只加载一次、没有导航；不是原始异步现场逐字复现。
- GoalStandoff.on_localization在不足4支持时直接返回，已通过几何质量门的当前测距没有参与弱目标观察选点。原publish_visible_step只传bearing给vehicle_observation_pose。两遍新7项反例均2fail/0error，证明该接口缺口；不是已证明本缺口独自造成全部超时。

## 两个运行文件的修改

GoalStandoff仅在无待确认目标、最新合格测距与当前mask完全同stamp、方位一致且仍通过既有2秒mask门时，将测距交给原vehicle_observation_pose。沿用校准相机最小观察距离/12m提案转弯弧、当前测量地图和车体检查；当前测距只约束新观察点，不缓存目标成功或放宽最终4帧/2米到达。新的mask不继承旧距离，错误query/歧义/重复/过期消息不能增加确认。

VisibleTargetApproach在原通道接收range_candidate（完整有限三维点、lidar/multiview来源、显式未确认），沿用原GraphNav→Smac→整路FollowPath和控制器权限。没有新增直接控制通道、强行中断动作或禁用碰撞检查。状态消息如实区分提供了距离与仍需视觉确认。

## 验证范围和待验收

61相关两遍、原828离线通过。首次真实ROS批量进程在旧ContinuousRetryAdmission用例附近段错误，未见断言错误；原日志保存real_ros_review_initial_segfault.log。随后v47同套826通过，候选833两遍、保留实际C++32两遍/console通过。首次段错误未复现，不能断言原因已查明或把它写成已修复的运行崩溃；本次未为它修改运行代码。

原v47关闭录包按记录顺序和真实地图/车姿回放：旧124条bearing_only，新123条bearing_only+1条range_candidate，仍0确认；两遍报告相同。该观察点(737.229,-71.407)、自身导航yaw -1.184，来自当前测距点(738.365,-79.533,55.051)，未拿程序确认充当语义真值。回放保留原车辆后续轨迹，无真实Action/GraphNav失败反馈反事实，不等于整车验证。

实际query后180秒现场待验收。此前已修的控制器整路/退役边界/接近反馈保留，未推广/commit；静态对照和候选证据test_artifacts/range_candidate_20261027。
'''
(repo/'docs/RANGE_CANDIDATE_20261027.md').write_text(summary,encoding='utf8')
with (repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8') as f:f.write('''
### 2026-10-27 继续：v48测距到靠近观察候选（现场待验收）

- 用户暂停时服务器项目进程/X148锁空；恢复后重查仍空，v48仅本机未部署状态起步。没有将暂停期间候选当作已通过版本。
- 固定6记录视点×16/32/64线×两遍静态对照，原几何下各12采样定位6/2/3次；增加线数会丢原有效位置，未修改运行rig或地图。服务器证据depth_coverage_probe_20261027，本地range_candidate_20261027/diagnostics/static_coverage_report.json。
- 新确定接口缺口：当前已合格3D在未满4支持时不参与观察选点，仍只按bearing；新7项两遍2fail/0error。v48从v47隔离，只改GoalStandoff和VisibleTargetApproach：同一新鲜mask的合格测距约束原车体观察步，新mask不继承旧距离；range_candidate仍显式未确认，经原GraphNav/Smac/整路控制，不改4帧/2米门、不禁用碰撞/物理。
- 61相关两遍/原828离线通过。真实ROS首次批量进程在旧retry用例附近segfault（未断言失败），原日志保留，原因未定；随后v47原826通过、v48候选833两遍及32C++两遍/console通过，未为未复现段错误改运行代码。不能写成首次回归一次全过或崩溃根因已查明。
- 同v47关闭录包真实Producer和地图/姿态两遍：旧124 bearing_only→新123 bearing_only+1 range_candidate，0确认不变；只证明当前范围进入实际观察点计算，不等于完整动作/语义/到达成功。现场待补，未推广/commit，主703和v47保留。详docs/RANGE_CANDIDATE_20261027.md。
''')
print('recorded pending field; no promotion or commit')
