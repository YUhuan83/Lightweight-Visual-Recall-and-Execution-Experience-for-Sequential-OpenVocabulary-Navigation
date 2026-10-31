from pathlib import Path
import json,shutil
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/localization_range_20261027';d=root/'diagnostics';d.mkdir()
shutil.copyfile(repo/'test_artifacts/range_candidate_20261027/diagnostics/static_range_report.json',d/'static_range_report.json')
r=json.loads((d/'static_range_report.json').read_text(encoding='utf8'));base=[x for x in r['rows'] if x['lines']==16 and x['max_range_m']==40];wide=[x for x in r['rows'] if x['lines']==16 and x['max_range_m']==80]
assert len(base)==len(wide)==12 and not any(x['located'] for x in base) and all(x['located'] for x in wide)
report=json.loads((root/'candidate/test_artifacts/offline_navigation/callback_report.json').read_text(encoding='utf8'));assert report['tests']==828 and report['passed']
for n in ('probe_static_lidar_range.py','probe_static_lidar_range.sh','prepare_range_probe.py'):shutil.copyfile(Path(__file__).parent/n,d/n)
s='''# 定位量程与局部地图范围解耦（2026-10-27）

本轮v48现场仍80mask/0LiDAR/1multiview/0确认，新增测距辅助分支未触发。继续沿输入侧排查，保留旧版，隔离localization_range_20261027/candidate（服务器v49_localization_range），不更改模型/确认门/到达条件或连续控制。

## 根因证据：不能把建图半径等同于定位有效范围

固定6个记录相机视点和原mask、同16线/60度/density30/相同定位算法，两次40m采样全部没有合格定位；只将硬件扫描量程改80m，两次共12/12取得22–36支持点的有效定位，位置集中在(747–749,-130)房屋区域。原40m多数0–4回波。此前只增加线数的36采样对照出现回退，因此没有改线数、密度、相机或mount。此为静态原视点几何证据，不是目标语义真值或完整异步/车辆验收。

原数据链同时存在两层40米边界：物理rig.max_distance=40；Adapter.process_lidar又将同一<=40m过滤云发给建图和三维定位。不能只把硬件量程增大而保持定位订阅旧近场过滤云，也不能直接将远处点全部交给局部terrain。全扫描限点发生在距离过滤前，会使远回波占用近处障碍点预算。

## 修改与全链路约定

- 物理rig仅max_distance 40→80，其余传感器/车体参数保持。
- Adapter在相同State acquisition pose下只做一次世界到虚拟LiDAR坐标变换，按范围分别采样：导航40m、定位80m，各最多30k。occupancy与terrain仍使用完全相同的40m点集，远处点不会污染近场或挤占预算。
- 新定位topic /spot1/ouster/front/points_for_localization 与旧points_filtered使用相同spot1/ouster/front/os_lidar虚拟frame、相同state_ros_stamp，不挪TF/原点，不以接收时间重打测量戳；原skew/clock-reset/查询/多层歧义/支持门保留。
- Triangulator切换到新topic；控制器/costmap仍订阅旧近场topic。记录器增加新topic，方便核验真实传输。没有直接控制或替代规划通道。

## 验证状态

7新增最终改前两遍6fail/0error，改后61相关两遍/原828离线/4物理rig通过。初始离线double缺PointCloud2和PointField、message(name)与标准字段name冲突，两个早期错误保留，补齐标准消息字段并只改test double工厂参数名；没有放宽运行断言。真实ROS及C++回归进行中，未开始现场、不推广/commit。

新范围会改变输入采样分布，不能用“地图半径不变”宣称所有地图结果逐点不变；须现场核对更新频率、新鲜命令、障碍约束与完整接近。额外定位DDS每帧最多360KB，queue/同步缓存维持原界限，新增流负载也须现场核验。
'''
(repo/'docs/LOCALIZATION_RANGE_20261027.md').write_text(s,encoding='utf8')
with (repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8') as f:f.write('''
### 2026-10-27 40米量程根因两遍对照，v49分流候选（真实ROS进行中）

- v48零LiDAR后固定6原相机视点/mask，16线/density30不变，两次40米0/12、80米12/12定位（22–36支持，约747–749,-130房屋区域），非语义真值或完整车辆验证。静态证据depth_range_probe_20261027及localization_range_20261027/diagnostics；此前增线对照会回退，未改线数/密度。
- 最早输入限制：rig和定位/建图共用过滤云均40m，远目标回波根本未进入定位。v49从v48隔离：rig仅量程80m；Adapter按40m导航与80m定位分别限30k，occupancy/terrain严格同40m样本、各自预算不被远点挤占；新points_for_localization沿原虚拟TF/state测量戳，Triangulator订阅切换，costmap/控制仍近场旧topic。新增记录该流，未改门限/模型/物理/连续控制。
- 7最终反例两遍6fail/0error后实现，61相关两遍/828原离线/rig4通过；初始离线fake缺PointCloud2/PointField及name工厂冲突日志保留、按标准消息修正，非运行问题。真实ROS/C++待补，未现场/推广/commit。
- 采样分布和额外DDS负载会变化，不能假称地图结果逐点相同或无回退，后续须核验频率/时间戳/障碍数据与完整流程。详docs/LOCALIZATION_RANGE_20261027.md。
''')
print('static 40m 0/12; 80m 12/12; local regressions passed; real ROS pending')
