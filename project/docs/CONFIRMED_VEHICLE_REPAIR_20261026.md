# v35确认后中间接近与车辆航向交接

2026-10-26。隔离候选，完整流畅验收尚未通过，不commit。保留v34及用户主目录。

## 原因，至少两遍复核

v34关闭轮222040已得到28条LiDAR定位和24条确认，但13条无接近点，最小确认时车身间距仍29.93米。24组同帧地图/车姿/目标的独立快照两遍均复现13组选点失败，使用既有车辆圆弧提议可为其中8组提供地图有效的中间点。原生Smac三组各两遍：2组0换向，1组1换向，均可规划。回放未重建原拒绝/last_goal全部历史，公开raw图非现场私有锁定图，不冒充整车因果精确重放。

另有明确接口缺口：producer和consumer都强制目标朝向，确认后路径候选又再次覆盖为面向物体；可行中间点的车辆航向无法传到底层。新增干净回调反例（真实地图失败、航向覆盖、非有限航向）改前两遍均3fail/0error。

## 修改范围和契约

相对v34只改3个运行文件：

- GoalStandoff抽出弱提示已经使用的车辆观察点提议，供确认后受地图截断的中间接近复用。保留当前地图/配对车姿的新鲜度、扫过车身、占用/未知余量；已确认物体还须得到至少0.6米实际距离进展，并保留车身/相机最小目标间距。最终观察位姿仍按原规则面向目标。找不到车辆候选时保留原可用候选/失败路径，不删除障碍约束。
- 可选`standoff_navigation_heading`随确认消息与approach_refresh传递；原观测戳不更新、不伪造新鲜度。重复保留旧点时也保留属于该点的旧航向。未确认/已确认身份、实例和源quality不改。
- ConfirmedTargetApproach保存与所选中间点绑定的航向，活动动作仍独占；新点/航向延后交接。GraphPathBridge校验字段只适用于clamped点且必须有限，relay与ComputePath候选都使用相同航向。旧消息没有该字段时沿用面向目标规则。

不改变可靠支持帧数、车身2米最终到达、真实物理、地图、C++ Smac/RPP、车速、ACK或看门狗。新增1个测试模块与真实快照夹具。修改/源SHA见source_scope.json与confirmed_vehicle_delta.zip，主目录703运行文件不覆盖。

## 验证和状态

- 13项新回归与相关159项两遍通过；包含真实producer→确认→consumer→ComputePath请求的航向一致、最终旧消息行为、延迟更新、query切换和非有限数据拒绝。828项原离线通过。
- 实际v35 producer在同24快照两遍得到19组ready（原11组），新失败5组仍保留；两JSON同SHA `f21cdb1434940e4562c5d5aeac63f15b1d88c036ea0880dbb1713783bc653ea6`。三组原生Smac各两遍仍为0/1/0次换向，未声称全部无换向。
- 两包构建和真实ROS回归通过，整车180秒搜索待记。组件通过不替代最终视觉/车身到达和稳定性，不推广、不commit。

证据：`test_artifacts/confirmed_vehicle_repair_20261026/`；服务器独立`/home/ly/execution_continuity_review_20261022/v35_confirmed_vehicle/`。默认无视频，保留日志和必要图像。尚未证明最终近距离阶段不存在其他缺口，须按最早接口违约点继续排查。

## 现场关闭与未解决事项

- 首次084031在预检已通过后，关闭控制门的SetParameters服务超时（enable，10秒），goals=0；没有进入搜索。SIGINT之前没有节点运行异常traceback，关闭后的KeyboardInterrupt不能算运行崩溃。证据startup_failed_084031.json与服务器原session。相同冻结版本复测启动通过；精确服务延迟来源仍未定位，不能称已修。分析包装器因没有goal日志又报统计错误，原结果和退出证据保留。
- 复测084439已关闭，startup39.993/search180.192/shutdown15.018秒，超时/完整验收false、session errors=[]、console0、项目进程/X148锁无。757真实ROS已通过，不推广/commit。
- 净54.808/路程110.157米；近停33.294%/60.267秒，最长10.135；stale37.081，其中planning30.284、gear3.607秒。114mask/2定位（1LiDAR+1multiview）/0确认，新确认后分支没有现场触发，不能认定已整车验证。
- 停顿在临时观察阶段：多次no_vehicle_path、两次换向后footprint_collision、一次发送前motion_section_invalid/cusp_position_outside_tolerance。本轮没有RPP bounded-arc abort。比v34表现差，不能称没有回退；四个固定v33弱提示快照在v34/v35的实际producer输出逐值一致，只支持这些输入的重构等价，不支持所有现场行为完全相同。
- 两次Native/events报告同SHA `0dc48f93bf5dca625097a31993ddce7123a8e9a1b7fb0bde15d5c7c6de9683a2`，field_result、field_phase_audit、closed_arrival_audit和trial_20261026_084439_core.tar.gz已取回。

另有仅受控接口反例，尚未修：到最终观察点仅0.348米、当前车身距对象2.273米、车姿与要求航向差45度时，on_object_confirmation只报告object_visible_not_arrived、没有新导航意图；v34对照也有此行为。两遍JSON同SHA `286dd25fdf6d8270e86086834a1e75fbbd5d1f1d16ac7593345ee90b120d0207`。该输入使用有效帧/时间和空闲地图，是构造的接口反例，不是本轮现场几何；本轮零确认，不能拿它解释本轮超时。后续须检查位置接近与整车最终位姿/2米到达交接，不能用原地旋转绕过真实车体。

v33/v34关闭Native在speed>.5区间的模拟时间/墙钟比分别0.9930/0.9938，Native速度积分与位置路程一致；这两轮不支持“BeamNG整体模拟变慢导致低位移”。证据两目录simulation_time_audit.json，未据此排除局部RPC或ROS服务时延。
