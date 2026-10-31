# 观察点候选部署与探索走停复核（2026-10-21）

## 本轮结论

`reachable_observation_v11/full/trial_20261021_221131`：181.02秒timeout，
没有目标mask/接近/确认。本次观察位姿修复没有在现场触发，不能称现场修复有效。
探索走停仍未解决。用户质疑后继续对同一轮关闭bag取证，没有重跑随机搜索或追加导航修改。

## 版本与验证范围

- 原v10保留；服务器隔离目录 `/home/ly/execution_continuity_repair_20261021/reachable_observation_v11`。
- 两包构建通过；实际ROS定向20、既有visible22项通过。初次真实ROS有两处
  Mock夹具错误，改测试后重跑通过，生产逻辑未为测试改动。
- 本机回调682项，681通过、1项改前已存在的look-back断言失败；不报全绿。
- 421项冻结成员校验通过；运行桥SHA `8265169d13cd8cd7228785bd161f025cb258b56697e5d42ce4cc2aaa1939d5f7`，
  GoalStandoff `a6276ac4bacdc2ee8dc4a5343470ba0452b361a83ad1ae69b52549ccf56950b5`。
  安装路径与现场verified_sources匹配，实际Smac库仍为原89ae42c4…版本，反向惩罚0。
- 没有修改车体、速度、转弯半径、碰撞或到达判据。单地图加载，完整物理车体。
- 启动前与收尾后残留进程均为空、X148锁清除。GPU0另有其他用户作业，未干预；
  本轮不足以量化该负载的影响。未录像、未commit。

## 同轮180秒窗口的实际停驶

|指标|结果|
|---|---:|
|Native速度覆盖|179.994秒 / 3600样本|
|近停（绝对速度<0.1m/s）|91.433秒，50.8%|
|最长连续近停|7.303秒|
|Native采样里程 / 净位移 / 最大离起点|70.57m / 14.48m / 24.00m|
|gear_change近停|36.887秒|
|stale_command近停|29.463秒，其中planning期间24.346秒|
|tracking / ackermann_stop / execution_hold近停|10.519 / 7.170 / 7.266秒|
|planning驻留 / following驻留|34.815 / 122.857秒|
|ComputePath / empty_path|47 / 11次|
|计划段派发 / 完成机动 / 实际D-R切换|23 / 14 / 18次|

3600个Native样本telemetry_ok均true、source_rejection均None，最大样本源时间间隔
0.159秒。不能把stale_command标签直接解释成网络拥塞；其大部与规划阶段停发控制重叠。
following期间近停47.834秒，不能将following当作持续行驶。里程不是净位移，
planning驻留不是CPU时间。与212436没有同口径near-stop对照，不能据此断言本次修改造成变化。

## 换向与推进被截断的已证链

23次planned是9个探索owner的首次派发、13次原队列续段、1次碰撞重算。
18次Native实际换向对应12次续段cusp、5次新owner方向翻转、1次碰撞重算。
23个派发段中11段不足1米。并非车辆自行无故倒车。

具体反例：owner:2的完整路径为D1.209m→R0.938m→D10.469m。
前两段于查询后5.047、8.373秒派发，9.940秒GraphNav发出`frontier_resolved`，
桥按既有退休规则保留当前段、丢弃后续队列。最后10.469m前进段没有派发。
5.047–14.080秒Native累计移动约1.747m、净约0.462m，已经支付换向成本。

owner:18同类：完整R0.515m→D3.148m→R19.56m，161.112秒
`frontier_resolved`，尾段未派发，随后新owner再次从短机动段开始。
这证明前沿生命周期与整车多段机动之间存在效率缺口；不证明frontier=false本身错误，
也不支持删除退休或碰撞保护。应在同一反例上验证如何连续交接和兑现安全推进收益。

## 规划选择的源码问题与证据边界

- `connector.cpp`按栅格位置逐个遍历全部72航向，再换位置。0.25m图下，
  即使达到6000次上限，前4圈81格已消耗5832次，6m允许范围大部未覆盖。
  本轮180秒内47次continuous_start全在0.2秒期限截止、每次评估1020–1667组合；
  这是空间/航向预算分配偏置。
  它可能让近处多cusp接驳优先入选；需要同一真实地图/起姿的受控对照，不能直接认定
  它解释全部换向。未改惩罚或提高预算掩盖问题。
- Smac确实按完整路线的长度和cusp成本比较；本轮35次selected中31次报告5–6个完整候选，
  不支持“完全没有比较”或“总是只有一条解”。
- bridge精确探索终点+observation_heading候选首条可行时会跳过normal_best普通航向比较；
  普通分支也只比较同端点航向，不对全部推进端点做统一选择。
  3–5段在现有6段上限内合法。相关选择策略与动态退休的组合仍未修复。

## 本机证据

`test_artifacts/reachable_observation_repair_20261021/server_v11/full/trial_20261021_221131/`
包含结果、运行身份、规划/感知日志、原始事件和Native样本、失败末帧。
`fluency_audit/closed_trial_summary.json`、`segment_chains.json`、
`representative_retirement_chains.json`、`path_geometry.json`保留时间、对象owner、
实际路线和上游退休原载荷。原关闭bag留服务器，SHA
`000bbd18bad8a4f4167b0ff585b1e87de7d36a5bea633dced3252efc1dc99dc9`。

当前优先级由用户纠正回探索执行连续性。观察点候选保留为待现场验收，
不将其宣称为探索走停修复。完整搜索及视觉记忆多目标验收均未通过。
