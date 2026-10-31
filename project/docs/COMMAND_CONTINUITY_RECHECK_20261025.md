# 命令连续性退化与到达假设复核

2026-10-25。只读排查与冻结v28整车复测；未修改生产代码、配置或创建commit。

## 冻结v28复测

服务器`v28_intent_handoff/full/trial_20261025_200844`。预检项目进程空/X148锁无，原候选清单SHA和配置在运行前后相同，实际入口仍是`continuous_graph_path_bridge`。目标下发后搜索180秒，启动、收尾另计，无录像。

- 近停32.033%，最长8.952秒，净位移90.404米、采样105.907米。
- 无新鲜命令近停32.097秒，其中planning22.730秒，换挡4.234秒。
- 完整搜索仍超时，console正常关闭，session errors为空，后检全部项目进程/X148锁清空。
- 历史v28为15.668%/stale8.190秒，v30为40.916%/stale48.484秒。原样v28本次也有长停，说明不能仅凭版本前后就认定全部退化由v29/v30引入；也不能把历史15.7%当作已经稳定的基线。路线和负载未完全控制，此轮不证明定位修改无任何间接影响。

## 旧修复与空窗

v28/v30两个bridge文件SHA相同；持续探索健康动作保留机制没有被删除。原v30按记录elapsed划分，前90秒近停15.04%，后90秒65.48%；非目标发送精确裁剪，覆盖分别87.30/89.43秒。恶化主要集中后程恢复，不是所有时刻命令发送都失效。

原v30约91.3秒后台可行路径被保留以比较另一朝向，1.728秒后按新车姿检查出现wrong_start（偏移1.233米）。只读录包裁剪可得路径，但最新全局图仍报collision_cell；下一次重用时已不能接入首分支。不能只删除wrong_start或放宽起点门来宣称解决12秒空窗。公开图复现不等于插件私有锁定图真值。v28原轮未观察到同类extension_rejected；此反例不解释全部长停。

## 用户提出“接近后一直判不到达，所以停着重规划”

本次v28复测首次确认约154.7秒，其后录到26条search_confirmed=true消息，全部approach_goal_ready=false、standoff=null、approach_failure_reason=no_reachable_standoff。按同帧odom复算至确认目标坐标的车身距离约32.24→29.32米，不是已证明车身进2米但终态误拒。距离是对估计的三维点，不是建筑真实表面真值；未用单张图认证该目标或语义识别完全正确。

`GraphPathBridge.on_object_confirmation`在无standoff且未达2米时只发布confirmed_target_waiting_for_approach并返回；该分支本身未取消动作，也未发起规划。该状态覆盖期间统计出的7.32秒近停不能直接全部归因到达判定。154.5–160.2秒实际仍有规划候选和短探针，后台状态标签不能代替动作因果。

另一个8.95秒停顿在90秒、确认出现之前：maneuver_done→续段footprint_collision→串行候选0–6→重新following。原v30有效3D/确认均0，也已有严重走停。因此最终2米门不能解释全部停顿。

目前证据支持把接近点生成/车辆可达性、执行失败后的替代观察/探索交接作为重点。尚未闭合no_reachable_standoff究竟由地图边界、连通性、车身朝向、射线截断或已拒候选排除中的哪一门导致；不改到达距离、不绕过碰撞门，先保留反例。

本机证据：`test_artifacts/command_continuity_regression_20261025/`下baseline_recheck.json、confirmation_wait.json、cache_reuse_audit.json、stop_windows.json、status_comparison.json与三份core归档。新增脚本仅为只读审计和原样测试驱动；无运行修复、无commit，未宣称稳定或完成目标。

## 26次接近点失败的逐门回放

实际安装GoalStandoff源码SHA `412b914893ed1418b583fb44bdcf2832dd78e9685257ee3a39010b4ae04a9cfc` 与冻结v28一致。用消息接收时刻之前的最近local_map、global_costmap、odom快照调用实际publish_standoff，未载入历史被拒区域（因此是偏宽松的接受条件，不冒充整个回调序列）。两遍报告SHA均`e6a6cc30d55e0722d9dcf5ae582d81dcc9837b1435fc0bf01bd12d2f5466450f`。

- 26/26复现no_reachable_standoff。目标在80×80米全局图内，首尾局部图龄约0.38/0.34秒、全局图龄0.56/0.003秒、地图odom配对戳差均0；不支持这些快照的主因是地图边界或普遍时间断传。
- 每次24个目标环候选加1个直线截断，共650条射线，均先碰到local_map的已占用100格；不是未知格或越界。候选环要求整条直线无阻挡，尚未提交GraphNav/Smac绕行。
- 最终唯一截断候选26个：23因vehicle_footprint_blocked、3因not_connected_known_free被过滤。这些是提议检查，不等同Smac对所有道路路径证明不可达。
- 在相同快照用既有窄锥observation_step找中间观察点，8/26可通过地图过滤。3个原截断点在当前车身朝向可容纳，但面向目标时不通过；仍未证明该朝向/位置可被12米半径Smac实际到达。
- 用记录中的新鲜掩码、受控“已建立3帧连续性”状态对照，2个快照无pending_confirmation时可发布观察步，同输入加pending后被完全抑制。这是实际回调的受控反例，非完整历史mask/实例状态逐条精确复现。
- 源码交接缺口：GoalStandoff有pending_confirmation时抑制publish_visible_step；bridge在确认但无standoff且未到达时，先记录等待并return，未保存可用于接近的active_object_confirmation或生成新的导航意图。强确认未能接成导航任务，原来的观察步也可能被停发。它不等于整车永远锁死（GraphNav原探索仍可能行驶），但确认→接近的接口未闭合。

证据`standoff_filters.json`。本轮只读调查，没有更改运行代码、碰撞门、2米到达或部署；原版本保持。

## 原仓库到达逻辑与当前机制的区别

读取本机原始参照`C:/Users/Yu/nebula2-wildos`，不是按记忆猜测论文实验协议。

- `graphnav_planner/src/planner_node.cpp`默认goal_radius=3.0，官方launch同为3.0；在TF到odom同帧后以目标和odom位置三维距离小于该值清除goal_pose。此为导航目标完成，不等同当前新增的人工图像/物体实例验收。
- 原ObjectMaskTriangulator有定位后直接发导航PoseStamped，orientation.w=1；该路径没有当前GoalStandoff四条同簇确认、物体周围接近点筛选和车身外缘距离门。
- 仓库另有GoalNavigator默认2米二维路点判定，但WildOS_Nav继承TFLookupSubscriber；GoalNavigator由LRN/GeoFrontier/ImgFrontier模块使用，不能混称它就是WildOS当前运行链的到达实现。
- 视觉记忆不必替换基础到达判定。多帧/实例/新鲜度约束来自后续识别可靠性与可信记忆、跨目标状态管理；车身外缘距离和接近点来自真实车辆及目标表面不可直接驶入的执行需求。不能把全部变化归咎记忆，也不能称现有复杂链是原文必需。
- v26已允许车身满足2米且证据合格时跳过standoff直接完成；问题在尚未到达时接近任务仍强依赖standoff，而不是最终成功仍一律要求先完成指定接近点。下一步应保持导航进度、最终到达、记忆写入各自的证据职责，避免确认成功反而阻断继续靠近。
