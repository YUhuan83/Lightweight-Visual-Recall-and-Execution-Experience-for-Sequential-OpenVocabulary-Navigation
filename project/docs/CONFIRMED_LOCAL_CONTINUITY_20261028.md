# 确认目标的局部连续接近（2026-10-28）

完整绿房屋到达尚未通过。当前隔离候选为 `test_artifacts/confirmed_local_continuity_20261028/candidate`，服务器v56_confirmed_local_continuity；继承v55的评分端点修复，不修改主703运行源码，不提交。

## v55现场结果与更正

trial_20261028_083200启动62.095秒、搜索180.031秒、收尾22.034秒超时。session errors=[]、console0，所有项目进程/X148锁清空。实际加载GraphNav f573bcd1...新库，原Smac/RPP库保持。Native181.460秒（含边界/收尾）近停24.747%、最长9.200秒，净46.307m、累计128.728m；49mask/39有效LiDAR/36程序确认，确认帧最小车身56.851m，未到2m。

本轮没有实例释放或Native租约故障，但仍折返，不能把“目标保持”当作完整修复，也不承诺本轮无回退。证据在 `test_artifacts/frontier_endpoint_contract_20261027/field`。当前累计范围不等于严格裁剪到query180秒。

## 最小接近修正

- physical_vehicle_control.launch将实验性的远目标graph_destination_approach设false，复用已有GoalStandoff近距可观察姿态选择。真实三维物体坐标继续独立锁定；运动路径仍经GraphNav/12m Smac/整路RPP移动交接。
- TargetObservation先消费已排队的同实例后续接近点，再判断是否等下一段路线。v54先返回partial等待，会跳过已排队交接；新增反例已复现。
- 中间接近点不是最终物体视点。新增明确的standoff_intermediate消息字段，按实际选点到物体距离是否仍超过安全standoff+.2m生成；它与standoff_clamped分开，后者还含车身安全距离调整。仅保留与当前执行点相符的中间点，不把旧点元数据借给新点。最终点仍要求新鲜观测，车身外缘2m标准保持。
- 确认/刷新路径通过带query/instance的JSON入口，调用publish_standoff(...,publish_pose=False)，避免同时发无身份Pose。几何函数默认旧API保留，直接几何单元测试不改。近距入口的重复Pose反例已复现。

运行变更共3文件：target_observation.py、goal_standoff.py及physical launch；2个测试文件新增/更新。未新建导航状态机，未合并目标实例，未取消物理/代价地图检查。

## 验证范围

- 原始中间点等待/排队接续反例两遍2fail；近距重复入口反例1fail。明确中间点字段前又补充“仅车身安全距离调整仍属最终点”反例，修改前1fail+1error（缺该字段），修改后相关204项两遍通过。
- 第一轮真实ROS回归中，旧GoalConfirmation三个测试仍依赖裸Pose/旧mock签名（2fail+1error）；更新为检查确认JSON身份、几何点、时间戳并不发裸Pose，mock明确要求publish_pose=False。首次失败报告保留。随后882真实ROS及32控制器测试通过；追加中间点字段后的最终真实ROS验证进行中，不用先前通过冒充最终结果。
- 同一关闭v55传感器录包，真实producer两版本各两遍：旧36个远目的地，最远68.441m、0clamped；新36个局部点，最远8.321m、36clamped、裸Pose0。使用历史车轨迹，不模拟新动作，因此不能证明新车轨迹会到达。首次诊断时钟缺to_msg错误已补全，原日志保留，非运行崩溃。最终字段版本需重放复核。
- 既有蓝房屋实例隔离、延迟/旧消息、确认/控制交接、车身到达回归保留。最终完整180秒现场待验收；未推广commit。

## 后续更正与停止（最新）

最终字段版本883真实ROS通过后启动v56 trial_20261028_085117。继续核查真实消费者发现，graph_path_bridge.py::on_object_confirmation把active_object_confirmation重建为query/时间戳/目标XY/instance的几何子集，并未保存standoff_intermediate/standoff。新TargetObservation从该容器读取标记，实际链路不能使用它。此前新测试手工设置该容器，只证明分支行为，没有覆盖producer消息→实际消费者→路线完成；不能据此宣称端到端已接通。这是本候选新增接口放置错误，不能归因BeamNG或原文。

发现后向当前v56驱动器发送正常结束信号，由既有/quit停车和收尾。启动56.228/搜索80.057/收尾13.219秒，主动中止，非180秒超时或完整验收。session errors=[]/console0/进程X148空。Native81.508秒近停16.306%/最长3.562、净61.535/累计68.669m，仅短轮数据，不能与完整180秒排名或认定稳定。当前停止时净位移61.5m，不能仅凭末值确认/否认更早是否返回出生点；用户质疑的返回行为须按对应轮次轨迹逐段核实。

用户随后明确指出纯WildOS原文在真实车体上已能看到目标后直接接近，反对继续擅自叠加机制。暂停沿本候选继续补丁，不推广v56。已创建confirmed_route_metadata_20261028基线副本，但未修改运行实现、未部署v57；不能把目录名当新修复版本。

已读取本机原文参考：`C:/Users/Yu/Desktop/Beamng/wildos_controlled_experiment/repository/reference/nebula2-wildos`。原obj_mask_triangulation.py::publish_navigation_goal_and_marker直接发布三维位置；原Planner::update_graph用traversability_cost建立无向图，无新增robot物理评估剪枝；原PathFollowerNode在每次有效odom上更新前视路径目标。当前的额外物理剪图、确认/接近点队列、观察超时与释放恢复属于后加适配。后续先以用户跑通过的原文链路核实差异及可回退边界，记忆提供历史目标/候选，不能借加记忆重写接近执行。已修坐标/时间戳/消息契约和车辆执行约束的去留需分别核验，禁止把整套回退等同于取消碰撞或一刀切清理用户未提交代码。
