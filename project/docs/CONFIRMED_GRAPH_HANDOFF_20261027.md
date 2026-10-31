# 确认目标交给GraphNav并保持归属（2026-10-27）

v53/v54均为隔离候选，主703运行文件/原ZIP及此前轮次保留。尚未完整通过，不提交、不推广。

## 两张末帧的含义（用户于本轮指出）

第一张来自v51 trial173048，画面左侧确有清楚的绿色房屋，不能解释成目标未出现；车身2m到达尚未完成。第二张来自不同轮次v52 trial175000，不是同一条轨迹的下一帧。

v52日志：115.680s临时点(735.934,-73.946)结束后awaiting_visual_confirmation；121.059s请求释放，121.060s释放green residential house实例；121.095s恢复图探索；151.720s跟随(716.375,-62.625)，目标frontier约(710.735,-59.560)，157.589s继续朝该区域。查询始终green residential house，没有切换成gas station。转向加油站区域是释放房屋约束后的普通探索偏离，不是目标到达，也不是已证明把加油站识别为房屋。

源码monitor_observation在5秒等待后，start_live_observation不能找到替代观察点时会release_unconfirmed_target。它把中间点/替代视点失败升级为释放目标。目标已知、路线失败、最终到达应分开处理。这正是用户指出的目标接近与失败交接缺口。

## v52关闭

搜索180.150秒超时，启动43.204/收尾23.033，session errors=[]、console0、项目进程/X148锁空。Native182.243秒（含收尾）近停27.894%/最长15.949秒、净54.933/累计115.073m。未到达，未通过流畅/不回退要求，不推广。独立Native/events两遍复算已保存。

## v53复用既有图规划目的地策略

核对当前保留的graphnav_planner/src/planner.cpp::plan_to_goal：没有完整已知路线时已有目标方向的frontier评分/选择分支。此前GoalStandoff把远确认目标不断替换成<=8m中间点，限制了这条全局通路。

v53新增单个实验参数graph_destination_approach（兼容默认false，真实车体launch显式true，并贯穿官方launch的bool参数）；远目标使用原standoff几何作为GraphNav目的地，近处保留已有可观察姿态筛选。未改GraphNav源码/评分或Smac+RPP控制。它复用当前图规划分支，不声称还原论文全部执行器。

新增6用例最终改前两遍4fail/0error，79相关两遍/828离线/871真实ROS两遍/32实际C++两遍/console/rig通过。初期误加载导入的9个测试、桥接夹具缺odom_received_at导致的假失败均保留，修正后才记上述结果。现场只读参数确认为True。

v53 trial180515启动51.381/search180.116/收尾22.624秒超时，session errors=[]、console0、项目进程/X148锁空。Native183.113秒近停30.190%/最长13.017秒、净43.777/累计124.760m。现场确实提出约(747.640,-125.848)的远目的地，仍未跑通；不能用策略改变或组件通过代替整车结果，不推广。

## v54继续修正交接（现场中）

- 远目标原先同时通过裸Pose及带query/instance的确认JSON输出。实际桥接on_waypoint会先reset已有可见接近，再设置expected；后到确认因点相同只补锁定。v54去掉远目标裸Pose输出，仅由已有确认JSON进入桥接，避免双入口抢同一目标。不是新增状态机。
- 对已确认目标，route_complete=false表示仅到一段frontier前缀；await_object_observation不再把它当最终物体视点并启动释放倒计时，而是保留expected及实例等待后续GraphNav路径。真正完整目标路线及未确认普通观察点仍要求原最终观察。不放松碰撞、12m规划/8m控制或车身2m标准。

新4项改前两遍2fail/0error；改后298相关两遍/828离线/875真实ROS两遍/原32实际C++两遍/console/rig通过。主703/ZIP与控制库不变。预检清场后开始v54实际query后180秒绿房屋，无视频，结果待补。不能声称本次四项组件已证明完整目标接近修复。

证据：test_artifacts/confirmed_graph_destination_20261027、confirmed_graph_handoff_20261027；前两图及原始事件分别仍在confirmed_action_handoff_20261027/field、approach_local_map_20261027/field。

## v54现场关闭：目标接近仍未完整通过

trial181924启动50.983/搜索150.958/收尾20.837秒。不是完整180秒搜寻；150.663秒goal_result以session_exit失败，原因是Native actuator unhealthy超过.4秒，车辆Lua报Vehicle control lease expired or closing。console1，完整到达false，所有项目进程与X148锁已清空。没有把控制租约异常归因到本次目标代码，也没有放宽/重置过期租约；确切喂租间隙或closing来源仍待查。

47mask/37有效LiDAR/14程序确认，确认帧最小车身距50.354m。Native149.590秒（含边界/非精确query裁切）近停27.585%/最长9.995秒、净81.757/累计106.124m；独立轨迹两遍一致。数据不能证明完整到达或无回退。本轮object_standoff_approach=2，target_instance_released=1，confirmed_target_partial_route等待分支触发=0；未触发的分支仍只有组件验证，不伪称现场已覆盖。

远目标进入已有GraphNav的改动已实际触发，但此前planning/执行失败仍存在，不能把本轮所有停顿推给最后的Native异常。本候选不推广、不commit，主703/原ZIP保留。后续需要继续验证目标接近路径的可执行性与持续推进，并分别复核Native租约喂租/关闭来源，再完成180秒整轮；不通过延长倒计时、降低确认/到达门或关闭碰撞冒充修复。

证据test_artifacts/confirmed_graph_handoff_20261027/field与validation_summary.json。上述图像问题的直接解释仍是v52释放目标后恢复普通探索，不改写为识别目标没出现。


### v54释放事件更精确复核（更正，不与v52混同）

v54确有一次target_instance_released，但原因不是v52的中间点后5秒观察超时。137.554s先发生target_instance_superseded，替代簇约(750.620,-129.983,58.180)、4/5支持，随后137.555/137.560s按现有实例替换协议释放旧实例；旧目标60.186s约(747.196,-130.085,54.793)，两者XY相差约3.43m、Z相差约3.39m。144.654s重新进入接近时又确认约(761.999,-130.031,58.783)。尚不能仅凭这些坐标判为同一房屋不同立面或另一个物体，须结合对应掩码/RGB/投影复核；不擅自扩大关联半径、取消已修复的跨实例隔离或把重绑定认作正确。

普通partial前缀等待分支本轮未触发，不能声称已现场验收。先前看到released计数时只统计，不能据该计数认定仍是相同的5秒释放故障。目标接近路径/稳定绑定仍未通过完整验收，Native异常发生更晚，不能拿它解释之前的规划停顿和实例变化。当前所有进程已退出，所有候选及证据保留，未commit。
