# 2026-10-14 换向曲率与Smac平滑契约修复

## 定位范围

上轮完整绿色房屋搜索仍是失败；本轮先用其封闭bag定位最早违约点。记录中3条不同原始输出形成4个事件案例：整条路径通过，实际方向段却因excess_curvature拒绝。旧代码与原始证据保留，不将这些输出都假定成已选执行路径。

trace_curvature_windows.py现固定读取before/graph_route_dds06bb.py。拒绝窗口全部来自换向段末端的heading检查：例如原始139姿态路径65～68的1.06245m窗口/0.13847rad；原始118姿态路径38～41的1.72643m窗口/0.22241rad。全路的2m窗口跨越换向点，加上下段弧长并抵消转角，遗漏了同一终端违约；切片后窗口止于cusp，才拒绝。不是本轮碰撞图或实际车姿造成的几何复现。

## 实际插件平滑对照

服务器实际nav2_smac_planner版本1.1.20，隔离域89、导航/模型停止、BeamNG仍保留暂停。用记录输出端点和最近的已发布global costmap，原冻结插件分别开/关平滑；另加自由图对照。请求action没有录入，OccupancyGrid恢复内部cost存在量化，因此这是记录输入重构，不能宣称逐字节重现原搜索。

三组记录地图请求中，raw输出各方向段均通过8m/12m曲率约定；smooth输出均有方向段excess_curvature，而旧全路检查都允许。三组自由图在旧8m校验下没有这些拒绝，说明此前单个自由图平滑开关对照不能排除其他地图/机动的平滑问题。

官方[Smac 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_hybrid.cpp)忽略smooth的bool返回；[smoother.cpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smoother.cpp)即使失败也会更新路径与近似航向。自定义插件此前只在输出检查碰撞、预算和连续起点，没有校验每个方向段的曲率。这里定位的是Smac平滑输出与本项目车式执行准入之间的契约，不证明视觉记忆或WildOS语义探索原理失败。

## 修改与全局影响

- graph_route.py完整准入按motion_sections共享的换向边界，调用每段独立弧长/航向检查；量化不确定度、微小解析接续保护及几何圆拟合阈值保持。先拒绝有问题的候选，防止先开车、后到cusp才发现同一几何缺陷。探索和记忆路径使用同一入口。
- 新path_contract.cpp/.hpp在实际Smac插件内执行相同的方向段曲率约定。Python/C++使用共同6条raw/smooth夹具验证两种半径判定；已覆盖圆弧边界、刚体变换、倒向和重复姿态。
- planner.cpp只进行一次native搜索，临时关闭native内部平滑、异常时恢复smoother所有权；保留该次raw。有剩余预算时对副本平滑，只有成功且曲率/当前地图轮廓/预算通过才接受。否则检查同一次raw再回退；raw仍不合法则拒绝，不再搜索一遍来伪造回退。连续connector与suffix合并后再次查曲率，保持当前地图检查与总预算。
- 未修改真实车身/轮胎碰撞、8m实际执行半径、12m名义规划半径、固定3m前视、速度、到达条件、视觉模型、目标实例与动作归属逻辑。平滑回退可能增加路径量化折点；真实动态跟踪必须另验。
- 本候选也包含上一轮已离线验证的连续停稳退出修复；不能将它写成此前06bb完整轮次已执行的功能。

## 已完成验证

本机27项graph_route测试、363项生产回调替身、全局41PASS/0FAIL/6WARN通过，范围有重叠。服务器实际Smac20项测试零失败/错误/跳过，含记录地图的实际Planner回退、碰撞拒绝/清图恢复、参数冻结及起点契约；Python27曲率/8DDS/9停稳策略通过。

原插件12组输出保留geometry_replay1；新插件相同12组输出另存repaired_outputs。新结果整路及每段均允许，3条记录地图smooth配置输出回退为相应raw几何。安装和实际进程加载库身份已核对，旧RPP库保持不变。

冻结413成员候选SHA71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660，服务器curvature_contract_candidate1独立构建，不覆盖旧版本。该候选的真实车体组件和完整绿色房屋现场验收尚在进行，未commit。

## 工具失败与证据

构建成功后，首次测试缺--merge-install；接续脚本又遇CRLF和colcon PATH缺失，均保留deployment.log与validation_attempt2/3.log，修正后validation_attempt4.log通过。首次现场准备工具缺tools导入路径，在移动/瞬移/进程启动前失败；保留physical_component_session1，第二次准备用session2，1次准备复位、0地图重载。不能算车辆失败或新的DDS故障。

本机证据根test_artifacts/tracking_contract_repair_20261014：green_house_dds_trial1/curvature_window_trace.json、geometry_replay1输入/原输出/新输出，curvature_candidate1安装身份及validation_attempt4.log，curvature_offline_check.log。前次完整失败与20轮/80%门槛均保持，成功轮数未增加。

## 后续终态补记

组件13真实4段执行通过，终段误差.422883m、前/倒向均有实际运动、SDK持续停稳末.005695m/s；观察者125s终态13278消息。仅证明该几何组件，未进行完整绿色房屋验收。随后清理工具被自动审批额度错误拦截、未执行，服务器最后已知暂停/门关闭，不能声称残留已清理。

2026-10-15新用户要求转入离线测试/更名/RViz布局。之后测试更正与最新候选见docs/OFFLINE_RECALL_RVIZ_20261015.md；当时全局41组没有覆盖两条不符合局部采样要求的合成链夹具，后续已修正并纳入入口。旧失败证据保留。
