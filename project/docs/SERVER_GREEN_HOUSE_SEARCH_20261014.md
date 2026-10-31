# 2026-10-14 Town 绿色房屋服务器实测

本轮完整单目标诊断试跑失败。绿色房屋查询已ACK，真实SBR实际行驶，但420.0869秒内没有有效目标掩码、三维定位、接近点或视觉到达。不能计为成功，也不开始多目标测试；20轮稳定性验收仍未启动。

## 版本与准备

- 冻结候选：服务器 /home/ly/town_memory_validation_20261013/navigation_transactions_candidate，candidate.zip SHA256 f9561be85a67cc1b47044f090e8fbb80cda00e14fbb0b0bf52e7c49d116e526f。本轮没有修改生产代码、模型、车速设置、碰撞、轮胎或8m转弯半径。启动时8个导航/控制/租约文件逐字节核对；正式preflight核对实际Nav2配置、参数、进程/节点单一归属、相机外参及输入健康，passed=true。
- 上次2026-10-13的准备失败证据已取回：test_artifacts/green_house_search_20261013_trial1/。暂停复位后立即读出的速度为3400.997m/s，挂载器以未停稳拒绝启动；尚不能将读数解释为真实高速运动，也不能由随后SSH超时确定引擎崩溃。本次连接时旧BeamNG和测试进程都不存在，旧日志末尾仅有SDK连接关闭记录，退出来源未知。
- 新BeamNG实例PID1202233，今日加载Town地图一次。出生瞬间车身落地速度0.883m/s使原初始检查失败；保留scene_preflight.json，不改为通过。独立普通全制动检查要求连续停稳，随后新SDK物理快照通过并写scene_after_settle.json。传感器挂载自己的严格静止检查也通过。搜寻期间0重载、0瞬移。
- 原车体723节点、0固定节点、467个车身碰撞节点、四轮各32个轮胎碰撞节点；真实动力学、8m半径、CarLike规划、Ackermann RPP和双代价地图障碍约束保持。旧短waypoint状态由新导航栈隔离。相机960×640、LiDAR、三维定位节点及do_object_search=true视觉节点正常启动。
- 仅发送green residential house一次。视觉日志确认door→green residential house并编码目标；ACK候选views=[]。该轮没有选择历史目标候选，不能把失败归因为曾见目标检索或宣布整个VisualMemory无关；尚未做无记忆控制变量对照。

## 实测结果

| 项目 | 结果 |
| --- | --- |
| 搜寻时长 | 420.0869秒，search_timeout_without_strict_arrival |
| 里程 | 里程计累计176.0398m；Native车辆参考点累计179.7896m，两种参考点分别保留 |
| 最远离Town出生点 | Native世界位置32.0485m；累计里程不能等同探索覆盖 |
| 最大记录速度 | 1.5943m/s |
| 视觉链 | query 1、ACK 1；object_mask/localization/confirmation/visible_target_approach/imgnav_waypoint均0条；memory_selection 0条 |
| 路径动作 | following 31；4次segment_done和21次maneuver_done；6次follow_failed、3次no_vehicle_path、4次replanning_after_maneuver |
| 执行失败分类 | 4次Ackermann单方向保护中止、1次bounded command arc碰撞检测中止、1次tracking_deviation（横向误差1.002m） |
| 曲率候选拒绝 | 68条excess_curvature日志；是日志计数，不是68次独立实验 |
| 长停驶采样 | 6个速度<0.05m/s持续≥5秒区间，最长约21.35秒；包含规划与交接等待，不能直接归类为受困 |
| 末帧 | 源龄0.180秒的原始RGB显示近距离树林、草灌与路边；不是到达证据，不能单独判定车身碰撞或被卡死 |

## 最早已对齐的执行失败

第58.429秒FollowPath动作ABORTED，控制器明确报告Ackermann lookahead would reverse a single-direction path。实际DDS记录保存在first_abort_interface_fixture.json：

- 上一段终点：spot1/odom中(712.09499,-3.39056)，yaw2.44346。
- 中止前最新里程计：(711.64492,-3.60171)，yaw2.56859；与段末点距离0.49714m、航向差0.12513rad。
- 该段使用cusp_goal_checker，位置容差0.20m；未满足段末完成条件。
- 控制器实际转换路径为spot1/nav_base_link，末点(-0.26687,-0.42147)，已位于车辆后方。该转换路径接收时刻距中止约5.6ms；里程计源龄约0.1956秒。
- Bridge及时处理此终态并开始下一条GraphNav路线，未观察到本例following遗留。仍不能替代S01/S02/S03所需缺响应/乱序/延迟的故障注入验收。

这确定了本轮一个重规划来源：有限转弯车辆没有在该单方向段的容差内完成，继续追踪将要求改变方向，因此保护中止。仍需回放末端跟踪、前视/曲率和时间延迟的共同影响，不能只放宽到达容差或取消方向保护。已有P03/P04/G01几何与交接问题仍待修复；P01/P02恢复预算和候选归属也未由这轮排除。

末帧的植被场景需要继续复核T01：GraphNav边可行性、Nav2弧线路域与terrain约束之间的关系。本轮碰撞弧检查确实中止过一次动作；当前不能把再次出现树林画面解释为碰撞检查未开启。未录local_terrain或原始点云，不能仅由末帧确定WildOS的travability输出或LiDAR是否遗漏对应植物。

## 时间戳与证据边界

- 里程计源龄中位数0.271s、P95 0.423s、最大0.548s；原始RGB中位数0.287s、P95 0.440s、最大0.539s。最早中止附近没有多秒旧里程计，仍不能忽略这些亚秒延迟对0.2m段末容差的影响。
- planned_path接收源龄中位数约0.8ms、最大约3.0ms。plan_status表中最大41.1s是repeat_status复用原状态事件stamp的统计，不能直接作为传感器或DDS停顿证据。
- 本轮只保存少量图像、接口bag与日志，无MP4。没有全程RGB录像，不能断言途中从未出现绿色房屋；确定的是有效目标链消息为0且未到达。
- 初始analysis_summary.json的collision ahead=0只表示该旧检索文本未出现；实际自定义控制器的碰撞文本为Ackermann RPP detected collision on its bounded command arc，明确1次。diagnostic_review.json保留更正，原报告不覆盖。
- 本机证据核验首试已通过归档和原始成员哈希，随后附加绘图因缺matplotlib退出；保留原日志。核验工具去除附加绘图后退出0、40个原始成员逐字节哈希匹配。此工具依赖失败不属于现场导航失败。

## 最终状态与证据

正式试验完成后两执行门关闭、真实车辆停稳、仿真暂停；停止本轮注册的stack/vision/triangulation/sensor四个owner。round_shutdown.json通过，最终速度0.0000650m/s、物理/碰撞保持、相机和LiDAR已移除，无本轮导航/模型/录制残留。BeamNG1202233与已加载Town保留暂停。关停后adapter日志有context invalid异步发布警告，归于关停阶段，未据此解释搜寻失败。

本机入口：test_artifacts/green_house_search_20261014_trial1/。可发送的完整证据包town_memory_validation_20261014_evidence.tar.gz为18,288,077字节，SHA256 168fa598892c2c18e48f9d5a4e04b6a2d41d9a1d6ea5758b4fa8dda9934d4659；40个原始成员128,126,387字节全部哈希核对。evidence_verification.json与verification_final.log为复核结果。evidence/green_house_trial1/包含完整报告、首个中止接口夹具、失败末帧与过程中诊断帧；完整接口SQLite保存在压缩包与服务器原始目录。

本轮生产候选未改，未修复上述剩余机制问题。现场验收失败，按AGENTS.md不创建commit，也不开始20轮或多目标重复试验。
