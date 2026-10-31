# 2026-10-16 探索目标失效交接：修复与真实车体复测

## 目的与控制条件

继续处理探索范围与恢复候选 23675e28。每次启动前检查服务器残留和冻结源码；生产 e6f28118 保持，两个实验均从独立目录启动。

固定 Town 加油站、真实 SBR、原传感器、速度、8m 车体转弯半径、原 Smac/RPP、碰撞检查与两张障碍代价图，目标 `green residential house`，搜索预算 420 秒。

本轮基线只在外部 YAML 中关闭 `visual_memory_enabled`、`visual_memory_map_enabled`、`semantic_landmark_memory_enabled`。构造后确认 ExecutionExperienceStore、GraphObservationStore、PassiveLandmarkIndex 均为 None；ConfirmedViewpointCache 在新进程初始为空。其余共同适配逻辑继续存在，**不是未经修改的 WildOS 原仓库对照**。

## 23675e28 第一轮完整结果

- Driver：field_scope_baseline_search_20261016_194719；session_20261016_194721。
- 421.850s 超时，未找到绿色房屋。Native 总路程 205.121m，最大离站 52.202m；40 条 Following 状态、30 次机动段完成、5 次普通段完成。
- 三次实际 RPP bounded command arc 碰撞中止，时间对应 186.814、266.071、332.849s；四次无可用车辆路径。退出时 SIGINT 对应的进程 exit -2 不当作测试期间崩溃。
- 六段速度低于 .05m/s 且超过五秒的静止，最长 27.351s。592 条路线决策，连续路线发布间隔没有超过五秒。本轮未再见旧轮 72 秒无路线窗口，但轨迹和记忆开关不同，不能据此推导稳定性或成功率改善。
- `recovery_started` 状态出现两次，其中一次是退役回调重发同一状态；**实际为一轮恢复**，约 347.255–353.396s，实测倒车约 1.701m，纵向最低速度 -1.123m/s，随后 recovery_done 并继续探索。不是两次倒车成功。
- mask、定位、确认均为零。不存在目标到达验收。新记忆先验关闭时仍暴露公共链路问题，不能把这一轮失败归因于记忆检索。
- session errors=[]，SDK 持续停稳，相关进程清理完成。启动前一次诊断包装器路径仍指旧入口，严格预检拒绝，未发目标、无录像；修复测试入口身份映射后重启，未放宽门禁。

## 新定位的状态缺口

探索 ID `5498c9e3-0771-e340-eddc-eb4ae619e270:341559553430148:7` 被持有约 222.009s，车辆在该阶段走了 109.287m，但到目标的最小 Native 参考距离仍为 12.683m；296 次图决策仅 12 次完整图路。

有界抽取首、中、末三份 scored 图，按源戳、节点数、UUID 和位置匹配原始拓扑，并保留六份 CDR。中段原始拓扑中同一节点已经 `is_frontier=false`、visited=0，末段节点已缺失。旧 choose_exploration_goal 的持有分支只检查完成、frame 和退避，随后直接返回保存的目标，不检查“已经不是 frontier”。因此继续把过时的探索意图当坐标目的地，走部分图路。

诊断初版用 source stamp 到单个拓扑消息的索引，重复戳覆盖造成一次假失配；已改为同戳全部版本的 UUID/位置/节点数精确匹配。未改生产图拼接保护。不能将这个诊断错误写成已证的接口坏包。

该证据证明失效目标没有退役的机制缺口，不能单独解释所有碰撞、树林输入和搜索失败。

## 最小修复与全局影响

候选 **274c3ef2b5cecd1522579ccd4eea2b5a3c4bab649f5e1669c9d79226de7079a6**，相对 23675e28 只改四个成员：ExplorationTarget 头文件、PlannerNode、C++ 路线测试、Bridge 路由测试。

- 相同目标 UUID、位置在原 .25m 身份容差内，收到更新的有效同 frame 图，明确报告非 frontier，才退役为 `frontier_resolved`。
- 使用现有五秒观测有效期，目标选定前、重复采集戳、乱序、过期、未来及不同 frame 的更新均不授权退役；每个目标保存有界的源戳水位。
- 节点暂时缺失、评分变动继续保持目标。明确对象/记忆目的地不进入这一探索失效流程。
- 退役沿既有 ID、取消与 quarantine 交接；不加 visits、不发布到达 ACK、不添加障碍或区域退避、不把它训练成失败地形。
- 正常运动时发生退役可能增加取消交接，需通过真实试验核对是否造成抖动或探索效率回退；不能仅凭组件通过排除这个风险。

## 验证

- 旧 236 实际 C++/DDS 节点上，新反例失败：节点已非 frontier 仍保持旧目标。旧消息隔离及对象目的地两项正常，保留 resolution_before.log。
- 本机全局 50 PASS / 0 FAIL / 6 WARN；447 项生产回调配 ROS 替身通过，不能当作 447 次仿真。
- 新实际 C++ 构建和路线测试通过；真实 ROS 交接/代价 51 项，既有归属/代价 DDS 9 项、范围 DDS 2 项、新失效退役 DDS 3 项均通过。
- 新 GraphNav 库 SHA db52f00e06d7c598ecba3e8c42d1b34019693cd879f03537a5a36e9f6e1c6b8f。Smac/RPP 库身份保持原验证值。
- 修复前 DDS 脚本第一次覆盖 PYTHONPATH 导致 rclpy 导入失败，仅修正测试环境继承后重跑；未改变生产实现以满足夹具。

## 录像与证据

第一轮真实双栏 RViz：`C:/Users/Yu/Desktop/拟真/captures/20261016_scope_baseline/rviz_live.mp4`，426.25s / 3410 帧 / 8fps，全解码通过；SHA 8237fa49a199147017cb340ea2ad446ce17c1adfa0014c9a2bc72c7af65feca0。右侧包含 camera，无独立 front_camera.mp4。

第一轮核心归档：test_artifacts/exploration_scope_repair_20261016/scope_baseline_field_evidence.zip，141 成员，SHA bfc858cabb255d969adea9fa5537db1c7b89f47da7fb7d87126a90428b7c1328。本机 CRC、140 有效载荷逐 SHA、录像一致性通过。3.264GB 原始 sensor DB 留服务器并保留路径、长度和 SHA，不称全部原始数据库已经取回。

新修复、测试及下一轮证据位于 test_artifacts/frontier_resolution_repair_20261016；冻结包位于 deploy/20261016_frontier_resolution_candidate。

## 修复后完整现场结果

相同条件的 session_20261016_200923 已关闭，driver、session、goal 三个终态存在后才分析录包。

| 指标 | 236 基线 | 274 修复候选 |
|---|---:|---:|
| 完整目标到达 | 未通过 | 未通过 |
| 搜索结果耗时 | 421.850s 超时 | 421.901s 超时 |
| Native 路程 | 205.121m | 123.653m |
| 最大离站 | 52.202m | 48.341m |
| 实际动作中止 | 3 | 0 |
| no_vehicle_path | 4 | 0 |
| 超过五秒的静止 | 6 段 | 14 段 |
| 最长静止 | 27.351s | 16.256s |
| 完整图路 / 路线决策 | 292 / 592 | 696 / 696 |

两轮初始条件和配置相同、修复候选只改四成员，但传感器与模型异步运行、实际轨迹不同；这是两轮诊断，不能作为成功率或严格统计因果结论。

新版本源图退役原因：37 次 frontier_resolved、9 次 vehicle_cost_selection；Bridge 收到并处理 34 条匹配退役状态。实际 source、库及配置均核验，失效目标释放路径已经现场发生。最长单目标约 91.964s，28.499m 实际行驶，最近 Native 参考距离 8.004m；该目标三份原始拓扑抽样仍为 frontier，不能把本阶段称为同一个“已经非 frontier 仍被持有”的旧反例。

但这轮 **21 条取消 FollowPath 请求、23 条控制器停车日志**（不同层事件，不能当作相同计数），14 段静止总约 109.852s，动作交接过多的问题仍存在。`execution_recovered=23` 是取消隔离解除相关状态，不是 23 次倒车恢复。代码中的普通探索退役目前仍走 reset/cancel；下一步应检查“探索意图作废”和“已验证运动段的安全交接”能否合理解耦，并覆盖旧动作终态、后台续接、对象/记忆接管、query/frame 改变及碰撞停止。不能简单关闭取消或直接放行旧队列。

mask、定位、确认、非空召回均为零，709 次实际评分没有因为测试关闭记忆而停用；不能根据零 mask 断言模型识别错误，更没有视觉记忆多目标到达验收。

新视频：`C:/Users/Yu/Desktop/拟真/captures/20261016_frontier_resolution_retest/rviz_live.mp4`，423.625s / 3389 帧 / 8fps，全解码通过；SHA c15ee1497fc4b21d89b80cead061b85a1414d20158fe9d5b8a9293cf9cb826ae。真实 RViz 右侧 camera，没有独立 front_camera.mp4。

第二轮核心归档 110 成员，SHA b76698eb3ce04edc370132345412f972928c4027beaa62a40f9c4b07d53486c5；本机 CRC、109 载荷逐 SHA、录像一致通过。3.295GB 原始 sensor DB 仍在服务器，外部 manifest 保留。服务器测试后独立 ps 核验相关进程零残留，候选与生产各 439 成员再次逐 SHA 通过；生产入口仍 e6，未替换。完整搜索仍失败，**不计 20 轮成功、不做多目标、不 commit，不宣称规划稳定性已修好。**
