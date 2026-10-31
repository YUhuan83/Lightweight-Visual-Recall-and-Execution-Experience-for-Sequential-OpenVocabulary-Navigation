# WildOS / BeamNG 排查记忆

更新：2026-10-16。这是本项目的持续工作记录；后续结论应按新增证据更新状态。密码、API 密钥等凭据不得写入。

## 最新状态（2026-10-16 探索范围修复候选）

- 详末尾新增记录及 `docs/EXPLORATION_SCOPE_REPAIR_20261016.md`。候选23675e28已完成离线和服务器隔离组件／同图对照，未部署、未做本候选真实车体搜索；现场生产仍e6f28118，完整实验未通过，未commit。
- 新拒绝策略只暂缓具体目的地及失败起点上下文，取消误推断的12m／8m区域和图边屏蔽；恢复增加有界意图及空闲重试，所有动作安全门保持。其它heading／覆盖／经验及树林地图来源问题仍未闭环，不重复宣布规划整体已修好。

## 当前活动状态摘要（2026-10-10）

- 真实SBR已恢复：车身/四轮碰撞和动力学开启，0固定节点；SmacHybrid/REEDS_SHEPP与受限曲率RPP使用8m规划半径，禁止原地旋转。源码修复65929a3、真实交接组件及证据69efff5已验证；完整绿色房屋3轮物理试跑均失败，按用户要求停止完整实验，进入严谨修复；20轮固定统计尚未开始。
- 最新用户要求：后续不录视频，以接口日志和少量目标到达图像定位故障、验收；达到20轮至少16次真实视觉/执行成功后，才进行绿色房屋→公交站→加油站等多目标。多目标阶段仍须同进程、同地图、不预先下发记住加油站指令。
- 当前实际换向点完成用StoppedGoalChecker的0.2m/0.35rad与实测停稳阈值；普通语义观察终点保留0.6m。不能从早期FIX-05的理想载体条目恢复旧物理交接容差。
- 两张Nav2代价地图必须保留障碍mark/clear观测及碰撞检查。FIX-04记录早期无观测源条件下的current修复，活动真实配置使用完整观测，不得退回无障碍输入。
- 每次改动必须有对应commit和相关测试，交付前所有必要验证通过；组件、离线与完整验收范围分别记录。最新无录像/制动诊断候选通过本机及服务器隔离检查，完整试跑效果待验。
- 以下“当前实验条件”等早期条目记录2026-10-08状态；活动运行、已验证范围与待修复事项同时查阅本摘要和末尾最新记录。

## 使用规则

- 开始排查前先读本文件，再读相关证据；历史配置不能覆盖最新用户要求。
- 用户新增要求：每轮现场测试前必须检查服务器状态，核对残留进程、重复 ROS 节点、控制来源和 PID 身份；发现残留后先核实精确命令行再清理。预检报告随该轮证据保存，未通过不得启动测试。
- 已验证的修复先检查运行进程、安装包、配置、源码哈希是否仍生效。出现新反例才重新诊断，不能仅凭同类症状回退已验证修复。
- 区分四个状态：现场组件已验证、仅离线验证、实施但完整验收未通过、待修复。编译通过、生成路径、发现掩码、object_found 日志分别只证明对应环节。
- 房屋/加油站共享通用的检测→定位或观察接近→全局路线→FollowPath→现场确认流程。禁止以固定目标坐标、特定 Town 地形规则、调低识别门槛代替接口修复。
- 每次新发现记录：症状、模块、原因、修复位置、验证条件、证据、未覆盖范围；旧结论用“已被后续证据更正”保留来源。

## 已确认的需求和当前实验条件

- 主代码：`C:\Users\Yu\Desktop\拟真`；辅助工作区：`C:\Users\Yu\Desktop\Beamng`。
- 要在 WildOS 基础上增加轻量视觉记忆：同一进程和同一次地图加载中，复用曾见物体的位置与地图，更快选择合理路线；不要求原路返回，不要求跨进程保存。
- 当前 Town 出生点：`(734.294, -11.946, 52.154)`，四元数 `(0, 0, 0.7658087977066911, 0.6430683364581344)`。
- 当前顺序：`green residential house` 真正到达后再查询 `gas station`；此前不预置要记住加油站的指令。两目标之间不重启视觉节点、不重载地图、不瞬移。
- 用户取消离站 20 米验收门槛；不额外限制车速。活动控制配置速度为 1.2 m/s，参数值不能代替实测速度。
- 当前诊断载体为 `ideal_point`：固定全部 ego 节点、关闭包括轮胎的碰撞、隐藏车体网格，Camera/LiDAR 保留。不是恢复物理车辆模式；碰撞关闭不会自动清空规划地图障碍。
- 当前 RViz 视频重点显示 RGB + Target，不要求显示全部 frontier/traversability；每轮保留 RViz 和前摄像头 MP4。
- A/B 比較需同条件、同共同修复、仅记忆功能开关不同。不能将历史修改版或用户命名为“原文”的视频直接认定为论文原版。

## 已修复并有证据的问题

| ID / 验证范围 | 原因及修复 | 证据与避免返工的检查 |
|---|---|---|
| FIX-01 现场固定路径 | ROS 正角速度与 BeamNG 理想载体旋转反号，导航原点补偿又按另一方向计算，造成约 `2×2.172m` 的横移项。Lua 使用 `rosYawToClusterSign=-1`、`world-z-passive-v2`，按真实轴补偿导航原点。 | `test_artifacts/server_validation_20261007/coordinate_probe/REPORT.md`：直行 3.641m，正/负旋转约 ±0.91rad、原点漂移 <0.001m；61 点固定路径 FollowPath 成功，最大横向误差 0.044651m。仅证明坐标和该路径，不证明完整搜寻。复查实际 Lua 标记与正负角速度响应，不能只看 YAML。 |
| FIX-02 现场目标握手 | trial3/4 在视觉模型初始化前只发一次 volatile query。规划端目标为绿色房屋，视觉端仍为默认 `door`。测试脚本现在等待 `wildos` 订阅建立，并等待同查询、发送后时间戳的视觉 ACK 才启用运动。 | `test_artifacts/server_validation_20261008/query_contract_review/REPORT.md`、`full_trial.py`。trial3 door 掩码 49 条、trial4 33 条，house 为 0；trial5/7 已收到 ACK。**trial3/4 无法验收绿色房屋识别，更不能证明模型能力不足。** 同配置回放 trial3 80/82/85s 可生成房屋掩码，但 MP4 是有损帧。 |
| FIX-03 回归及现场组件 | official adapter 的占用点按 ≤40m 过滤，但 terrain 使用未同步过滤的 world_points；极大有限输入旋转/float32 转换可使 terrain 非有限并异常退出。现以同一范围掩码过滤两份数组。 | `official_adapter/beamng_wildos_adapter/node.py`；服务器 `town_memory_validation_20261008/lidar_range_regression.json`、`followpath_probe.json`。合成生产回调原版失败、修复后成功；3m FollowPath 2.803s 成功，端点误差 0.563m。原崩溃帧未保存，不能断言其一定含 FLT_MAX。 |
| FIX-04 现场规划服务 | Nav2 Humble 1.1.20 清 costmap 后启用的 InflationLayer 不能恢复 current：关闭的 ObstacleLayer 没有提供更新范围，PlannerServer 等待 costmap，不受算法 max_planning_time 限制。保持障碍层 enabled、无观测源、footprint clearing 提供更新范围。 | `test_artifacts/server_validation_20261003/COSTMAP_READINESS_FIX.md`。同起终点清图前后对照：原配置 GridBased/CarLike 均超时；修复后分别 2.55ms/1.29ms 成功。避免重新采用“禁用障碍层但保留膨胀层”的组合。 |
| FIX-05 离线确定性修复 | 换向段完成容差 0.6m、maneuver_join 却默认用 0.3m；14 次 actual_cusp_pose_differs 在 0.5606～0.5896m，被提前拒绝。将位置检查绑定同一 0.6m 常量。 | `test_artifacts/server_validation_20261006/stall_physics_findings.md`，12 项运动段检查通过。后续曲率、朝向和扫掠仍会拒绝不安全接续；当前 ideal_point 不应误恢复 Ackermann 换向约束。 |
| FIX-06 离线接口修复 | 记忆候选校验接受可转 float 的 JSON 字符串，但后续仍保留字符串，距离计算可能失败。统一转换 object_position/radius/FOV。 | `test_artifacts/offline_navigation/five_layer_audit_20261006.md`；修复前新增用例失败、修复后通过。历史录包没有证明现场收到过这种字符串。 |
| FIX-07 离线配置修复 | 运动后端接受修改 carrier_model/minimum_turning_radius/max_angular_rate，但缓存实现不更新。对需重启项明确拒绝，参数批次整体校验，防止拒绝时部分启动车辆。 | 同上五层审计；参数更新成功返回不代表实现已更新。预检运动模型和真实运行模式。 |

## 测试基础设施已处理的故障

- BeamNGpy 重连 `StartVehicleConnection` 响应缺少 `result`，导致 KeyError：改为使用已核实的车辆连接端口。端口是运行态信息，不应永久硬编码旧端口。
- 重生 ego 后碰撞旁路配置可能丢失：检查真实节点数、零碰撞掩码、fixedNodeCount、profileAvailable、相机/LiDAR句柄。2026-10-08 检查为 723/723 节点固定且掩码 0、125 网格隐藏。该数量属于当前车辆，不是任意车型的通用常量。
- Lua 热重载清空 reports，而 JBeam 初始化不重跑：start 根据 verify 重建当前节点报告；见 20261007 coordinate_probe。
- 重复/跨日旧控制、传感器桥进程和端口监听会导致连错/多控制源：先核实精确 PID、模块命令行、端口、源码哈希；按身份清理，禁止用宽泛 `--ros-args` 匹配杀进程。
- venv 下 nvidia_radio 的 symlink install 清理失败：该包单独非 symlink 构建。colcon 指定 base-paths 避免 reference/repro 同名 visual_navigation 包重复发现。
- 新测试误用不存在的 `general_goal_checker`：使用该运行配置实际存在的 `maneuver_goal_checker`；启动拒绝不能记成 FollowPath 算法失败。
- trial3 harness 将 `telemetry_age_sec=None` 误作 3.1s 超时；改为保留最后有效遥测时间并实测缺口，故障需明确控制错误支持。未放宽生产端 0.5s 新鲜度保护。
- **录制中不可打开/查询 rosbag SQLite 做诊断**：trial5 曾因本次实时读取引发 database locked，记录器退出。使用 harness JSONL/状态日志；bag 停止后再分析。trial5 MP4 与 harness完整，rosbag 仅部分；91 是 harness 的掩码计数，71 是部分 bag 计数，不能混用。
- verify_experiment 要求干净的测试 worktree：trial6 因源码未提交导致视觉启动被拒绝，未下发搜索目标。这类启动失败须与导航失败分开。

## 已实施但完整流程仍未通过

### 2026-10-08 通用可见目标观察接近

- 只有 2D 掩码/方位且缺乏稳定 3D 定位时，新增 `visible_target_approach`：多帧新鲜且方位一致，沿可见方位的已知自由空间给出有界观察步；仍经过 GraphNav/全局规划/FollowPath，不将临时点当成目标已到达。
- 新临时目标执行期间锁定，防止每帧取消路径；超时退回探索并短时避开重复失败点；记忆路线和已确认目标有优先权。
- 三角定位 view 增加时间戳和 5s 过期清理，禁止无限混用旧视角。未证明已解决所有大型物体或多实例关联。
- 关键文件：`official_adapter/beamng_wildos_adapter/visible_approach.py`、`goal_standoff.py`；`nav2_adapter/wildos_nav2_adapter/visible_approach.py`、`graph_path_bridge.py`、`target_observation.py`；权威 triangulation 位于 `wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py`。
- 2026-10-08 本机/服务器模拟 ROS 回调 220 项通过。真实 partial bag 回放产生 75 个临时观察提案、0 个确认目标；不能当作仿真到达成功。
- 服务器派生修复 worktree：`/home/ly/wildos_ab_20261008_B_fix`，commit `aeac77aa7be5ae011d93c91e5bd55e41936015fd`，branch `experiment/shared-visible-approach-fix`；前一 LiDAR 修复 commit `38e877b`。**这不是论文原版或正式已完成的 A/B 对照。**

### 轻量被动记忆

- `semantic_landmark_memory.py` 无预设目标类别采样少量特征+几何支持；默认 1024 条、1152 维 FP16，核心数组约 2.326MiB，数组预算 8MiB，临时 LiDAR 缓冲 ≤2MiB。数字不包括模型/GPU/地图/ROS/对象开销。
- 不存 RGB 视频/完整特征图，不跨进程保存；查询切换保留记忆。记忆位置只作候选，新查询后的现场观测才可确认成功。
- `test_artifacts/server_validation_20261003/lightweight_memory_repair.md` 有实现与验证边界。曾生成记忆路线、选择候选不等于完成返站。

## 未解决问题：后续优先处理

1. **确认接近点更新导致重复重规划**：trial7 记录 56 次 object_standoff_approach，原代码每次 standoff 位移 >0.6m 就 reset。已新增 `ConfirmedTargetApproach` 锁定健康路线、延后更新，处理完成终态、候选比较隔离和 1m/0.6m 终点死区。首轮本机/服务器234项通过，commit `2a77d4607c508b95da55faf77dd80cf8ed6b766e`；trial8 house未到达，但没有FollowPath失败，暴露“延后提案未交接、生成端已更新last_goal”的缺口。后续缓存已验证提案，在当前段完成后接续导航，绝不作为新鲜到达证据；本机/服务器236项通过，服务器commit `6822cbfbd690ba62cd6c59dacea0259ec8f4a580`，trial9单独实测。证据：`test_artifacts/server_validation_20261008/confirmed_approach_patch/REPORT.md`。
2. **记忆区域与新确认不一致**：gas station 阶段出现 `confirmation_outside_memory_region`。记忆定位多个局部立面/实体点可能与新确认不是同一点，候选区域关联与退出/重选状态需诊断；不能直接无限扩大半径或关闭关联校验。
3. **定位不稳定**：有效 trial5 全程有 house 掩码，但部分 bag 的 17 个位置回放均未形成满足条件的稳定簇，confirmation=0。不是“没有看到”也不是 FollowPath 首先失败。研究时间同步、实例关联和几何观测质量，模型能力短缺尚不作专门适配。
4. **代码部署一致性**：`wildos_port/patches/overlay` 的 triangulation 副本较旧，当前运行使用 repro_check。避免用旧 overlay 重建覆盖新代码；同步副本须按语义差异单独处理。
5. **时间同步边界**：相机/odom 0.05s、mask/LiDAR 0.1s 等保护已有离线测试；真实 DDS/message_filters 漏配与时延仍需数据。源时间到 ROS 的常数偏置不是延迟，bag接收年龄不是控制回调年龄。不能把所有停驶归因于通信负载。
6. **不应混淆物理模式**：历史物理车存在轮胎悬空/空转证据；理想点已旁路牵引和车身碰撞。继续停驶要首先沿路径状态和数据流诊断，不能直接将灌木/车辆物理作为通用根因。

## 最新验收结论和证据索引

- 20261007 房屋阶段曾成功到达，但 harness 后续崩溃，独立站点续测失败。`captures/green_house_success_20261007/wildos_原文测试.mp4` 是用户重命名，不代表论文原版。
- 20261008 trial3/4：目标 query 丢失，属于无效 house 验收；trial5：query正确、有掩码，无严格到达。
- **20261008 trial7：原自动验收 passed=false，报告 house=true / station=false；后续录包回放发现验收脚本漏报。** 有界历史精确匹配可恢复 house 和 station 到达，station 的候选编号一致，standoff 0.476m、物体3.430m；房屋近景为浅绿色墙板、station视频约641s可见棚顶/燃油泵/站牌。原报告保留，不回写成功；修复脚本后另用 trial8 现场验收。详见 `test_artifacts/server_validation_20261008/trial7_review/REASSESSMENT.md`。
- trial7 地图重载0、两阶段瞬移0、station_preseeded=false、vision_process_retained=true；记忆选中约1.181s、首次路径约1.195s，证明调用/路径生成速度，尚不能证明成功找到加油站。
- trial7 开始后、首个掩码前手动重启 triangulation 并重发同一查询；应保留此运行偏差，下轮所有关键节点纳入统一启动。
- 服务器 trial7：`/home/ly/town_memory_validation_20261008/B_green_house_station_trial7`，包括 `full_test.json`、视频、bag及日志；不要把“house日志到达”写成“两阶段成功”。
- 本机主要证据根：`test_artifacts/server_validation_20261008`；历史源：20261007 coordinate_probe、20261006 five_layer_audit/stall_physics_findings、20261003 COSTMAP_READINESS_FIX/lightweight_memory_repair。

## 开测前最低检查顺序

1. 核实 Git commit/干净工作树/运行源码哈希和活动配置；冻结的 A=`ddbd491d2640e8581c25d253f9ceb8eaf796dd38`、B=`2889389ab9806a2b67c50be3edde9e3718e2740c`，共同修复另建版本。
2. 精确检查单一运动后端、真实载体模式、角速度符号/导航原点、传感器源时间与帧、odom/地图新鲜度。
3. 视觉订阅就绪→query 发布→同 query 新鲜 ACK→允许行动。记录有效模型 query，不能只看测试脚本目标名称。
4. 先小路径验证 ComputePath 与 FollowPath，再完整任务；组件成功不能跳过目标语义、3D位置、到达截图验收。
5. 同地图/同视觉进程切换目标，记忆候选≠确认；保留完整视频、harness、状态、最终结果；录制停止后再读 bag。
6. 按需要运行 `tools/offline_callback_check.py`，检查 JSON 报告，不只看 PowerShell 最后命令退出码。本机无真实 ROS，不把 fake 回调测试当现场实测。

### 2026-10-08 trial9 及有界续测结果

- 服务器版本 `6822cbfbd690ba62cd6c59dacea0259ec8f4a580`。原 trial9 的 house 搜寻 420.385s 超时，house=false，station 未下发；同地图、同视觉进程且未预置 station，完整双目标验收未通过。现场状态已显示多次延后接近点在旧段后交接，但不能据此宣布目标到达。
- 独立 `B_green_house_station_trial9_continuation` 保持地图、视觉 PID 和当前位置，追加 house 150.107s，仍无严格到达，未切换 station。续测日志 mask_count=0，末段位置保持 `[744.0688,-109.9725,52.1540]`，状态为 execution_ready；尚未定位此续测的数据流停滞原因，不能推断为模型识别失败或 FollowPath 算法失败。原 trial9 失败结果保持。
- 结果证据：`test_artifacts/server_validation_20261008/confirmed_approach_patch/trial9_result_summary.json`；服务器原始日志 `/home/ly/town_memory_validation_20261008/runner_continuation9.log`；两轮独立目录保留。
- 已取回 trial9 前相机和 RViz MP4：`captures/20261008_B_green_house_station_trial9/front_camera.mp4`、`rviz_live.mp4`。视频可查看，尚未完成本轮逐帧语义验收。

### 2026-10-08 用户指出trial9约5分30秒目标可见

- 已检查前相机330/335/345/420s和RViz330s：335/345s明确可见远处浅绿色住宅，420s更近；不能把本轮超时解释成全程没有看见房屋。两段视频同秒不同画面，前相机固定5fps写帧与harness时钟存在差异，尚无精确逐帧PTS对齐。
- harness349.498s发出visible_target_approach，349.705s规划7.924m、62点，349.712s进入Following；373.100s首个3D确认的接近点因地图裁剪距估计物体17.130m，后续确认位置明显变化，443.469s仍距估计物体18.787m。位置是否同一目标尚未核实，不能仅凭日志确认语义成功，也不能说完全没有规划。
- 优先核对掩码→物体几何关联、地图裁剪接近推进和端点交接。本次仅取证，无修复。证据 `captures/20261008_B_green_house_station_trial9/review_330s/REVIEW.md`、关键帧和timeline.json；服务器完整events.jsonl保留。原失败结果不变。

### 2026-10-08 从底层检查：语义区域与物体实例的契约缺口（尚未修复）

- 用户现场视频观察：绿色房屋被识别，同时蓝色房屋也被标为绿色。记录为用户指出的语义误检；尚未建立每个掩码像素与原始RGB/三维坐标的精确对应，不能断言全部定位跳变来自蓝色房屋。
- 已确认代码行为：ObjectMaskWithTf只有query和每相机二值掩码，没有实例ID或语义分数；supported_mask_components保留多个合格连通区后合成二值掩码。visible_direction对整张掩码求像素中位数；triangulation add_views对整张掩码求bbox和中位射线；foreground_depth在整个掩码内选择最近且点云支持充分的深度面。这不等价于同一物体实例，也不验证颜色语义。
- GoalStandoff以位置聚类形成确认，quality.confidence_kind明确为geometric_support；query字符串表示搜索指令，不能当作分类正确的证明。新ConfirmedTargetApproach已锁定单个健康路线，但confirmed_instance_matches在graph_path_bridge仅busy时拒绝异实例，route_confirmed_target接续会刷新对象锚点，仍不提供整次任务的实例跟踪。
- 真实停止录包统计：trial9收到86条object_mask，其中51条包含至少两个分离区域（每区>=50像素仅为诊断统计门槛，无生产参数改动）；这不证明每个连通区是独立建筑，也不证明具体蓝色建筑造成失败。
- 合成输入调用当前生产visible_direction：两个相隔的区域中心u=14.5/84.5，全掩码中位像素u=49.5落在掩码外，仍输出0rad中间方向。证明该方位计算存在可复现多区域反例；不等于trial9确切根因。证据 `captures/20261008_B_green_house_station_trial9/review_330s/mask_components.json`及`instance_contract_counterexample.json`。
- 通用修复方向（尚未实施）：保留区域级候选及语义/几何分开的证据；跨帧关联实例后再求方位/深度/确认；明确实例锁定、失配、失观和重选的状态契约。不能靠green-specific颜色门槛、固定坐标、调整单个场景阈值解决。实例管理也不能单独修复模型的颜色语义能力。
- 用户要求：从底层定位通用问题，再修复；不为这一栋房屋做适配。本次仅检测/记录，没有改变生产代码或参数。

### 2026-10-08 通用语义竞争与目标实例锁定已实施（未验证）

- 用户要求修复底层通用问题。WildOS 查询阶段目前只对 traffic light 启用负类竞争；加入从带颜色属性的任意文本查询生成同名词、替换颜色的 counterfactual prompts，并对每个特征单元要求目标相似度高于对照；复用原 mask threshold 和五格最小支持，语义margin默认0且接受可配置的非负值。它可排除模型把蓝色区域同等或更强地响应为绿色的情况，但不能保证纠正模型对颜色本身的稳定错误排序。
- 对定位聚合：在可见目标方位和 LiDAR/多视角定位前，每台相机的掩码仅取最大连通区域；每次观测只取支持像素最多的一台相机，避免无实例ID的多个区域或不同相机目标混合成中间方位/位置。多房屋同时可见时该策略是候选选择启发，尚未证明始终选最近/最相关房屋。
- 对接近状态：GoalStandoff在首次形成可执行确认时锁定固定 query-local 物体锚点，后续位置必须满足已有通用 `3m + 两次几何半径` 关联门；切换 query 清除锚点，防止相邻建筑确认将锚点逐次漂移。
- 文件：`wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/utils/object_search_utils.py`、`.../utils/semantic_mask.py`、`.../wildos/nav.py`、`.../explorfm_triangulation/obj_mask_triangulation.py`、`official_adapter/beamng_wildos_adapter/visible_approach.py`、`goal_standoff.py`。
- 当前只完成源码修改和代码阅读，未运行测试或现场实验；更未证明 RADIO 对绿色/蓝色复播纠错。上线/完整搜寻待离线检查和现场验收。局部遮挡、多个相连实例、多相机视角下最大区域选择与短暂离开实例范围都需关注。
### 2026-10-08 trial10现场失败、更正统计与强制服务器预检

- 生产派生版本为 `70c4b2e6e25015f01b9758a6f75b7d023d95004f`（通用语义竞争/区域选择/固定锚点），服务器干净工作树，构建与离线门禁通过。此前“仅源码未测试”是当时状态；现已现场测试，但完整验收失败。
- `B_green_house_station_trial10_color_fix_retry2`：query ACK正常，地图重载0；约160.8/162.0s形成LiDAR几何确认，最终接近点约距锚点3m，179.8s路径完成，185.1s无新鲜到达确认而恢复探索；房屋搜索420.35s超时，未切换加油站，passed=false。启动失败轮次与此有效轮次分开记录。
- 更正：harness旧字段 `mask_count` 是累计收到的掩码消息数，不是当帧有效像素数。不能将143解释成143像素，后续增加 `latest_mask_pixels`、消息计数和源时间戳。
- 更正：trial10视频434.2s而事件总耗时442.2s，固定5fps写帧遇阻塞丢失时间。视频PTS180s不能直接对应事件elapsed180s；旧“接近结束时掩码消失”的结论缺乏精确对应，撤销确定表述。离线回放PTS160s支持绿色立面，PTS180/200s无掩码，只适用于这些视频帧。
- 更正：ROS导航位置和BeamNG native参考点有约2.172m纵向杆臂。相近录包段ROS端点误差约0.53m、物体约3.51m，不能用native点直接判定0.6m端点容差失败。精确到达失败来源仍需同步RGB/掩码/里程计证据。
- 服务器预检已实施于本机 `tools/server_test_preflight.py`，部署 `/home/ly/town_memory_validation_20261008/server_test_preflight.py`；run_trial在录制/运动前强制执行。检查域40关键进程与PID登记、直接rclpy节点发现、重复节点、单一传感器发布源、cmd_vel来源、运动已停、Nav2活动配置哈希、BeamNG端口和磁盘。仅记录/阻止开测，不自动按模糊命令杀进程。
- 清理前逐个核实命令/PID，清掉7个跨日孤立诊断进程与旧RViz；保留当前RViz显示`:137`。先前口述`:1337`是终端换行误读，原脚本已用`:137`，现改为从运行RViz读取显示号。CLI缓存节点列表可能失真，预检使用直接rclpy发现。
- 新trial11开测前报告passed=true、剩余约102.3GiB，报告和清理证据随轮保存。测试基础设施增加 `front_camera_frames.jsonl`（PTS/事件/ROS源时间映射）与同步 `mask_snapshots`，不改生产判断阈值。trial11仍在运行，尚无最终结论。
- 证据：服务器 `/home/ly/town_memory_validation_20261008/B_green_house_station_trial10_color_fix_retry2` 与 `B_green_house_station_trial11_synced_preflight`；本机trial10视频/关键帧在 `captures/20261008_B_green_house_station_trial10_color_fix_retry2`。录制中的rosbag不读SQLite。
### 2026-10-08 trial11同步取证：接近目标后的观测几何失败

- 开测前强制预检通过，无检测到的重复关键进程/控制源/旧录制；本轮生产仍为70c4b2e，未调整车速、到达阈值或视觉模型。全流程house阶段420s超时、station未下发、passed=false；地图重载0、过程瞬移0、视觉进程保留。不能宣布视觉记忆返站通过。
- 现场同步快照：elapsed183.204s掩码覆盖浅绿色住宅墙板（45035像素），187.774/189.942s等待确认时仍有掩码（13633/11890像素）。源RGB和mask同时间，不再使用MP4同秒猜测事件位置。前相机432.6s、RViz443.8s均取回并可解码，RViz显示RGB+Target。
- 执行186.773s route_done并awaiting_visual_confirmation，192.104s无新确认退回探索；最后有效几何观测源时间1791461271852502420。此后triangulation回调仍接收掩码，但LiDAR定位无输出；多视角回退因停驶、baseline约0.14m跳过，五秒过期后仍缺足够视角。
- 停止rosbag后离线投影复现：源时间1791461271852502420时624个点落在相机图像内、183个命中所选区域，生产foreground_depth成功；1791461272278002441时仅8个点落图像内、目标命中0；随后1791461273064502478至1280347502824的10条观测落图像点为0、深度无输出。这些最近LiDAR扫描与mask源时间偏差为0，不能将该段失败笼统归为时间戳延迟。整张mask的深度同样失败，不能单独归因于最大区域选择。
- 新几何契约缺口：实际相机相对ROS导航原点body偏移为[3.672,-0.330,2.350]m；等待确认时相机世界位置[748.7043,-130.8283,54.5040]，此前稳定墙面估计y约-130.0846，记录的相机已越过该墙面。当前GoalStandoff.approach_distance在ideal_point且front未显式配置时按0前方尺寸，3m接近目标可让保留的前置相机进入/越过目标表面；关闭载体碰撞未消除传感器空间偏移。这是本段有证据的观测几何失败，尚未证明每次失败都同因，也未验证传感器TF与真实采集位姿在越面后是否完全一致。
- 后续通用修复方向：把运动碰撞尺寸与观察传感器偏移/观察净空分开，按真实标定/TF设计可观测接近点；联合检查FollowPath停点容差、相机有效视场、LiDAR投影支持和到达状态切换。不得硬编码此房屋坐标、仅放宽成功距离或拿过期几何作新鲜确认。尚未实施此新修复。
- 本机证据：`captures/20261008_B_green_house_station_trial11_synced_preflight`内full_test/events、front_camera_frames、mask_snapshots、front_camera.mp4、rviz_live.mp4、server_preflight及清理报告；depth_diagnosis.json与trial11_depth_diagnosis.py为停止录包后的离线复现。服务器同名目录保留原rosbag，不复制数GB录包到本机。
- trial11结束后服务器复查也passed=true；enable=false、follow_path=false，未检测到残留测试/录制进程。证据服务器同轮server_after_test.json；本机临时传输归档已清理，保留展开后的证据和视频。
### 2026-10-08 trial12重复现场测试与真实相机位置核验

- 用户要求继续测试。开测前强制服务器预检passed=true（约96.9GiB），版本仍70c4b2e、干净工作树；同Town出生点、同query顺序、无生产参数修改，录像同步索引保留。该条件相同不代表随机探索轨迹相同。
- house阶段约420s超时，未切换gas station，完整验收仍失败。有效query发送21.736s、vision_query_accepted22.051s；模型进程正常接收相机并输出未检测到目标日志。本轮沿另一条探索路线行驶，尚未产生目标mask/3D确认。不能用本轮验证trial11接近结束故障已修复，也不能把这轮无mask归因于query丢失或FollowPath完全不动。
- 新只读诊断：通过BeamNGpy Camera.get_position/get_direction、Lidar.get_position和GE车辆position/direction/up获取真实位姿，不创建/移动传感器、不修改控制。辅助probe在运行中补充仅odom条件后重启一次，仅影响该诊断日志；完整trial未重启。
- 停驶并暂停后3次测量完全一致：相机相对按现有SBR_BASE_OFFSET得到的导航原点为[2.751995,0.0000083,2.209999]m；Adapter.camera_offset默认仍为[3.672,-0.330,2.35]m，位置误差0.987378m。相机朝向与车辆前向差约数微弧度。配置nominal安装点不等于BeamNG真实挂载后位置，已确认相机位置标定不一致；原因细分（挂载吸附/参考原点等）尚未确认，不改变已验证导航原点/角速度修复。
- 更正trial11：记录TF相机越过估计墙面，不能再当作真实相机越面事实；真实挂载测量说明原TF不准确。按trial12实测body偏移对trial11停止录包做对照重投影，末段落在图像的LiDAR点从0恢复到约177–178，但所选目标区域命中仍0、整张mask命中仅3–4，foreground_depth仍失败。标定偏差是真实缺陷，单独纠正它尚不足以恢复该末段确认；近距离观测重叠/语义区域/定位支持仍需联查。此次仅离线对照，未部署标定修复、未现场复测修复版本。
- 预检已把sensor_pose_probe.py和stationary_sensor_calibration.py加入残留测试进程检查，避免新诊断工具留连接后直接开下一轮。本机语法检查通过；现场结束后同一预检再检查。
- 证据：服务器 `/home/ly/town_memory_validation_20261008/B_green_house_station_trial12_repeat_geometry`；本机对应 `captures/20261008_B_green_house_station_trial12_repeat_geometry`。关键文件stationary_sensor_calibration.json、sensor_pose_probe.jsonl、trial11_depth_diagnosis_calibrated.json、full_test.json、events.jsonl、server_preflight.json、前相机和RViz录像。尚未进行源码修复或宣布两目标成功。
- trial12结束后预检passed=true，enable=false/follow_path=false，未发现残留诊断进程；两段MP4取回并解码。视频PTS180s树丛、300s道路/田野（不是同秒事件位置），所查帧没有绿色住宅。补充离线对照：源时间1791461272278002441目标命中0→643、1791461273064502478命中0→15，均恢复foreground_depth；最终静止段仍无足够命中。trial11本机报告已附更正，原失败结果保留。

### 2026-10-08 仅分析：目标失观恢复与实例锁生命周期缺口

- 用户要求先分析，不修改代码、不新开测试；新分析报告 `test_artifacts/server_validation_20261008/mechanism_analysis_20261008.md`。
- 代码核查：无记忆候选的等待超时只清路径桥状态并reset GraphNav，GoalStandoff固定实例锚点仅不同query时清除，缺少放弃实例→允许重选的交接。恢复探索不等于上游解除实例锁。
- trial11停止录包只读统计：恢复探索后138条原始定位中130条超首个ready锚点关联门约3.709702m；8条门内，后续无新confirmation。130条的语义是否正确未验收，不据此放宽关联门。原始统计服务器trial11/post_resume_anchor_analysis.json。
- 等待段elapsed187.774s，ROS到standoff0.516617m、到先前物体XY3.193457m；按实测挂载修正的相机距估计墙面法向约0.234m，最终仍没有足够mask内LiDAR命中。可达终点缺少可观测性保证；LiDAR失效时多视角需要baseline而停车等待没有新视角。
- 时间证据核对：过滤点云header使用matched state时间，header相同不能单独证明原始采集同步；此次另查1791461273–1280s adapter_record，LiDAR/state/camera各59条published，raw lidar/pose skew均0，10条mask/LiDAR配对。该段不支持将时间戳延迟作为首因。
- trial12约10秒粒度的42条进度采样，41条速度>0.3m/s，不支持整轮FollowPath不动；该粒度不能排除短暂停驶。探索评分/植被区域选择原因仍未定。
- 部署一致性：统一LF后本机bridge、standoff与现场哈希一致；vision仍有4个函数源文本差异，已核实服务器运行文件、评分/heading函数源文本一致。不能用第一次跨Python版本的AST dump差异作依据，也不据文件哈希不等直接认定功能故障。后续以现场版本为基准核对。没有修复或新增完整验收结论。

### 2026-10-08 处理：传感器净空、主动观察与实例释放（仅离线验证）

- 按用户“处理”实施源码修改；详情 `test_artifacts/server_validation_20261008/geometry_lifecycle_fix/REPORT.md`。此前“仅分析未修复”是历史状态，本次更新为已实施、Python仅离线通过；未部署、未进行新现场测试，不修改trial11/12失败结论。
- Adapter/GoalStandoff/GraphPathBridge共享当前实测SBR相机偏移[2.751995092,0.000008318,2.209999084]m；保存导航原点/yaw/虚拟点云修复。传感器观察净空独立于碰撞footprint，至少杆臂+1m观察范围+0.6m停车容差（约4.352m）；支持距离过近时地图检查后退让。不能把平面净空当作完整可观测性保证，不降低定位/成功阈值，不调速度。
- 新鲜确认缺失时最多4个新视点、每个导航30s有界，经原全局/局部规划与地图校验、终点面向目标；旧几何仅用于导航，不能宣布到达。
- 稳定instance_id + release/ACK统一放弃/重选生命周期；未收到匹配回执前不clear GraphNav，重试幂等且不清新实例。消息按查询/实例/请求身份及时间界限校验；旧clear服务回调也检查释放时间界限。记忆重选保留该事务，地图和同进程记忆保留；等待状态为unconfirmed_target_release_pending。
- 新增几何参数热修改拒绝和服务器预检三节点标定/两规划端最小观察范围一致性检查，后者真实服务调用尚未现场验证。
- 额外源码发现：GraphNav planner.cpp的is_scored_graph跨节点累积，缺少分数的一个frontier使后续有评分节点也退回距离评分，选择受遍历顺序影响。改为逐节点判定，非有限bin作该节点无分；新增C++节点顺序回归。本机无ROS C++编译环境，该项未编译运行；尚未证明是trial12选树丛的根因。
- 离线：250项生产回调接口替身检查通过，包含14项新增回归；综合43检查37PASS/6WARN/0FAIL，部署包已刷新。证据callback_report.json、report.json与geometry_lifecycle_*日志。Windows默认临时目录权限导致SQLite测试失败，用项目内TEMP/TMP后通过；多目标AST测试缺已有color_counterfactual_queries依赖已补齐，非现场模型故障。旧合成到达用例配置同点相机/4m距离，真实传感器约束另测。
- 待办：现场70c4b2e基线移植/构建/哈希、C++顺序回归、真实挂载及DDS回执检查、小路径、完整house→station及trial12实际评分候选还原。不能整包覆盖本机/服务器存在4个函数差异的vision nav.py；本次未修改视觉模型/该文件。完整两目标验收仍未通过。

### 2026-10-09 稳定性审查：新增回归与服务器隔离验证

- 用户允许服务器，并质疑修改引入新 bug。本轮确认昨日待部署候选有两个新增运行缺口：①生成端锁定实例但消费端未获ID时，空ID释放被拒绝、消费端等待ACK，导致挂起；②新确认接管路线未清主动观察旧计时器，后者可取消新路线。均有修复前后反例，不能再仅凭250项替身通过判定部署就绪。
- 修复：`target_observation.py`请求保留固定`observation_fence_ns`；`goal_standoff.py`只清界限前未知实例，对已缺席旧锁ACK但保留新实例，重复幂等；`confirmed_approach.py`在实际提交新确认路线时清旧观察状态/目标/截止时间。查询、时间、身份校验及所有既有坐标/query/costmap修复保留。
- 新C++评分顺序用例误将临时Eigen对象传给非const引用，服务器构建才暴露；改为局部变量。测试代码编译故障不等于现场车辆故障。候选隔离构建成功，23/23 gtest通过；此前“C++未编译”现已补充验证。
- 本机253项生产回调替身通过；综合43项37PASS/6WARN/0FAIL，部署包刷新。服务器隔离ROS_DOMAIN_ID78两反例修复前复现2、修复后0；仅实例释放采用真实DDS request/ACK，动作/清图为替身，观察超时为直接回调。不能宣称真实FollowPath或完整目标搜寻已验证。
- 活动基线仍`70c4b2e6e25015f01b9758a6f75b7d023d95004f`、工作树干净，没有本次新增释放/主动观察计时源码，故这两个新增缺陷不能解释trial10–12。Nav2/载体旧工作区路径与部分新适配器并存须核对实际导入，但不能仅凭cwd断言版本故障。本轮未部署、未重启活动栈、未开车辆运动或完整搜寻。
- 前后服务器预检均`passed=false`，原因只有RViz进程/节点缺失；两控制开关false，残留测试进程0。完整两目标验收仍未通过。当前主要机制风险为分散状态归属、旧计时/取消权限和异步ACK交接，不能将所有停驶泛归因于BeamNG负载、ROS延迟或轮胎物理。
- 证据：`test_artifacts/server_validation_20261009/stability_audit/REPORT.md`及`server_evidence`（真实DDS前后JSON、C++XML/构建日志、基线与前后预检）；服务器`/home/ly/town_memory_validation_20261008/stability_audit_20261009`。工具`tools/audit_stability_boundaries.py`只用于隔离反例，不驱动车辆。昨日修复报告追加更正，历史现场失败保留。
- 下一步：以活动基线逐文件移植冻结候选，避免整包覆盖vision四个函数差异；真实挂载/安装来源预检、恢复RViz、小路径和动作交接后再完整house→station。此验证尚未执行，不把本轮隔离通过写成部署验收。

### 2026-10-09 冻结候选部署及 trial13 现场失败

- 已按全局回归要求，由活动70c4b2e逐文件移植19文件并构建三包；核心commit587b8d0，预检工具增强后的冻结现场版本`d62752a582fd92a9f11db8439d12ab29aa9db2b4`，服务器工作树干净，Python导入和planner实际加载库来源核实。vision nav.py未覆盖。本次改变前一“未部署”状态，完整验收仍未通过。
- 开测前发现旧BeamNG崩溃存留：端口监听/ROS节点/心跳不能证明活性。增强`tools/server_test_preflight.py`，实际采样camera/odom/native并检查遥测、状态、执行器健康。精确恢复模拟器、实际车辆端口38197及all-fixed-v1；723节点固定/碰撞mask为0，camera/LiDAR保留。序列开始前恢复地图不计为序列中重载。
- 真实相机偏移与三节点配置一致，最终静态标定误差约0.046mm。现场GridBased 3m ComputePath成功25poses；3m FollowPath成功约2.907s、ROS端点误差0.549m。仅组件通过，不代表目标机制和完整流程通过。
- trial13绿色居民楼搜索约392.784s后no-progress终止，最大离站102.537m，无严格到达、加油站未下发；同地图/视觉进程、无预置station、无序列内瞬移/重载、无调速。0个object_mask、0确认/观察接近，未进入新目标接近和记忆返站分支，不能用这轮验收它们。
- 26次planned_path、24次following、21次segment_done，无follow_failed状态。elapsed369.774s旧探索路线完成，此后约62s静止；GraphNav连续报告`NO FRONTIER IN PATH!!!!!`，无新路径。最终nav_graph仍有166 frontier（1878节点、32300边），原始有限非负边无向诊断发现current idx64分量仅10节点、0 frontier；尚未定位断开原因，下一步沿建图/建边/当前节点归属/恢复契约分析，不直接关闭保护或堆补丁。
- 更正本轮中途口述“末段数据流异常”：停止录包后核对，测试末段odom/局部图/RGB/native仍更新，telemetry/state/actuator=true，无control_error。停止并暂停后的预检发现adapter/输入缺失、遥测过旧及DDS SHM报错；该结束后异常不能当作末段停驶原因，下轮需先恢复实时数据再通过预检。enable/follow_path均false。native与ROS导航杆臂仍需区分，ROS最后路径端点误差约0.566m。
- 证据`test_artifacts/server_validation_20261009/geometry_rollout/REPORT.md`及server_evidence；服务器trial13目录`/home/ly/town_memory_validation_20261008/B_green_house_station_trial13_geometry_lifecycle_20261009`。两MP4已取回`captures/20261009_B_green_house_station_trial13_geometry_lifecycle`并确认可解码，RGB时间索引留存，原rosbag留服务器。
- 用户持续要求：以后每轮失败，主动展示本轮RViz与前相机MP4，提供可发送的本机绝对路径，记录失败阶段；如录制损坏/缺失须明确说明。已写入`C:\Users\Yu\.codex\AGENTS.md`。不得用旧轮视频替代或把失败录像称作成功。

### 2026-10-09 trial13 林中行驶：已有配置遗漏与验收范围更正（仅分析）

- 用户指出trial13驶入树林，要求核对原文可通行性设计与关闭物理的影响。已查看前相机PTS120/240/360s，均在树木/枝叶间；这些PTS不直接等于事件elapsed。关键帧`captures/20261009_B_green_house_station_trial13_geometry_lifecycle/forest_review`。
- 已确认该轮使用的`nav2_adapter/config/nav2_official_control.yaml`哈希为`2f3e6ff269e7362ec50ab201a72e2fd4d4b0b0bb6d5ef0274fa6fafc525c5df8`，与轮次server_preflight和当前服务器文件一致：FollowPath `use_collision_detection=false`；两costmap `observation_sources=''`、无static层、`track_unknown_space=false`，robot_radius=0.01m。桥接器的global/local路径检查读取Nav2 costmap，启用检查并不补上缺失的障碍数据。Adapter上游仍从LiDAR形成local_map/terrain，不能说所有层的可通行性均关闭。
- 这些是此前已存在的理想载体配置，离线检查也明确输出`obstacle_avoidance_disabled=True`，并非本次几何生命周期修复新引入。审查遗漏在于没有将此配置与原文局部安全导航和本轮验收要求核对，未明确报告无避障模式不能验证原文可通行性执行；不能把配置原先存在说成新发现或归咎用户。
- 更正归因边界：BeamNG宿主节点固定/关闭碰撞属于动力学简化，Nav2障碍观测和碰撞预测属于独立的软件规划约束。当前配置同时放宽两者，允许错误/捷径路径在理想载体上执行；仅恢复物理碰撞不保证规划选择正确。尚未逐帧还原林区首次选择的图/视觉评分/LiDAR占据及路径，不能认定这几个配置是入林唯一原因，更不能据此判定视觉记忆原理失败。
- WildOS官方方法说明包含几何安全建图、视觉可通行性评分、分层规划与局部安全运动（https://leggedrobotics.github.io/wildos/）；当前BeamNG适配配置缺少Nav2局部障碍保障，与其完整设计不能等同。本轮只分析、更新记录，未修改生产代码或参数、未新开测试。

### 2026-10-09 恢复软件障碍约束及RPP旋转卡死组件复测

- 用户要求继续测试。保留BeamNG all-fixed-v1/物理碰撞关闭，独立恢复Nav2软件约束：FollowPath碰撞预测true，局部/全局cloud标记与clearing_cloud清除分别接入/wildos/points和/wildos/clearing_points。绝对高度边界保留Town海拔约52m的已过滤观测；未改速度、模型或同进程记忆。
- 首次部署9280c6bc9d6bcf1244c955d87578d0592f761ee8，现场预检通过，障碍云/清除云持续发布且两图有占据格。3m动作被接受但接近终点后未返回完成，端点0.391111m，随后局部清图超时，动作线程约98.1%CPU。首次组件测试未录MP4，明确为证据缺口；失败JSON/日志保留，后续录像不替代失败现场。
- Humble RPP1.1.20纯旋转采样公式sin((resolution/2)/max_radius)在局部0.2m栅格/旧0.01m半径时得到负时间步，构造可复现不退出循环反例，与现场CPU/锁阻塞相符。未获现场线程调用栈；不能归因为此前关闭碰撞预测时所有停驶的原因。
- 两图改为0.15m规划代理/0.3m膨胀，满足局部0.2m和全局0.25m栅格的采样数值域；规划代理不是BeamNG车体碰撞恢复或转弯半径。离线及服务器预检检查数值域、实际运行半径/膨胀/障碍源。冻结现场f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0，构建和预检通过。
- 相同3m FollowPath成功返回STATUS_SUCCEEDED，2.802703s、端点0.540690m、最大横向0.000199m、29反馈；清图后两服务约1.05/0.67ms，GridBased规划23点约1.28ms，全局图517占据格，路径无占据命中/越界。组件复测两路录像已录。只验证该输入及服务，不代表完整搜寻/森林语义/记忆通过。
- trial14_obstacle_restore_20261009已启动，统一开测预检passed=true，两控制初始false/无组件测试残留。保持绿色居民楼→加油站、严格房屋到达后切换、无预置station、同地图/视觉进程。最终结果待记录。
- 证据和回归边界：test_artifacts/server_validation_20261009/obstacle_restore/REPORT.md；服务器/home/ly/town_memory_validation_20261009/obstacle_restore及proxy_revision。未知区域仍按原配置，trial13图连通断开未解决，不能将LiDAR接入写成全部可通行性修复。

### 2026-10-09 trial14结束：自动通过与人工验收边界

- 现场冻结f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。trial14原始自动passed=true：house142.563s、station返程233.292s，同地图/视觉PID，无station预置/序列内瞬移/地图重载/调速。返站首次记忆选点0.402s、首路径0.426s；经历首选候选与确认约28m失配、观察、旧实例释放和候选重选，最终14~approach0匹配距离0.303m。无本轮无记忆对照，不声称已证明全程更快。
- RPP现场collision ahead中止一次（elapsed323.631s），约3s后重试完成，真实软件碰撞保护已触发。组件新数值域未再卡死，但不能据单轮声称所有障碍约束/稳定性修复。
- 原始日志两目标到达不能替代人工验收：house接近帧有浅绿色房屋、掩码覆盖其区域；house到达帧前相机PTS169.4s主要贴近蓝色墙，绿色房屋在右侧背景，3D支持与目标实例/遮挡需进一步核验。暂不认定完整绿色房屋到达成功，不回写自动报告，也不仅据此断言模型颜色失败。返站接近及到达画面明确为加油棚/泵/柱，视觉支持返站完成。
- 结束enable=false/follow_path=false、模拟器暂停。完整人工双目标验收仍未通过。最新详情test_artifacts/server_validation_20261009/obstacle_restore/REPORT.md，server_evidence/trial14保留自动JSON、掩码、时间索引和组件日志，原bag保留服务器且录制中未读取SQLite。
- 两MP4已取回captures/20261009_B_green_house_station_trial14_obstacle_restore：前相机142651907字节/2017帧/403.4s，RViz262505856字节/4155帧/415.5s；服务器哈希一致、末帧可解码。review/video_review.json保存真实时间索引及四帧复核范围，非全片逐帧审查。用户要求视频时应展示本轮两段，不能称完整成功视频。

### 2026-10-09 trial14蓝房旁停车已复现：近表面定位与到达误判

- 用户问为何停蓝房。本次只读停止录包/当前源码，无生产修改或新仿真。详见test_artifacts/server_validation_20261009/obstacle_restore/BLUE_HOUSE_REVIEW.md。
- 精确观测1791513863084888339对应前摄像头845帧/PTS169.0s/harness179.017s，RGB与mask时间戳完全相同。掩码内62个去重点有两组：近表面33点、相机光轴中位2.336m，世界[746.33197,-112.56855,54.94038]；远表面29点、19.698m，世界[751.17981,-130.08669,58.21225]。
- foreground_depth按最近合格表面选近墙；按mask携带同次采样姿态/虚拟LiDAR杆臂复算与现场原始定位数值完全一致。近表面与蓝墙位置相符，缺少遮挡物与目标实例区分；早期固定锚点在近墙，后续远住宅定位约18m外被关联门拒绝，最后近墙观测再次入簇。为什么近墙点投影进入绿房区域仍需区分粗掩码边缘、实际采样和投影偏差，不固定归因于颜色模型或时间戳。
- 到错误接近点0.51151m、到错误目标5.12001m，clamped到达阈值由4.5m调整为5.21019m，object_found触发停止/切station。远表面到车仍约23.27m。当前几何支持confidence不证明语义身份，实例锁和到达没有独立同实例复核；house普通graphnav链路发生，记忆返站尚未开始。
- 更正此前重投影18m差距的解释：录包TF插值姿态比采样姿态差2.14度；精确采样姿态可完全复现现场，不能据首轮差距认定现场TF错误。该帧配对mask/LiDAR/CameraInfo戳相同；不等于证明原始采集绝对同步或所有时序无故障。
- trial14原始自动passed=true保留；现在有更强证据判为绿色房屋阶段人工验收失败，完整双目标失败。返站视频仍支持完成。取证server_evidence/trial14/blue_house_review/surface_selection_summary.json、投影NPZ/原始定位/配对/TF、review/arrival_measurement_frame_index.json。原视频路径不变，应将两路MP4视为本轮未通过完整验收的证据。
- 未实施修复：在定位端处理深度表面歧义/遮挡，保留多候选而非盲选最近或最远；固定实例有界反证释放/重选；到达验证语义-3D支持一致性。先以此反例覆盖正常、遮挡、多物体、失观和交接，不能只放宽距离或加绿房专用适配。

### 2026-10-09 蓝房误停修复进行中与真实车辆/稳定性新要求

- 用户要求确认并按全局准则修改；保存进度后补充：上轮姑且作为基线完成，仍须修复蓝房误停；恢复真实物理车辆，不能原地掉头/不能忽略体积，需转弯曲率；固定条件20轮重复绿色房屋搜寻，至少16轮实际验收成功后才开展绿色房屋→公交车站→加油站等多目标测试。成功轮只留日志指标，失败轮生成/取回RViz及前相机MP4，需临时画面缓冲以支持失败回看。不能把旧理想载体轮次计入新物理版本成功率。
- 服务器最后只读核实HEAD f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且工作树干净，geometry/triangulation与本机修复前哈希一致；本次新修复尚未部署或开新现场测试。已有SSH句柄62320后续丢失，不能据此断言服务器掉线；须新连接重新核对。
- 已保存修复前反例 `test_artifacts/server_validation_20261009/surface_association_fix/before.json`：精确采样姿态2.336090m、几何confidence0.864435、生产到达回调false_arrival=true；原NPZ哈希866fd7340e720b40b56f33e02df4122c4e5fa73124a1f026dccd8710eac3f49f。紧凑回归样本visual_navigation/test/fixtures/trial14_ambiguous_surfaces.npz，原始证据不覆盖。
- 本机候选源码已实施但尚未完成专项回归：localization_geometry保留所有通过同一支持/相干门的表面，并区分association_validated与纯几何confidence；obj_mask_triangulation在歧义时拒绝3D目标并清多视角旧射线，发布带采样时间的拒绝事件；GoalStandoff严格要求有效关联，拒绝事件撤销旧观测/待发确认，实例锚点由原有release/ACK管理；当前新观测不属于历史胜出簇时不能刷新历史簇成功。GraphPathBridge收到拒绝后设置证据时间界限，阻止迟到旧确认/旧刷新到达，不直接取消已接受路径；同query候选比较期间也更新拒绝界限。质量协议三端必须同时部署，不能只上线其中一端。
- 初步本机253项既有生产回调替身检查通过（5处合成消息助手显式添加新关联契约）；日志surface_association_fix/callbacks_initial.log。仅既有回归，不代表新增歧义/恢复/乱序专项已验证，更非DDS或完整现场验收。下一步补专项测试、真实录包回放和跨端拒绝/恢复/交接，再处理真实车体配置与上线。
- 已核实真实车辆恢复风险：tools/restore_physical_vehicle.py绑定20261006旧进程/路径，不能直接运行；仅将wildosKinematic carrierModel改ackermann仍fixedNodes且physicsBypassed=true，不满足真实物理要求。nav2_ackermann_control.yaml虽已有SmacHybrid/8m半径/矩形体积，却仍关闭RPP碰撞预测且两图无障碍源，必须保留当前软件障碍修复一并对齐。恢复需原始车体/轮胎/悬架物理与native控制链、统一后轴坐标/曲率/footprint、RPP禁原地旋转并允许合法换向、传感器实测标定、预检与所有执行分支；不能只换规划器名字。
- 尚未恢复真实物理、尚未统计20轮成功率、尚未进入多目标阶段。此前trial14自动passed=true与人工蓝房反例均保留；用户暂认可基线不等于修复后的验收。开发工具默认沙箱启动出现setup refresh错误，require_escalated普通宿主读取可用；不是项目仿真故障。

### 2026-10-09 蓝房修复专项与真实DDS隔离通过（尚未部署/实测）

- 以上“专项尚未完成”为中途状态；现五个生产文件候选（geometry、triangulator、GoalStandoff、GraphPathBridge、TargetObservation）完成本机268项生产回调替身检查，包含15项新增专项；综合43检查37PASS/6WARN/0FAIL。详情 `test_artifacts/server_validation_20261009/surface_association_fix/REPORT.md`，原反例与trial14结果不覆盖。
- 全局审查发现并在部署前修正新增冲突：不能把深度拒绝直接写入原release/ACK界限，否则会丢弃同一帧仍有效的bearing提示。新confirmation_evidence_not_before_ns只阻止旧3D/到达，release界限继续控制全部旧实例消息；重选保留、新query重置。专项已验证同帧方向提示保留且拒绝不取消当前路径/改旧计时归属。
- 真实服务器域79隔离生产三节点与实际DDS，8项检查通过：录包双表面不出raw_goal、保留两候选、旧错锁不偷偷改成另一物体、迟到错误到达被阻止、同帧bearing保留、release/ACK解除错锁、新单表面观测确认、蓝墙位置不宣布到达。单表面恢复输入由录包远表面构造，只证明有效新观测及交接，非自动消除遮挡/车辆找到绿房的现场成功。未运行Nav2动作/代价地图/模型推理/车辆运动。
- 两个历史station投影样本仍association_validated=true（一个合格相干表面）；背景密度没有被简单当作否决规则。近/远两个独立合格表面仍拒绝确认，不盲选最远。单表面支持本身不证明语义类别正确，后续成功仍须同步画面与真实位置验收。
- 首次DDS诊断工具保存numpy.bool_报JSON错误，保留dds_attempt1.log；修正工具后dds_after.json passed=true。这是测试报告序列化故障，不是生产节点故障。证据已取回本机同报告目录，verified_source.json核对五生产文件哈希；服务器隔离目录/home/ly/town_memory_validation_20261009/surface_association_fix。
- 已重新连通服务器并复查活动HEAD仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且工作树干净，未部署此候选或开新运动测试。三端新质量契约须一起部署/核实运行来源，保留vision四函数差异。用户最新要求先修蓝房再测试，当前为源码+离线/隔离已验证、完整现场待验收。下一步恢复真实车体与全链路Ackermann约束，然后组件验证及固定20轮（至少16成功）；只保存/取回失败MP4，成功留日志指标。尚未开始20轮、多目标或宣称80%。

### 2026-10-09 用户再次要求先修蓝房：当前源码复核

- 先读项目记忆，再核对五个生产文件；当前文件原始SHA256均与已完成268项离线回调/8项真实DDS隔离检查的候选一致。证据 `test_artifacts/server_validation_20261009/surface_association_fix/current_source_review.json`。本次是版本与接口审查，没有重复计为新测试，也未部署或开启车辆运动。
- 确认结论仍限定为trial14近墙/远房深度歧义被盲选、错误锚点阻挡修正、错误到达的已复现链路。近墙点进入绿色掩码的最初来源以及单表面颜色误检尚未定论，不能将关联修复写成模型颜色能力已修复。
- 持续遵守用户全局修改准则：保留原反例、检查定位/确认/执行及释放交接、时间与实例界限，保持方向提示与路径计时归属，三端同时冻结部署；修复后的完整搜寻与真实车体现场验收仍待完成。

### 2026-10-09 真实车体接口修改、服务器恢复与原始 SBR 现场检查

- 按完整目标继续推进，详情 `test_artifacts/server_validation_20261009/physical_profile_fix/REPORT.md`。Ackermann 配置恢复 RPP 碰撞预测和两图标记/清除观测，保留后轴矩形车体、SmacHybrid/REEDS_SHEPP/8m曲率；新增独立 physical_vehicle_control.launch.py，原生速度环和 GraphPathBridge 默认关闭，未调速度。
- 新 runtime_contract/预检区分理想和真实模式，核对实际加载曲率、footprint、观察、终段容差与控制话题；禁止两种后端并存，真实 StateSensor 直接检查，不要求原生遥测具有理想控制器的 state_ok。新物理恢复工具取代固定20261006旧PID/命令，默认只读、明确原始完整配置、应用前跨域检查车体客户端，恢复后停驶/暂停，不自动重启旧进程。
- 离线最终44检查38PASS/6WARN/0FAIL，15项新增物理契约反例通过；服务器隔离候选两包构建和15项检查通过。真实Humble启动展开8项通过（抑制Node执行，无DDS/运动），保证嵌套include参数实际为物理车体。构建/原始报告/候选v1-v3已取回上述目录。生产HEAD仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且干净；蓝房五文件未部署到域40。
- 新现场基础设施状态：SSH可用但BeamNG已退出/25252无监听，旧传感器桥报Not connected to the vehicle。核实并停止实际PID1443674(sensor_bridge)和1443781(理想kinematic)，再用原保存的正确二进制/headless/userpath启动BeamNG PID1876608，GE连接成功。加载Town原始SBR一次作为新实验准备，未启动搜寻或统计20轮。旧域40的其他ROS节点仍在，不能未经重新预检直接开测。
- 新诊断反例：普通访问extensions.wildosKinematic会触发BeamNG __index自动加载，读取本身导致扩展出现。初次stock_physics_boot报告因此failed，保留原结果；extension_autoload_probe实证后改为rawget，恢复工具同改。连续两次现场读取不再加载扩展，三项物理/只读回归通过，不把诊断缺陷误诊为车辆物理关闭。
- 当前真实SBR现场结构：723节点/0固定节点、4538梁、1449.72kg、467车身碰撞节点，四轮各32胎面碰撞节点/0固定节点；车辆ID16266，初次车辆端口36165，位置约[734.29299,-11.94647,52.06423]，停车/模拟器暂停。现场physical_snapshot_v3通过，但camera/LiDAR尚未重新挂载，原生执行/转弯/传感器标定和完整搜寻仍待验收；端口/PID只属本次实例，后续须重查。
- 下一步：当前旧sensor_bridge.py硬编码38197，须用当前实例重新连接并核实真实挂载；蓝房与物理候选统一冻结、登记PID、核对导入来源；完成真实控制与小路径/曲率/倒车组件检查。现有town_sequential_memory_trial.py仍硬编码理想载体和总录MP4，须先改成真实单目标20轮/失败画面缓冲与实际视觉到达验收，不能直接用旧脚本统计成功率。尚未开始20轮或多目标。

### 2026-10-09 SSH恢复后：真实传感器、直行/倒车/转弯组件与新证据

- 用户确认可SSH后，重新核实当前原始SBR/BeamNG仍有效，没有新加载地图。新增tools/attach_current_vehicle_sensors.py，从当前StartVehicleConnection查询身份和有效端口（本次36165），拒绝旧端口兜底/已有客户端或传感器；只复制保存的sensor规格，不应用旧rig文件的关闭碰撞车体属性。按拥有者清理和先停轮询后关闭连接。4项新纯契约反例本机/服务器通过；本机综合45检查39PASS/6WARN/0FAIL。详见test_artifacts/server_validation_20261009/physical_profile_fix/REPORT.md。
- 相机/LiDAR已挂载。三次静止相机标定位置误差最大约1.25mm；12秒域79 DDS观测：native状态241、sensor状态120、相机108、LiDAR116，戳推进，最大接收间隔约0.056/0.104/0.301/0.200s；原生遥测与执行健康，看门狗缺指令时制动。该窗口不证明移动采集绝对同步或长期稳定。
- 完整launch首次失败：生产安装的dummy_pose_graph_publisher.py链接源权限664，ROS无法执行；仅在隔离候选复制/构建graphnav_mapper并给脚本执行权限，原生产HEAD/工作树不变。失败launch留下adapter/standoff两个子进程，核实域/命令/PID后清理，证据failed_launch_leftovers.json。check_physical_launch.py补上ROS实际libexec解析及执行权限检查，8项通过；仅参数展开曾遗漏此部署问题，保留更正。
- 候选物理栈在域79启动，全部实际runtime_contract参数匹配，两门初始false，两图有障碍占据，CarLike生成3m路径。静止physical_components_v1检查通过；它不是含视觉/所有进程的完整preflight。原生产仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且干净，11候选源码/配置与本机哈希一致，蓝房协议三端尚未统一开启完整搜寻。
- 实际FollowPath组件：forward3_v1成功，前进2.545m、D挡、终点0.50033m；reverse_v2成功，反向3.440m、R挡、终点0.36042m；right_arc_v4前进右转成功，前进投影2.486m、D挡、yaw约-0.222rad、终点0.53251m。每次完成后关闭速度环并验证停车；没有调速或关闭软件/车体碰撞。1.2m/s是期望速度，短动作瞬态最高约1.71m/s，不能写成实测速率上限。
- 失败反例left_arc_v2：实际前进0.818m后RPP collision ahead中止，终点2.18265m，native健康；尚未核验具体碰撞格来源，不能认定误报或之前全部失败同因。失败两MP4已主动取回，绝对目录C:/Users/Yu/Desktop/拟真/captures/20261009_physical_components/motion_component_left_arc_v2；SHA256与服务器一致，failure_video_hash_match.json保存核对。所有六组短组件12段录像均取回，末帧可解码；组件画面无视觉query掩码，不能作为绿房搜寻验收。
- 诊断工具自身错误保留：left_arc_v1将PoseStamped当二维数组，启用车辆前TypeError；修正工具输入后v2才执行。right_arc_v3负转角构造负前向位移，实际完成倒车弧线，不能算前进右转；v4修正有符号圆弧目标后验证真正D挡前进。这些不是生产规划器新故障。
- 新未解决接口证据：mapper运动时偶发terrain_base_link的TF未来外推，请求比自身缓存最新值新约0.10–2.02s。独立right_arc_v3见证收到110条terrain/base/nav TF、最大接收间隔约0.157s且无戳倒退；状态、相机、LiDAR持续发布。不能据独立监听正常证明mapper接收缓存正常，须查具体接收/排队及同次发布次序，不能直接替换成latest TF或归因BeamNG时钟。尚未修改该源码。
- 测试结束真实车体结构/四轮碰撞/相机LiDAR复核仍通过，暂停前速度0.00018m/s，车辆停车、模拟器暂停。当前域79的候选物理栈/传感器/新RViz仍在；旧域40其他ROS/视觉节点仍在，任何下一轮必须重新查实际进程/运行来源，不直接混用。证据server_components_v4和captures/20261009_physical_components；失败/诊断原始报告不覆盖。
- 下一步先查左转具体障碍与mapper TF缓存，再统一视觉定位三端、完整预检和真实单目标20轮工具/失败画面缓冲，随后固定20轮至少16次实际到达才开展多目标。20轮尚未开始，完整蓝房修复/绿房到达仍待现场验收。新版部署包63363FBC15084ECB1ECD813B7062F6C7D9CA49BF7410DF7DECB889372FA6AE13，284文件，Linux执行权限部署时仍须核对。

### 2026-10-10 恢复工作：上次 TF 修复与暂停状态补记

- 上次暂停前已修改 graphnav_mapper/src/mapper.cpp：地形图按采集时间查询 TF，专用监听线程存在时最多等待 0.25 秒；没有专用线程保持零等待，缺失仍拒绝，不使用最新姿态替代。实际 Humble TransformListener 默认独立监听线程，不能把地形处理耗时直接解释为同一个 TF 回调线程被阻塞。
- 真实 DDS 隔离反例：原逻辑在地图先到、对应 TF 延迟 0.03/0.10/0.18 秒时丢图；候选全部接受，完全缺失仍拒绝，之后收到精确 TF 并重发地图均恢复。两个新增 C++ 契约反例及全套 17 项 mapper 测试通过；首次测试编译 unique_ptr/SharedPtr 类型错误已修正并保留失败日志。这仅证明接口修复，不能证明所有现场未来外推已解决。
- 服务器 motion_component_forward3_tf_v5 在暂停前实际完成：FollowPath status=4、终点误差约 0.48668m、9.436s。左转 TF 修复后尚未执行；先前清 costmap 辅助断言失败，不能声称清除完成。user_pause_20261009_tf_fix.json 记录车辆速度约 0.000205m/s、仿真暂停且取消动作。parameters 中 true/false 是设置响应结果，不能脱离实际参数复查解释为参数值。
- 2026-10-10 用户要求继续后重新 SSH 核查：原 BeamNG PID 1876608 已退出，原生产仍 f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0、工作树干净；域79传感器/导航及域40旧视觉仍有残留。按 PID、完整命令及域逐一确认后停止对应项目进程，恢复仿真作为新的实验准备，不计入完整测试/20轮。恢复证据服务器 /home/ly/town_memory_validation_20261010/resume_recovery.json。
- 发现域83两个隔离诊断 mapper 孤儿进程：旧工具终止 ros2-run 包装进程没有可靠终止实际子节点。check_mapper_tf_delivery.py 改为从当前安装前缀直接启动 mapper 可执行文件，finally 只清理自身进程；不是生产规划器修复。需验证工具退出后没有残留。
- 当前部署包仍是上节旧包，未包含最新 mapper TF 源码。后续冻结/部署须重新核对源码、二进制和包哈希。绿色房屋真实到达、蓝房现场纠错和 20轮成功率仍未验收。

### 2026-10-10 真实执行复测与 TF 接收策略修复

- 现场报告 `test_artifacts/server_validation_20261010/REPORT.md`。恢复工具准备阶段遗漏真实换挡，默认街机模式把刹车转换为倒车油门；正式 native_vehicle_control 已有 realistic_automatic，未修改正式速度环。明确设置真实自动模式、空挡和制动后，在 Town 出生点停稳；stock_boot 的 passed 仅表示结构检查，不代表当时已停车。初次 SDK/序列化诊断错误和恢复日志保留。
- 当前原始 SBR、车身/四轮碰撞和传感器有效；本次端口 38903。实际参数/地图/CarLike 规划检查通过。直行终点误差0.10559m，倒车0.39933m；保留软件碰撞保护和真实体积，未调速度。
- 左转新失败发生在 ComputePathToPose 阶段，未进入 FollowPath。请求终点车体覆盖局部30个lethal格，后续全局快照覆盖25个。LiDAR 点约52.22m，独立 BeamNG 静态下视射线约52.2241m，对照附近路面约51.929m，支持真实约0.29m抬高表面，不支持删除这些点或关闭碰撞。统计是格中心落入带余量车体，不是完整 Smac 搜索复刻，也不能直接倒推上次 left_arc_v2 的所有格。失败前相机/RViz MP4 已取回并与服务器哈希一致：captures/20261010_physical_left_arc。拉开空间后左转成功，实际位移2.8617m、转向约0.325rad、终点0.03423m；这是组件成功，非完整搜寻。
- TF有界等待修复仍有反例：倒车时独立监听器已收到被 mapper 拒绝的三个采集时刻TF，但 mapper 缓存落后。真实 FastDDS 同节点双QoS静止60s见证：可靠600条、最大接收间隔1.951s；尽力590条、最大0.187s，共同消息最大接收差1.796s。第一组90s建图比较原节点3次失败、独立尽力节点0；两节点同时新启动的后续60s窗口均接受108图、均无窗口内失败（可靠节点启动预热期1次失败）。保留这个无差异窗口，不把QoS修改说成总能提高吞吐或完全消除丢样。
- 最小生产修改 graphnav_mapper/src/mapper_node.cpp：只将动态 TF listener 改为 best_effort，静态 TF 继续可靠/TRANSIENT_LOCAL；发送端、地图时间戳、采集姿态和缺失拒绝不变。保留 mapper.cpp 的专用线程有界等待和旧反例，不改成latest TF。影响范围是建图订阅；Nav2/其他节点的监听策略没有同时修改，不能声称全系统传输稳定性已解决。
- 候选重新构建、全套17项C++通过；增强真实DDS检查验证静态标定在 mapper 启动前发布仍能晚加入接收，动态TF延迟0.03/0.10/0.18s接受，缺失拒绝且后续恢复。新候选90s现场接收162张地形图、160张处理输出、0空图/0 TF lookup error，包含真实倒车复测，终点误差0.45971m。运行订阅QoS与实际共享库已核验。源码本机/服务器规范化内容一致，C++node/test原始哈希仅换行不同；记载原始差异，不冒称原始哈希一致。
- 新证据 components/diagnosis/tf_final 已取回。原生产Git仍干净f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0，改动只在本机/隔离候选。组件结束速度约0.000104m/s、真实物理检查通过，两控制门关闭、仿真暂停作为验收工具准备；任务仍继续。临时双节点/隔离DDS节点已自行退出，需下次预检再核实。
- 本机部署包已刷新到285文件、SHA256 36765D20C0999800C6C918E7D1CDCF9293C7E2FDA01E268EA22D5F630EE70765，包含TF诊断工具；之后验收工具修改尚需再次更新包。
- 新发现验收脚本的真实模式不兼容：旧 Harness 只接受 ideal_point/GridBased/0半径，速度门硬转发到 kinematic 后端；到达截图使用最新RGB、掩码叠加可能重用较旧RGB。已抽取 import-safe 的 tools/town_trial_harness.py，明确真实/理想参数与控制归属，保留旧两阶段/续测调用；到达截图要求精确源帧和有效关联，保留64帧有界缓存，旧overlay不覆盖新RGB。停驶分别尝试关闭两门，一个服务拒绝不跳过另一个。8项离线回调契约通过；测试助手第一次字段name与factory形参重名，修正并保留attempt1日志，不是生产故障。
- 共享验收工具尚未服务器部署/完整验证；失败画面临时缓冲、真实单目标driver及完整视觉预检仍待完成，20轮与蓝房现场纠错未验收。不得直接运行旧理想载体主流程统计成功率。

### 2026-10-10 真实单目标验收工具与失败录像流程

- tools/town_trial_harness.py 的真实/理想控制归属、采样帧精确匹配已部署候选，补充停驶必须取得新鲜静止遥测、参数结果必须一一响应及实际僵尸状态检查。10项离线回调检查在本机/服务器通过。未改变正式导航/控制算法。
- tools/failure_video_capture.py 临时将有界JPEG写到磁盘，仅在失败时生成MP4，精确ROS纳秒及单调接收时间索引分别记录。视觉到达暂定时先封存临时帧、保留同步到达RGB+mask，人工确认后才计成功并丢临时帧；误到达则从同轮临时帧生成失败录像，不拿最新画面替代源帧。9项实际编码检查本机/服务器通过，含间隔重采样、初始缺帧、大小界限、时序/单位、无帧、分辨率变化、待审后失败/成功。失败导出缺失或损坏须报告，不伪造成功。
- 服务器静止真实DDS/录制组件：前相机20帧、RViz32帧，两真实控制门False、车停稳，实际ffmpeg管道成功退出、临时帧清理且无MP4。范围不含视觉query/模型或运动；结果 /home/ly/town_memory_validation_20261010/recorder_component/result.json。
- 新 tools/town_physical_single_target_trial.py 在创建控制发布者前运行完整预检，严格核验Town出生点/原始SBR，query订阅就绪+ACK+实际CarLike模型再启用两门；420秒有界搜索，三次mask与同观测确认/到达匹配；任何异常尝试停驶/停车、暂停/保存日志及失败录像。自动到达仅为待人工验收，不自动计入20轮，不下发station。
- 新准备已在同地图独立轮次前回Town一次、重启本轮自己拥有的物理launch和mapper；不是搜寻序列中的瞬移。新物理/视觉源来自隔离candidate，vision/nav.py保留服务器原版差异。视觉启动首先缺explorfm模块，属于候选的模型依赖搜索路径遗漏；检查实际导入后仅补既有原生产core依赖路径，候选视觉/geometry及配置保持优先并核对，不修改模型。原启动失败和源检查日志保留，不当作现场搜寻失败。
- 最新本机部署包292文件，SHA256 55F40C6F874214BA80547936372D7DF76B8A909F7799D6F6A37B2EE563A97E3C；本轮5份验收工具已原始哈希核对。20轮统计及蓝房修复现场到达仍未开始/验收。

### 2026-10-10 真实绿色房屋 pilot1 失败与曲率接口修复候选

- physical_green_house_pilot1 完整预检和 query ACK 通过，但未找到/到达绿色房屋。约387.878s，RViz临时JPEG达到工具旧512MiB界限，录制异常结束试跑，不能写成420s搜寻超时。没有目标掩码/确认；累计约184m含往复运动，不能等同于离站距离。20轮尚未开始。
- 现场32次 maneuver_done 后均 replan；31次 join_curvature_exceeds_turning_limit，1次目标在后方。保留33组可匹配完整Smac路径/首段，其中17次失败交接有唯一原路径及实际姿态。到达姿态通常偏0.56～0.59m；将同一交接复放到精确规划cusp仍有12/17被旧曲率检查拒绝，不能只收紧goal checker距离作为根因修复。
- 新确认契约差异：Humble RPP1.1.20按请求曲率预测碰撞/下发，native执行再按物理转弯能力限制曲率；旧交接检查要求精确3m圆弧可达。候选新增 wildos_ackermann_rpp 插件，在原RPP速度调节后、碰撞预测和发布前统一限制曲率到规划8m半径；保留真实体积/两图障碍/反向行驶，禁止原地旋转。交接预测使用同一受限圆弧，仍须0.6m/0.35rad有界重接和既有两图扫体检查；不删除障碍，不调速度或缩小半径。旧RPP配置与新协议混装须预检拒绝。
- 本机268项生产回调、31项motion/runtime/physical契约通过；录包17样本几何重放仅10次通过有界重接，7次仍合理拒绝。此范围不包含地图碰撞、实际插件或运动验收。C++插件固定依赖1.1.20，实际构建、前后向碰撞反例及复合路径组件验证进行中；不可宣布重规划全部解决。
- 新工具故障修复候选：临时界限每路2GiB、最低6GiB剩余空间；录制界限/写盘异常后继续排空ffmpeg管道并最终kill+wait，11项编码/生命周期检查通过。driver异常也保留累计行程/时间；native静止后再等引擎实际速度<0.05才暂停，未放宽物理门。pilot旧结束瞬态约0.116m/s、暂停后约0.102m/s；两控制门关闭独立续仿真5s后约0.000473m/s并重新暂停。原报告不覆盖。
- 失败两MP4已取回并展示：C:/Users/Yu/Desktop/拟真/captures/20261010_physical_green_house_pilot1/front_camera.mp4（102235080字节、SHA256 d3e116fd3fdf23c35f61797d5389d908135055bf536e3e5867605ffa55e44298d）、rviz_live.mp4（146781893字节、c469b7efacccc84021e7f3c84c835bc0b2780b4dad313df666219af3dd502aa6）。精确帧时间及失败日志/见证在同目录；本轮约385.8/387.625s，首次录制偏移各2.2866/0.3733s。只复核部分帧，没有凭无掩码断言模型原因。
- 用户新规则：每次失败先修复再实验；连续3次完整试验失败先暂停实验严修；服务器证据核验取回本机后可删服务器重复文件。pilot1是当前配置第一次完整试跑失败，已停止自己的物理/视觉/triangulation进程，保留传感器/RViz、车停稳暂停。核验本机完整哈希后仅删除服务器本轮两MP4及重复tgz，释放493723497字节；原日志/冻结源码/录包保留，media_archival.json留证。新候选尚未开下一次搜寻。

### 2026-10-10 受限曲率插件实际组件验证

- wildos_ackermann_rpp 候选实际构建，6项C++检查通过，含pluginlib实际加载、前/后向请求曲率过大时保持原速度但限制发布曲率、原RPP漏检而受限圆弧命中同一lethal点的双向反例、动态半径更改拒绝。测试使用小点体积与构造静态地图，验证曲率/碰撞接口，不代表SBR现场全部障碍。第一次测试fixture缺独立TF线程有警告，补真实独立监听后6项仍通过且0该警告；生产TF未因此改动。
- 服务器回归最初缺历史fixture，保留失败日志；补齐本机样本和当前全部测试源码后268项通过、0跳过。15项motion及11项录像检查通过。实际RViz管道4s容量故障注入：保存1帧、随后排空31帧、生产者已回收/读线程退出，未控制车辆。新组件后台继承TTY导致ffmpeg SIGTTIN，前台恢复完成；工具新增-nostdin/DEVNULL避免后台挂起，须部署后核验，不归因为导航故障。
- 现场实际控制器已加载libwildos_ackermann_rpp，静止物理组件参数/地图/3m CarLike规划通过。新check_ackermann_cusp_execution.py先停实际GraphNav planner/vision/triangulation，保持传感器及全部代价图；只读规划筛选短多方向路径，再由生产GraphPathBridge规划/FollowPath/交接。真实车辆在相对[4m,2m,0.8rad]组件完成两次maneuver_done、4次following、route_done，0replanning_after_maneuver/0follow_failed，终点误差0.631611m，末态真实速度0.042391m/s、车体/四轮物理有效并暂停。没有瞬移/清障/调速/原地旋转；两门关闭，成功临时画面丢弃无MP4。证据bounded_cusp_component1/result.json及实际日志。
- 这只证明一条短复合路径，不代表全部17交接样本、蓝房修复现场验收或完整绿色房屋成功。准备冻结版本重新单目标试跑，20轮仍未开始。

### 2026-10-10 pilot2完整420秒失败与新的排查证据

- 新插件/配置31文件及实际共享库冻结freeze_pilot2.json；原生产仍干净f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。本机规范化核对除历史服务器nav.py四函数差异外一致，beamng_ablation为服务器配置，不冒称整树一致。Town序列前复位一次、地图不重载，完整预检 passed=true。只有绿色居民楼query，无station预置。
- physical_green_house_pilot2真实执行420.004s超时，0有效掩码/确认/到达，累计124.229m包括往复。11次maneuver_done中6次交接重规划，7次follow_failed（包含execution_stalled及collision ahead中止）、2次no_vehicle_path；这不是稳定性或绿色房屋成功。2轮物理单目标试跑均失败，尚未进入20轮固定统计；按每次失败先修复，暂不再开实验。
- 结束两门关闭、引擎速度0.038584m/s、暂停，实际车体/四轮检查通过；本轮自己拥有的物理栈/vision/triangulation全部停止，保留传感器/RViz。新的2GiB录制无丢帧错误、FFmpeg自行回收；失败本机目录 captures/20261010_physical_green_house_pilot2，两路MP4取回/哈希一致并展示，前摄像头5fps/2107帧、RViz8fps/3373帧。完整闭包及接口bag也取回 test_artifacts/server_validation_20261010/pilot2_closed_evidence.tgz；核验后仅删除服务器两MP4/重复tgz，释放560241000字节，原日志/bag保留，pilot2_media_archival.json留证。
- 新证据不能直接归因物理受困：约221.435s及270.440s execution_stalled前选取约11秒窗口，84～86条实际native状态显示sent.throttle=0、sent.brake=1、native watchdog=False，实际gear分别D/R，车速几乎0；上层到故障时却报告target_speed=1.2/-0.2及actuated_no_path_progress。证明该窗口执行端主动制动，具体上游原因/消息顺序待查；不能宣称汽车物理卡住或已经定位完整制动原因。stall_windows.json保留选窗范围。
- 该轮odom/RGB接收间隔最大约0.240/0.350s，ROS戳最大年龄约0.493/0.499s；measurement_timing最大年龄0.626s。只描述harness观测，未记录control_status/cmd_vel，不能证明速度环未过期或各订阅缓存无延迟。plan_status最大年龄61s包含旧状态心跳重发，不能误诊61sDDS延迟。原witness2仅记录310秒，有界见证不覆盖末段，不能写成完整420秒全接口追踪。
- 同一MP4 PTS260s，前摄像头看树/停车场、RViz RGB看便利店/道路；两流首帧偏移不足以解释该差异，需要检查 visualization_bridge实时画面与运行来源，不以RViz右画面替代当时真实RGB。部分帧0/60/130/210/260/330/415及RViz260保存review；未逐帧全片审查，暂未发现绿色房屋的有效到达证据。
- 新修复前重点：执行反馈是否将安全制动/等待跨时段累积为连续物理停滞；控制状态与真实native制动的时序/契约；RViz更新链路；残余交接的实际偏差。新的受限曲率按规划8m限制，与native保留的更小物理半径/跟踪修正余量仍须评估，不能盲目缩小规划半径或放宽扫体保护。先保留反例、离线/隔离验证，再重新运动。

### 2026-10-10 pilot2后离线计时反例与最小修复（现场待验）

- execution_feedback.py生产回调两反例复现：监控因等待/数据恢复未调用期间，False→True控制状态不能原来清计时，恢复立即被计11s停滞；native已接收全制动时，仍按上层意图计作已驱动。callbacks_braking_before.log保留两失败。此源码反例与pilot2制动窗口一致，但不证明完整现场制动来源或所有execution_stalled同因。
- 最小候选：收到actuating=False立即清stall_anchor；真实ackermann收到/持有新鲜sent.brake>=0.9也清连续驱动计时，不干涉动作、制动保护、goal checker、地图或车速。on_control_status按ROS纳秒、on_native_status按wall_time拒绝较旧更新，防止迟到旧制动打断新计时。乱序保护前两新增反例失败日志callbacks_braking_order_before.log保留；最终272项生产回调通过，包含原连续真实停滞/恢复/换向与4项新间隔/制动/乱序反例。只是离线替身，未部署或新运动验证。
- 单目标接口bag候选补control_status、cmd_vel、native_control、execution_feedback、StateSensor、TF，避免再缺少解释全制动原因的中间数据；不重复存RGB/大点云。当前部署包305文件已落后这一最后候选，下一次部署须刷新并冻结；不得直接用pilot2版本统计20轮。
- 可视化三文件本机/服务器原始SHA一致，实际RViz订阅和visualization_bridge发布RGB都是BEST_EFFORT/VOLATILE，不支持直接归因QoS不兼容。物理栈停后raw RGB无发布者是当前预期；不能据此倒推搜寻期间无RGB（harness已收到4011帧）。RViz260与真实RGB260不一致的具体缓存/处理/订阅来源仍待查。


### 2026-10-10 服务器隔离计时验证、静止显示诊断及用户删除录像

- 计时修复已部署到隔离候选，服务器272项生产回调（替身）与实际域85 DDS六项通过；检查连续真实驱动无进展仍触发，等待/全制动结束计时、旧消息拒绝及重复消息不能重启计时。不涉及Nav2动作或车辆运动，完整绿色房屋现场效果仍待验。证据 test_artifacts/server_validation_20261010/stationary_diagnostics。
- pilot2 两处全制动前11秒窗口，harness各收到220条native状态，传输年龄最大约0.0068/0.0081秒、native遥测年龄最大约0.0510/0.0495秒；仅能排除该订阅者在这些窗口持续延迟，不能证明速度环收到的StateSensor/control_status新鲜。review/native_braking_freshness.json保留计算范围。上游制动原因仍未定位。
- 静止合成图像探针不连接车辆、不发导航或控制命令，图像标注DISPLAY PROBE。旧桥：大图48 raw/0 preview，小图也0；进程非SIGTTIN停住、输入输出QoS兼容、候选源与build链接同一源码。新独立话题桥输出20/48大图、已正确退出；仅重启官方桥，大图最初0，小图48/48；实际960x640尺寸输出13/48。不能仅凭重启称已解决。新桥线程采样在executor等待，未证明卡在publish；gdb受系统ptrace限制未取得旧线程栈，不修改系统策略。
- 同一960x640发送流的双实例统计：BE原始订阅收到45并完成45次输出，RE原始订阅48并完成48次输出；输出订阅收到的帧数另存诊断JSON。支持继续调查大图输出/接收链路，不能把输入端丢3帧解释成长期画面冻结，也不能直接将全系统QoS改为RE。没有进行第三轮完整试跑。
- 预检新增原始RGB/预览戳和frame_id匹配、至少两帧推进和末帧新鲜；拒绝停止/被调试/死亡进程。image_stream_health.py四项反例覆盖正常丢样/预览先到、重复冻结、缺失/错误身份/过期、乱序/有界缓存，本机通过；只在测试准备层，不改车辆控制或导航状态。候选工具已复制服务器；真实全栈预检待验。
- 用户明确要求删除当前失败录像：本机pilot1/pilot2四个front_camera.mp4、rviz_live.mp4已删除；两证据tgz移除同名录像副本，其余文件逐一SHA校验保留。共释放1044340658字节约0.973GiB。SSH核实服务器同轮MP4/重复闭包归档此前已删除。删除清单 failure_video_deletion_20261010.json；历史录像路径保留作为曾有证据记录，但当前不可再打开/展示。该要求适用于当前这些录像，后续失败录像仍按既定约定取回，除非用户另有指令。
- 用户提出全局路线与真实车体局部规划不匹配：当前配置有SmacHybrid/REEDS_SHEPP/8m、SBR footprint、FollowPath及native转向接口；不是缺少所有车辆接口。局部RPP按前视点算并限曲率、调速/碰撞预测，未做包含转向/换挡动态的局部轨迹优化。交接依赖0.6m/0.35rad动作完成，再从实测车姿检查下一段；pilot2的6次交接重规划支持这里是重点，但并非所有故障唯一原因，且尚不能归罪视觉记忆（本轮主要source=graphnav，无记忆返站阶段）。短复合组件已能实际执行，不等于完整路径可稳定执行。
- 当前309文件部署包 SHA256 1E7AF2EC4B63C8C135E62A32BDE56DFCB7E2B58D9332A1C38BF0C449390CD1EB，刷新计时/取证/显示预检候选。原生产未更改；20轮验收仍未开始。

- 显示双实例对照输出端实际接收25帧（BE输入45/输出45；RE输入48/输出48）；本次静止诊断闭包display_diagnosis_final.tgz已取回并安全展开，不含MP4。旧桥重启后小图显示恢复，仍保留大图丢帧/初始无输出反例，不能宣布根因修复。

### 2026-10-10 真实换向点完成契约与终点制动候选

- 最早违约点：真实换向交接沿用语义观察终点的0.6m位置区域，SimpleGoalChecker允许车辆仍移动时完成，下一段再从偏离规划cusp的实际车姿重接。把8m规划半径缩到实测约7.447m，对17份录包几何重放仍仅10份能接上，不能靠缩半径解决。将录包位置偏差人为缩为1/3、保持原航向，几何重放17/17通过，仅为定位敏感性实验，不是地图碰撞或实际运动证明。证据 test_artifacts/offline_navigation/cusp_stop_candidate。
- 最小候选：真实且后续有运动段的交接独立使用cusp_goal_checker（StoppedGoalChecker，0.2m/0.35rad，线/角停稳阈值0.05m/s、0.05rad/s，stateful=False）。graph_path_bridge.py的交接成功独立复核及短初段拒绝与0.2m一致；普通/语义终点保留旧检查器，理想载体分支不变。runtime_contract.py核验配置及实际第三检查器，拒绝旧、新配置混装。转弯半径、速度、车体、障碍输入、身份/异步取消保护未放宽。
- 仅换StoppedGoalChecker不足：实际上游RPP进入终点仍输出非零最小靠近速度，旧插件上的新制动反例失败已保留。ackermann_rpp/src/controller.cpp只对声明停稳速度容差的检查器，在真正全局终点的位置/航向范围内输出零线/角速度；不以裁剪后局部终点停车，变换按当前车姿时戳，变换失败拒绝。真实停稳由实际GoalChecker继续验收。
- 首版零速候选引入回归：静止后不扫描未来运动圆弧，原足迹周边扫描遗漏车体中心lethal格，实际C++测试失败。交付/运动前补当前中心格拒绝，并保留上游unknown-space约定及周边检查；没有改成扫描假想非零圆弧，新增车外障碍时可以合法零速停车的前后向反例。修复前失败XML/日志与最终通过XML均保存，未隐藏中间回归。
- 本机274项生产回调替身、17项运行契约、15项运动几何通过。服务器实际插件7项加曲率3项全部通过，0失败/跳过；含原曲率一致性、碰撞预测、冻结半径以及新增停稳/裁剪/错误航向/中心周边障碍/合法零速。小体积构造地图不代表真实SBR障碍执行。16份相关源码本机/服务器原始SHA全部一致（README随后仅补文档说明，现场冻结前同步）；服务器原生产HEAD仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且干净。证据 test_artifacts/server_validation_20261010/cusp_stop，另含21KiB闭包zip；无车辆运动/无完整搜寻成功，新的真实交接组件待验证。
- 用户新增提交/验证规则已写入主/辅助AGENTS.md，规则提交251ac18、3项文档一致性检查通过。原主仓库无提交历史；从已冻结309文件包恢复16份相关旧源码到Git基线a9dd4e3，未改运行文件或夹带21份历史暂存条目。这仅是相关模块基线，不是完整项目的干净版本。候选须独立提交并冻结后才继续组件；2次完整物理试跑仍失败，20轮未开始，显示大图更新及上游制动来源仍未闭环。

### 2026-10-10 停稳交接真实组件通过（完整搜寻仍待验）

- 修复提交65929a3，修复前相关模块基线a9dd4e3。提交包含新增测试、失败反例和项目记录；原21份历史暂存改动保留。Git检查保留原CRLF源码和原始诊断日志格式，仅清理graph_path_bridge.py末尾旧空白；原运行逻辑未因格式检查修改。README和末尾空白已同步服务器后冻结。
- 每轮前检查服务器：无原导航/理想载体/视觉命令竞争者，真实723节点、0固定节点、4538梁、1449.72kg、467车体碰撞节点及4×32轮胎碰撞节点，camera/LiDAR有效；可用59.3GiB。启动自己域79物理栈并停止该栈的GraphNav planner，组件由生产GraphPathBridge接管，不下发视觉query。静止参数、双costmap、传感器及CarLike规划检查通过，实际controller_server加载隔离候选libwildos_ackermann_rpp.so。
- cusp_stopped_component1在25.540s走完真实换向复合路径：1次maneuver_done、2次following、route_done，0交接重规划、0follow_failed。独立见证的交接位置误差0.181694m、航向误差0.115185rad、线速度0.009428m/s、角速度-0.004596rad/s、odom年龄0.101281s；完成前约0.3s多次零线/角命令，前一条仍为-0.2m/s，证明实际制动后再交接。路线末端误差0.515890m（末段仍是既有普通终点检查），真实末速度0.020563m/s，非原地掉头/瞬移/清障/调速。
- 成功组件按用户要求丢弃临时画面：front125帧、RViz208帧，未生成MP4，不冒充视觉搜寻录像。闭包cusp_stopped_component_evidence.zip约226KiB已取回本机并SHA一致、安全展开到 test_artifacts/server_validation_20261010/cusp_stopped_component；本机另有cusp_stopped_component_stack_stop.json。13个本轮栈进程自行退出、remaining=[]，引擎Pause ACK收到、停栈后速度0.025965m/s，保留BeamNG/传感器/RViz。原生产未变。
- 新tools/test_cusp_component_evidence.py的4项检查全部通过，独立复核归档哈希/退出、实际停稳位姿与零命令时序、前后向8m曲率、真实车身/轮胎及组件范围。它验证已录这条组件的证据，不能证明所有现场路径或完整搜寻。完整绿色房屋2轮仍失败、20轮/80%未开始，蓝房修复/多目标未验收；剩余显示大图丢帧与全制动上游来源继续排查后才能重开完整试跑。旧309文件部署包尚未包含本次停稳交接修复，下次整体部署前须重新刷新；本次组件直接使用已核验的服务器隔离源码/共享库。

### 2026-10-10 用户取消录像：无视频验收与制动诊断

- 最新用户目标明确后续不录制视频，只需定位失败原因；此新要求优先于此前“失败必须取回MP4”约定。主/辅助AGENTS.md已同步入口。保留事件、接口bag和少量严格匹配的到达RGB+mask图像；完整视觉到达及真实执行仍要验收，20轮至少16成功后才多目标。用户未要求删除其余已有证据。
- 本机先建相关源码基线：ad5a2f7保存原RViz链路7文件；aeac9c1保存录像工具、输入健康及速度环7文件。压缩RViz方案尚未实施，用户改要求后转向行驶数据流与无录像流程，不把显示准备当成生产修复。
- 正式tools/town_physical_single_target_trial.py移除JPEG磁盘缓冲、ScreenCapture/ffmpeg及其异常终止搜寻逻辑，Harness默认record_video=False；仍保留64帧到达关联缓存、真实query握手、独立停驶/引擎停稳、超时与真实物理验收。报告video_recording_enabled=False/recordings={}。接口bag加入StateSensor、cmd_vel/control/native/execution_feedback、TF及双costmap/plan，便于复查真正的第一违约点；不存整段RGB或点云。
- server_test_preflight.py默认不将RViz/visualization进程与预览输出列为车辆启动必要条件，显式--require-visualization仍可完整检查显示；核心规划、两控制门、单一执行端、模型/query依赖、障碍mark/clear和StateSensor健康保留。相机独立要求合法身份、至少2个推进时戳和末帧新鲜，不能因去掉显示检查就接受真实相机冻结。磁盘下限改为1GiB用于接口bag/到达图像，后续按实际大小控制并核验取回。显示大图丢帧仍未解决，但不再为了录像阻塞车辆流程。
- cmd_vel_to_control.py仅增加诊断字段：command/state/native单调接收年龄，StateSensor源ROS戳/身份/年龄，native源wall_time、健康与当前/待换挡；不改变0.5s守护、控制公式、速度、车体、碰撞或换向策略。不同钟的字段明确命名；源年龄仅用于证据，没有混入原单调安全计时。
- 本机/服务器276项回调替身（新增过期状态制动/恢复与缺席字段null）、11项Harness、7项图像健康和3项无显示准备策略检查均通过。服务器实际域85 DDS合成输入检查经过healthy/state_missing/state_recovered三阶段，输出14/17/10条实际速度环状态；缺状态时仍brake=1、actuating=False，cmd/native接收仍新鲜；恢复后tracking/brake=0。无Nav2/BeamNG连接或真实车辆运动，不能据此声称pilot2制动唯一原因已确认。原生产未修改。
- 复核pilot2日志：第二次execution_stalled在1791604369.236新FollowPath开始、1791604369.436取消，约0.2秒；前面处于多个规划候选/地图等待。结合实际全制动11秒窗口，支持已经修复的跨阶段停滞积时反例，仍缺旧速度环订阅的完整状态原因。下轮有完整控制/时间戳/地图证据再核验；不把vision在完整试跑结束后引擎暂停期间重复TF告警当作搜索中的模型失败。
- 无录像候选交付前复核：本机276项回调、27项准备/图像/Harness/规则检查通过；Harness补3项失败静帧反例，现14项。失败只留一张原始RGB，标注源戳、frame_id、源ROS年龄和is_arrival_evidence=False；无相机明确缺失，不用旧掩码覆盖新画面，也不把过期帧当到达。静帧保存失败不覆盖原始失败原因，停驶仍独立执行。2026-10-10再核实域79仅保留BeamNG、传感器、RViz/显示，无残留控制/导航/视觉节点，可用59.3GiB；下一轮部署前须核对新工具/速度环和实际插件来源。上述是离线及服务器隔离验证，尚未重开第三次完整搜寻，不计20轮成功。

### 2026-10-10 第三次真实绿色房屋失败：路径方向反转已复现，停止完整实验

- 无录像/诊断修复已提交8baa675680735db7dadec47e62e3df109db9491d。选择性部署10份工具/速度环文件并核对原始SHA，服务器新版14项Harness通过；保留服务器vision/nav.py已有差异，没有用本机整体包覆盖生产。完整生产HEAD仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且干净。
- pilot3准备先检查无旧控制/导航/视觉竞争者；在同地图轮次准备阶段回Town原出生点，明确真实自动挡/空挡/制动，停稳速度0.000206m/s、723节点/0固定/467车体碰撞节点、四轮各32碰撞节点及camera/LiDAR通过。启动本轮自己域79栈和视觉/定位，冻结38文件及既有停稳组件源码，核对实际加载隔离RPP库；完整预检passed、query ready+ACK后才启用两门。没有station预置、搜寻期间瞬移、地图重载、调速或视频。
- physical_green_house_pilot3在420.089s超时、累计行驶143.808m、仅1条mask、0confirmation、0严格到达。完成5次maneuver_done，3次FollowPath中止（两次实际受限圆弧碰撞、一次Nav2 Failed to make progress），1次交接因footprint_collision重新规划，末段反复no_vehicle_path。没有execution_stalled误取消；不能仅据此证明全部停驶问题消除。停门/SDK停车/暂停均成功，末速度0.0000699m/s，无shutdown_errors；本轮栈/视觉/定位按身份停止，remaining=[]。已连续3次完整物理失败，**停止完整搜寻实验，严谨修复完成相关接口/组件验收前不启动下一轮**；目标仍活动，20轮/80%尚未开始。
- 证据闭包pilot3_closed_evidence.zip约16MiB，SHA256 a432f35a214f955bbe4c584aa529c9833e1692f8bea4f22e772d0fb9189bf343，46文件逐份核对，安全展开到test_artifacts/server_validation_20261010/pilot3_closed。包含已关闭94MiB原始接口bag、事件、源码/库冻结、进程清理、两张途中及一张失败原始RGB，不录视频；失败图源年龄0.458s，明确不是到达证据。服务器取回前约59.18GiB可用，不删除唯一原始日志。
- 最早执行反例在约193s：路径已裁剪为单一终点，车体局部坐标约(-0.0098,-0.2052)m，yaw误差约0.0235rad；横向误差略超0.2m换向点门槛。RPP以carrot.x的符号决定速度方向，把同一前进运动段自行反转；之后175–270s共29次正负速度翻转，1151条gear_change、741条tracking（20Hz），循环制动。状态/原生接收年龄最大0.104/0.061s，命令最大0.361s，该窗口没有stale_state/native_unavailable，不支持通信延迟或物理卡死作为此故障根因。StateSensor原始源戳与ROS约29853s偏置是仿真暂停后的钟域差，不能当成真实接收延迟。
- ackermann_rpp/src/controller.cpp候选：setPlan从完整几何+车体heading保存单段方向，拒绝混向/非法路径，路径与方向同锁更新；剪枝后非零命令不能自行反转，抛既有Nav2 PlannerException交给路线归属者重规划，不扩容差、不伪造到达、不引入隐式换向。真正进入停稳终点区域仍先输出零；新合法路径可显式换向。既有速度环换挡、碰撞、曲率、目标/记忆身份及取消权限不变。失败执行反馈原逻辑只允许成功且运动证据有效时学习，未新增地形失败学习。
- 修复前真实插件11项中2失败，保留XML/日志；修复后12项插件+3项曲率全通过、0失败/跳过，含前后向侧偏裁剪反例、终点在身后但真实已到时停车、显式新路径换向、混向替换拒绝不破坏原路径，以及本轮17运动段/462姿态的真实几何仍可接收。服务器域85合成地图、无BeamNG动作；本机17运行契约通过。direction_validation_evidence.zip保存源码/库SHA、构建和前后日志；它证明方向契约反例已修复，不证明现场到达或80%稳定率。
- 另一未解决的规划边界：末段218条返回路径中212条在最近先前发布全局地图上存在车体碰撞，典型第一姿态格网量化为(706.375,-46.125),yaw=-1.047197，边界阻塞约body(3.41,-1.10)m；实测末位姿(706.387983,-46.142000),yaw=-1.010699在新鲜双地图center=0、footprint_blockers=[]。位置/航向离散化造成的车体角点差异是需要复现的接口问题；记录使用的是发布快照，不是Smac私有地图，尚不能断言唯一根因、误报或删除障碍。后续优先查实际起点→格网量化→返回路径→接续的物理可行性，同时保留真实碰撞保护。仅1条目标mask的视觉覆盖/识别原因也未闭环。

### 2026-10-10 Smac 全局膨胀与真实车体碰撞检查约定冲突

- 连续三次完整失败后的严谨排查，未恢复完整搜寻或控制车辆。SSH现有22451会话与闭包快照核实；320×320、0.25m、origin(666.25,-86)的先前发布地图对应plan1791631204.666751/地图1791631204.562189/odom1791631204.602905。位置连续与离散首点相差约两厘米，航向约2.09度；实际安装Nav21.1.20 C++边界检查实际pose代价253、只换位置253、只换heading254、首点254。量化航向影响已在同一快照复现，但发布地图不是当时私有地图，不能认定所有停驶的唯一原因。
- 更直接的约定冲突已复现：全局inflation_radius=2.0、cost_scaling_factor=2.0，后轴SBR加垫角点距离约3.571m；安装版InflationLayer计算circumscribed阈值1，Smac中心cost0<1便跳过完整矩形检查。实际checker在完整检查为254时，开启该配置优化返回无碰撞。仅依据“已开启碰撞检查”仍不够，必须满足优化所依赖的膨胀覆盖范围。
- 实际安装SmacPlannerHybrid+InflationLayer对同一致命/未知格重建地图，旧2m中心0、错误放行量化起点、返回73点；3.75m中心3、完整检查判碰撞、拒绝返回0点。相同输入证明配置/优化机制冲突，不是原私有地图逐字节重放，软代价及现场地图竞争不冒充已验证。原GraphPathBridge拒绝危险首点是有效保护，不能删除该保护。
- 本机候选：nav2_ackermann_control.yaml只改全局inflation_radius为3.75，按加垫最大角点+半格对角线推导；runtime_contract.py增加Smac覆盖检查和live resolution/cost_scaling_factor核对，防短范围回归。局部1.6m、footprint、真实车体/轮胎、8m曲率、车速、观测、unknown与停止/交接策略保留。扩大低代价场可能影响规划代价/计算量，尚无现场性能验收。
- 新独立tools/cpp_navigation_probe链接真实1.1.20插件，不给生产RPP包增加Smac依赖。实际C++两CTest目标通过，含9碰撞控制+4规划/膨胀控制；覆盖错误放行、正确拒绝、无障碍/移除后恢复。诊断初两次失败因仅setRobotFootprint未改变Costmap2DROS radius模式，已改显式footprint并断言getUseRadius=false；失败日志保留，不误诊生产。19本机运行契约、5归档/源码/实际结果证据、3规则及3无视频策略检查通过。
- 证据test_artifacts/server_validation_20261010/smac_boundary_evidence.zip，149774字节，SHA92569ea1e9b9f3e03f03f255accdd65eb30fbc9714869306b821442c9783d64a；实际构建/二进制/源码/版本/成员哈希、前后JSON、快照与失败尝试保留，取回校验通过。说明PILOT3_SMAC_INFLATION_REPAIR.md。仅增加约150KiB压缩证据，不录像。
- 边界：候选使错误起点被正确拒绝，**没有让离散化后碰撞的起点可行，也未修复连续实际pose→物理路径接续**。下一步修连续起点/接续，保持全路径曲率和两图真实车体碰撞保护；不能以放宽容差、缩车、删障碍掩盖。候选尚未上线实际车辆；完整实验继续停止，20轮/80%和多目标未开始，目标保持活动。

### 2026-10-11 本机离线检测、连续起点接续证据与校验精度

- 最新用户仅要求离线检测；本次没有连接服务器、启动车辆或录视频。详细报告`docs/OFFLINE_PLANNING_AUDIT_20261011.md`。正式GraphPathBridge仍use_start=False，尚未接入连续起点曲线；扩大膨胀后正确拒绝坏格网起点不会自动恢复可行执行，不能声称该问题已修好。
- 补记上次未收尾诊断：真实OMPL1.7/Nav21.1.20隔离工具在pilot3闭包同帧两图寻找8m半径/完整车体可行接续，57528个anchor中1444个几何可行，最优倒车0.9228m/0内部换向点/21采样姿态，实际Smac后续71姿态。3CTest目标（9碰撞、4膨胀规划、8曲线控制）通过。工具无BeamNG/action，未修改实际候选栈的graph_route；数量不是成功率，穷举也未测生产时延，不能直接部署。
- 新已复现校验缺陷：精确航向前缀+离散Smac后续在旧validate_vehicle_path中因all-bin条件失败，被整条解释为精确航向，5条均excess_curvature。上次待提交修改改为端点半bin不确定度、microedge仅双方量化时处理；本次回归4条通过，1条仍严格拒绝。8m正常、7.99/6m紧曲率拒绝、精确区间不授全bin、车体障碍拒绝均覆盖；碰撞、方向分段和到达机制未放宽。该校验修改仅离线验证，正式接续与车辆现场仍待实现/验收。
- 新接口风险：首次send_motion_segment没有after_maneuver的实际pose接入检查；validate_vehicle_path仅限制首点距离，validate_local_prefix检查当前车体与前缀，不证明中间接续。合成反例当前姿态90度、0.5m外0度直线，两门均通过。只作为接口范围缺口，不认定pilot3因此失败；修复时应统一首次/换向/替换路径接入约定。
- 坐标旧修复仍存在：官方adapter后轴补偿、速度补偿、yaw-only/z=0 nav_base_link与传感器3D分离；源时钟映射、旧帧拒绝、时钟重启清历史仍在。pilot3方向振荡窗口接收新鲜，原始时钟偏移不能当通信延迟；未证明全程传感器/推理无延迟。仅1条mask且无严格确认/到达的视觉原因仍未定位，不能因几何通过宣布找到目标。
- 归档anchor_connector_evidence.zip已取回：254704字节，SHAf3416451adaea3aa6b1fe388d737657f0bec2b29ac51f628398fd8ee21130882；本次校验成员/相关源码哈希。两图spot1/odom同实际起点、local age0.361812465s；发布地图重建不等于私有地图逐字节复现。本次没有重跑Linux C++，引用归档实际结果。
- 本机本次9接续/精度/初始接入范围诊断、24graph_route、19runtime、5既有Smac证据共57项通过；另修复前归档源码隔离24项通过。上轮276生产回调替身结果沿用，未重计现场或本次新增。初次TEMP权限和基线漏PYTHONPATH是诊断环境错误，已恢复且保留attempt1日志。证据offline_navigation/anchor_connector_tests.log、graph_route_recheck_20261011.log、runtime_contract_recheck_20261011.log、smac_snapshot_recheck_20261011.log、graph_route_baseline_20261011.log。
- Git风险：graph_route原为未跟踪既有模块，从原完整bundle精确取出修复前SHA81fb530aaa0aa00fd93b378100c64a6d42c00b986ceceb2e45b2c3c2cbaf9383，只有三处本次精度修改差异；基线提交7881b5e保存源/24项测试/两fixture及日志，不覆盖当前文件。其他历史暂存和未跟踪模块保留，后续部署必须全组件哈希冻结，不能只依赖HEAD。完整实验仍暂停运行、目标活动，20轮/80%及多目标未开始。

### 2026-10-11 重连服务器：实际起点故障确认与短接续部分执行

- 用户授权连接服务器继续，SSH新会话75196有效。原BeamNG2437498/传感器2440374/显示仍在；原生产/home/ly/wildos_ab_20261008_B_fix仍f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0且干净。真实723节点/0固定/4538梁/1449.72kg/467车体碰撞/四轮各32碰撞及camera/LiDAR有效，初始speed0.0001196。沿用地图，无瞬移/加载/关闭物理/录像；详细test_artifacts/server_validation_20261011/REPORT.md。
- 将a9f3b5b全局膨胀/契约及f7680e3航向精度选择性部署到原隔离候选，保留before_update和SHA；服务器19runtime+24graph_route通过。实际加载配置全局3.75m、CarLike/8m、RPP碰撞检查开启、两门false；RPP库b6aacf2af48914085f132df76b6c8d69bb2e9b7817927dc863790d5cdf20810e核实，方向修复仍有效。原生产/视觉文件未覆盖；不是全栈完整验收。
- 现场静止传感器/地图/参数均通过，实际CarLike前方3m请求status6，日志Starting point in lethal space。同发布地图连续实际pose边界253、离散首点254；真实Smac重建3.75m也拒绝。此前量化起点故障从闭包推断推进到当前实际动作复现，发布地图仍不是私有地图逐字节还原。live_collision.log的radius2是诊断硬编码旧公式，不是本轮实际配置。
- OMPL对当前双快照57528anchor中1471个几何可行，最优倒车0.922771m；穷举耗时1.992652s超过1.5s预算，不能直接接入生产。统一几何目标(725.375,-47.875,-1.134464)：实际pose直接请求status6/0点，5个显式安全anchor的真实CarLike均status4，拼接后4条通过两图/曲率/换向门、1条excess_curvature保留拒绝。不是视觉目标或到达证据，正式GraphPathBridge仍没有自动连续起点接续。
- 仅最优精确前缀真实FollowPath组件：5.057s/status4、倒车0.753462m、endpoint误差0.169576m、heading误差0.012949rad，命令半径约束与反向保持；但SDK即时speed0.056313>.05，**整体passed=false保留**。新诊断立即取最终SDK快照，缺既有完整工具有界等待；Harness.stop初步0.1门不替代最终0.05门，后轴投影速度与SDK总速度非同量。不能据瞬态宣布生产控制新bug或改验收阈值。
- 独立停稳恢复在保持制动/恢复模拟后0.112s降到0.03625，持续低于.05超过1s后0.01106m/s，Pause ACK；原部分组件不重标成功。后续组件应复用正式有界停稳与新鲜度/身份核验。只停止本轮PGID3825803所有栈进程，remaining=[]；域79无本轮导航/控制，仅保留传感器/显示。首次过宽断言碰到其他UID1009 NavGPT ROS节点，环境不可读，不触碰；证据保留，后续按所有者+实际域节点筛选，不能说服务器所有ROS全无。
- 小证据闭包reconnect_component_evidence.zip 183328字节/47原始成员，SHAdd7289deb2f68dcb8bb2cad90894746968a12cc6d91217c535a2a1bd8f66ae57取回并核验，本机6项新独立证据检查通过，含保留部分失败和实际停稳，不代替组件整体/完整到达验收。服务器约86.85GiB可用，没有视频。连续3次完整失败后仍不启动第4次完整搜寻，20轮/80%和多目标未开始，目标活动。

### 2026-10-11 自动连续起点接续与真实 bridge 组件闭环

- 在授权重连工作上继续，新增 ackermann_smac/Planner（真实Nav21.1.20+OMPL），保留Smac后续搜索，只在安全连续起点量化后碰撞时短曲线接续；8m半径/当前完整加垫车体，0.2s/6000次/6候选，Smac保留1.5s、统一2s总预算。当前图复验，精确anchor拼接，时间过期拒绝；锁/原生搜索不是硬实时抢占。参数运行冻结，停用改参数须cleanup/configure，分辨率变化/下采样/圆形足迹拒绝。配置/源码依赖/构建链接/打包入口/live契约已纳入新插件，原生产目录未改。
- GraphPathBridge首次/替换/cusp统一实际pose受限圆弧及双图扫体检查，保留各自失败归属和epoch/取消/异步响应约定，不放宽0.6m/.35rad或到达门、不删障碍/调速/改物理。首次90度heading缺口反例已被正式分发拒绝；正常小量化误差、替换失败归属、cusp重规划及双图检查回归通过。
- 开发新反例：初版第一圈够6候选就停，真实插件碰撞检查通过但115姿态拼接被生产曲率拒绝。保留拒绝，改通用距离下界停止条件，受原时间/数量限制；最终91姿态通过实际生产全局/局部/初始join校验。没有针对Town指定anchor/放宽曲率。编译Footprint声明和测试替身/临时重放构造错误已更正，失败日志保留，非生产根因。
- 本机140相关回调/契约、既有276回调回归、64历史几何/证据/运行契约、3停车失败恢复控制、5闭包证据最终通过；部署后143Python、真实10项C++通过、0失败/错误/跳过。组间重叠，不相加宣称独立总量；回调替身不等同DDS。真实插件覆盖预算/无候选/碰撞/恢复/配置生命周期/量化起点与正常路径。
- 现场 automatic_entry_component1 passed=true/errors=[]：同地图原始SBR车身/四轮碰撞及传感器有效，实际自动接续日志两次（4194/4270评估、6完整候选，搜索deadline触发），生产bridge真实倒车2.429204m、终点误差.582554m、两次maneuver_done/3following/route_done，0follow_failed或交接重规划。不是诊断指定anchor/瞬移/理想载体。SDK初停仍.103971，持续制动重新读至低于.05连续至少.5355s、最终.044311后才暂停。旧前缀passed=false保留，不重标成功。
- 栈PGID3918384按拥有者清理remaining=[]，引擎/原传感器/显示保留，车辆停稳仿真暂停；未触碰其他用户进程。原生产/home/ly/wildos_ab_20261008_B_fix仍干净f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。候选实际新插件来自/home/ly/town_memory_validation_20261011/continuous_start_build/install，运行前source旧候选setup后新插件local_setup，并重新核实来源/域/残留。
- 核验更正：构建库与安装库SHA不同是CMake安装移除RUNPATH；ELF Build ID和.text SHA一致，实际加载SHA精确匹配安装库975d5bae5fd4b1fa51c54b3a61c314adf7fa40411d80af4d5c5300a2a24b1faa。原误比较断言保留，不误诊部署旧版本。历史现场证据测试改为核对当时冻结部署包，不再随当前配置升级产生虚假回归。
- 详细CONTINUOUS_START.md，闭包continuous_start_evidence.zip 252556字节/62原始成员，SHA7402251f50f92263aa1a5a16509dcbbede0c8774fae5897006c9c2f7d4024547已取回核对；无录像/大bag。只完成短几何执行，没有视觉query/到达，不能算绿色房屋或视觉记忆成功；完整三次失败后的暂停运行状态保持、目标活动，20轮/80%和多目标未开始。下一步核验剩余失败/恢复链路及视觉覆盖，再恢复完整单目标；全局返回仍须下游完整曲率/新鲜局部图检查。

### 2026-10-11 真实绿色房屋 pilot4：双图漏观测结构与执行停滞

- 用户授权重连继续后，核验旧query ACK、视觉输入、实际导入及当前冻结63eda3d；同地图轮次准备回Town一次，真实SBR物理/车身四轮碰撞和8m半径保留。完整预检通过，正式420.077s搜寻失败，0mask/定位/确认/严格到达，累计81.511m含往复；25次execution_stalled、1次follow_aborted、1次cusp footprint_collision。20轮和多目标未开始，不把本轮当成功。详情test_artifacts/server_validation_20261011/PILOT4.md。
- stall_interface_audit录包关联证明多次R挡、油门约.35/刹车0、四轮downForceRaw>500N、遥测/执行健康仍低速无进展；odom/两图/Twist/控制状态匹配接收年龄均<1s。这些窗口不支持指令丢失/看门狗/悬空作为直接原因，不外推整轮绝无传输故障。
- 同次159.718s停滞的后轴pose附近561条静态引擎向下射线发现31个>52.8m采样点，附近道路约51.99、结构最高约53.065；同次最新全局/局部发布图在这些点全部cost0，0～2m短倒车扫体均无lethal。只读探针前后车辆ID/位置/速度相同，不移动地图/车辆。支持真实场景结构与软件障碍地图不一致，尚无车身接触力/历史原始点云，不宣布物理碰撞唯一根因。
- 结束后另一位置的暂停原始LiDAR见证19帧，最后105303点，结构附近110点、其中72点z52.8～53.4，说明结构能够产生回波；不是早先停滞时采样证据。下一步按原始→转换/限流→自体/高度/地面过滤→标记/射线及足迹清除追首个违约点，保留有界同源戳点云见证；尚不针对Town降低阈值/提高油门/关碰撞。完整重跑暂停，先修复新失败。
- 新诊断字段缺口：物理速度环未发布target_angular，ExecutionFeedback旧消息兼容默认0；因此stall日志0不能解释为直行，bag Twist实际曲率±.125，原生转向存在。当前只记录，未修改正式协议/控制。BeamNG源戳与ROS墙钟暂停后约103094s偏置不是接收延迟；实际接收年龄另记。
- nvidia-smi内核595.91.07/NVML595.99.02不匹配，但实际torch2.7.0+cu128、RTX4090 CUDA小运算通过，视觉持续处理（813条无目标含预热）；不误诊GPU不可用，不改驱动。
- SDK最终.011592m/s、两控制门关、仿真暂停；本轮3个拥有PGID/子进程清理remaining=[]，原生产f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0干净。日志/bag/少量图像/52份冻结来源闭包pilot4_closed_evidence.zip 18155711字节/106成员，SHAd961fec210e0c053e7a372251fd5c79b819e8b0f51f92d186a2b2affd5992729本机取回核验，无录像。5项新证据检查最终通过，只是诊断一致性，不是修复/完整视觉记忆验收。初次测试误用downForce字段改为真实downForceRaw，attempt1保留；初次归档别名映射缺失已更正，最终逐项来源匹配。

### 2026-10-11 LiDAR 上游漏观测定位与真实传感器候选

- 延续授权服务器诊断，无新完整搜寻/录像。停滞历史姿态的暂停重建与恢复物理、全制动对照：旧挂载和新建同配置LiDAR均0结构回波，同世界位置/方向临时静态LiDAR约1200；静态传感器有/无vehicle参数均能看到。仅下游源点云重放时现有过滤及实际Nav2会标障，不支持过滤/costmap阈值作为该漏采首因。不是历史精确变形/扫描，仍不证明所有停滞都由接触造成。
- 最早已复现约定违背在BeamNG挂载扫描配置：3.6m、dir(0,-1,-.45)的360模式受方向影响，不能根据SDK文档假设dir无意义。仅水平化恢复75回波/5标记点；仅放大90度或降低1.7m保留下倾均0。1.7m+水平1695/33，四诊断航向1700/1401/967/496；只有方向/高度差异，没有Town目标/障碍专用阈值。四航向脚本原scope误写2次teleport，归档实际脚本为6次，保留更正；这是传感器姿态诊断，不是真实自主转向或搜寻。
- 新config/sbr_physical_sensor_rig.json仅传感器，保留相机/身份/采样/量程，无地图/出生点/车体碰撞覆盖字段；避免复制旧rig里的wildosBodyCollisionDisabled。只是候选准备，实际桥接双图/真实执行待部署验收。原生产未改、原车恢复停车位置，SDK .018337、camera[0]/lidar[1]保留。
- 实际独立Nav2重放1.7m水平数据：31静态采样点13 lethal，其余99、0 free。但独立costmap完成生命周期清理后SIGINT exit -11，初次-6；该诊断退出故障未修复，不宣称所有运行验证通过或把它当生产规划崩溃根因。源采集戳未保留与真实清除射线安装点的契约缺口仍待独立反例，当前原点对照未显示差异。
- 本机10配置/原始NPZ/四方向/恢复/归档检查、4既有挂载失败身份/端口反例通过；服务器4配置检查通过，全部为传感器诊断范围。21.74MB闭包lidar_chain_evidence.zip，SHA534b8b40280308bde3dea209bd81d19280daa3aa2117cd48d4ea6d990db56563，91原始成员已取回核验。tools/test_lidar_chain_evidence.py、lidar_chain_tests.log及docs/PHYSICAL_LIDAR_REPAIR_20261011.md为入口。脚本初次端口key/type错误已恢复并保留attempt1/2，未当生产新故障。
- pilot4仍完整失败，0严格视觉到达，未进入记忆返站；完整试跑暂停运行，目标活动，20轮/80%及多目标未开始。下一步同地图/同车体更换本任务rig，核查实际原始/标记/双地图，再恢复完整测试；相机、真实车体轮胎/碰撞、8m半径、规划/控制速度不改。

### 2026-10-11 LiDAR 实际部署、短执行与服务器重启

- 新传感器配置已在旧引擎同地图、同一真实SBR部署；旧传感器已清理，SDK空集合可能返回`{}`而非`[]`，最初空集合断言失败保留，不当作传感器泄漏。新相机28/LiDAR29、实际拥有者4033214（不是已退出的setsid包装PID4033213）。停滞历史姿态真实全制动组件收到1678结构回波、35标记/清除点；31个静态采样点全局12/local8 lethal，其余99、0 free，两图及各输入持续更新。整个真实栈退出0、拥有进程remaining=[]。这证明本次漏采修复到达双图，不证明历史全部停滞唯一原因。
- 首个短执行`lidar_execution_component`在参数/服务/动作准备阶段超时，未进入实际FollowPath，原因尚未定位，保留失败图像且不是到达证据。第二个`lidar_execution_component2`九项参数和真实CarLike规划通过；生产桥实际倒车2.436835m、终点误差.569522m，0follow_failed/0交接重规划/0no_vehicle_path。原始车体723节点/fixed0/车身467/四轮各32碰撞有效；制动等待实际速度<.05连续至少.5s，最终.0128251后暂停。仅3m短直线组件，不算绿色房屋或视觉记忆成功。独立costmap诊断退出-11仍未解决，不能因本次栈退出0覆盖旧失败。
- 闭包`test_artifacts/server_validation_20261011/lidar_rollout_evidence.zip`1553798字节、42原始成员，SHA256 `1e2cc12d388143aa7f793e5405204ee2adb377cebe79ce594541353396b1ac95`；配置/原始链/现场证据15项、既有过滤11项、适配器32项通过。首次clock测试文件模式未匹配导致0测试/exit5保留，正确文件32项通过，不把0测试当成功。
- 准备pilot5前SSH被远端关闭；重连确认服务器重启（boot_id `0d39fe7e-0d7e-4e4b-b49d-4ea6ac007dea`），旧引擎/传感器/全部任务栈已消失，25252未监听，pilot5_session未创建，故不是新增完整搜寻失败。服务器证据`/home/ly/town_memory_validation_20261011/server_reconnect_after_restart.json`。按授权恢复原始真实车体/新rig；恢复必须重新加载一次地图，作为新的实验准备记录，搜寻期间不重载，不能假称旧进程记忆延续。原生产上次核验仍干净f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。
- 用户最新提交规则已同步主/辅助AGENTS.md：只有本次改动所要求的测试及验证全部成功后才commit；失败、未执行或必需验收待验证时保留工作区和证据、不提交。三项规则一致性检查通过。此前已存在提交不重写；本次新记录和未完成验收改动暂留工作区。后续不录视频，20轮/80%及多目标仍未开始。

### 2026-10-11 恢复后 pilot5：共同接口响应空档，未完整成功

- 重启后原始引擎15057、原始SBR在Town准备停稳，新传感器15918/端口46381；新地图加载1次仅属于重启准备，搜索期间0重载/0瞬移，车身/四轮碰撞、8m半径和速度保留。candidate缺config目录导致首次rig复制失败，补目录后成功，准备反例保留，无车辆运动。视觉候选三模块和原生产explorfm依赖来源核验，新插件实际映射SHA与冻结一致、原生产仍干净f6d7ab。
- physical_green_house_pilot5完整预检/ACK通过，实际搜寻168.590s后因Native actuator unhealthy >.4s提前终止，累计74.454m含往复，0mask/确认/严格到达。早期2次follow_failed（tracking_deviation1.003m及motion_section_invalid）、7maneuver_done/3交接重规划、21等待地图更新、4segment_done，0execution_stalled。不能因后者比pilot4减少就宣称全流程成功，20轮/多目标仍未开始。
- 同录包167.54～172.24s原始StateSensor和odom约4.7s共同接收空档；native状态约2.052s空档，首次执行RPC2002.672ms超时，随后执行连接关闭并永久不健康。ROS control_status/native_control仍持续（最大间隔约.061s），支持先查BeamNG共同响应阻塞，而非单纯重规划/ROS控制整体冻结。缺故障期间GPU/引擎主线程/请求级记录，具体GPU竞争、同步传感器、物理计算或通信原因未定位，不盲目放宽超时。重启后nvidia-smi已正常；kernel日志读取受用户组权限限制，无entries不能证明没有GPU错误。
- 最终SDK速度.006002m/s、两门关闭、仿真暂停；本轮栈/模型/定位清理remaining=[]，保留引擎/本任务rig。无视频，保存一张失败RGB（源年龄约2.83s，不是到达证据）、全接口bag/时序/53来源。闭包pilot5_closed_evidence.zip8020554字节/92原始成员，SHA256 `2de4b391160ddf86ee874eb5fc56d3d92b0a11979310861320b0e4b72beeda4c`本机取回全员核验；首次误抄PTY折行hash断言失败，按完整metadata更正通过。REPORT为test_artifacts/server_validation_20261011/PILOT5.md；4项新证据检查通过仅为一致性。本次新规则/文档/测试保留未提交，完整失败不commit、不盲目开始下一轮，先排查共同响应空档及早期跟踪/交接。

### 2026-10-12 重连：Native 新鲜度契约修复，引擎空档尚未闭环

- 用户授权连接服务器修复。重连确认 boot_id 未变，BeamNG15057/本任务传感器15918保留，车辆16266/端口46381；无残留本任务导航/控制/模型进程。原生产仍干净 f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。同地图、真实723节点/0固定/467车身碰撞/四轮各32碰撞及新1.7m水平LiDAR有效，不录像、不调速、不瞬移、不重载。
- 200s静止完整物理栈/模型对照未复现pilot5共同空档，native3955/state1998/RGB1526接收；最大控制总耗时61.193ms、状态最大接收间隔.160s、RGB.800s，0执行错误。55s后启动GPU0视觉、110s后重复同一green query，不等于同负载移动对照，不证明GPU竞争/引擎冻结已排除。SDK典型视觉/GE/VE源码已记录：独立客户端可共用46381监听，但GE请求运行onPreRender、VE控制onDebugDraw，共享引擎回调；更正native“新监听端口完全隔离”的旧注释，不把它当功能修复。
- 四个新增反例在原代码全部失败：慢轮询以接收时间重置样本年龄并赋新StateSensor戳；控制RPC前算telemetry_age、返回后用新wall_time，使旧状态仍报fresh；速度环分别检查采样和传输年龄而未累加；接收后重新给完整.5s寿命。上述确定是逻辑错误，**尚未证明它们造成pilot5引擎4.7s响应空档**。
- 两模块最小修复：native以请求开始时间保守计龄，超过已有.5s保护不发布新StateSensor；控制返回后对同一缓存样本重算年龄，不替换并发新样本身份。速度环累加采样/传输/接收后稳态时间，重复旧消息不续命、明确未ACK不能用于启用；补control_acknowledged/control_rpc_ms/poll_roundtrip_ms/当前样本年龄和target_angular。ACK只表示输入接收，不等于实际运动。已有.5/.8/2s保护、超时锁定、曲率/碰撞/换挡/速度/目标交接不变。候选native SHA34a147703f2ea12cb94276343be9f575faa46948f7a0914f7486b74ea0ccdd0f，速度环 SHAfe0efe120509d5c72d50d9324087f140a4b01f9927f19a05d6d46baad7e599ce。
- 本机283项相关回调替身、服务器25项相关替身通过；域85真实DDS七阶段通过（累计超龄、重复、未ACK、恢复等），没有车辆执行。域79真实BeamNG/DDS五阶段通过：制动保持、实际读取/ACK后人为650ms延迟触发失效制动、新鲜数据恢复跟踪；合成1.2m/s意图仅流向无执行器订阅的私有话题，实际native全部零油门/全制动，末SDK.00018量级、低于.05连续至少.5s后暂停，组件拥有导航/控制进程remaining=[]。这不是自然引擎冻结或实际FollowPath到达验收。
- 保留诊断失败：补诊断字段时局部变量作用域错误被离线回归捕获、修正后再启动，未运行错误版本；首次真实组件“最后poll耗时”已被新轮询覆盖导致验收失败，独立保存注入交易后通过，原false保留；直接在ROS环境运行替身单测未装替身失败，正确安装25项通过，DDS另验；200s驱动未及时回收退出子进程的zombie导致原false保留，驱动退出后独立PGID核查零残留。不能将这些诊断尝试误记为原生产根因或静默改成成功。
- pilot5曲率审计682个重叠移动窗口支持正反向符号基本一致，不是独立成功数，不证明全部标定/跟踪无误。tracking_deviation、实际pose不满足8m接续，以及相机/LiDAR独立GPU采样戳缺口仍未闭环。下一步有界移动组件加请求/引擎/GPU见证，追共同空档首个违约点后恢复完整搜寻；20轮和记忆返站未开始，完整验收未通过。
- 详细docs/NATIVE_FRESHNESS_REPAIR_20261012.md；闭包test_artifacts/server_validation_20261012/freshness_interface_evidence.zip，1853406字节/51原始成员，SHA33031426f58eb75b1cd5bd7024411111b03dea5a1722d339bd8bb4a126b06ddb。本机逐员核验、5项新证据检查通过，无视频。服务器/home/ly/town_memory_validation_20261012保存源码前后、实际参数/请求、日志/资源采样/失败尝试。因完整移动故障及所需搜寻验收仍待验证，按AGENTS.md条件提交规则保留工作区、不commit。

### 2026-10-12 引擎原生崩溃、传感器桥生命周期修复与短执行

- 后续重连 boot_id 未变，但旧引擎15057和显示 :138 均消失，25252拒绝连接；旧传感器桥15918仍轮询并追加错误日志约62.5MiB，已核实拥有者后SIGTERM停止。13:29引擎日志明确记录minidump。异常线程15057/code11，指令落在libmimalloc.so.3+0x87c5释放例程；栈顶候选引擎+a41d38接近std::vector<char*>扩容符号，未完整展开，不能归因于特定模块，也不证明显示退出先导致崩溃。不是服务器重启，不与pilot5的4.7s空档合并成同一确定原因。
- 独立会话恢复Xvfb422169和BeamNG422172，Town准备加载1次、原始SBR16266/新端口34567，723节点/0固定/467车身/四轮各32碰撞有效。此后组件测试0重载/0瞬移；旧进程记忆不延续。未录视频、未调速度/8m半径/保护，原生产仍干净f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。
- 最小候选修复tools/attach_current_vehicle_sensors.py：CLI必须--engine-pid，仅本机，接入前核对命令/端口；pidfd固定进程身份，每.2s检查无需SDK心跳，全部SDK发送含重连/删除先查同一身份，禁止旧桥进入新实例。poll()/close短锁保护，不锁网络调用。ROS SignalHandlerOptions.NO让自定义停止顺序先停轮询、清自有传感器/桥、再关上下文。新增tools/check_sensor_attachment_lifecycle.py真实SIGTERM/SIGINT检查，两阶段退出0、空传感器、ID/位置/物理保持。真实pidfd存活通过，替代进程死亡/删除拒绝通过；不是实际引擎自然崩溃现场复现。
- 保留本次新增反例：首次默认ROS信号导致上下文先关闭、桥清理创建执行器失败，源码修正后重验；测试输出flush位置错误导致驱动退出1，原result及日志保留，改脚本后退出0。首个运动驱动错误把SSH shell当冲突进程，运动前失败，按角色精确筛选修正。新增pidfd监控首次被多SDK线程触发concurrent poll() invocation、桥退出、组件无RGB且0计划/0运动；这是本次新逻辑引入的问题，不是旧pilot5根因。新增并发反例在旧候选失败，加锁后本机/服务器最终各14相关测试通过，源码前后及field false均保留。
- physical_execution_probe3实际CarLike→桥→FollowPath短倒车位移2.491667m、终点误差.509517m；physical_execution_curve_probe1相对目标(5m,1m,+.25rad)短单向曲线位移4.411764m、终点误差.688115m。两组件均通过、0follow_failed/0交接重规划/0残留，本任务传感器退出0并移除。模型/定位未启动，不算绿色房屋/记忆返站/20轮成功数；短曲线没有覆盖换向交接，原失败仍未闭环。
- 历史pilot5接续fixture：8.013s下一段起点737.050537/-12.774487/yaw2.879793，邻近实际pose736.942609/-12.752077/yaw2.944523；几何诊断能返回有界接近弧，不证明全段跟踪收敛。按received_global_plan的真实车体frame计算，32～43s90条名义请求全部超.125，Twist实际被8m约束；8～16s粗略名义计算已超限。只是接续/曲率余量线索，不是精确插件重放或增大转角能治愈的证明，规划/控制尚未改。初次诊断错误将车体frame当世界坐标重复变换，虚假约725m距离文件保留并作废，由pilot5_tracking_body_audit.json明确更正；不是生产坐标错误。
- 最终SDK速度.0084487565、暂停、无本任务导航/控制/模型/传感器残留、传感器空，错误日志最终66,701,965字节且不再增长；引擎/显示保留。共享磁盘余量约46.73GB，本轮证据压缩后仅779244字节。engine_lifecycle_execution_evidence.zip/133原始成员，SHA4779f4e82784a45e294c9cb3b1d44fe799e934c3839f1c39458c86ac121800cc本机取回逐员核验、7项新证据检查通过。详情docs/ENGINE_LIFECYCLE_REPAIR_20261012.md。首次归档找错Smac源目录失败，补冻结实际continuous_start_build源码后归档，无删除旧反例。
- 继续保持完整试跑修复门槛：引擎共同响应阻塞/原生崩溃来源、长曲线路径跟踪/换向接续、独立costmap退出异常未解决；需要历史契约反例和相关恢复/交接验收后恢复完整搜寻。20轮/80%和多目标未开始，按AGENTS条件提交规则未commit，工作目标仍活动。

### 2026-10-12 转向 SDK 滤波定位、候选修复与移动响应空档复现

- 继续授权 SSH 工作。读取实际 BeamNG0.39.4 的 techCore/inputFilters/input/hydros 源码：普通 SDK Control 对转向使用 FILTER_PAD=1，ACK 后仍有输入滤波；FILTER_DIRECT=2 能直接下发输入。真实全制动/零油门/空挡 probe2：PAD 的 ±.5611 输入 t90 为 .4372/.4094s；候选 DIRECT 首个 .0063/.0069s 采样已匹配请求，ACK 都约1～3ms。hydros state 仍按原1.25速率追赶 cmd，真实转向机构/车身/轮胎保持。probe1 的 rack 实为 steering_input，probe2 更正标签并补 hydros；不把输入当轮角，不宣称所有 DIRECT 辅助均关闭。
- direct_vehicle_control.py 只改 Native actuator 实例：一次同步 Lua 请求，转向DIRECT、其他输入保留PAD及原挡位，有限/范围/类型与滤波常量先验，协议ACK错误/超时走现有锁定失败。native_vehicle_control.py 安装候选并发布接口版本、steeringUnassisted。没有调速度/.5/.8/2秒保护/8m半径/车体或碰撞；原生产仍干净 f6d7ab。新增输入、单请求、错误ACK、实例隔离、真实tick锁定不重试及制动挡位回归；本机/服务器相关替身各52项通过，真实BeamNG/DDS五阶段全制动延迟注入/恢复通过，尚非实际路径修复验收。
- 实际安装RPP+pilot5的55姿态/19.119m段，在FREE_SPACE理想自行车闭环中实际/名义起点均到达、最大偏离.614801/.234789m，原预期实际起点失败的断言被推翻，exit1/result false原样保留。饱和本身不足以解释现场失败；未更改RPP/放宽曲率。
- direct_steering_curve1/2 在起步前没有满足原约束的有界路径，控制门关闭，无运动。一条Nav2成功返回路径被生产 excess_curvature 拒绝，纯几何重放定位heading窗口16～20：2.11766m上.370957rad，.17517/m>.125；量化/解析接续/平滑的具体来源未定，不把该拒绝直接解释成真实道路不可行。其他采样规划失败，不能归因于转向候选。
- direct_steering_short_curve1 选择5.26176m安全倒车曲线并真实执行，因 Native receipt stale 失败。原生状态请求和 QueueLuaCommandVE 同窗口约2.002s超时；独立传感器请求4.78363s、GE读取.896s。无模型/定位进程，不能归因于视觉记忆/模型负载。GPU低利用率窗口只是线索，缺主线程堆栈，未确定渲染、物理或通信根因；与旧native崩溃不合并归因。
- 已确认新的停止链缺口：客户端超时锁定后，不再发送RPC，ROS watchdog_stop/sent制动意图不能代表引擎收到制动；实际brake0、速度仍>.5m/s。本轮原停车和持续SDK停稳失败，原false保留；独立新连接全制动后低于.05连续至少.5s，最终.001495m/s、仿真暂停。随后出现route_done也不能作为健康执行/停车成功，需查动作终态与actuator health交接，并增加模拟器内过期指令制动租约；两项尚未修复。全部引擎回调冻结期间Lua也不能立即运行，须验证恢复后的行为，不宣称watchdog可治愈引擎冻结。
- 最终boot_id未变、引擎422172/显示422169存活，723节点/0固定/467车身/四轮各32碰撞有效，0重载/0瞬移；全部本任务组件清理、传感器空、原生产干净。共享磁盘余量约23.88GB，新增闭包仅3,328,064字节/124原始成员，direct_steering_evidence.zip SHA d5ecfe949c0d29bb134ad81ee3a740ff668f6fd56ed245c47187075511872462，本机逐员核验、6新证据测试通过，无视频。运行helper与最终只更正docstring，运行版本另存并匹配三个组件冻结SHA；两个历史证据测试改核对归档源码与当时部署，5+7通过，旧包未修改。
- 详细 docs/DIRECT_STEERING_REPAIR_20261012.md。本轮短运动仍失败、长曲线/cusp及完整目标流程待验收；独立costmap退出故障及引擎空档来源未解决，保持完整搜寻修复门槛，20轮/80%及多目标未开始，按用户规则未commit。下一步先做模拟器控制租约与失效终态回归，再排空档来源、恢复路径执行验收。

### 2026-10-12 执行健康门控与迟到动作成功交接修复

- 按活动目标继续，服务器boot/引擎422172/显示422169未变，无新完整现场试跑。4项离线反例/5失败子检查确认：失败actuator仍进入端点/动作成功终态，重复Native消息续接收年龄，未ACK制动意图被当已送达输入。transport_counterexamples_before2.log保留；首个runner导入旧TestCase带入43无关测试，第二次模块导入只计4项，不放大故障数。
- 修改GraphPathBridge/execution_feedback.py：启用执行的真实Ackermann才要求Native健康、ACK及样本+传输+接收后累计年龄≤原.5s；重复/乱序不续命。启动/分发/监控/接受/完成及经验学习共同检查健康。明确actuator failure使动作epoch失效、取消所属动作、清队列/替换/恢复，迟到成功不能推进完成；execution_unavailable保留目标/地图/记忆，不计路线不可达，不增加几何重试次数，保留旧退避。ACK仍不是实际停车，取消请求也不能保证车辆停下。
- 新9项回调覆盖故障/缺席/未ACK/累计超龄/重复/迟到接受/旧成功/正常恢复/几何退避；本机完整runner299通过、相关195通过，服务器相关195通过，覆盖重叠不相加。工具runner纳入新健康和先前DIRECT测试，车体/碰撞/速度/8m半径/到达阈值不改。
- 服务器独立localhost域86真实DDS/action result1、最终result2通过：人工Native健康时实际action接受，人工失败后桥请求取消并保留目标；服务器故意拒绝取消且迟到SUCCEEDED，旧epoch结果不能产生route_done/segment_done/maneuver_done。恢复人工健康后目的地仍保留。坐标/地图/状态/action server全是隔离测试输入，未连接BeamNG执行器，不证明真实FollowPath运动、停车或视觉到达。下一项车辆端过期指令制动租约尚未实施，共同引擎空档根因仍未定。
- 保留开发失败：旧夹具缺ACK/刷新健康、导入路径缺official_adapter、恢复夹具缺服务就绪，纠正后通过；大批base64工具输出含省略导致测试解码拒绝，两个核心源码SHA核对正确，改SCP并先验SHA后才测试。没有运行损坏候选。执行经验成功学习还需健康证据，防止失效污染图边评分。
- 最终SDK .001495、暂停、原始723节点/0固定/467车身/四轮32碰撞有效；无本任务组件/传感器/域86残留，原生产f6d7ab仍干净。新增闭包execution_health_evidence.zip 113656字节/21原始成员，SHA8c0329b51138def848f4c8d4829510062cb8965d04fd5ef7320d8febec8b38a0本机取回逐员核验、7部署源码一致；无视频。详情docs/EXECUTION_HEALTH_REPAIR_20261012.md，历史证据只核对冻结版本；20轮/80%及多目标未开始，必要车辆端停止/长曲线/cusp/完整到达未验收，不commit，目标活动。

### 2026-10-12 车端指令租约与真实运动失效停车验证

- 延续授权 SSH，同 boot/BeamNG422172/Xvfb422169、同真实SBR16266。新增 vehicle_control_lease.lua/.py，以真实 HighPerfTimer 墙钟、原 .8s command_timeout 在车端 GFX/physics 回调检查过期并设置普通全制动/驻车/零油门；不改物理、车体/轮胎碰撞、hydros、速度、8m半径或到达阈值。源文件/加载源码固定 SHA，未知版本拒绝覆盖；扩展存在检查用 rawget，避免自动加载副作用。setup.py 已包含 Lua 包资源。
- Native 在改 AI/挡位模式之前取得 owner；有效控制请求先检查 owner/递增 sequence/有效期，再设置输入，ACK 匹配 owner/sequence。旧 owner 过期不可续命，新 owner 须真实停稳；移动中释放保持制动，静止才解除。执行失败不自动重连，退出制动失败仍通过现有遥测连接尝试释放自身租约，并保留错误。只读 status 不触发被测制动。SDK 外部控制仍可用于恢复，租约不是引擎所有客户端的权限隔离。
- 本机/服务器完整回调 runner 各307通过（重叠不相加）；新Python租约5项和退出3项包含其中。实际VE Lua VM 14项用隔离假输入/时钟/速度测试，不控制实际车辆；wheel 构建资源核对通过。stationary2实际全制动回调过期、旧请求拒绝、新owner释放通过；freshness1实际BeamNG/DDS五阶段650ms延迟/恢复通过，无行驶输入。
- short_curve1真实CarLike→Bridge→FollowPath正常短曲线通过：5.02375m路径、实际移动4.54211m、端点误差.558157m，SDK速度持续<.05至少.5s后暂停。只是几何执行，不是视觉到达；此前DIRECT运动失败未重标成功，偶发空档没有因此解释为已消失。
- moving_fault2在真实双图验证后的安全规划路径上，只SIGSTOP本轮拥有的Native。独立66份故障采样首速1.61376、油门.02710/刹车0；.782008s观察到租约过期全制动，峰值2.03953，末速.00425059且持续停稳。故障后总位移1.82539m，.8s不是零停车距离。读status/退出制动之前已停稳；SIGCONT后新鲜遥测age.004764、speed.0088214、实际油门0/刹车1，旧owner被拒绝，actuator/ACK false。桥execution_unavailable，无route_done。证明本轮控制进程停摆后的真实停止及终态保护，不证明BeamNG全部回调冻结时立即制动。
- 保留测试工具失败：stationary1混用旧helper签名，无运动，完整冻结部署后2通过；server_related误把替身测试放进真实ROS类型/未init环境，8失败9错误，正式runner307通过；moving_fault1原result true但故障数组被退出数组覆盖、恢复记录过期，diagnostic_review判证据失败，原不改写。moving_fault2分离数组且要求新鲜恢复，最终通过。首次wheel临时目录权限错误，改项目内目录通过。没有为这些启动/采样错误修改生产行为。
- 归档vehicle_control_lease_evidence.zip本机取回：795309字节/118原始成员，SHA25f108cd01452c611030f8b5cdb8561eef9c5b2373a6392ee249d982c3c84c07。逐成员核验、10个本机部署文件一致，6项新证据检查通过。源码/运行/备份和失败均保留，不录像/无大bag。详细docs/VEHICLE_CONTROL_LEASE_20261012.md；实际车端加载源码SHAa02898bdd7bfa94458a2b5447d75797b3bd8404b9fdb10a7f2f024f0bfc16377。
- 最终SDK .00570656、停稳/暂停、租约已释放、传感器空、本任务组件remaining=[]；原生产仍干净f6d7ab，723/0固定/467车体/四轮32碰撞保持，磁盘约25.41GB可用。回退须先确认所属控制停止、实际停稳及租约释放，不能带活动守护启动旧客户端。引擎请求共同空档/旧崩溃来源、独立costmap退出故障、复杂路径曲率来源、长曲线/cusp及完整搜寻仍待验收；20轮/80%及多目标未开始，不commit，目标活动。

### 2026-10-13 切换离线后的执行契约审计与未闭环接续

- 用户最新要求“现在离线检测，彻查原因”。本轮未重新连接服务器，未修改运行代码，新增 tools/offline_execution_contract_audit.py、对应6项诊断测试及 docs/OFFLINE_EXECUTION_AUDIT_20261013.md；证据 test_artifacts/offline_execution_audit_20261013/。冻结的15项生产源码/配置前后未变，当前8个文件与此前 handoff_endpoint/candidate2.zip 一致，核对见 verification.json。必要现场门槛待验收，不commit。
- 补记上一阶段现场交接证据范围：cusp1未选出满足组件长度的换向路径；cusp2完成前两段后下一交接因实际姿态不能接续而重规划，旧测试工具拒绝任何该状态；cusp3第二短倒车段实际FollowPath报“Ackermann lookahead would reverse a single-direction path”。其前后3个新鲜SDK样本已保存 nav2_adapter/test/fixtures/physical_short_cusp_join_20261012.json（SHA bf17d756b716289d160bb1c497189b76499976e542c57f0a4afa5119703b710d），不是精确bridge回调pose。完整cusp1～4原始目录尚未本机归档，不将对话摘要等同完整原始证据。
- 已有候选修复：motion_path诊断增加endpoint_position_tolerance，GraphPathBridge先选实际goal checker再以同一位置容差校验短段；.6m旧接近预测不能替代.2m物理cusp停车盘。增加精确下发pose/stamp及checker日志，17项几何、309项回调已通过。当前服务器候选此前已部署，本轮只核对本机包；现场换向修复验收尚未通过。
- 新离线几何复现：3个记录姿态对上一cusp误差约.172m/.140rad，允许完成，但旧join预测偏差.20737～.21009m仍被接受。零延迟/瞬时转向/无障碍的端点pure-pursuit自行车重放均错过.2m停车盘，最近.20730～.21001m后触发方向保护；名义起点同路径可达。1/2/5mm步长和刚体坐标变换结论一致；正式三维后轴变换仅比原二维近似差约.17～.19mm，仍失败。该重放非实际C++/DDS/BeamNG，但证明此反例存在独立几何交接问题，不依赖模型、地形或通信延迟。当前紧容差会提前拒绝这些路径，不能据此称新路线可执行。
- 新发现正式恢复范围缺口：graph_path_bridge.py:1534的after_maneuver拒绝直接start_segment，不计recovery_attempts/退避。真实生产回调+ROS替身对同一拒绝注入8次，start_segment调用8次、计数仍0、blocked为空、0动作发送；是分支预算反例，不是现场自主循环8次。测试工具有界不能替代正式逻辑有界。
- 新发现候选失败归属：实际pose接入检查在候选比价后；首次下发前失败进入follow_failed而非继续候选。离线2候选反例0动作发送、候选索引仍0、next_candidate0、整路退避2s；不声称第二候选已验证可行。应先把实际接续可行性纳入候选筛选、保留下发时复验，并统一有实际进展的恢复预算，避免只把跟踪失败提前成重复规划。
- 当前正式校验器在足够大空图重放旧direct_steering_curve1的55姿态成功返回路径，仍excess_curvature；窗口16～20约2.11766m/.37096rad/.17517每米。无同请求平滑前路径，无法在量化/解析接续/平滑/解释间定责；插件普通分支只保证碰撞/预算，不保证下游相同曲率与接续约定。原理/模型或道路不可达不能由此推断。诊断首轮人工空图太小先报collision_cell，attempt1保留并更正，非新生产碰撞。
- 离图地形风险仍独立：当前双图已接障碍、FollowPath碰撞开启，旧“完全没障碍”不适用；但只有obstacle/inflation层、unknown=false，图traversability没有直接约束Nav2整片空间。默认通道19.4m；合成8m半径半圆离图边8m，空图接受、标障后拒绝。不是实测灌木漏检证明，说明图边可行不能替代周围运动区域可行。不得用关物理或硬缩通道掩盖。
- 坐标/时钟32项、几何17项、组件工具8项、当前回调309项及新诊断6项通过，覆盖有重叠；诊断测试通过只说明能重现未修问题。本轮未运行真实C++/DDS/物理。旧共同请求空档2.002/4.784s及引擎崩溃、独立costmap退出仍未定根因；长路径固定3m接续预测与实际3～6m前视范围需另验。
- cusp4切离线前已读结果：发布者计数断言 /wildos/native_control_status 失败，未进入规划运动；没有实际数量/端点证据，不能判定DDS发现延迟、重复owner或Native崩溃，也不是修复后换向失败证据。最后读到本轮拥有者清理/传感器空、暂停，速度约.00005m/s；不是本轮重新核实服务器状态。简要回读已保存cusp4_readback.json并标注非完整归档。
- 下一步先处理候选可行性与统一恢复归属，再验证短换向/长曲线，追原始与平滑后路径，再补地形和独立引擎延迟。完整绿色房屋/多目标仍未验收，20轮成功率统计未开始。

### 2026-10-13 理想点载体与真实车体差异核对（离线）

- 用户指出ideal车体不出现当前问题。当前ideal_point不仅关闭物理：NavFn/原版RPP/原地转向/0.15m点代理/整段动作；真实为CarLike REEDS_SHEPP/自定义Ackermann RPP/8m半径/整车轮廓/前后向分段与StoppedGoalChecker交接。目标接近点前车体安全长度也不同。不是只改碰撞的控制变量实验；当前两者均开启Nav2障碍检查，不重复“没接障碍”的旧诊断。详见docs/IDEAL_PHYSICAL_COMPARISON_20261013.md。
- 更正术语：上轮“零延迟理想自行车失败”仍有8m半径，不是用户的ideal_point。新增compare_carrier_models_offline.py读取当前Lua XY算式，以相同3个cusp3历史姿态构造原地旋转/直线/旋转序列，位置误差<4e-12m；8m半径endpoint-carrot反馈最近.208819/.210013/.207305m，未进.2m终态。仅证明点载体存在允许动作和有限半径当前反馈失败，未运行真正NavFn/RPP完整流程，也不证明任意车辆机动不可达。
- 点载体可原地消除姿态偏差，真车须沿弧运动；已证实的完成区/下一短段接续、路径几何契约、候选失败归属和接续恢复预算缺口位于真实模式经过的分支。真实动态可加重偏差；此反例无延迟/碰撞/轮胎仍出现，不能全归因于物理负载，也不能推出视觉记忆原理失败。引擎空档/崩溃仍独立未定位。
- 新诊断4项通过，证据test_artifacts/offline_execution_audit_20261013/carrier_comparison.json（8输入SHA）及carrier_comparison_tests.log；未修改生产源码/未连接服务器/未新增现场成功，未commit。后续控制变量固定路径、姿态、轮廓、RPP/cusp规则先验有限半径零延迟执行，再加动态；Lua ackermann选项与Native健康/ACK/停稳接口不能直接假定兼容。

### 2026-10-13 继续离线检测：后台规划超时误失效当前执行动作

- 检测新增确定回调反例：ContinuousPath后台延长与当前FollowPath共用全局epoch；recover_timed_out_plan递增epoch，普通延长失败随后保留following/旧handle/队列，而on_follow_result按旧epoch忽略原动作成功与ABORTED。同回调输入实际pose在终点仍不推进；有新图路线时start_segment调用被following入口保护挡住，extending_path true/pending false。仅离线生产Python回调+ROS替身，未证明历史现场发生此序列。详见docs/OFFLINE_EXTENSION_OWNERSHIP_20261013.md。
- 正常对照：普通empty_path延长失败不增epoch，原动作成功能清队列并进入下一段；前台规划超时取消迟到规划接受保护仍有效。ideal_point注入相同条件也复现，属于公共Bridge动作归属问题，不能将所有停驶都归因真实物理，也不与历史ideal未观察到条件性故障冲突。
- 修复方向：独立规划请求/执行动作身份，后台规划超时只失效所属请求；保留目标切换、输入/执行器失效对整个事务的失效和旧消息保护，覆盖替换/取消/候选/恢复路径。本轮按离线检测要求未修生产代码。
- 旧紧容差复核：同cusp3路径343组允许范围起点扰动（位置0～.19m、角-.3～.3rad）中220接续接受且端点重放可达，123拒绝且不可达，0检查/重放分类不一致；是单路径模型采样不是现场343轮，不推广到全部长路径。新工具offline_extension_ownership_audit.py及7项诊断测试通过，结果extension_ownership.json/extension_ownership_tests_final.log保存；早期6项日志保留。15项冻结生产文件未变以extension_ownership_verification.json核对，无服务器连接、无新现场成功、未commit。

### 2026-10-13 整体离线检查与多组件目标链串联

- 用户要求继续检测整体能否离线跑通。当前Windows本机无rclpy/message_filters/ros2/Docker、WSL未安装，NumPy/PyYAML/PyTorch可导入；未安装下载/未连服务器，不能真实启动C++ GraphNav/Nav2/DDS/BeamNG全链路。docs/OFFLINE_SYSTEM_CHAIN_20261013.md、test_artifacts/offline_system_chain_20261013/为本轮入口。
- 全局offline_navigation_check首轮36PASS/3FAIL/6WARN，两组视觉记忆测试在沙箱Temp SQLite打开/清理权限失败；改项目内TMP/TEMP、实际SQLite写探针后重跑，38PASS/1FAIL/6WARN，通过项572测试执行含重叠，不算仿真次数。原失败attempt1保留，未改数据库生产实现；不能将初始Temp问题诊断成VisualMemory逻辑失效。
- 剩余FAIL为旧通用official_wildos_beamng_bundle.zip与当前源不一致：32已含成员不同，还缺当前Smac planner/connector、vehicle_control_lease.py/.lua、direct_vehicle_control.py；bundle_inventory.json保存完整清单。不是服务器冻结候选包或实际部署来源结论，未覆盖旧包/未重打包部署。
- 新offline_goal_chain_replay.py以真实记忆类、当前nav.py抽取方法、GoalStandoff/Bridge及动作回调串联。Ackermann配置供应直线路径/完美运动/健康反馈，完成station→green residential house→confirmed ViewpointMemory station（3动作）；ideal配置未给station查询先捕获合成场景特征，完成green house→passive_semantic_landmarks检索/选路/执行/新观测确认（5动作）。身份保持同进程，路径成功后未提前宣告object_found。特征/定位/图路线/规划/运动均供应，不是真实RADIO/GraphNav/Smac/RPP/速度环或车辆动力学验收。
- 同两条链在green house执行中注入后台延长规划超时，都复现epoch误失效旧FollowPath、旧成功被忽略、following true/队列遗留/无法推进；因此正常供应链可连通不等于整体可靠跑通。goal_chain.json保存正常/故障轨迹，4项新增诊断集成测试通过（要求故障存在，不是修复通过）；此前15生产文件冻结核对未变。未改生产代码/未commit，无新现场成功，20轮及完整记忆返站验收仍未完成。

### 2026-10-13 按顺序修复规划归属与冻结包（仅离线通过）

- 用户要求先处理后台扩展超时，再刷新旧通用包；无服务器连接。生产仅改graph_path_bridge.py与continuous_path.py。实际规划异步回调用独立单调serial并检查epoch/index/serial，超时只退役所属规划，不使合法FollowPath终态失效；重复接受/终态不重复消费，同index重试仍拒绝旧响应。导航切换、输入/执行器失效、实际动作失败仍保留整事务失效。
- 相关交接事件顺序已处理：动作先完成、扩展后timeout或普通拒绝，统一extension_failed通过原start_segment保护恢复；仍活动的动作保持归属。未改速度、碰撞/轮胎、8m半径、坐标/时间戳、新鲜度、视觉确认，不将其归因物理，也未顺带修cusp/terrain。
- 首批320项生产回调替身（含11项新增）及7项历史反例回归、4项目标链回归通过。正常与超时注入下，确认视点station→green house→station与未预置station查询的passive memory链均连通，仍供应特征/定位/图路线/直线规划/完美运动，非真实RADIO/Smac/RPP/BeamNG验收。历史要求bug存在的测试已更新要求恢复，原历史JSON/日志保留。证据test_artifacts/planner_ownership_repair_20261013/。
- 打包5项通过；资源缺失/越界/源变化拒绝替换旧包。首轮误预期plugins.xml，按真实CMake plugin.xml修正测试后通过，attempt1保留。全局首批40PASS/0FAIL/6WARN，588测试执行含重叠。项目内TMP/TEMP用于SQLite，未改数据库。新包独立解压320回调和4链通过，383成员逐字节一致，补齐此前32差异及当前Smac/lease缺失文件，不是服务器实际部署结论。
- 初始冻结deploy/20261013_planner_ownership_candidate/official_wildos_beamng_bundle.zip，SHA48b2f4c64d62f3c948d4d4a34254f6447085d4c7ffc23076d8903b72e4a8ba05；manifest/CANDIDATE同目录。旧SHA1e7af2ec4b63c8c135e62a32bde56dfcb7e2b58d9332a1c38bf0c449390cd1eb在before和主目录rollback保留。用户随后要求继续离线测试；若补测更新包身份，以后续记录为准。
- docs/PLANNER_OWNERSHIP_REPAIR_20261013.md为详细入口。现场仍待验收、未commit。下次现场先核实际来源及残留，明确真实SBR physical launch，保持Town与原物理参数，先测真实后台超时/交接，再恢复完整搜寻，不录像。20轮/80%及多目标未开始。短cusp、after_maneuver预算、候选实际姿态接入归属、曲率/terrain、引擎空档/崩溃及costmap退出仍未解决。

### 2026-10-13 继续离线：交接保护补测及最终包身份

- 按最新指令继续离线，不安装WSL、不连接服务器。新增4项回归验证统一extension_failed的实际下游：动作已完成时健康输入能发送新规划；过期位姿或执行器故障阻止下发；旧动作终态在替换ACK前到达时保留deferred结果，不争抢交接。这4项通过，无新增生产代码改动。
- 最终15项归属测试/完整324项回调替身通过；全局40PASS/0FAIL/6WARN、592测试执行含重叠。独立解包最终324项回调通过；此前首包4条合成目标链证据保留，两包间生产及链脚本字节一致，仅补回归测试。详见ownership_guard_tests.log、full_guard_report.json、callback_guard_report.json、unpacked_callback_guard_final.log、summary_guard_final.json、source_manifest_guard_final.json。
- 交付路径不变deploy/20261013_planner_ownership_candidate/official_wildos_beamng_bundle.zip，383成员，最终SHA94f6e301ec17d2b1d884830362c64e034d34bdba4303aac0858c47ca45c9bb42；此前初包48b2...和旧通用包1e7a...都保留rollback，旧记录是历史身份，不作最终包身份。manifest/CANDIDATE已更新。现场门槛未通过、未commit，未解决事项和20轮/完整搜寻要求保持。

### 2026-10-13 集中机制审计：先汇总故障，再按依赖修复

- 用户询问为何持续发现bug，接受集中冻结/审计/清单流程后要求继续。本轮无服务器/WSL/运行代码修改。冻结126项生产源码/配置/启动/构建资源，前后hash一致；上轮383成员候选包仍逐字节一致，SHA94f6...不变。docs/NAVIGATION_STATE_AUDIT_20261013.md为入口；test_artifacts/navigation_state_audit_20261013/audit.json、source_manifest.json、source_locations.json、summary.json、issues.json保存反例/身份/11组问题和修复依赖。
- 新确定离线公共Bridge缺口S01：初始FollowPath接受响应缺席（两种载体），替换ACK缺席且旧动作成功deferred，及已到端点/供应制动反馈但终态结果缺席，均无对应等待期限。通过生产输入回调持续供应新odom/双图/native/control状态，在一致时钟虚拟推进60s后仍following或replacement_pending；迟到响应后能正常处理。不是真实60s车辆实测，不证明历史所有停驶由此引起。
- 新确定S03：GraphNav清目标Trigger响应缺席后仅call_async1次、query_reset_pending/inflight保留，60虚拟秒内没有重试期限，新目标不能进入路线；迟到成功能恢复。必须保留generation/confirmation fence，不能伪造ACK。S02为重复终态回调注入反例：旧结果已消费后active_follow_serial未退休，下一动作尚待接受时重复旧回调清新队列/置following false；没有历史DDS确实重复的证据。
- 旧P01/P02再次复现：after_maneuver拒绝8次仍预算0；首条实际接入拒绝归follow_failed而不推进候选。新增更强对照：同一记录pose/空图独立供应6m直线替代，生产下发可接受，原流程不主动尝试。替代非Smac生成、非车辆完成，不扩大结论。
- P04风险：固定3m接续预测与RPP配置3～6m不一致，同45组8m弧线/位姿，Python预测器用3/6m有7组接受分类不同，非实际C++失败。P03旧55姿态返回路径仍excess_curvature，平滑前来源不明；T01图边可通行不保证周围弧线路域可通行，当前障碍/碰撞已开启；G01短cusp、E01引擎空档/崩溃、E02costmap退出继续保留。旋转地图origin对照100/0正确、正常动作释放交接正确，本轮不认定汽车坐标或VisualMemory原理失败。
- 10项新增诊断检查通过（要求故障可重现，不是修复通过）；324项旧回调结果按当前源码hash相同复用，不重复当新增。本轮工具首试函数名误写、次试重复装ROS替身被保护拒绝，改隔离子进程后通过，attempt1/2保留；初版直更receipt age结果保留，最终通过真实on_odom/on_map/on_local_map/on_native_status/on_control_status供应更新并复验。
- 后续顺序：统一FollowPath/GraphNav服务事务身份、期限与终态消费（S01/S02/S03）→统一恢复预算及候选实际接入（P01/P02）→几何/前视/goal checker契约（P03/P04/G01）→运动区域terrain约束（T01）；引擎/退出独立定责（E01/E02）。替换ACK超时不能盲目恢复旧动作，因为服务端可能已接受新动作，须核UUID/取消权限和实际停止。新增诊断文件未加入冻结包；现场及20轮门槛仍待验收、不commit。

### 2026-10-13 真车迁移目标与当前耦合边界

- 用户明确未来需要真车测试。本轮仅核对源码、分析迁移边界并记录要求，尚未实施S01/S02/S03事务修复、车辆接口解耦或真车部署，没有新增运行验证/commit。
- 视觉记忆检索/候选比较、目标身份与确认、GraphNav到Nav2的任务交接，以及动作请求身份/超时/迟到响应/终态消费应保持通用；BeamNG作为实验后端，不能将其暂停、瞬移、理想运动或引擎API当作导航核心的运行前提。
- 当前并非直接可用的真车栈：execution_feedback.py的ackermann执行受/wildos/native_control_status自定义JSON及硬编码新鲜度门约束；runtime_contract.py物理契约固定SBR轮廓、8m半径和BeamNG节点/话题；sensor_geometry.py默认SBR相机外参；cmd_vel_to_control.py依赖beamng_msgs，native_vehicle_control.py/direct_vehicle_control.py/vehicle_control_lease.lua依赖SDK/Lua、挡位和归一化控制。迁移须替换驱动与状态适配，统一车辆配置/外参/时钟契约，不能只改车辆名字或话题。
- 真车还需实际定位输入（不能沿用引擎真值）、相机/LiDAR标定和采集时间戳、执行器实测转向/制动/倒车能力及独立停机机制。运动学规划可复用框架，但当前8m、RPP前视/交接容差及动力学跟踪未经目标真车验证；不宣称BeamNG或离线通过等于真车可行。新后端必须提供真实执行/停止反馈，禁止伪造健康或ACK绕过保护。
- 后续S01/S02/S03优先按ROS动作/服务事务修复，停止、取消确认和执行健康通过明确后端契约接入；本条是迁移设计约束，尚非已完成解耦。核对证据为上述当前源文件及docs/NAVIGATION_STATE_AUDIT_20261013.md，历史现场/20轮验收状态不变。

### 2026-10-13 S01/S02/S03事务修复（仅离线）

- 用户要求先着眼当前bug，真车迁移只作为约束。本轮未连服务器。docs/NAVIGATION_TRANSACTION_REPAIR_20261013.md及test_artifacts/navigation_transaction_repair_20261013/为入口；before保存旧源码/94f6包/原审计和5规范测试修复前6处断言失败，旧证据未覆盖。
- 新NavigationTransactions区分FollowPath请求/活动/退役身份：默认接受/替换5s、当前活动反馈缺席10s、已进端点容差且停驶5s无结果超时，不按行驶总时长取消正常长路径。接受/终态只消费一次，旧serial不能清新队列，后台规划独立归属旧修复仍有效。监视/服务重试STEADY_TIME与monotonic，源戳/确认围栏不改。
- 不确定动作发布execution_hold（owner/revision/源龄），物理制动、理想载体零运动，拒绝新速度，释放清缓存、周期重发。全部不确定动作实际终态+新鲜停稳连续0.5s后经原保护恢复；取消ACK不等于停止、迟到成功不等于视觉到达。永久缺ACK/终态仍明确保持，不能安全自动恢复，不声称所有停驶已解决。
- Trigger默认5s、1s退避、最多3次，有独立serial/query/fence；耗尽明确graph_reset_failed而非无限inflight。服务端无请求身份，旧请求可能晚执行，故最新成功且排空全部旧回复才relay目的地；异常不当服务端结束证据。MemorySelection/TargetObservation清目标入口统一退休请求/预算，不改目标语义。
- 复查关闭FollowPath时停稳可能复用旧原生反馈，mode_stop_freshness_before.log确定失败；for_stop保留执行健康/源龄保护后最新358回调（含34新方法）通过。早期7处/2处cusp测试失败来自fixture跳过接受或同步接受后状态预期，按真实接受/结果流程更正、未放宽互斥，日志保留。
- 10审计回归、4合成目标链、全局检查和独立解包验证见报告；感知/规划/完美运动均供应，非RADIO/Smac/RPP/BeamNG验收。新工具配置/历史证据依赖补齐，冻结deploy/20261013_navigation_transactions_candidate/，精确身份见CANDIDATE.json。必要ROS/DDS/物理现场未验收、未commit；P01/P02/P03/P04/G01/T01/E01/E02未顺带修复，20轮80%及完整视觉记忆返站仍未通过。
- 最终全局40PASS/0FAIL/6WARN，626次测试执行含重叠；358回调通过。独立解包358回调/4链/10审计均通过，390成员逐字节匹配；最终包SHAf9561be85a67cc1b47044f090e8fbb80cda00e14fbb0b0bf52e7c49d116e526f，128生产来源冻结核对一致。summary.json/candidate_verification.json/full_report.json保存验证范围；中间包和旧94f6版本按身份保留rollback，最终不以中间hash部署。

### 2026-10-13 服务器文件清理复核

- 用户要求清理服务器无用文件。检查 `/home/ly/town_memory_validation_20261013` 后，仅删除本轮探针运行产生的29个 `__pycache__` 目录（合计2.3 MB）；删除后该测试目录内 `__pycache__` 数量为0。旧录像目录 `/home/ly/nizhen_viz_capture` 无文件。
- 保留当前BeamNG进程、Town场景、活动ROS/传感器栈、导航候选包build/install、失败与通过的诊断日志、清理清单；未触碰pip/NVIDIA缓存，因为它们可支持后续部署或仿真。服务器根盘清理前后均显示约280 GB可用（显示精度未体现2.3 MB变化）。证据：`test_artifacts/server_validation_20261013/server_cleanup_20261013_followup.json`。本次为文件清理记录，不代表规划或现场测试通过。

### 2026-10-13 服务器候选 FollowPath 实段与缓存清理补记

- 继续已授权的服务器验证。测试前先核查候选安装进程：12个进程使用 navigation_transactions_candidate/install，0个进程使用其 build。build 的真实路径核验为 /home/ly/town_memory_validation_20261013/navigation_transactions_candidate/build，包含2,296个文件、227,425,712字节；仅删除该构建中间目录。删除后 build 不存在，候选 install、候选 zip、测试报告保留，安装进程仍为12个。/home/ly 可用空间由299,631,452,160增至299,864,813,568字节。
- 实际CarLike/Ackermann（8m半径）短FollowPath段报告通过：起点至目标约14.033m，实际行驶13.550m，端点误差约0.489m，36姿态规划；动作状态到segment_done/route_done，车体制动停稳。运行时FollowPath与速度控制门已关闭。报告 physical_follow_trial_retry1.json，SHA256 D2253EADE151653D721184A3595A1AE3B7A9D45DA094C331668A544D75170D11；本机副本在 test_artifacts/server_validation_20261013/。
- 首次物理段报告 physical_follow_trial.json 未启动动作（armed=false、无action states、无运动样本），保持两控制门关闭。其错误记录称执行前健康守卫失败，但同一嵌入快照的telemetry_ok、actuator_ok、control_acknowledged为true，速度约0.000288m/s且制动1.0；这是测试预检结果与快照不一致的证据，不能归类为车辆运动失败。原报告SHA256 C4F8EF69E1C3922C2BC3B03E79368E50B5FFA7A6724CCA0FDF8323EC0B78F9BA，已从服务器取回保留。重试的监视器字段适配已通过实际短段；不将首轮失败擦除。
- 5秒运动前接口采样报告 physical_stack_pre_motion.json 显示state 41、odom 50、RGB 41、LiDAR 50、local map 50、terrain 10、global costmap 6、local costmap 5、native status 100条；采样期传感器/控制健康字段正常。SHA256 A0FCC8C16656EC77D1F36C50F755F750FA58591CF9AA12843B62DB95A3896F38。
- 此实段使用审阅waypoint，不是绿色房屋语义目标；visual_arrival_confirmed=false，不计入20轮目标搜寻的80%门槛，也不证明双目标记忆交接。此前候选部署现场已确认路线生成，但完整视觉搜寻和双目标流程仍待验收。未录视频，符合2026-10-10后测试留少量图像与接口/控制日志的规则。详细证据索引：test_artifacts/server_validation_20261013/server_validation_cleanup_and_followpath_20261013.json；较早29个pycache清理记录仍见server_cleanup_20261013_followup.json。
- 本次未修改生产代码；不创建Git commit。删除是针对服务器内可再生成且经进程核验未使用的构建目录。
### 2026-10-14 Town绿色房屋实测失败：段末方向保护与受限探索

- 继续授权SSH。上次准备证据已取回test_artifacts/green_house_search_20261013_trial1/：暂停复位后瞬时速度3400.997m/s，传感器以未停稳拒绝挂载、未发送目标/运动；不将读数解释为真实高速运动。后续SSH超时与引擎退出来源未知，当前连接时无旧引擎/栈/录制残留，旧日志只有SDK连接关闭。
- 新BeamNG1202233加载Town一次；出生落地瞬时0.883m/s使原初检失败，旧scene_preflight.json保留。独立普通全制动连续停稳后新SDK快照通过，scene_after_settle.json和挂载检查通过，不放宽传感器保护。冻结f9561be...候选生产未改，8个安装导航/控制/租约文件与包源一致，正式preflight实际参数/节点/外参/输入检查通过。真实723节点/固定0/车身467/四轮各32碰撞、8m半径、双图障碍、Ackermann RPP保持。
- 完整green residential house单目标运行420.0869秒超时；query 1和ACK通过、views=[]，mask/localization/confirmation/visible approach/imgnav_waypoint均0、memory_selection0。不把它误诊为query丢失，也不能据此排除VisualMemory所有评分作用或RADIO能力；无无记忆对照。
- 里程计累计176.0398m、Native车辆参考点累计179.7896m，最远离出生点32.0485m、峰速1.5943m/s。31次following、25个分段/机动成功终态，6次follow_failed（4方向保护/1碰撞弧检查/1横向偏离1.002m）、3次no_vehicle_path、68条excess_curvature日志；6个停驶≥5s区间最长21.35s，不等同受困。该轮仍集中机动、没有视觉到达，不计20轮成功，暂不开始20轮或多目标。
- 最早58.429s失败已按真实DDS夹具对齐：cusp_goal_checker要求0.2m，实际段末误差0.49714m/yaw0.12513rad，控制器spot1/nav_base_link终点(-0.26687,-0.42147)位于车后，因Ackermann lookahead would reverse a single-direction path中止；odom源龄0.1956s、转换路径距中止5.6ms。Bridge消费终态并开始新路线，本例不再遗留following，但缺响应/乱序现场事务验收仍未做。P03/P04/G01与P01/P02未修，不通过放宽到达或关闭方向保护掩盖。
- 末帧新鲜RGB显示近距离树林/草灌与路边，非目标到达证据；须继续复核T01运动路域terrain约束，当前不能把植物画面单独当作碰撞/被卡死或关闭碰撞的证明。该轮确有1次bounded command arc碰撞中止；旧summary的collision ahead=0只是旧文本未匹配，diagnostic_review.json更正，原报告保留。未录local_terrain/原始点云，travability和植物观测归因尚缺证据。
- odom源龄中位0.271/P95 0.423/max0.548s；RGB中位0.287/P95 0.440/max0.539s。首个失败无多秒旧odom，仍不能排除亚秒延迟对0.2m容差的影响。plan_status最大41.1s为repeat_status重复旧事件stamp，不能误作传感器停顿；planned_path接收源龄最大约3ms。
- 无MP4，少量图片/接口bag/日志保留。详细docs/SERVER_GREEN_HOUSE_SEARCH_20261014.md；本机test_artifacts/green_house_search_20261014_trial1/town_memory_validation_20261014_evidence.tar.gz，18,288,077字节、40原始成员128,126,387字节、SHA168fa598892c2c18e48f9d5a4e04b6a2d41d9a1d6ea5758b4fa8dda9934d4659，全部成员核对。首个中止first_abort_interface_fixture.json保存真实pose/世界路径/车体路径，可离线重放；不存在全程RGB，不能声称途中从未出现绿色房屋。附加绘图工具缺matplotlib不属于导航失败，去除后纯证据核验退出0，旧日志保留。
- 最终两门关闭、停稳并暂停；本轮stack1203711/vision1203713/triangulation1203712/sensor1203632全部有归属关停，Native速度0.0000650m/s、原物理/碰撞保持、传感器空、无本轮模型/导航/录制残留。保留引擎1202233和Town暂停。adapter context invalid异步发布警告在关停后发生，未作为搜寻失败原因。没有生产源码变更；现场失败、不commit。
### 2026-10-14 规划执行契约修复候选：软件验证通过，真实车辆待验收

- 继续授权服务器测试；预检仅BeamNG1202233保留，Town暂停，无旧导航/视觉/传感器/录像进程。docs/TRACKING_CONTRACT_REPAIR_20261014.md为入口，test_artifacts/tracking_contract_repair_20261014/保存修复前源码、真实失败夹具和所有对照日志。上一轮完整搜索失败保持，不计成功率。
- 确定Smac自由图起点也被离散化；只有精确格网状态才允许绕过现有连续接续，旋转/预算/恢复实际插件回归通过。原50姿态和实际下发位姿在零延迟受限自行车+真实RPP中仍失败（最近约.412m），不是仅网络或BeamNG动力学；缩短前视更差，Frenet尝试更差已撤销/保留失败证据，未上车。
- 候选保留实际8m指令半径/真实车身轮胎物理及碰撞，将名义规划半径设12m，固定3m前视与预检一致；运行契约、动态参数冻结和速度状态字段command_minimum_turning_radius同步。相同起点/原终点的8/12m与平滑开关对照，同半径输出一致：8m后续段仍失败，12m四段可捕获；未更改平滑默认或把该例外推为P03唯一原因。
- GraphPathBridge新增单方向整段几何预检，实际pose候选在定价/执行前拒绝，最终/替换/cusp再次核验；after_maneuver三次重试计预算，只有实际向路线端点进展>=1m才恢复。旧动作归属、显式/记忆目标和到达门保持；几何预检不证明动态/障碍/视觉成功。
- 服务器隔离真实Nav21.1.20 C++39测试零失败/错误/跳过；相同目标四段真实RPP/自由图/瞬时自行车在0/.1/.2/.4秒反馈滞后16段均到.2m且停稳，非BeamNG实车。363回调替身、22运行契约、6预检、全局40PASS/0FAIL/6WARN通过；组间重叠不相加。长两点控制器夹具补.25m采样，历史记忆夹具补记录航向，初次失败保留。TMP权限/构建merged布局/测试SCOPED_TRACE等工具错误不算机器人新故障。
- 冻结新396成员包SHA46ff51f2355a15fb4f92c79299b774d532dd72432380420e72dae7613978be8e，旧f956回退保留。新代码尚未真实车体现场验收、不commit。接下来核对安装/输入→真实几何执行→完整单目标；terrain/旧曲率/真实动力学与延迟/引擎故障仍待检查，20轮80%及多目标未开始。

### 2026-10-14 现场组件未开车及用户停止

- 46ff候选已在服务器tracking_contract_candidate独立构建，安装Python/租约/配置与源包一致，库SHA已记录installation_identity.json；本轮同地图准备复位1次、地图重载0次，原车身/轮胎碰撞保持。新源的真实行驶仍未验收。
- physical_cusp_component1生成约.5m换向段，诊断工具旧.8m最短段门拒绝，armed_at_ns为空。工具对齐既有connector .4m（两个.2m到达半径）并增加各段几何跟踪，9项回归通过；没有放宽生产到达门或重标首轮。component1_result.json与component1_failure_last_rgb.png已在test_artifacts/tracking_contract_repair_20261014/。
- physical_cusp_component2在/wildos/native_control_status发布者数量断言失败，armed_at_ns仍空，没有动作/真实运动验收；尚未收集发布者身份定责。不要先认定重复进程/网络/车辆动力学。服务器原结果tracking_contract_candidate/physical_cusp_component2/result.json保留。
- 用户明确杀掉相关进程、不再测试。按PID起始身份/组/安装路径核归属后先Pause ACK，再关闭13个导航栈、sensor1290435和BeamNG1202233，共15进程全部停止，errors=[]；ps仅用户系统服务和关闭检查SSH，25252无监听。服务器user_stop_20261014.json保存清单；目标暂停，没有继续跑测试或修复。
- 后续工具更新作为独立诊断来源上传，不在旧46ff冻结包内；check_continuous_start_execution.py依赖单独补给组件目录，恢复时需要统一冻结与打包回归。此前40PASS/0FAIL/6WARN是更新前包身份检查，不覆盖之后工具打包。停测证据归档脚本已准备但首次执行因SSH密钥文件路径拼写错误未运行；日志/源码仍在服务器，未完成本轮总归档取回。新修复仍待现场验收、不commit。

### 2026-10-14 用户恢复服务器测试：真实换向组件3失败

- 用户重新授权连接并继续测试。先核实无旧项目进程；新引擎1709748、新传感器1712701、导航根1712802，Town新会话加载1次、瞬移0次，真实车体/轮胎碰撞保持。先前停测的同进程地图/视觉记忆不延续。
- 诊断工具补发布者身份记录，最短段/整段预检沿用已验证门；打包补check_continuous_start_execution.py依赖及诊断测试。10项探针/5项打包测试、全局离线检查通过。新398成员包SHA25cd0289aa21c2f711e36beb63a6e3f46187ea9056ec08883457603b5d4f1af2，服务器source_identity.json逐员核对，仅5个tools文件新增/变化，生产仍为46ff候选及原安装库。不能把工具更新称作生产停车修复。
- physical_cusp_component3实际前两段maneuver_done，第三段约2.504m路径触发单方向lookahead反向保护；result.passed=false，两方向均有实际运动，最终SDK持续停稳并暂停，两执行门关闭。此轮发布者身份满足预期，不能据此追溯确定component2失败原因。
- 本机test_artifacts/tracking_contract_repair_20261014/physical_cusp_component3保存结果、接口时间戳、原生输入/遥测、事件及失败图片。diagnostic_review.json用已存在body_rotation/SBR后轴外参重建：第三段Native后轴最近误差约.00928m，但仍有速度；随后越过终点且反向保护中止。几何预检最近.19216m且reached。重建不是精确控制器odom/TF回放，尚不能确定终点制动判断/延迟/动态响应谁是主因；已安排只读DDS观测下一轮控制/实际pose，不关保护、不放宽到达门。
- 本轮无视觉模型/定位，不算绿色房屋成功或20轮成功数；未录视频，必要真实执行与完整搜索仍未验收，不commit。新增启动脚本首个工具导入路径缺失在引擎启动前失败，修正后启动；Native重建工具首个数据类型错误修正后运行，均不当生产故障。

### 2026-10-14 控制器内部TF旧输入复现、DDS配置对照和诊断预检修正

- 详情docs/CONTROLLER_TF_DELIVERY_AUDIT_20261014.md，本机test_artifacts/tracking_contract_repair_20261014保存component3..9及DDS观察、GDB线程快照。默认DDS下组件4即使执行通过，控制器位姿最大源龄2.12150s；Town复位组件5四个换向段完成、末段失败，最大2.43480s。外部odom/导航TF广播持续更新，但received_global_plan/前视点使用旧robot_pose戳，末段重复约1.8s相同.46568m/s指令，更新时目标已到车后。不是plan_status旧事件戳，不关闭反向保护，不把反馈源龄错记为车体卡死。
- 父GDB捕获一次滞后快照；各线程主要在DDS/executor等待和正常睡眠，未证明点云回调是根因。组件6报告通过但有调试中断，不能算正常验收。直接ptrace/sudo均被服务器权限限制，未改变权限。共享内存63GiB仅23MiB使用，不支持总容量耗尽。
- 源码核对实际rmw_fastrtps6.2.10：ROS_LOCALHOST_ONLY=1会自动追加回环UDP与SHM。XML额外UDP组件8通过且位姿最大.30155s/外部odom新鲜时过龄tick0，但仍有SHM句柄，不能声称彻底关闭。组件9用ROS_LOCALHOST_ONLY=0+UDP仅127.0.0.1白名单，controller fd无SHM通道，执行通过，当前运动记录最大.21607s/过龄tick0；观察者125s封闭结果尚须核实并重新取回，当前比较不是完整封闭归档。DDS传输配置影响已获证据，默认恢复/再次UDP/完整搜寻仍待做，未改正式运行入口。
- 组件7在立即查询native发布者为空时未开车失败，稍后独立观察者收到原生状态，证实此预检有异步发现竞态。工具改成最多5s等待、精确节点/命名空间/类型/数量连续稳定.5s；陌生/重复立即拒绝，永久缺席超时且控制门关闭。14相关单测本机/服务器通过，组件8/9实际预检通过；不能据此追溯确定component2当时原因。
- 新398成员诊断包SHA94e37140f6e06d2dc9ad01ea68c300cce4cc61d86fcc70dd1eebf10cddcec19c，旧25cd/46ff按身份保留；逐员核对只5个tools变化，安装生产继续46ff候选。全局离线检查通过；无MP4，真实视觉到达/20轮/记忆返站仍未验收、不commit。
- 新引擎1709748/Town加载1次，后续复位4次均在拥有者清理后，执行中0重载/0瞬移。当前session5 sensor1743484/stack1743566/controller1743587，组件9结束两门关闭、实际停稳、暂停。用户目标继续活动；下步先封闭证据并做默认/回环UDP复验，再统一环境与完整搜索。控制器最大源龄保护和其余历史未闭环问题未顺带解决。

- 封闭补记：9观察者125s结束/result.json13417消息，PID1744310已退出；重新取回完整记录后最大源龄仍.21606834s、过龄tick0。tf_delivery_evidence_20261014.zip本机取回10,359,177字节/145原始成员/SHA41a32bfae0c8c6859c6e7c2f60c2c4be0702dd7c726300617be132e78fec40c0，全部成员SHA核对及4证据回归通过；session5活动栈/传感器日志只称归档时快照。主运行component_env.sh仍强制ROS_LOCALHOST_ONLY=1，单拷XML不能复现9；默认恢复对照后应统一所有ROS拥有者入口、冻结部署并核验实际环境，再做完整搜寻。保持活动目标、不commit。

### 2026-10-14 默认恢复反例、统一DDS入口及完整绿色房屋失败

- 详见docs/FULL_DDS_SEARCH_20261014.md。恢复默认组件10再次出现最大2.152s旧位姿/40个外部odom新鲜时过龄tick；回环UDP11和冻结入口12分别最大.21017/.21554s、过龄tick0，真实换向通过。新DDS模块/安装XML/ROS初始化前统一入口和逐ROS拥有者/传感器预检，8环境与6初版策略测试、本机全局/服务器回归通过。06bb47afd4d32df40afd85fef7aa0f11c97e89efff0c9dcf1e120916da4b64d0为实际部署402成员候选；只安装Python覆盖层，C++库与算法源保持46ff身份并逐员核验。
- 首个视觉进程因包内同名配置缺失退出，未发目标/未运动；改用已冻结适配包配置绝对路径，保留失败vision.log并只重启已退出模型1771495。不要误诊成RADIO识别失败。随后完整preflight/唯一green query/ACK通过。
- 完整420.0846s超时，里程计222.5813m、Native225.1266m、最大离站50.9382m；46following/36maneuver_done/4segment_done，4follow_failed（包含曲率准入/2个明确碰撞弧中止）、4no_vehicle_path、4after_maneuver重规划。旧单方向反向保护本轮0；静止最长21.337s，不直接当物理受困。保持真实723/固定0/467车身/四轮32碰撞、无速度调参、执行中0重载/0瞬移，不计成功。
- 后半程28掩码/18bearing approach/一个实际观测步，3个localization_valid=true但object_confirmation0，严格到达0。第一个multiview位置与后两个LiDAR位置约18m差、两个LiDAR互相一致但数量不足以稳定确认；不能说没识别/没定位，也不能以此宣布找到。mask dimensions[1,1,640,960]/CameraInfo戳差0；暂不归因尺寸/钟域接口错误或放宽确认。
- 模型负载下3775控制器tick，源龄median.24827/P95 .49938/max.62627，外部odom<.3而控制器>.5的tick0；旧控制器独立TF积压未复现，原始odom尾部max.84197仍待分析。额外RGB诊断在19:38上传，试验19:36已结束暂停，拒绝旧帧是正常保护；不得当QoS/相机失灵证据。
- 封闭真实geometry在自由图重放4案例：整条准入None，motion_sections切片后部分excess_curvature。首个接续拒绝55.068s；173.512s首个follow_failed更晚。来源是分段约定不一致已复现，但Smac/平滑不可执行与验证器量化/窗口错误尚须分辨，不关闭保护、不凭此责怪VisualMemory。
- 退出旧完整工具只取一次SDK速度<.05；末.042975，暂停后独立.051303、位置差约.0007m，清理拒绝。保持两门关闭/原位置/挡位/物理，以普通制动连续.5s<.05，末.010074再暂停；第二次关闭stack1768659/vision1771495/tri1768660/sensor1768568均通过、传感器空，只有BeamNG1709748/Town暂停。两次清理记录和制动恢复都保留。
- 完整退出改用现有settle持续停稳并记录样本，9策略/全局离线通过；本机最新包40796ca9b1f0be850de25ed40d8beca28d4c33d2d9bcf08e8874a13885d95124尚未部署；真实独立同函数制动通过不等于新完整退出路径验收。full_dds_trial1_evidence.zip本机39378675字节/62成员/SHA bfc0d4f14e40a94cb3ddba3932837e7def142c5b3eaf1dcda96bc8e862a70e07，ZIP及全成员SHA核对；含230MiB SQLite/755地形消息/新鲜失败PNG/原日志/部署身份。20轮和多目标未开始、不commit，活动目标继续；下步修复分段曲率契约，再接近/确认及完整复测。

### 2026-10-14 换向曲率定位、同次raw回退与组件13（补记）

- docs/CURVATURE_CONTRACT_REPAIR_20261014.md为入口。3条不同记录输出/4事件反例的拒绝均落在方向段末端heading窗口；旧整路窗口跨cusp后加入下段弧长并抵消转角，造成整路准入、分段拒绝。保留before/graph_route_dds06bb.py与curvature_window_trace.json，不能再把这批反例仅归因碰撞/真实姿态。
- 实际Smac1.1.20、域89隔离重构3组端点/最近发布地图，raw各段8/12m均允许，smooth均有段超曲率；自由图对照未暴露相同8m拒绝。请求action未记录，OccupancyGrid恢复cost有量化，不称逐字节复现原搜索。原插件忽略smoother bool返回且失败仍可能改航向；自定义插件此前只有碰撞/预算/起点检查。
- graph_route按实际motion_sections独立查曲率；新增C++同约定检查；planner在一次native搜索保留raw，仅采用成功且几何/当前地图/预算通过的smooth，否则检查同次raw再回退，合并connector后复验。保持8m实际/12m名义半径、3m前视、物理/碰撞/速度/确认/动作归属保护。新候选413成员SHA71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660，服务器curvature_contract_candidate1独立安装。
- 服务器Smac20测试、Python27几何/8DDS/9停稳策略通过；相同12组插件重放修复后整路/各段均允许。本机363回调及当时全局41PASS/0FAIL/6WARN通过，尚未涵盖后述两条合成链夹具缺陷，不扩大历史验证范围。
- 真实组件13passed=true，4段前/倒向执行完成、终段误差.422883m（普通终段.6m），无交接重试；SDK连续停稳样本末.005695m/s，原真实节点/碰撞保持。只读观察者125s终态13278消息；本机curvature_candidate1/physical_cusp_component13与observer已取回。无视觉到达/无新完整目标成功，不计20轮。
- 之后上传/执行组件清理脚本被自动审批额度错误拦截，工具明确未执行；不能声称栈已清理。最后已知sensor1840081/stack1840196、引擎1709748，组件两门关闭/停稳/暂停；之后服务器当前状态未经重新核验。工具merged-layout/换行/PATH/首次导入失败均保留，不当作新增车辆故障。未commit。

### 2026-10-15 用户三项离线工作：测试、更名与RViz双栏

- 用户要求先完成离线测试、按职责重命名原四个memory、准备左2D图/右实时camera的未来RViz布局。本轮未连服务器/未开始新现场测试。详细docs/OFFLINE_RECALL_RVIZ_20261015.md；当前术语入口docs/VISUAL_RECALL_COMPONENTS.md。
- 整体称Visual Experience and Recall；VisualMemory→ExecutionExperienceStore，VisualMemoryMap→GraphObservationStore，ViewpointMemory→ConfirmedViewpointCache，SemanticLandmarkMemory→PassiveLandmarkIndex，MemorySelection→RecallCandidatePolicy。索引确实保存历史特征/位置；semantic门槛、关系与候选比较属policy/rules，不再统称四种记忆。生产imports更新，旧模块/类为同对象兼容alias；SQLite结构、配置键、ROS/evidence协议、内部状态与同进程生命周期未改。
- 更名7生产模块经名称/import规范化后AST与71f9一致，15个C++源码/头文件/CMake字节一致。当前新Python命名尚未现场部署；不能混称服务器已运行本轮423成员包。
- VisualizationBridge从/spot1/nav_graph绘制真实节点/边/frontier到/wildos/viz/topology；显示定位/确认物体名称到object_labels，区分localized和confirmed，拒绝无位置、错frame/query、围栏前/过期/未来、非有限数据，独立实例不混，临时标签5s/缓存128上限，切query保留确认名、切frame清空。只有掩码不能编造地图位置。名称来自有定位的目标查询，不是新增全场景通用识别器；显示进程须测试前启动，尚无任意晚启动历史名称快照。
- 相机逐帧独立刷新，保留采集戳/新鲜掩码约束；图更新2Hz与相机解耦，显示节点不发导航/控制/确认。旧局部执行路径用最新车姿变换存在显示错位，改按源戳匹配历史odom（<=150ms），无匹配跳过显示副本。导航原始消息/控制不改。
- 默认beamng_navigation.rviz改TopDownOrtho、左右50%、1920×1080；启用拓扑/frontier/名称与路线，右原生RGB + Target。Qt离线restore成功（各957px），预览为合成图+旧失败RGB并标fixture，不是实时RViz/到达证据；实际Qt5/RViz/Ogre/流畅度需上线验收。不自动开录像，未来用户要求录像时使用新布局。
- 严格DDS路径测试在沙箱4处WinError5；相同生产源码在正常本机权限8项通过，未修改路径保护。更名前71f9包合成链同样2失败：测试供应20m两端点倒车路径、8m局部窗口仅起点；仅测试供应规划器按既有.25m采样、补间距/长倒车检查，保留生产准入后4合成链通过。首轮/基线失败和诊断保留，不误诊更名/模型/物理。
- 最终本机全局47PASS/0FAIL/6WARN（缺ROS同步与历史入口/验收边界），含371回调替身、4合成链；测试范围有重叠，不当仿真次数。独立解包371回调/4链/Qt布局均通过、423成员逐字节核对。证据test_artifacts/offline_recall_viz_20261015/summary.json及对应日志；冻结deploy/20261015_offline_recall_viz_candidate/，SHA dfdb40d7493274da9b0397c8a8dc3e3473368ff2cbdff3a12ef11c3ccd65eb53。
- 必要实时RViz/完整真实车体视觉到达仍待验收；20轮80%与多目标现场未开始。现有工作区含历史未验收修复，未commit。下次上线先核残留/版本，不能在旧组件栈上叠加部署。

### 2026-10-15 自助完整实验交互脚本（离线准备）

- 用户要求自行启动完整循环、输入目标、结束进程。新增tools/wildos_console.py（持久交互入口）、wildos_console_scene.py（一次新Town场景）、wildos-console.ps1（本机SSH入口）、setup_wildos_console.sh（一次独立构建），配置config/wildos_console.example.json，使用说明docs/SELF_SERVICE_CONSOLE.md。没有连接服务器/执行构建/实际启动车辆；不要声称用户当前服务器已经可一键使用。
- setup继承明确基础setup并独立重建adapter/visual_navigation/Smac/RPP五包，记录新库SHA。运行在ROS上下文前统一setup/DDS；检查Python安装与源、launch/config及库身份，跨域检查旧车体/导航/视觉拥有者与单入口锁。发现残留只列清单并拒绝叠加，不宽泛pkill。全新入口拥有引擎与单次Town加载，保留真实SBR/轮胎物理；没有杀掉GraphNav planner，没有调速度/关闭保护。
- 每目标独立日志/接口bag（不录MP4）、Query规范化→记录器发现→一次发送→视觉ACK围栏→新鲜Bridge profile→开启两门。一个模型/地图/图贯穿不同目标；不按目标复位/重载，不强制原路返回。/stop关门、SDK持续停稳0.5s<0.05m/s并暂停但保留栈；/quit、Ctrl+C、EOF、SIGHUP、SIGTERM按停车、暂停、释放SDK、清理所拥有进程组顺序退出。身份由boot ID/PID起始时间/已知存活组员核对，处理launch退出而子进程遗留；无法证明身份不发信号并记错。实际服务器退出/SSH中断仍待验收，SIGKILL/断电不能被脚本捕获。
- 已发现接口边界：模型search_query_callback与Bridge on_query对相同query不创建新generation，重发还会导致旧ACK不满足新围栏。入口拒绝连续相同目标并解释；独立同目标复测需要退出重开。模型原配置初始query为door，首个目标不能与其相同。没有发送虚假中间目标/改动生产目标协议来掩盖问题；输入按casefold/空格规范化与两端一致。
- 通用输入暴露Harness.save_target_images直接用query作为文件名存在路径逃逸/非法路径风险；改为安全字母数字下划线短名，否则SHA短名，保持正常旧名称。配套测试覆盖路径文本、长目标、准确源帧；图片保存失败不再让新入口跳过停车。目标目录只用序号，不拼目标文字。
- tools/test_wildos_console.py最终26项纯/SDK替身测试通过：两目标复用、超时、ACK缺席/中断、重复/初始query拒绝、旧状态不能开门、停车故障锁定、失败图片错误仍停车、PID复用/无身份拒绝、已有子进程清理、场景加载/失败暂停、配置shell参数按数据传递等。Harness15项通过。第一轮17项中1项因Windows测试环境没有SIGKILL常量失败，补Linux常量替身后通过，生产Linux退出代码未为此放宽；first_console_tests.log保留。Bash语法在沙箱MSYS命名对象权限拒绝，正常本机权限-n通过；不是服务器构建或启动测试。
- 最终全局离线48PASS/0FAIL/6WARN、371回调替身/4合成链（范围重叠，不相加成试验数）；PowerShell解析/Bash-n通过。430成员逐字节核验并独立解包26交互测试通过。冻结deploy/20261015_self_service_console_candidate/，SHA5c6e501c457ab59b70637e1e15c499b01f4af5825ba32261168b6edf6f2374ae；旧dfdb更名/RViz候选保留。完整证据test_artifacts/self_service_console_20261015/summary.json及日志。
- 真实安装、模型/传感器发现、目标ACK与切换、实际停车/退出组清理、实时RViz和完整视觉到达仍待现场验收。新入口到达一律automatic_arrival_pending_visual_review，passed不会自动置true；不计20轮成功，不进入正式多目标验收。必要现场验证未执行，按用户规则未commit；当前服务器残留状态仍须先核实。

### 2026-10-15 自助入口服务器验证：启停通过、绿色房屋搜索失败

- 用户授权连接；先核实旧stack1840196/sensor1840081/engine1709748身份，关门、SDK停稳/暂停后清理；旧cloud_filter约3950%CPU随栈退出，未证明历史规划根因。当前本轮测试结束且独立ps无相关残留。
- 新/home/ly/wildos-console独立五包安装，修复构建PATH无colcon（显式SDK venv bin，Linux替身安装回归通过）、新场景制动前未设置realistic_automatic（原流程与单变量对照，未放宽.05m/s连续.5s门）、静默事件dict重复query参数（真实Harness回调回归）。27console/16Harness及全局48PASS/0FAIL/6WARN通过，原生Smac/RPP测试通过；范围重叠，不相加成实验数。
- 当前431成员源SHAff0fa6e512f21a67e1605ea960df15148edac385db6b2a55c84cb1f5f8785a43；安装生产沿17adb4ae，之后仅tools/测试变化且核对一致。真实723节点/0固定/467车身碰撞/四轮各32/1449.72kg、8m实际半径和碰撞保护保持，没有速度调参。lifecycle144900实际启动/预检、/stop、/quit通过，非搜索成功；外部停止/断连路径未现场验收。
- session_20261015_145008绿色房屋420.8641秒超时，Native169.7334m，35following/26maneuver_done/5segment_done/8follow_failed，无严格到达。末四次motion_section_invalid/local_footprint_collision拒绝同终点(684.5423,-17.3379)，2/4/8/8秒退避，末次后约74秒无新执行状态。源码follow_failed没有为此类重复几何拒绝调用reject_exploration_route，而恢复最多3次，是待修复交接缺口；尚未生产修改/反例回放。更早确有一次frontier拒绝换目标，不能说整轮反馈均失效。进入绿化的terrain/costmap来源未闭环，不归咎视觉记忆/物理。
- bag缺rejected_route_endpoint/traversal_feedback，不能把未记录当未发布；Harness事件未含确认不证明全程没有mask。失败新鲜RGB源龄.44865s显示树木/绿化，非到达证据，无MP4。末次SDK持续停稳并完整清理，errors=[]。20轮成功计数未增加、不做正式多目标、不commit。
- 详docs/SELF_SERVICE_FIELD_20261015.md；本机test_artifacts/self_service_server_20261015/closed_evidence_full.zip SHA5bbbf0314b33393da619c2e6fcf2e5480548fccdf3c3bee796bf790fd32b06ca，128原始文件/ZIP/逐员SHA通过，解包同名目录；final_search_review.json含封闭统计。保留旧入口失败/单变量对照与日志；当前ff0fa6尚未独立冻结deploy目录，不混称原5c6候选已更新。

### 2026-10-15 用户要求继续：局部碰撞恢复交接候选

- follow_failed对重复motion_section_invalid/local_footprint_collision在recovery_attempts>=2时复用探索拒绝接口，原早期重试、3次预算、碰撞/真实物理、显式/记忆目标/查询/新鲜度保护保持。原ff0fa6方法反例复现缺拒绝，当前374回调通过；真实ROS routing118/console27通过，全局48PASS/0FAIL/6WARN。fake消息对象比较、旧Time夹具类型两次测试问题均修正，仅夹具，不放宽生产保护。
- 431成员SHA751218fde4ef9575a606b53e440719c6b737bb2840d54fa2ebaebe0738527f8c，deploy/20261015_local_collision_recovery_candidate冻结；服务器仅重建wildos_nav2_adapter、C++库身份保持。补录拒绝/遍历反馈/route_status。
- 第二轮session_20261015_151045绿色房屋420秒搜索进行中；初期正常离站约31m/4机动1段/0FollowPath失败。必要完整现场验收未完，不称修复已验收，不commit。详docs/SELF_SERVICE_FIELD_20261015.md；新轮结论须后续封闭补记。

### 2026-10-15 续测封闭与用户要求视频

- 第二轮151045在174.257s中断：Native79.0526m/24following/20机动3段/0follow_failed，rosbag SQLite database is locked。与外部诊断全表只读统计同时，诊断读取可能阻塞写入；不能归为规划失败。工具改成goal与session均关闭才接触SQLite，活动试验拒绝保护已验证。该轮晚启动无帧，无可用录像。
- 用户中途明确要求视频，本轮覆盖不录像约定。新151859先收到camera写started再发query，录实际前摄像头；另45秒墙钟预览已给用户，431帧/43.1秒，有源戳与新鲜度验证。不是RViz屏幕，未来RViz双栏约定保持。
- 第三轮151859完整420.8003s超时、199.2655m，34following/27机动3段、4执行失败（动作中止2/偏离1/无进展1）、2恢复；末段10no_vehicle_path/10拒绝消息/GraphNav10次接受，仍无法完成探索。新重复local_footprint_collision分支未触发，不能称该分支现场通过。bag query1、mask/localization/confirmation/visible approach0；是否错过可见目标需人工复核全程RGB，不擅判模型唯一根因。
- 持续停稳、session errors=[]、独立ps无本轮残留。SSH曾reset但driver/console/录制独立继续，最终正常结束。完整H264视频captures/20261015_green_house_recovery_trial3/front_camera_full.mp4，112806673字节/4231帧/423.1s、全部解码、新鲜比例99.905%、SHA67cd22e3d3cba279dcbe3beadd331620caf9914a8358a2bd82f11ea09eac2f07，前摄像头真实画面，无RViz二维面板；逐帧JSON/validation齐全，失败验收不称成功。
- test_artifacts/self_service_server_20261015/closed_continuation.zip，91原始文件，SHA4abdb622e62b0d243b21e44325acb08ac98720ca674aafdb3214c387a6c605a4，ZIP/逐员SHA核对，视频单独存。详docs/SELF_SERVICE_FIELD_20261015.md。20轮成功未增加/多目标未开始，必要现场未过，不commit。

### 2026-10-15 用户优先要求核实评分与记忆参与：发现输入范围与采样耦合断点

- 详docs/SCORE_MEMORY_PARTICIPATION_AUDIT_20261015.md。用户5:00远距房屋线索后接受可能过远/短时可见，不据此降低目标阈值；先核评分/经验，不叠加停车修复。本轮生产算法/参数/包751218保持，GraphNav来源9个CPP/HPP与bundle一致。未commit。
- 旧151859全程17执行反馈：16无近期样本/1节点时间不匹配/0存储。新155034诊断：启动期+120s目标期349次处理与检索，真实score_geofrontiers调用0、命中0、3反馈0存储0pending。110份1Hz记录最近frontier最小10.871489m>配置9m，within_config_range均0。数据流候选匹配255/255 route_status/7目标，但实际使用回退分数，不能称WildOS视觉评分在正常生效。
- 明确耦合：nav.py994仅raw_score_rows非空才采样，几何frontier在9m外→视觉评分不调用→有图像特征仍无执行经验样本→反馈无法学习。缓存默认分数下一帧在1194/1205被标为is_default_scored=0，非默认标记会误导诊断，不能用它单独判模型有效。
- GraphObservationStore确有8观测/3遍历/5覆盖；本次候选效用中的记忆项差0。PassiveLandmarkIndex520条持续采集；确认缓存/记忆候选0，单目标短测不验证后来新query召回，不能一概说四模块均失效。以上是当前集成/采样契约问题，不证明论文或记忆原理失败。
- 显式诊断包装记录原方法返回、私有DB行数，不改输入/结果；3包装回归通过。首诊154833预检因包装进程身份未知拒绝，未开车；独立诊断preflight登记准确包装路径并保持原1拥有者/DDS/输入保护，增加RViz流检查。第二诊155034实际通过、120s正常超时、SDK持续停稳、errors=[]，独立ps无仿真/RViz/录制残留。
- 用户要求后续录真实RViz：本次已实际录制captures/20261015_score_memory_audit/rviz_live.mp4，1920×1080/992帧/124s/全解码，左2D节点frontier路线/右实时camera，SHA1ec7a6a85986f47822c2e3982c4396f7caa64a8e1fff470516eb0d9dbbd64043。非到达成功。80成员封闭test_artifacts/search_mechanism_audit_20261015/score_memory_audit.zip SHA75e8088db178a67b94f7470f8b63aaafe6b9a8eb75710865a9271a4a000c60da，ZIP/成员哈希核验，原数据解包evidence。
- 另停车图连接器重放6000和100000预算均0候选，自由图可行；不是私有地图精确快照，不能认定只是搜索预算问题。连接器/停车恢复未改。后续先对齐frontier观测范围、解耦监督样本采集/保留未知方向、修正评分来源，再受控回放/现场验收；不将调常量或编译当完成。
- 用户询问沿途自动命名：当前PassiveLandmarkIndex不存标签，只存空间支持特征；RViz名称来自已定位/确认目标，未实现任意沿途物体自动命名。无名特征锚点不能冒充独立物体；自动物体发现/语义命名/图关联尚未实现，本次未新增此功能。

### 2026-10-15 先设计再实施评分/经验修复候选

- 用户授权先思考再修改；先收敛观测范围/执行采样/缓存来源三契约，不混入连接器或停车恢复。详docs/SCORE_MEMORY_CONTRACT_REPAIR_20261015.md。当前434成员042ee7af47bd9e24d57cf9cee38081de6fe3122ebe9bb0effdbfb2052d876a94，deploy/20261015_score_memory_contract_candidate冻结；相对751218只14文件，C++/速度/物理不改。
- YAML单源40m（已有点云上限）同时驱动frontier投影与cloudfilter，保留FOV/正深度等；模型只读range参数+现场双端契约防再错配。不是将40m内区域宣告可通行。
- 描述子采样不依赖frontier，按执行端最近有效节点/6m/同frame/源时间匹配；.25s节流+512条/60s TTL，1152维满描述子约2.25MiB，目标无关、不存图片。经验新增监督mask，仅实际执行bin学习，其余乘法中性；旧库NULL兼容。生产不把目标偏置frontier分数当监督；正反馈写入命中可能仍中性，不伪造排序变化。
- 默认/视觉缓存来源、raw/统计/观测时间区分，默认计数0、视觉首次重置默认统计，5s处理后缓存过期回退，派生properties替换防旧query重复。源戳年龄另行现场核验，不把cache age当传感器新鲜度。
- 新6机制回归含真实反馈回调、投影反例、严格匹配/延迟/重复、缓存恢复等；经验5项/runtime23项服务器通过；全局49PASS/0FAIL/6WARN。初版bf3ee中间包保留、随后目标中性修订3文件再通过。当前正在120s现场组件复测并录真实RViz，未称完整修复/到达成功，不commit；后续补封闭结果。

- 评分候选042ee现场161453暴露本次引入的缓存时序缺陷：写observed_at晚于该帧now，fresh_visual负年龄立即覆盖默认。保留递增1ms时钟反例（旧常量夹具漏检），统一单次评分事务now修正，未来保护未放宽。新434成员ef3fc7f814683f640e2d4a93dfc5ef601f76f7f1a6e85e56187631f8fec5a54b，仅改Nav与对应测试2文件；全局49/服务器6+5+23通过，当前161910组件复测进行中，已见90评分/7真实视觉缓存，待封闭。不能将042ee未采用真实分数称修复成功；不commit。

### 2026-10-15 评分/经验修复最终组件封闭（仍未完整验收）

- 最终ef3fc7f814683f640e2d4a93dfc5ef601f76f7f1a6e85e56187631f8fec5a54b，实际40m双端范围/源/配置预检通过，原C++和车辆物理保留。全局49PASS/0FAIL/6WARN；服务器6机制/5存储/23runtime通过。真实组件161910启动+120秒243评分调用，最多10真实评分节点；202路线状态目标中160匹配真实视觉/42回退，不能等同受控消融或任务成功。
- 6反馈6写入/6经验/146命中；本轮正面标签的先验仍中性，memory_modified_scores0属如实行为，不能制造负面标签强行改变排序。GraphObservationStore记忆效用项记录最大差.1893，说明参与候选数值，不证明最终路线改善。
- Native37.3512m/18following/14机动，8.349/12.735/18.839秒3次FollowPath单方向lookahead反向中止；评分修复未解决实车执行。120秒目标超时无确认，成功计数未增加、不做多目标、不commit。后续要对齐这些实际路径端点/控制器内部TF/源龄与刹停交接，反向保护保持。
- 外部odom源龄median.14275/P95.33350/max.79914s，RGB median.15454/P95.33969/max.83609s；模型图快照年龄median1.28459/P95 1.81605/max2.15204s，处理延迟仍需查，未独立观察本轮控制器内部TF，不扩大时间戳验收。cache年龄是处理后年龄，不能混成采集源龄。
- 两轮SDK持续停稳、session errors=[]、独立ps无本轮残留。104文件test_artifacts/score_memory_repair_20261015/field_evidence.zip SHAc78c7f62b9a1aa1e6d47c5a3c17bde57575b545f74a9935c6c05a27266464fd1，ZIP/逐员SHA通过。真实RViz视频captures/20261015_score_memory_repair/rviz_component_retest.mp4 123.5s/988帧/SHAf4c4db4ac808d6c1aed97aed75d5dc07cb6e0cb87363d22ee14cc899fe4c174b；本次引入时序错误的反例rviz_clock_failure.mp4 124.5s/996帧/SHAcbcba42d8f20879d10b50355a08b54e136426cc9349a80a5995ccb2d570fbb50，两者全解码，非成功验收。详docs/SCORE_MEMORY_CONTRACT_REPAIR_20261015.md。


### 2026-10-15 评分修复后420秒完整搜索：连接器阻塞

- 详docs/FULL_SCORE_SEARCH_20261015.md。生产ef3fc7不变，session163334单次Town/真实车体/green residential house，420.932秒超时，Native26.0263米、6机动完成、31no_vehicle_path/11等costmap，没有mask/定位/确认。本轮FollowPath反向中止0，不当作旧问题解决。退出SDK持续停稳、errors=[]、独立ps无本轮残留。
- 1041真实评分、最多14非默认节点；2反馈2写入/899检索命中，正标签先验仍中性，memory_modified_scores0。GraphObservation效用差最大.18999是数值参与，不是任务或消融成功。控制器535tick源龄median.1602/P95.2497/max.4613秒，外部新鲜内部>.5计数0；模型图快照median1.8239/P952.4272/max2.8239秒，处理延迟未解决。
- 最早59.825秒车辆路径失败；531连接器搜索529无候选/2超时。封闭首/中/末3样本，最近先前发布地图/odom重放6000与100000均0，自由图均6。起点safe=1但前后直行.2米拒绝，自由图候选在原图移动约.13-.18米即拒绝；附近lethal/inscribed单元来源未定，不擅定自车点/地面/真实障碍，不关闭碰撞。重放不是私有地图逐字节快照。
- 本机72成员field_evidence.zip SHA2ce1b9d4bfbba2ea74d0a93676961c7c5e0c07d6dc882282bdecbdbecf503c93逐员核验；captures/20261015_full_score_search/rviz_live.mp4真实双栏423.5秒3388帧全解码，SHA8e4f8e84dce1cd0f80c120b6dcc6572eee21a8225e8210ec28b13f15e10d5a1b。失败录像，不计成功，不commit。
- 用户继续后准备180秒点云来源组件诊断，额外只读bag采集marking/clearing/适配LiDAR/TF/odom；新轮尚待封闭，不能把增加诊断负载与短时限称原完整试验同条件验收。


- 续测封闭补记：新增sensor_bag若在预检前启动，session200325被原“旧记录器”检查拒绝，query0/控制门关闭/无搜索MP4；诊断入口错时序，保留sensor_startup_rejected_preflight.json。独立包装改为预检通过后、begin前启动owned记录器并等待订阅，原预检/清理不变；4新增生命周期回归通过并留日志，不改生产源码。
- 重试session200533为180秒点云组件诊断，182.564秒超时、Native111.7438米、21following/17机动3段、no_vehicle_path0/follow_failed0，7连接搜索都有候选（5触及时限仍保留候选），15反馈15存储/537真实评分/409命中；无mask/定位/确认，不算绿色房屋成功。1775控制tick源龄max.4584s、外部新鲜内部>.5计数0；模型快照median1.5131/P952.1050/max2.6479s。停止剩余执行、SDK持续停稳、errors=[]，独立ps无残留。
- 新轮未复现首轮阻塞，cloud_origin_review.failure_events=0，不能用另一轨迹的点云断言首轮障碍来源；短时限/额外日志负载也不当受控消融。首轮障碍单元究竟真实/自车/地面/残留仍未定位。连接器重放3日志已取回，生产/碰撞/车速保持，无commit。
- 补录原始102成员sensor_evidence.zip SHA3b22109e38ae35af92b806c2abf5823f4b393392fa6bb1dd525017ec69782b1f逐员核验。captures/20261015_sensor_origin_audit/rviz_live.mp4真实双栏185.375秒1483帧全解码，SHA3991162421f34840896ff8f1abe4982ee2aec4efefbdd420e738636d6fd289d5；是超时失败录像。完整20轮/多目标未通过，详docs/FULL_SCORE_SEARCH_20261015.md。

- 人工抽帧补记：补录轮frame_1200约150秒相机前方接近树丛。无FollowPath报错不等于路线/避障正确；也不单凭此图断言车体已穿入。该风险仍未解决，见完整报告。


### 2026-10-15 用户继续修改：LiDAR射线原点接口

- 详docs/LIDAR_RAY_ORIGIN_REPAIR_20261015.md。确认点坐标frame与传感器光心混用：Nav2输入已转base_link但未设sensor_frame，默认后轴发射清障；虚拟os_lidar也只是StateSensor原点。SDK只读低位移样本(<.1mm)测实际SBR光心相对后轴约[1.252,0,1.560]m；原occupancy/terrain的StateSensor射线XY同样不符。不是以此断定所有停驶根因。
- 增独立lidar_ray_origin静态TF/可配置lidar_offset，原虚拟点坐标/采集戳保持；实车两个costmap的marking/clearing指定真实frame；occupancy与terrain对齐同源光心，terrain保留采集时roll/pitch转换。ideal_point既有原点约定不变，速度/半径/车体/碰撞/目标阈值不改。
- 实际Nav2 ObservationBuffer受控反例证实端点不变但默认后轴和光心射线经过不同单元，不能当首轮私有地图精确重放。新增3原点回归先失败后通过；旧ROS几何测试2/4失败是夹具没flush已有消息队列，补flush仅测试；新增占据图mock浮点失败改int8，仅夹具。原失败日志保留。
- 修复版203653预检发现新增DOUBLE_ARRAY外参被旧参数读取器按STRING_ARRAY解读None，未开车/query0/无搜索录像；增加严格ROS类型解析与回归。最终435成员SHA0d145100656f87d692b6a81fdfa45a97b11c6ee30d98e46929cdf4051ea8328a，离线50PASS/0FAIL/6WARN；服务器3原点/5真实几何/24runtime/32occupancy/5terrain通过，生产C++不改。374回调范围已包含，不相加为仿真次数。
- 修复前session201924完整421.715秒/215.416m超时，7follow_failed=4反向保护+3碰撞弧，连接54均有候选（25触及时限），控制器max.5968s、外部新鲜内部>.5 tick1；不当作163334同类零连接复现。captures/20261015_sensor_origin_baseline/rviz_live.mp4失败录像424.625秒3397帧全解码SHAf0a38ee458d0c5a39dd8570e0d905671e3fd759a0f223dbb8d43370ecee2c9c8，已取回展示。
- 最终修复版session203918已过真实光心TF/参数预检，420秒完整搜寻进行中，待封闭；不称全部导航修复/目标到达成功，不commit。后续不得通过改点坐标/虚拟os_lidar外参来补偿射线原点，以免破坏已验证world投影；仍需同轮地图输入证据和完整验收。


- 射线修复现场203918封闭：421.850秒超时，Native176.337m、48following/42机动4段，1次碰撞弧中止、反向保护0、no_vehicle_path0；69连接均有候选（63触及时限仍有候选）。对照前轮7失败轨迹不一致，不称成功率改善。最大离站36.663m/结束21.645m，无mask/定位/确认，完整目标未过。
- 6段speed<.05且>=5秒静止，最长19.793秒；4段开始replanning_after_maneuver、1waiting_for_costmap_update、1following，需查整个窗口，不能凭起始状态断言全部原因或物理受困。1060评分/17反馈17存储/898命中，正标签中性未改变经验评分。控制器3579tick max.5339s、外部新鲜内部>.5计数0；模型快照median1.524/P952.070/max2.574秒，延迟待查。SDK持续停稳/errors=[]、独立ps无相关残留。
- 同8份封闭真实点云只换射线原点，82-170单元变化、障碍端点逐格不变；旧free→新unknown41-73格，只证观测覆盖差异，不作为真值/全部碰撞根因。分析浮点纳秒采样初版首查询舍入早于首消息，改整数；生成脚本一次换行转义错改直接传已解析文件，两者仅诊断未入生产。
- baseline_evidence.zip完整116成员SHA9838ce807a078fe242ca448a549166c4c205312a0955095dc40d0fdc2c4e49ba已取回逐员校验；修复版148成员归档SHA6b7dd4c8748a7d94726a2558ef8e043f0e0143d2fd625d57b7c60ceb4177bfdd本机下载校验待补。captures/20261015_lidar_origin_retest/rviz_live.mp4本轮失败真实RViz424.625秒3397帧全解码，SHA1311939d1b59ddd4bb116e415eadc27815fa559c703ff3c04f1a32a40613693c；右侧camera，无独立front_camera原始流MP4。
- 最终0d145100候选保留，真实光心接口组件已验证；完整视觉到达/稳定性未过，不commit。后续查机动后清图/重规划等待、碰撞弧输入与frontier执行代价交接，不重复更改点云虚拟外参/物理/保护去补偿。

- 最终封闭补记：修复版完整148成员field_evidence.zip已取回本机test_artifacts/sensor_origin_repair_20261015，SHA6b7dd4c8748a7d94726a2558ef8e043f0e0143d2fd625d57b7c60ceb4177bfdd；全部成员SHA/CRC及本机视频与归档视频一致性通过，field_archive_validation.json保留，原始SQLite仍在完整ZIP中。此前“下载校验待补”状态已结束。完整搜索仍失败、不commit。


### 2026-10-15 冻结版续测205942：后台续接晚到的交接缺口

- 详docs/PLANNER_WAIT_REPEAT_20261015.md。生产0d145100保持，单次Town/真实SBR/green residential house，420.768秒超时，Native144.555m、最大离站52.732m，29following/24机动5段/0follow_failed/0no_vehicle_path；仍连续静止173.005秒。677真实评分/17反馈17写入/896命中、GraphObservation效用差最大.191315；无mask/确认，不计成功。
- 时间线：旧FollowPath在252.009s完成，而后台候选5在1792069472.141返回51poses，候选6随后失败；之后pending=false/following=false，新GraphNav route持续接收但无执行。不是整个后段都等地图，也无证据称车轮空转。控制器内部源龄max.4671s/外部新鲜内部>.5计数0，模型快照max2.639s，不能据此定延迟根因。
- 生产execute_checked_path源码反例已复现：含换向的extending_path结果在旧动作已完成时仍缓存prepared_extension，旧完成回调已过去，无消费者；重复路线保护可继续抑制新计划。reproduce_late_extension.py以当前方法AST/合成路径/mock安全验证分别检查旧动作活跃/已结束；这是状态缺口复现，不是完整ROS/物理修复测试。现场bag缺full_planned_path/内部prepared字段，候选5形状未获证，现场根因与此吻合但未精确闭环；本轮未改生产。
- 更正上轮预算猜测：203918失败端点用最近已发布map/odom及匹配图heading，实际插件隔离重放0.511s成功，6连接中5可行；没有复现现场拒绝，不能断言预算是根因。input_scope.json/replay.log明确私有地图/实际请求差异，且重放与现场共享CPU，非受控性能消融。未调预算/关保护。
- 本轮SDK持续停稳/errors=[]、独立ps无残留；captures/20261015_lidar_origin_repeat/rviz_live.mp4真实双栏3391帧423.875s全解码，SHA b9d5380d37d84d120a8ce4606c18c14e52b7bcae6e6decf28c53ea465af1fb62，服务器一致。无独立前摄MP4。完整验收失败，交接缺口待修，未commit。

- 本轮完整归档test_artifacts/planner_wait_audit_20261015/field_evidence.zip已取回，163成员SHA/CRC全部校验通过，外层SHA cc684dca9ac6fbdcfe8c81bbf0697937f61c87885518f297cf793dd517f43573；本机MP4与归档视频一致。archive_validation.json保存结果。状态反例脚本与诊断脚本语法检查通过，原方法反例两状态均复现；这是故障检出，不是修复通过。


### 2026-10-15 后台续接完成顺序修复候选与现场复测

- 详docs/LATE_EXTENSION_REPAIR_20261015.md。仅execute_checked_path将缓存/替换限定为following仍活跃；旧动作先完成时走正常motion_queue/send_motion_segment，实际pose/双地图/新鲜度/执行器/隔离/换向保护保持。无速度/物理/目标/原生规划器调参。
- 新增10回归，原版8项初测4失败，修复后384回调替身全通过，全局离线50PASS/0FAIL/6WARN。实际安装32项相关交接/所有权和118路由通过。首真实ROS入口误纳替身专用整套测试，109项7FAIL/9ERROR证据保留，mock属性/typed message/重复init等不兼容；未为测试放宽生产。两验证范围分别报告，不以数量当成功率。
- 冻结435成员SHA262264de3fc9ed131fafd67a7a43901f01bf467c577c0abccdc8af6fd8535377，相对0d145100仅graph_path_bridge.py与test_planner_execution_ownership_ros.py。服务器保留bundle_0d145100及manifest，重建Python adapter；C++两库SHA不变。当前完整现场session213429已过预检并搜索中，闭环验收待补，不commit。新独立记录器补录full_planned_path，不在活动bag上跑SQL。


- 续接修复213429封闭：421.525秒超时，Native217.792m/max离站53.556m，41following/33机动6段、2follow_failed、1no_vehicle_path；5段静止最长13.302秒。本轮轨迹不同且22份full_path里completion_first_cases为空，不能称现场精确触发/验证该竞态或成功率改善。870评分/20反馈20写入/759命中，GraphObservation效用差最大.190015，无mask/确认，任务仍失败。
- 两执行失败在329.902s无路径进展、374.554s控制器碰撞弧中止。前后均见请求倒车/实际R挡/ACK和执行器正常却Native direction·velocity为正。新定位cmd_vel_to_control.on_state用hypot丢方向、drive_outputs及积分按模长误差；原方法+生产drive_outputs两状态反例证明“倒车1.2”和“前行1.2而请求倒车”不可区分且可输出0油门0制动。此接口缺陷已确认存在，地形/传动贡献未闭环，不能一概归为规划失败或断言所有失败同因。速度环尚未修改；后续保持换挡/超速/静止的模长保护，独立引入有符号纵向反馈及反向滚动制动，勿仅加油门。
- SDK持续停稳/session errors=[]、独立ps无残留。真实RViz captures/20261015_late_extension_retest/rviz_live.mp4 3407帧425.875s全解码，SHA4ab562df8aa1c1ce47282e54a8b3f5391f820dc0af4cab1e1470643335a33567；193成员core_evidence.zip SHA124cb3ac9e078063ea3c0c4f2dd9983d3a8f571ca74a4159d30fbe75dd589024逐员/CRC通过。大点云原库仅服务器完整194成员late_extension_field_20261015_213429.zip（SHAd86fe33b931711afd71080a39b64fb793d3f6ff86a15c157005359f160a19343）保存，外部成员manifest在本机core中，不混称已下载全部大点云。完整任务仍失败、竞态无精确现场验收、速度缺陷待修；不commit。


### 2026-10-16 有符号速度反馈修复候选

- 用户连接服务器继续，启动前无相关残留，旧生产262264de逐成员一致。核实真实StateSensor同时发布世界velocity/dir，同一Native poll来源。新增longitudinal_ground_speed归一化水平车头方向投影；模长仍管停车/换挡/超速，纵向PID使用请求方向进度，实际反向>.15m/s复用原direction_change_speed阈值全制动/清积分，停稳后正常驱动。方向无效则制动，不靠挡位猜方向/模长兜底。新控制诊断含signed值和制动原因，话题/类型/戳不改。
- 新9回归先在原版失败；首修复出现steering未初始化，被回归捕获并在部署前修正，恢复夹具dt=0改推进20Hz时钟，不改生产保护。最终393回调、全局50PASS/0FAIL/6WARN；实际ROS消息9新方向+32交接通过。冻结435成员SHA9bcaf8700786cbaf2ab40a22405121f20288fae5e8f51d6d777fbf5eb3d87997，仅drive_math.py/cmd_vel_to_control.py/test_cmd_vel_to_control_ros.py相对262264de变化。C++原生库/速度参数/物理/碰撞/半径保持。详docs/SIGNED_SPEED_REPAIR_20261016.md。
- session20261016_085025单次Town/green residential house/420秒完整搜索已过预检，现场还进行中；坡道制动后能否恢复、是否反复制动及完整到达需封闭验证。不得仅凭signed分支输出宣称物理或搜索成功；不commit。


- 20261016_085025封闭：421.967秒超时、Native225.141m/max离站40.634m，45following/39机动5段、0follow_failed/0no_vehicle_path，4段静止最长12.304s；与前轮轨迹不同，不推导稳定性改善。8986控制样本signed均有限、方向/制动违约0，但反向制动分支触发0，**反向滚动制动恢复仍未现场验收**。678评分/28反馈28写入/628命中，GraphObservation效用差最大.191248，无mask/确认，任务失败、不计20轮成功、不commit。
- 新时钟证据：控制StateSensor源戳落后median11.482s，但与独立Native速度8184样本对齐最佳偏移-.05s/MAE.02124m/s；按-11.5s比对MAE.87865。支持源戳时间域偏移，不支持“控制用11秒旧速度”；仅1D速度对齐，不外推所有字段或精确采集戳。源戳契约仍待核原ROS桥_make_header/场景暂停锚点，不能简单重盖当前戳。state_clock_alignment_review.json保留。
- SDK持续停稳/errors=[]、独立ps无残留。captures/20261016_signed_speed_retest/rviz_live.mp4本轮真实双栏3392帧424秒全解码，SHA9a7dca290a7886305f30317fc01365dd71d39bfd238163c8d003bc7be0ccfcc1，已向用户提供。196成员core_evidence.zip SHA7c47463a9a3f6d5172c4244a3c1a019c461fea38927aff7751cc778b4bd70191逐员SHA/CRC通过；大点云原库仅服务器signed_speed_field_20261016_085025.zip，外部成员manifest见core，不混称本机完整原库。完整报告docs/SIGNED_SPEED_REPAIR_20261016.md。


### 2026-10-16 用户质疑回加油站：全链决策审计（未改生产）

- 详docs/SEARCH_DECISION_AUDIT_20261016.md，证据test_artifacts/search_decision_audit_20261016。复用封闭085025及前轮213429，不新跑/不改生产，435成员仍是9bcaf870，服务器无残留。没有mask/定位/确认/召回，不是已证把加油站识别成绿房子；frontier分数不是目标检测置信度。不能把FollowPath0失败当整体规划正常。
- 确认上层/执行归属缺口：642决策分51连续目标组，Bridge Following只有5阶段；640首动作后决策449与最近Following graph_goal>4m（含等待/完成窗口，非全部活动中换目标）。当前被选择目标在采样时距车体最近9.965m、无<=6m，不能假定访问/观察退役闭环已发生；缺私有current_node/visits记录，不断言其始终0。
- 已进入执行的候选含48.1m/6段、74.04m/12段、70.07m/7段，图段max_segment_length24只约束图采样，不约束真实路径队列。GraphNav按heading/欧氏距离/访问数选点，无真实路径/换向/时间成本反馈；on_plan_result观测heading候选跳过普通比较，原方法隔离反例已复现（验证器mock，不证明存在更短安全替代）。旧路线多机动与新目标不断切换叠加导致绕行，不能删取消保护或给加油站拉黑。
- 另两记忆契约缺口：GraphObservationStore同node所有heading共用negative_evidence=max，方向0=1/方向8=.1却对8施加1，原存储反例复现；按UUID覆盖新节点不继承，无私有历史DB不声称本轮具体覆盖条目错误。mask被竞争/支持拒绝后仍用1-max(raw cosine)作absence strength，示例.99→罚.002与留当前目标+.15尺度不一致；不能把无mask当整个区域空或直接调权重。
- 更正先前经验功能描述：消费端能处理负反馈，但Bridge finish_execution_feedback以succeeded为eligibility，只发OUTCOME_SUCCEEDED。实际前轮2执行失败只有learning_eligible=false诊断、20学习反馈全成功，本轮28也全成功；628命中/28写入/0改分不能解释为已学避失败或重复搜索。可信几何/控制/动态/停止/旧消息须分类，不能将所有失败训练成障碍，搜索无目标应归覆盖。
- 修复顺序：探索目标交易/归属→车辆实际执行代价→按方向覆盖与分类失败反馈→统一时钟和有界可追踪日志→组件/封闭重放/完整现场。全部未实施，不commit；当前规划整体仍未通过。Mapper visited不重新添frontier保护已存在，排除“清了立即重加”未证假设；StateSensor11.48秒源戳偏移仍不支持11秒旧速度，保留先前更正。

### 2026-10-16 探索目标交易/观察/退役修复候选（独立组件通过，未现场部署）

- 用户继续后按审计依赖先修第一层。详 `docs/EXPLORATION_OWNER_REPAIR_20261016.md`，证据 `test_artifacts/exploration_owner_repair_20261016/`。新 `ExplorationTarget` 保存目的地/观察朝向和进程实例＋序号身份；评分更新不抢占，frontier UUID 暂时消失不悄悄改目标。新鲜同 frame odom 优先计算实际距离；图 current node 的 6 米连接半径不能代表观察完成。
- `route_status` 增 exploration_id/endpoint_observed，Bridge 状态带同 ID/实际探索目标；完整探索图路延伸至真正观察点，不完整 prefix 不跳未知尾段，对象/记忆目标优先不变。Bridge 在实际 1 米内、朝向误差<=.35、速度<=.15、新鲜输入且无活动/待定动作时回传原路线 header/末点；GraphNav 精确关联交易后，还要同 frame/源龄<=.5s/有效姿态/实际 1 米范围内持续1.5秒采集时间的低速观察才加 visits。无 odom/无回传/暂停采集戳不能完成；图空路径失败不再加成功访问。
- 拒绝只影响当前交易；同交易最新图路可更新长动作后的反馈，不能借用新目标的相同 endpoint。增加 retired_exploration：观察完成、有效拒绝、图无路、frame/目标复位均明确退役。Bridge 复用取消隔离/hold 撤销匹配动作，最多64退役ID拒绝晚到路径；覆盖先退役后元数据/路径、重复退役、旧 query/旧目标/未来消息，旧消息不得复活或取消新目标。没有关闭物理/碰撞/曲率/取消保护或修改速度、转弯半径、对象到达标准；没有给加油站拉黑。
- 安装入口现在重建六包（包含 GraphNav），配置记录/启动核验 libgraphnav_planner_component.so，避免新 Python 搭配旧 tracking C++。接口 bag 增回传与退役。冻结436成员 `11af6c17b0335766c50ce892bf128af5ab88f9a7a82571e0799f3e555e06d9cd`，相对生产11成员变化，`deploy/20261016_exploration_owner_candidate/`；旧版本按hash保留。服务器 `/home/ly/exploration_owner_repair_20261016/final_install` 独立候选库 SHA `cb188e47921472dd6e598e20fac3be8874575cdde1ecb848a7b1449d15c15dc7`；当前实际生产435成员9bcaf870逐员核对未变，未部署候选。
- 最终本机全局50PASS/0FAIL/6WARN，410回调替身、console28通过；服务器真实安装C++27、Bridge新17＋已有交接32、实际C++节点/DDS5通过，范围重叠不相加成仿真次数。双向DDS用合成图/odom/free grid，仅观察接口隔离执行器健康，没有动作服务器/BeamNG；不能当真实车体或视觉到达验收。首轮fake Header对象比较、旧套件ROS init与map/spot1 frame夹具失败保留，未放宽生产保护。双向测试还覆盖实际位置领先图 current node，确认不会刚观察完即因旧图距离再选相同点。
- 已测试组件源da6aeea0与最终11af6c17仅安装说明不同，服务器逐员核对全部生产/测试源码相同，freeze_attestation保留。27成员组件证据包已取回本机，SHA `306023d6954420faecbfae3c96b9bf684e7c6cec66a1334d881b8f18d3b55a6f`，ZIP/26有效载荷逐员SHA通过。summary.json与最终离线/真实ROS日志已封存；最终独立进程核验无相关残留。
- **尚未解决**：观测heading直接接受高代价路径/真实执行代价未接回frontier比较、按方向覆盖证据串扰/raw cosine混用、分类失败经验生产、StateSensor时钟域。目标持有不会自动消除70米绕行，不能宣称整体规划修好；先处理这些依赖再冻结配套版本现场测试。无新现场试验、无新MP4、不计20轮成功、不commit；旧失败录像不能当本候选验收。

### 2026-10-16 用户要求配套部署及现场复测：交接组件通过，完整搜索失败

- 新要求覆盖前段暂不部署计划。11af6c17已部署，旧9bcaf870 bundle/manifest和console配置备份；自助配置最后source `/home/ly/exploration_owner_repair_20261016/final_install/setup.bash` 并核GraphNav库身份。实际 /proc/maps证明加载cb188e47新GraphNav、原f915a4fc Smac/bd96eb9d RPP。入口17真实消息测试、源码/配置/库门禁通过。详docs/EXPLORATION_OWNER_FIELD_20261016.md。
- 首启session095604在未发目标时控制门SetParameters响应超时，goals0；安全停稳/清理，未开始录像所以无该次MP4。原原因未闭环，同版本完整重启服务正常，不改门禁超时/保护，不归为模型或图规划失败。
- 完整session095740预算420秒/result423.913秒timeout，Native196.599米/max离站60.979米；39following/31机动4段、3follow_failed（均follow_aborted/status6）、5no_vehicle_path，未到绿色房屋。1024图决策/10交易，同ID目标漂移0，27观察ACK、4 observation_completed和5 vehicle_route_rejected。**真实车体的观察退役、无路换点交接已实际发生，完整搜索仍失败**，不计20轮、不commit。
- 首目标距出生点约14米却花171.932秒/行驶82.639米；最大完整发布候选42.068米/6段/位移15.423米，不能把候选都算已执行。末段静止61.766秒，5组候选3/7/10/9/15次空路径；Smac有最大迭代/无路警告，Bridge收到empty_path。用户明确指出低效打转和频繁不动；下一步按封闭地图/姿态重放朝向代价、连续起点和失败恢复，不继续只刷整轮。Native中心与odom参考点需区分，1米观察以odom而非Native中心核验。
- mask/定位/确认/visibleapproach均0；1068评分、27反馈/写入、926命中但memory_modified_scores0，不称识别/视觉到达/失败学习成功。SDK持续停稳、session errors=[]，独立ps无相关残留。仍保留真实723节点/467车身碰撞/四轮32碰撞/8米实际12米规划半径，无速度或物理修改。
- 已回传并展示本轮失败RViz：captures/20261016_exploration_owner_retest/rviz_live.mp4，3423帧/8fps/427.875秒全解码，SHA569c7e9ef23b63c96c1bb2df81120cfeace689866793aa9d33eb9695130f937e，无独立前摄像头MP4。本机core_evidence.zip SHA5e1d744072671f5eb82d06ccc5126a5d235ba1df64f71be7f45b5048546c86ff，117载荷/CRC/SHA均验证；原始点云/拓扑1个DB仅服务器原路径留存，外部manifest附路径/长度/SHA，不声称本机已全部下载。所有新分析均在driver/session/goal结束后开SQLite，避免此前录包锁故障。

### 2026-10-16 用户质疑效率/改动范围：重规划仍失败，封闭对照及恢复候选

- 已明确回复用户：此前修的是目标归属/观察/退役，不能称重规划与规划整体已修好；首近目标83米/172秒、末5目标44空候选/61.8秒停驶均是未解决的执行效率问题。暂停重复完整轮次。详docs/PLANNING_ESCAPE_REPAIR_20261016.md。
- 实际Smac、记录地图/车姿10组对照：首原heading45.834米6段，保持起点heading43.546米6段，朝目标方位角无路，故只换heading不是有效根治。末原目标3朝向均无路，前3米/正负6米无路，倒3米3.098米单方向段可行。发布OccupancyGrid重建有量化、请求部分由元数据推断，不称私有地图/请求逐字节重放或物理执行成功。
- 用实际odom/control/status重建保留跨frontier物理历史的近期前驶40点走廊，原reverse_escape生成2.167米弧，在记录全局/局部地图均通过。生产恢复只覆盖部分execution_stalled，不覆盖多目标规划均失败，是末段缺少局部退路的机制缺口；不能把空路径训练成静态障碍。
- 新候选仅3文件（graph_path_bridge.py/execution_recovery.py/routing测试）：两不同探索交易60秒内同2米区域耗尽且全部empty_path、低速/新鲜odom+两图+健康传输/无活动或不确定动作/无对象记忆query占有时，才拟合原近期已驶2–4米倒车弧；原曲率/碰撞/姿态/动作保护及同点3米内不重复保留。没有有效历史则正常失败，不造退路，不改速度/半径/模型。
- 同时发现11af退役reset会清走廊并取消“已拒绝目标后启动”的独立逃逸，2条源码回调反例先失败后通过；这是上一层修复的状态冲突，**现场没有逃逸触发，不称本轮停驶直接原因**。同frame探索退役保留有限物理走廊/失败历史，独立恢复不中断；frame/query/显式重置仍清理，恢复结束拒绝旧退役路线。覆盖碰撞不可调度、旧消息、查询/记忆/干跑/新鲜度和隔离动作。
- 冻结436成员3fe6f21557bf25408de7668f895bc53b55a56608992ef02e4d8b86d305069b14，deploy/20261016_planning_escape_candidate；服务器仅/home/ly/exploration_efficiency_audit_20261016/install独立候选，实际生产仍11af且逐员未变。全局50PASS/0FAIL/6WARN、422回调；真实ROS新29/既有32/DDS5通过。扩展75旧套件实际ROS出现7失败3错误，部署11af相同基线也同样失败（Mock接口/ROS强类型夹具），保留两份日志，不称该75项通过，不放宽生产。
- 47载荷服务器证据包已本机逐员SHA/CRC验证，SHA0faf6ed5c0a00e57c6f2db88a0d1fb50779b6a384ab51e67b2844694de4a202b，test_artifacts/exploration_efficiency_audit_20261016/summary.json。候选未部署、无新现场/录像、未commit；局部恢复现场效果待验收。**frontier间实际车辆代价比较仍未实现，首目标低效绕行仍未修复**，不得把恢复候选或单次退路可行称为整个重规划解决。

### 2026-10-16 继续修复恢复交接、补充实际frontier代价对照及清理磁盘

- 详docs/RECOVERY_HANDOFF_REPAIR_20261016.md。在3fe恢复候选中复现：控制器不可用或下发时地图过期，未发动作却残留recovering=True，后续start_segment被永久挡住。现在未分发明确返回False；无请求/活动/隔离动作才释放恢复状态和尝试位置，保留真实错误及重试入口。新增控制器恢复后可重新调度测试，不以清队列证明已发动作结束。
- 补取消隔离保护时，审查检出“受困恢复在隔离解除后没有消费者”的中间候选风险；保留f916行为反例。最终隔离期只存有界恢复意图，等旧动作终态/持续停稳后，按epoch/query/frame/探索ID/目标身份、60秒TTL和0.8米位置范围重新核验，在最新车姿/双图/健康传输上重拟合已驶退路。query改变、过期或移位即撤销，正常重规划；碰撞/曲率/取消保护、物理/车速/到达不改。不能把这些尚未现场触发的源码缺口当上一轮61.8秒停驶的已证原因。
- 最终436成员415c0bef49adf99f0fd7c64de75d7210973715090c6002fe8742064b92f7dde3，deploy/20261016_recovery_handoff_candidate，相对生产11af仅Bridge/recovery/transactions及2测试文件，共5文件。服务器/home/ly/planning_recovery_handoff_20261016/install独立安装，实际导入SHA符合manifest；生产436成员逐员未变。离线50PASS/0FAIL/6WARN、428回调通过；实际ROS消息32+35项及C++/DDS5项通过，均非真实车体测试。6新增回归覆盖下发失败、恢复重试、取消晚到、停稳、目标变化、过期/移位。36载荷证据ZIP本机SHA/CRC全部通过，SHA0ab2fc9c76adfc7d0df9551d2e44025a4ebf4fd45fc3dd6fe35effc5477ab5c6，test_artifacts/recovery_handoff_repair_20261016/summary.json。未部署/未现场/无新MP4/未commit；不计20轮成功。
- 同一首路径输出起点/已发布地图/实际Smac，精确匹配scored图与原始拓扑，8候选按初始图规则均连通：原目标45.834m/6方向段，对照22为14.132m/3段、25为27.351m/4段。语义/物理置信度不同，且该图比原目标首次选定更晚，不能称精确原决策重放或最短等于最好；地图量化/私有请求边界保留。heading motion_cost只为角度代理，相同约.174对应不同真实路线。**frontier车辆代价反馈仍未实现**，整体搜索仍失败；下一阶段需有界只读比较和明确选择提交/ACK，不能借显式目标接口或绕过图路线。
- 用户授权清理本人无用文件：删除可重新下载安装ZIP22,982,395,117字节；旧临时安装420个逐文件双SHA一致副本39,450,001,060字节，正式安装及唯一不同legran.zip保留。总约58.14GiB，模型/当前生产/未闭环原始bag和日志保留，清单见证据cleanup目录。最终df可用约131.1GiB，期间共享分区另有变化，不能将全部空间增量归于本轮；普通权限无法判断其他数TB归属。服务器最终无本轮仿真/规划残留。

### 2026-10-16 用户要求修复并重测：车辆代价交接已现场发生，完整搜索仍失败

- 详docs/FRONTIER_COST_REPAIR_FIELD_20261016.md。新增ExplorationCostComparison与ExplorationChoice消息：图层冻结最多4个24m内、现有图完整连接候选；Planner值拷贝探测不改变活动规划状态。Bridge空闲/低速/无对象记忆占有时固定真实起点、有界12秒只读比较实际Smac路径长度＋原3m等价换向，以原.004/m系数结合原效益；提交前共同最新图重验。比较路径不下发/不发假经验；图层核验路线ID/header/endpoint、候选、新鲜同frame odom和<=.25m/.1rad起点变化后，确认并保持或更换探索身份。Bridge收到确认的新图路后重走正常规划与双地图安全检查；丢确认走有界重发及真实clear-goal barrier，旧消息/新query/活动控制隔离。
- 首可行观察路径直接执行的旧行为在415c冻结代码上新反例真实失败；新增传输epoch后旧比较缓存覆盖新候选的反例也先失败后修正。13新回归、全局50PASS/0FAIL/6WARN、441回调通过；服务器真实ROS13新＋32恢复＋35交接、C++/DDS9（4新5旧）、路线C++27及Linux7包安装通过。安装增加graphnav_msgs，记录vehicle_choice/selection_ack。实际记录Smac候选24/25/20/21经生产比较器选择25，27.351m/4段（等价36.351m），原24为45.834m/6段（等价60.834m）；合成当前戳/只读，不算物理成功。
- 当前439成员e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb，deploy/20261016_frontier_cost_candidate，相对11af共16文件，已部署，未中途修改。新GraphNav/消息在/home/ly/frontier_cost_repair_20261016/install，最终Python在release_install，编译源逐员同冻结C++；运行/proc/maps GraphNav SHA33b48f3c20fc234a1c87b30801c2b6b30046247af24e897f6350e685ca31ea39，Smac/RPP仍f915a4fc/bd96eb9d。原bundle备份/home/ly/wildos-console/bundle_11af6c17_before_cost_20261016，console/source旧身份亦保留。
- 首启driver144940/session144941因诊断误复制main console的rviz=false被预检拦截，goals0/控制未解锁/无搜索MP4；保留startup_failure.json。修正为上一轮验证过的现场配置，只更新setup/libidentity，再重启。后续复用/home/ly/frontier_cost_repair_20261016/field/sensor_full_console.json，勿再混用默认自助console配置与录制现场配置，不放宽预检。
- 完整重测driver field_frontier_cost_search_20261016_145142，session20261016_145143，真实SBR/Town/green residential house，420.889s超时，Native93.147m/max离站54.440m/末53.183m。22有效代价评估、8唯一选择/8selected ACK、7目标替换、4探索观察完成；首点12.929m/22.986s与旧目标不同，不能当受控成功率提升。21Following/14机动4段，3次action6逐时对应RPP bounded command arc碰撞，19次下发前local_footprint_collision；合22follow_failed、12no_vehicle_path、2no_free_graph_candidate。最长静止216.248s，恢复started/done0，未找到绿色房屋，不计20轮、不commit。967评分/7反馈写入/840命中/memory_modified_scores0/objects0，不能称视觉记忆或目标到达成功。SDK持续停稳、session errors=[]，独立ps无本轮残留。
- 闭包恢复重建前4次无路：原策略可拟合，78前驶点最近年龄13.6/35.6/52.8/54.1s、距车<=.736m，记录双图均通过；local戳龄<=.595s/odom<=.368s/global<=1.644s，低速。不能因此声称私有门禁都满足：缺try_planning_escape内部失败历史/隔离状态和逐项原因，实际为何没有启动尚未闭环；反向走廊仅1点，不支持擅改前行逃逸或放宽TTL/碰撞。下一步先补有界门禁诊断，用此反例定位最早未满足条件，勿盲刷整轮。
- 已主动展示本轮失败真实RViz captures/20261016_frontier_cost_retest/rviz_live.mp4，426s/3408帧/8fps全解码，SHAe160ee31a949a5407134f2bce9fcb9b2c1e2229f649bc0a3924ce06d9e0c02c9，无独立front_camera.mp4。核心75载荷ZIP SHAda55d44a01266fceed8417934826c4ccea9b2a141ec5ef2994f08edaf6e9c306、组件30载荷SHA5838e69b7bb1074b462859f3e3e1827aea00341ee936246a0b8ab3d0c9f9e78c，本机CRC/逐员SHA及视频一致性通过，test_artifacts/frontier_cost_repair_20261016。1个大型原始点云/拓扑DB只留服务器原路径，外部manifest已附。恢复报告在封包后补充单存同目录；比较内部路径只留代价/地图戳与typed选择，不能冒充逐条几何已录。全部SQLite仅读封闭包。

### 2026-10-16 用户质疑探索被改坏：原版对照及真实图回放审计

- 详 docs/EXPLORATION_REGRESSION_AUDIT_20261016.md。本次仅审计，不修改生产、参数、物理或安全保护、不启动新仿真；生产仍439成员e6f28118，未commit/未增加成功数。原仓库只读核实HEAD ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572，五份核心源码从Git固定导出、统一换行作差异。服务器七关键源同本机/冻结清单，实际GraphNav库SHA33b48f3c同现场。
- 确认新增探索策略是主要责任之一，不可只归因真实车体：reject_exploration_route只发PoseStamped，丢失失败原因/局部运动段/地图证据；ExplorationBackoff统一屏蔽目标12m/失败位置8m/120s及邻近图边。局部车姿或前缀失败不证明16.84–71.81m外的目标邻域不可探索。本轮21退役匹配12无车路、7重复local_footprint_collision、2无候选，6条拒绝端点距最终目标>10m。
- 封闭145143抽样62 scored图按源戳/current/UUID/pose精确关联原topology，零失配；实际生产C++ Backoff/Planner回放，748实际选中目标重建屏蔽误匹配0。以bag接收query为0：240.410s有60frontier/51基本候选，区域退避屏蔽51，其中28仍有完整图路；418.288s有60/56，屏蔽56，其中29有完整图路。13份抽样候选全部区域屏蔽；图路不等同真实车辆可执行。图边屏蔽未使已有完整候选断连，不能把早期怀疑的唯一出口断开当本轮原因。
- 实际239.727–312.354s连续72.627s无新route_status/path，期间仍有frontier/视觉评分；首退避311.905s到期后约.449s恢复选点。choose_exploration_goal空候选→plan_to_goal直接return，无耗尽状态/到期或恢复出口，形成探索停摆。该窗口是216.248s最长静止的一部分，不能解释全部静止；try_planning_escape私有门禁/失败历史缺失仍未闭环，不盲放TTL/碰撞。
- 当前已含自定义自动选点、heading硬筛、visits、固定观察姿态/目标保持、已知图路优先和车辆代价比较；不能再称完整原生WildOS仅加记忆。748决策644 complete，967真实评分仍在运行，不说模型完全停用。保留既有归属/取消/坐标/握手保护；成本比较的24m/最多4是比较集合限制，不是全局探索半径。
- 新反例复现Python heading把低置信度/过期否定/缺assessment全部方向标不可通行，而C++允许未知继续探索；但本轮62图/3521节点帧该额外veto=0，不当现场原因。旧GraphObservationStore方向共享negative=max、UUID更替无覆盖继承及Bridge仅成功eligible的失败经验生产缺口仍存在并重现；本轮7写入/840命中/modified0不证明已学会避免失败。无目标mask/定位/确认或非空召回，不把绕回加油站说成记忆偷换目标。
- 本机test_artifacts/exploration_regression_audit_20261016，原版/diff/实际方法反例/verification齐全。证据ZIP22成员SHA bd532deb9a0d28560eae1f412a020cac4ee2ac566898e9dc2306e8319245cfc8，CRC/21载荷逐员SHA、7关键源核对通过；四份完整CDR已取回，另58份保留服务器路径/长度/SHA。审计程序首次批量反序列化大图内存约13.6GiB已终止自身并改有界抽样；构建路径/UUID数组/夹具初始化问题仅在诊断修正，无生产变更。最终无审计/仿真/导航残留。
- 后续优先修失败证据范围与候选耗尽恢复出口，再统一heading三态和覆盖/经验类别；同车/同地图/同接口基线正常扩展后只开记忆增量作对照。树林地形/代价图来源和全部静止原因尚未闭环。本次仅完成审计复现，未实施上述修复，完整绿色房屋/20轮/多目标仍未通过；旧失败RViz路径保持，不生成或冒充新录像。

### 2026-10-16 用户要求继续修：探索范围与恢复重试候选（尚未现场部署）

- 详docs/EXPLORATION_SCOPE_REPAIR_20261016.md。候选439成员23675e2881fc797e2e65699cd045cfb7942cf7ae7bdf43785b6f0beb6dd2b769，deploy/20261016_exploration_scope_candidate，仅6成员（Backoff/PlannerNode/Bridge/recovery及两测试）相对e6变化。服务器/home/ly/exploration_scope_repair_20261016/install独立安装；实际生产e6逐成员仍未变，无新仿真、录像或成功计数，必要现场验收待执行，未commit。
- 已查实拒绝接口无障碍证据范围，删除其推断的目标12m/车旁8m/图边120s屏蔽；仅暂缓实际目的地（.25m匹配）、原120s上限，具有失败位置时只在.8m起点上下文内保持；移位重新可选，缺位置不假定已恢复。身份/60s消息期限/query清理/显式目标隔离/容量继续保留；候选空增加节流最早retry诊断。真实地图障碍与碰撞约束不改。
- 新反例证明旧恢复在local map或执行器暂时不满足时连合法规划失败都丢掉，且idle monitor无消费。现在合法empty_path交易先入有界历史，两个不同交易在同处失败后保存有界意图；同frame目标退役保留，idle重验再拟合原已驶倒车走廊。重验query/frame、位置<=.8m、失败证据<=60s且仍两交易、odom双图/执行器新鲜、停稳、曲率/footprint/tracking及控制权；显式/记忆/停止/过期/移位撤销，不抢pending/following/quarantine动作。控制器缺席至少1秒间隔重试，恢复启动/失败正确释放。修正独立恢复无route_endpoint时planned/following状态异常，并实测到动作接受回调。此为源码机制反例，不把它直接等同旧现场每次未恢复的已证门禁原因。
- 旧e6实际C++库/头上新反例失败（错排其它目的地/失败位置/推断图边）；旧Python真实行为反例也失败（local恢复+退役后未调度）。本机50PASS/0FAIL/6WARN、446回调替身通过；服务器真实ROS交接/成本50、实际C++路线及DDS既有9/新增2通过。新增DDS证实4m外近邻仍可选、起点移位后再选，非车辆实验。完整头/消息/安装导入SHA已核。
- 同封闭62图+原21拒绝序列的实际C++对照：全候选屏蔽13份→0份，240.410s和418.288s原剩0个完整图路候选，新各22。只证明过滤规则修正，不是真实汽车可达或新轨迹成功。heading低置信度、方向覆盖共享negative、失败学习缺席等其它缺口不混改；车体/速度/控制器/Smac/模型/阈值/消息结构均未改。
- 初C++审计缺nav_msgs仅补诊断依赖；离线总检4个DDS错误为Windows沙箱strict路径解析拒绝，正常权限重跑全通过，无DDS保护修改。新节流使旧立即重试预期失败，补节流窗口验证后通过。日志均保留；不是生产新故障。
- test_artifacts/exploration_scope_repair_20261016：before/diffs/delta/本机与server验证/同图结果齐；26成员组件ZIP SHA958af239bfcb8852636859b99389d46f2addebaa89be9c804b5c6118ef447959，CRC/25载荷逐SHA/6源核对通过。候选GraphNav库SHAff8840117a16f836e3779f44087633f8f65bc7f2c3a1a027b4b73b488a95ff51。服务器组件结束无残留；生产旧库33b48f3c保留。
- 下一步现场应先固定真实车体/出生点/配置，建立无新增记忆先验干预的受控探索基线并验证恢复，再只开记忆增量；候选非完整原版WildOS对照。整轮216秒静止/树林输入来源与全部恢复门禁尚未闭环，不以回调或22个图候选宣布完整绿色房屋、20轮80%或多目标已通过。

### 2026-10-16 继续修复并测试：无记忆先验基线暴露失效frontier仍被持有

- 详docs/FRONTIER_RESOLUTION_REPAIR_20261016.md。生产e6逐439成员保持，236候选从独立目录跑Town/真实SBR/green residential house；三类新增记忆先验关闭，构造实际None、ConfirmedViewpointCache初始空，其余共同适配存在，不是完整原生WildOS对照。每次先查残留；首启动诊断预检路径映射错误被拦、未发目标无MP4，修正测试入口后重启。
- session194721完整421.850s超时，Native205.121m/max离站52.202m，40following/30机动5段，3次RPP bounded arc碰撞，4次no_vehicle_path；最长静止27.351s、592决策无>5s路线发布空窗，轨迹/开关不同不推断成功率改善。恢复started状态2次中含退役回调重发，实际1轮347.255–353.396s倒车1.701m、signed min -1.123m/s后done并续探索。mask/定位/确认0，仍失败不计20轮。
- 新定位ID7持有222.009s，行驶109.287m但最近仍距目标Native12.683m；296决策仅12完整图路。三份scored和原始拓扑逐戳/UUID/位置/节点数关联，保留六份CDR，中段同节点已经非frontier、visited0，末段缺失；旧保持分支不核验明确非frontier，持续追逐过时探索意图。诊断单戳索引覆盖重复版本造成一次假失配已改多版本精确匹配，不当生产坏包原因。
- 274c3ef2候选相对236仅4成员：ExplorationTarget/PlannerNode/C++测试/Bridge测试。仅新鲜同frame、严格更新采集戳、同UUID且.25m位置匹配的明确非frontier授权frontier_resolved退役；暂时缺失/改分仍保持，旧/重复/过期/未来消息不撤销新目标。沿原取消/隔离交接，不加visits/到达ACK/障碍区域退避/失败地形学习，不改变模型、物理、速度、半径或对象到达。活动中退役可能增加取消交接，现场须核抖动及效率回归。
- 旧实际C++/DDS反例失败后，新本机50PASS/0FAIL/6WARN、447回调替身，服务器C++构建/路线、真实ROS51、既有DDS9/范围2/新增3通过。初测试脚本覆盖PYTHONPATH误丢ROS路径已改继承；未修改生产保护。候选GraphNav库db52f00e06d7c598ecba3e8c42d1b34019693cd879f03537a5a36e9f6e1c6b8f，Smac/RPP保持。修复后session200923独立候选已过严格预检与实际加载核验，搜索进行中；完整验收待补，不commit。
- 本轮失败RViz已取回并打开：captures/20261016_scope_baseline/rviz_live.mp4，426.25s/3410帧/8fps全解码，SHA8237fa49a199147017cb340ea2ad446ce17c1adfa0014c9a2bc72c7af65feca0。右侧camera、无独立前摄MP4。141成员核心ZIP SHAbfc858cabb255d969adea9fa5537db1c7b89f47da7fb7d87126a90428b7c1328，本机CRC/140载荷逐SHA/视频一致通过，test_artifacts/exploration_scope_repair_20261016；3.264GB原始sensor DB仅服务器保留并附外部manifest。新修复在test_artifacts/frontier_resolution_repair_20261016。整体树林输入/碰撞和完整搜索仍未解决。

- 修复后session200923已封闭：421.901s超时/未到目标；Native123.653m/max离站48.341m，57following/33机动1普通段，follow_failed和no_vehicle_path均0。696决策全完整图路、无>5s路线发布空窗；源图frontier_resolved37/vehicle_cost_selection9，Bridge匹配退役34。最长持有目标91.964s、28.499m行驶/最近Native8.004m，三个原始拓扑抽样仍是frontier，不能将该段当作旧的非frontier仍保持反例。
- 本轮21条取消FollowPath请求、23条控制器停车日志，14段>5s静止总109.852s/最长16.256s；无长停不等于稳定，交接频率仍高。execution_recovered23是隔离解除状态，不是23次倒车。退役当前仍走reset/cancel；后续优先设计良性探索意图失效与已验证运动段安全交接，覆盖后台规划/旧动作终态/新query及对象接管，不能直接去掉取消或继续旧队列。两轮轨迹不同，不推断稳定性和成功率改善；完整搜索仍未通过，不计20轮、不提交/不替换日常e6入口。
- 新真实失败RViz已取回并打开：captures/20261016_frontier_resolution_retest/rviz_live.mp4，423.625s/3389帧/8fps全解码，SHAc15ee1497fc4b21d89b80cead061b85a1414d20158fe9d5b8a9293cf9cb826ae，右侧camera、无独立前摄MP4。709实际评分、mask/定位/确认/召回0、三类存储实际None，不把零mask归为已证模型误识，也不宣称记忆验收。110成员ZIP SHAb76698eb3ce04edc370132345412f972928c4027beaa62a40f9c4b07d53486c5，本机CRC/109载荷逐SHA/视频一致通过，test_artifacts/frontier_resolution_repair_20261016。3.295GB原始sensor DB仅服务器保留并附manifest。测试后源码/库身份复核、独立ps相关残留0、session errors=[]；候选274保留，生产e6不变，仍无commit。

### 2026-10-16 用户要求彻查：普通frontier退役被直接转成底盘停车，及新原因分类遗漏

- 详docs/CANCELLATION_CHAIN_AUDIT_20261016.md，证据test_artifacts/cancellation_chain_audit_20261016。此次只读封闭200923日志/包、原地不变的生产方法；未改生产、未新跑仿真/无新MP4、未commit。相关源码逐SHA同冻结274，日常入口仍e6。
- 21条FollowPath取消全部按同exploration_id/源退役/Bridge回报对上frontier_resolved；取消时Native速度median1.139m/s，取消到隔离解除median1.824s/max2.578s、到下一Following median6.282s/max17.047s。14段>5s静止中11段开始前3s即该链路；另9.711s从换向段终点开始但期间又两次取消，7.404s是段结束后候选/地图等待，6.006s是换向后新图footprint_collision重规划。主要为新增意图失效→reset/cancel→隔离停稳→再比较/规划，不能归为纯车体无法执行。这是上轮改法实际引入的副作用。
- 取消前已发布地图/车姿与接受目标匹配21旧路径，21全部前2m局部/全局检查通过；19剩余段全局检查通过，另外两例远端footprint_collision/collision_cell。仅读封闭包，使用原trim/safety函数；非插件私有地图/精确RPP弧真值，不保证全部实际可继续或取消障碍保护。odom源龄max.515s/global.908s/local.596s，范围写明。
- 静止109.852s按控制状态积分：stale_command81.360s/gear_change17.531s/execution_hold8.820s/tracking1.469s/ackermann_stop.672s。809静止Native样本794全制动、油门>.1为0；停车State接收龄max.129s/Native样本龄max.139s，Native遥测/执行器失败和watchdog为0。速度指令缺席符合取消后无活动控制目标，不能把stale_command解释为网络丢包；State源戳11.5s时钟域旧问题保持独立，不重写为旧速度延迟。
- 新确定遗漏：C++产生frontier_resolved，但Bridge.same_frame白名单未加入，30次匹配退役走完整reset，可清已驶走廊/规划失败历史，独立恢复也会被取消。原实际方法+ROS替身五组复现：新原因清空并取消独立恢复，旧vehicle_route_rejected保留，graph_frame_changed仍正确清理；这是机制/潜在回归证据，本轮实际无恢复，不能说现场已发生该恢复故障。
- 上次新增test_resolved_frontier_cancels_motion_without_success_or_obstacle_evidence明确assert取消，固化了不合适的连续性契约；447通过不能覆盖缺少的跨模块生命周期/物理历史验收。后续不能只补白名单或删除取消；区分frame/查询上下文、探索意图与动作段，安全核验后在有界运动段/有效替换处交接，继续保留障碍/过期/未知动作/显式停止保护和旧回调隔离。
- 该轮150计算请求/57Following；只读候选拒绝33含empty_path31/footprint1/pose_mismatch1，换向后footprint/collision_cell各1，纠正“0 follow_failed/no_vehicle_path=规划全正常”的推断。709视觉评分/query正确、三新增先验关闭、目标mask/定位/确认0，整轮没有已定位绿房屋接近链；零mask不单独定模型误判。当前机制含自定义探索和桥接，不能归为原版WildOS/视觉记忆原理失败。实施新交接方案及完整验收均未做，不宣称已修好。

### 2026-10-16 普通frontier退役的有界运动段交接修复候选

- 用户要求直接修改，按封闭取消链反例实施，详docs/FRONTIER_MOTION_HANDOFF_20261016.md；test_artifacts/frontier_motion_handoff_20261016保存before/diffs/反例与验证。frontier_resolved补齐同frame历史及独立恢复语义；归属明确、双图/odom/Native新鲜、安全且剩余<=现有8m续接范围的已接受段可完成，不扩展/不执行旧尾段。原epoch/动作serial保持、旧planner serial独立失效；实际终态/cusp检查通过后重新经on_route检查新意图，不给退役frontier到达ACK。其它不安全/不确定情况仍取消隔离。
- query/frame/停车、数据过期、Native失败、动作失败和旧回调保护保留。新增17回调使总数464；旧方法50项中6FAIL/3ERROR反例保留，最终全局50PASS/6WARN/0FAIL及464替身通过；真实ROS68交接/比较+25所有权、实际C++/DDS退役3通过。构建入口PATH和原有测试独立ROS init错误保留并修正诊断入口，未改生产保护。
- 冻结439成员12023042037205a79a57f0e76bb1d8544e94309f8ff9b80a3fe28f554f521f2d，相对274仅bridge/continuous_path/execution_feedback和路由测试四文件。独立服务器安装三个模块逐字节一致；GraphNav db52f00e不变，日常e6入口未替换。真实Town/SBR/green residential house同条件关闭三个新增记忆先验的420秒完整基线启动中；不是原版WildOS，现场效果/完整到达尚待封闭，不commit、不计20轮成功。

- 205443本轮已封闭：421.337秒超时，Native173.759m/max离站49.256m，42following/21机动；7条有界交接=4正常完成、2预瞄反向保护失败、1最终停车取消。7次普通follow_path取消均匹配frontier_resolved；10段>=5秒静止共96.513秒/max15.806秒。与旧轮21次取消轨迹不同，不推导稳定性/成功率改善，不能因4次交接成功称完整修好。
- 8次执行失败为5单向lookahead反向保护+3有界指令弧碰撞；前3次反向保护不处于本次新增退役段交接，后2次处于交接。既有规划准入预测与现场RPP执行仍不一致，几何/延迟根因待精确对齐，不关闭方向/碰撞保护或放宽到达。另2次no_vehicle_path均7候选empty_path；1次换向后footprint_collision重规划，1次测得独立倒车恢复完成。985路线/734完整，无>5秒发布断档；1032评分，mask/定位/确认0。完整搜索失败，不计20轮成功、不commit，日常e6入口保持。
- session errors=[]，三层关闭屏障通过；此前独立ps无相关残留。额外SSH复核自动审批超时未执行，使用已取回封闭日志核验，不将超时当作安全判定。captures/20261016_frontier_motion_handoff/rviz_live.mp4真实双栏425.375秒3403帧全解码，SHA81a14298f6c1c55c920736f783ac414559e8669662d68ac718a64edc88616bef；无独立front_camera.mp4。104成员coreZIP SHA b358c891e1db78eb45297e67add66080abbba70f2cd85b20ed6d39dcd524bd29，103有效载荷逐员SHA/CRC与本机视频一致通过；3.233GB传感器原库仅服务器保存，外部manifest已取回。summary/timeline/handoff_outcomes与详报保存；后续须优先检查预瞄终点/准入-执行契约，不把该残留误判为全部由frontier取消或BeamNG物理造成。

### 2026-10-17 用户要求纯离线彻查：源控制新鲜度与动态刹停契约缺口

- 详docs/OFFLINE_SYSTEM_AUDIT_20261017.md；test_artifacts/offline_execution_audit_20261017保存分析、4个诊断反例、source_snapshot、日志与机器摘要。本轮没有SSH/新现场/新MP4/生产实现改动；439成员逐SHA仍同12023042，已关闭205443核心归档SHAb358c891再次核验。此前16日准备的QoS比较被自动审批额度阻断未执行，不把其当成QoS修复或实验结果。
- 目标窗口8479速度环/8461Native样本，54采样点上游最近.1秒已发布throttle0/brake>.2，下游sent仍正油门/零制动。五次预瞄反向中止，从速度环最后持续制动开始到Native首次非零制动分别.678/.628/.585/.382/.430秒。前三窗精确匹配throttle/brake/steering/gear，12条旧源需求至少.274–.436秒才下发，此时Native报告接收后command_age<.05；重复payload取最晚源样本只给保守年龄下界，不冒充callback入口精确追踪。
- 全窗Native command_age median.0262/P95.1711/max.4821，RPC median5.388/P9516.792/max134.457毫秒，周期median50ms/P9551.5ms，遥测龄median.0555/P95.0672/max.1851秒；不能把全部延迟归为SDK慢RPC或未知地形。确定位于源发布后→Native消费/下发前及源龄契约，具体DDS可靠传输/回调调度/负载占比未闭环，不能宣称改BEST_EFFORT必解决。
- Native.on_control只接无源时间/序号的VehicleControl，command_at用回调当前monotonic；晚到旧消息会重新变年轻，.8秒缓存重发与租约心跳继续有效。Bridge.execution_transport_issue只核新鲜遥测/执行器/ACK，未将ACK关联当前源需求；RPC成功不证明执行的是最新需求。当前生产方法4项反例均复现：晚到旧消息被当新、.7秒旧油门仍可驱动、.44秒旧需求遥测fresh时健康门仍通过、5失败路径几何preview全通过。通过表示缺陷检出，不是修复通过；没有证明现场乱序。
- tracking_preview无速度/延迟/制动力，仅空间步长积分。5条失败路径当前预检均可达；4条记录到0.2m/.35rad内odom姿态而邻近Native速度约1.3–1.9m/s，另一条采样跨越未记录到，**更正先前5条均记录进终点的说法**；odom有延迟，不当同刻真值停稳。1.2m/s*.44s=.528m已超过.2m捕获直径，尚无制动距离。方向/碰撞/严格到达保护保留，不能通过删保护或仅降全局车速修补。
- 另复核目标35持有161.591秒、78.419m行驶、最小odom距12.991m，259图路仅24完整；中/末两份匹配拓扑中UUID缺失。持有分支对missing既不resolved也无持续有效期/整体进展出口，这是与“同UUID明确非frontier”不同的剩余逻辑缺口；两快照不证明全程连续缺失，不凭单帧消失立即取消。三个新增先验关闭仍失败，不能归为记忆理论失败；无mask/定位/确认仍未进入绿色房屋接近，模型/树林私有地图原因无新闭环。
- 新鲜度/目标有效性/动力学验收是此前测试设计遗漏。464原回调替身复查通过；4诊断反例通过仅表明原缺陷仍在。全局首轮45PASS/5FAIL/6WARN含默认沙箱Temp权限；授权工作区内tmp重跑后console28/包5/经验存储5/图观测11通过，最终49PASS/1FAIL/6WARN。剩余DDS8项中4因profile严格resolve的WinError5阻断，日志保留；没有删安全校验使测试变绿，不称全通过、不commit。
- 下一步优先控制需求源身份/源龄/截止与回执关联，再受控定位DDS/调度延迟，再以真实制动和时间约束验收cusp；随后补持续缺失/整体无进展的有界重关联退役。确认共同基线后再启记忆增量；不刷完整420秒替代组件根因闭环，不扩大原版WildOS/BeamNG责任结论。

### 2026-10-17 用户授权开始修改：源控制与回执成套离线候选

- 详 `docs/SOURCE_CONTROL_REPAIR_20261017.md`；证据 `test_artifacts/source_control_repair_20261017`。本轮纯离线，无 SSH/部署/现场/新 MP4/commit，不计20轮成功。新候选397成员 `a69ea0560b3008f53dbf21d3ac9b66758d261a36a1659517343b4f347a6d2f30`，`deploy/20261017_source_control_candidate`，本机通用包同步；基线120及旧包保留。服务器日常入口本轮未触碰，最后现场记录e6，不称服务器已更新。
- 速度环到 Native 改为独立 String `/wildos/native_control_command`、`wildos_source_control_v1`：源 unix_ns 生成/截止、随机 sender、generation/sequence、完整控制量，.5秒寿命从速度环输出起算，monotonic只转换剩余寿命；过期锁存，回拨不复活旧油门。无源/未来/过期/非法/重复乱序/旧代及非法跨进程消息不续命；启动/切代/失效恢复要求新制动屏障，跨进程接管另需旧流失效及新鲜停稳，旧身份有界围栏。历史各64项、包4096字符；不改QoS/线程/速度/真实物理/半径/到达/碰撞/方向保护。
- Native command_age含已知源龄，新增接收龄；RPC snapshot/实际控制/源身份配对，晚ACK不成有效源回执。`source_receipt.applied`仅匹配且未过期的SDK输入ACK，`applied_wall_ns`为ACK返回时刻，不是物理作用时刻。速度环核自己已发历史/当前代/载荷/截止；enable/disable/有效hold/release清旧速度换代即发制动。Bridge实际行驶要求源ACK与producer身份；停车隔离仍以全部旧动作终态、新鲜遥测/制动ACK/低速持续为据，发送端死掉不阻断确认停稳，但新运动仍需源ACK。已保留初候选停车隔离死锁、时钟回拨复活反例后修复，不当旧现场原因。
- physical launch/runtime contract/preflight唯一发送者与接收者/消息类型、旧native producer拦截及录包话题成套更新；legacy /control 保留但禁接 Native。归档原先继承46个过时build/egg-info生成成员，现排除且有旧库存回归，旧源和ZIP不删除。相对120改20既有载荷、加入归档4个（含原有探针），142个GraphNav/视觉/Smac/RPP/控制租约/速度数学/真实车体配置逐字节不变。CRC/逐员SHA/当前源与冻结候选全部通过，before/after/diffs与机器summary保存。
- 17协议测试、477实际回调+ROS接口替身（原464＋新13）、14探针自身回归通过；总检50PASS/1FAIL/6WARN，剩DDS8项中4项strict resolve被Windows沙箱WinError5阻断、4项通过，未删路径/hash保护。早期夹具命名空间、同刻dt、输入计时和采集编码拼写诊断错误已修并留日志。真实ROS launch展开、序列化/时序、SDK与物理执行尚未验证，不称全通过、不commit。
- 边界：源起点只到速度环输出，原RPP→速度环Twist仍无源header；.5秒窗内旧需求仍可执行，尚未减少/定位DDS与调度延迟。RPC跨截止只能令ACK无效，不能撤销已入引擎请求，不称硬实时停车。动态刹停/几何preview差距、frontier持续缺失及整体无进展尚未改；本轮保持C++生命周期不变，下一步先ROS组件与故障交接，再测动态控制，随后再处理探索有效性。完整绿房屋/记忆增量/20轮与多目标仍未通过。

### 2026-10-17 源控制服务器复测与6:36后视觉追踪交接反例

- SSH 专用密钥连接成功，首轮/复测前均检查无仿真导航残留，GPU0 空闲，未碰 GPU1 他人任务。独立 `/home/ly/source_control_repair_20261017` 候选最终398成员2e69583ced4ba5be3445d6c1c565e28812441ff2f76f5e7a410117af0fcc82d0；日常 e6 原包未替换，实际 GraphNav db52f00e、Smac f915a4fc、RPP bd96eb9d 核验。真实 Linux 54 协议/DDS/runtime/endpoint、54 源控制/Native回调、68 交接/代价回调与launch展开通过。早期测试夹具Time/重复rclpy初始化/错误Python缺SDK均保留并修正；不为通过测试放宽执行器保护。
- 首启151218无目标预检失败：新endpoint检查错误要求总订阅者=1，把合法只读observer视作歧义。改为唯一指定类型/命名空间生产者和唯一Native执行器，允许旁路订阅者；新增4项回归，反例/旧predicate保留，未白名单场景进程。随后真实制动保持的enable/disable源身份/代际ACK组件通过，零速度/零油门。
- session151623单次Town/真实SBR/green residential house，420.928秒timeout，Native142.286米/max离站48.881米，9段>=5秒静止合76.864秒/最长18.753秒；52following/28maneuver_done/无Bridge follow_failed或no_vehicle_path，但不能称整体规划正常。mask32、visible提示42、定位/确认0。三新增记忆先验关闭，非上游未适配原版/非记忆增量验收；成功仍0，不commit。
- 补充完整旁路8500 Native状态（原source_control_review只取3061抽样）：8407回执逐一匹配9303已观察源帧，错误匹配/无有效源ACK却确认正油门0；源龄median.05173/P95.06925/max.80652秒，回调时源龄median.000892/P95.01037/max.41246秒，RPC P95=16.43/max116.88ms，watchdog80消息、执行器故障/遥测不健康0。旁路仍可能丢消息，ACK不等于物理作用。4个execution_unavailable含source_expired/缺receipt，不据此诊断混版。
- 服务器最终持续停稳/errors=[]，独立/proc确认无相关残留。真实失败双栏captures/20261017_source_control_retest/rviz_live.mp4已回传，426.875秒/3415帧/8fps全解码，SHA8dc42d2af3f6add96ac8d4829f5bf4c89814bbcae9fa07d52e3237bccebe4f0d。核心ZIP 155成员SHA92c2e7e1135d16ca68b7695c370c930cd1a2bed2821039341593e95349f095c5，154载荷SHA/CRC/视频一致；3.39GB原始sensor DB仅服务器保存，外部manifest有路径/长度/SHA。
- 用户指出6:36房屋掩码，封闭路径/odom/控制重建：约376.19秒首次bearing-only靠近，12.017米候选拆三段；386.52–397.13秒9.450米主靠近段实际107份控制器变换路径/lookahead，odom偏差max.088米，正常执行到了临时点，不等于房屋到达。397.131秒完成时最近掩码采集393.957秒、年龄3.174秒；旧finish_visible_approach立即清expected/reset GraphNav。后续403.671/406.261/407.674等不连续掩码被三帧重启门槛抑制，424.340秒才再靠近。**2秒超时影响续接，不是中断正在执行路径。**
- 精确配对掩码/LiDAR/TF几何诊断32次：24次掩码内0点，余8次各1–5点，低于稀疏8点；未触发表面竞争分支。31组源戳相同/1组差98.5ms，无同步拒绝。多视角11次独立视点，常因<1m或5秒窗内视差不足而失败；唯一可交会一次协方差半径6.276m>3m拒绝，不能把其30–32m深度粗估当真值。40m量程/16线60°/30000点下采样是当前配置，点稀疏的范围/遮挡/采样/外参贡献尚未全部闭环，不只凭配置断言量程根因。
- 详docs/VISIBLE_PURSUIT_REPAIR_20261017.md。已按用户“疑似目标应靠近确定”修3实现＋测试：完成临时点后保留追踪所有权，最多停10秒重观测，旧GraphNav路线不抢占/不重跑已完成点；新鲜有效提示回来走原整套安全规划链，超时有界退避。已由3帧建立的方向候选允许15秒内由方向一致的新鲜帧重关联，初次仍3帧、单帧2秒/几何/到达标准不变。反例16项先3失败，修后20针对性和487全套回调替身通过；真实ROS/现场尚未测试、未部署、不commit。只有方位关联不声称可靠跨遮挡实例识别。
- 左转90°测试harness已准备并4项离线验证；保持Town位置/物理/相机，四元数按引擎被动旋转约定。尚未部署/启动，当前绑定2e，重测修复候选前须重绑冻结。用户后续连续追问定位和靠近机制，当前优先收敛上述交接修复与验证，不声称已完成左转现场测试。

### 2026-10-17 用户授权追踪交接修复现场验收（进行中）

- 新冻结398成员 `f9d1caa748373c8bd9106cde7f44105604d069a3b667b9dd3279b47b4d5defec`，`deploy/20261017_visible_pursuit_candidate`。相对2e仅visible_approach.py、graph_path_bridge.py、goal_standoff.py和test_visible_approach_ros.py四文件不同，逐员SHA/CRC与当前源通过。独立部署 `/home/ly/visible_pursuit_repair_20261017`，保留旧2e/e6；重建nav2_adapter与official_adapter。
- 现场前无相关残留，GPU0空闲/约45.7GiB磁盘；Linux54协议等检查、74真实ROS回调（含20项visible交接）、68交接/代价、launch展开及源门禁通过。真实消息回调仍不是物理仿真到达验收。
- driver `field_yaw_left90_search_20261017_155026`、session `session_20261017_155029` 正在单目标搜索。Town同位置左转90°，真实723节点/467车身碰撞/四轮各32碰撞、一次地图加载；制动保持的源生命周期探针/Native朝向门禁/预检通过，开始 green residential house。三个新增记忆先验仍关闭以验收共同接近链，不称记忆增量实验。
- 实际/proc/maps核GraphNav db52f00e、Smac f915a4fc、RPP bd96eb9d，参数来自本轮install；新Python安装源逐员一致。运行中不改导航源码/配置、不读活动SQLite。测试关闭后再核动态路径、视觉定位与真实物理到达，取回本轮双栏RViz视频、核停车/进程清理，补封闭结果；目前未宣称成功、不计20轮、不commit。

### 2026-10-17 左转90°追踪交接现场155029封闭：完整失败，未触发修复分支

- 详docs/VISIBLE_PURSUIT_FIELD_20261017.md，f9d1caa7运行中未改源码/条件。421.123秒timeout，Native108.697m/max离站13.181m，51following/45maneuver_done、57等costmap、8段>=5秒静止/最长20.593秒、2follow_failed，mask/visible hint/定位/确认/visible_target_reobserving均0。1440评分，三新增先验仍关闭；新交接修复部署与真实ROS组件通过，**现场功能未触发、完整搜索未通过**，不计20轮、不commit。
- 两次controller实报bounded command arc collision，query相对234.045秒与395.648秒（不是上一轮6:36掩码事件），随后FollowPath status6；不能直接归为真实车身碰撞或认定私有地图正确。RViz抽帧见树边/停车区，需对局部地图与跟踪实际弧进一步核验。
- 最长交易9指向(727.68336,-30.04983)，289.154秒/816完整图路、Native84.863m而最小odom距8.133m，未到1m观察范围；三份严格匹配raw/scored快照前中UUID存在/frontier=true，末缺失且邻近3节点非frontier。未证明连续缺失时长或单一根因；整体无进展/持续缺失有界退出仍是既有未解决项，不误诊visits0为已观察未退役。
- 完整旁路8457 Native回执全部匹配源身份/载荷/截止，错误/无有效ACK确认正油门/watchdog/执行器故障/遥测不健康均0；源龄P95 .04936/max .19657秒，回调源龄P95 .00798/max .03640，RPC P95 16.53/max161.35ms。此轮未复现速度环→Native旧源问题，不能外推上游RPP→速度环或把ACK当物理作用。
- 真实Native出生朝向-99.936°；一次地图加载/原真实车体。goal_001/stop.json持续停车末速度.000694m/s、session errors=[]，独立/proc无仿真/导航/录制残留。根目录stop.json是目标前预检，不能当结束位置；session video_recording_enabled=false只指console内置录制，本轮外部driver实际录像已校验。一次SSH读取因自动审批额度服务失败未执行，用户继续后经原通道恢复；未绕审批，driver自行按预算正常关闭。
- 本机失败视频captures/20261017_visible_pursuit_yaw90/rviz_live.mp4：423.5秒/3388帧/8fps全解码，116843072字节，SHA596d129a1f0cc664611cd29889a01e7aa0f4065d0684799427d0bfd24f446b7e，与服务器/归档一致。右栏实时camera，无独立前摄MP4。核心ZIP 176477499字节/121成员，SHA37b76b5265da579781808a5396afda5a1200e60999366ffbdd8de5990b157d9e，120载荷CRC/SHA通过；原始2.69GBsensor DB仍服务器，外部manifest有长度/路径/SHA。补充图快照ZIP SHA391d6eb8f557aa9f3f79d0f811621265b65622024e0318a15b40515e6ac9cbfa，7载荷通过，不覆盖原封存核心包。

### 2026-10-17 用户指出大部分停驶：执行循环反例与恢复次数候选

- 详 `docs/EXECUTION_PROGRESS_REPAIR_20261017.md`；证据 `test_artifacts/execution_progress_repair_20261017/`。重算封闭155029全部8420 Native样本：<.05m/s 214.337秒、<.2m/s 277.365秒。更正前述8段仅>=5秒连续长停，漏了频繁短停；不能用109米累计行驶或单动作成功说整体规划正常。
- 51实际下发段全部匹配完整候选，median1.047m/33段<=2m。10次换向后重规划中8次最近公开地图复现计划时可走→交接碰撞，队列年龄7–34s，部分中心冲突在前8–22m；2次实际姿态接入曲率/预瞄拒绝。公开地图不是私有缓存逐字节真值，地图变化是否真实/误障碍仍未闭环，不关闭保护。整轮三个新增记忆先验关闭，不归为记忆原理失败。
- 原方法同目标往返/完成prefix反复清零恢复次数已复现，9项初始新回归6失败；同时发现同交易图路/末点变化能清零、新交易同坐标继承旧耗尽次数。仅生产graph_path_bridge.py统一query generation/frame/exploration_id与整体goal归属；新鲜独立到达核验后的实际距离比已记账最近值再改善1m才恢复，累计短段可记、重复走同一段不重记；同交易换图/地图不续命，耗尽精确拒绝探索，新交易清旧block。显式物体/记忆目标的拒绝保护与动作隔离等保持。真实走廊escape独立成功恢复保持原行为。欧氏进展不能精确衡量长绕行代价，不声称单此修正稳定行驶。
- 最终500项回调替身通过；服务器无残留后独立目录/ROS域174，18项真实ROS消息回调通过，57载荷逐SHA核对，组件ZIP SHA dca3284ed871b8e0e2cad34d69fb788c3b3e25216390bb3bf1813a062743d9f5。初夹具时间域/重复收集修正、一次断言位置诊断修正均保留日志；未放宽生产保护。SSH首只读引号/服务器Python3.10无file_digest诊断失败已修，未触碰仿真。
- f9d1caa7整车部署保持，本机/服务器隔离组件候选未做整车验收、不commit、不计20轮；没有新MP4。多换向路线代价、长队列更新、地图变化来源、整体无进展/持续缺失有界退役仍待解决。此前“规划基本正常”判断不成立，不能再用组件通过替代完整搜索。旧视频仍为155029失败证据。

- 审查补充封存 `test_artifacts/execution_progress_repair_20261017/execution_review_supplement.zip`：32有效载荷，196133字节，SHA2a60b00755b72dce70c8f2c8b342e3ecb9cb7ce945a742ffdd605ea79e78eda7，CRC/逐员SHA通过；原图/队列分析与修复前反例一并保存，不覆盖原核心包。before生产源码与冻结f9d ZIP逐字节相同，本次生产改动仅GraphPathBridge一个文件，after快照/逐行diff保留。

### 2026-10-17 继续排查短段：完整路线选择候选

- 详 `docs/COMPLETE_ROUTE_SELECTION_20261017.md`；证据 `test_artifacts/connector_audit_20261017/`。本轮未新启仿真、未部署现场、无新MP4，f9d保持。沿上轮51段中33段<=2m反例审查，不以累计里程或动作成功代替进展。
- 查明自定义Smac连续起点包装层按起步连接代价排序、拼出第一条可行完整路径即返回，未比较完整后缀绕行/换向。19份封闭公开地图/路径端点控制对照两版均16生成/3失败(0/5/7)，最终候选9例成本下降、6例减少换向；case11旧12.17m/4换向/24.1725成本，新11.24m/2换向/17.244948。重建公开图不等于私有地图和精确请求；3失败不是现场当时失败的证明。简单朝向剪枝仅2/19改善首选连接，未采纳。
- 本机仅新增此轮6文件改动：planner/path_contract实现与头、两个测试、一个封闭图fixture。既有候选/时间预算内比较完整长度+3m每换向成本，最后重验当前地图，最便宜已受阻则验下一条；首次搜索原余量保持，额外搜索预留.05s。起点/曲率/polygon碰撞/raw回退/帧/时限不放松，不改物理、速度、模型、目标和Bridge动作权限。增加耗时和有限候选最优性仍是限制，跨候选成本不等于最优障碍净距。
- 实际旧planner反例按成本24.1725失败；候选真实C++21项(4 connector+10 planner+7 contract)通过，新反例额外3次通过。colcon总24含3CTest包装，不重复计。最终源码19案回放范围与原型一致，500本机回调替身通过。最终库SHA9665ced92f9baf172a4da7384a396329391cd62390c84b85b8b8ea7dc7285826，独立regression_install_candidate，逐源归档一致；不是整车验收。
- 自动审批服务额度曾阻断一次重试，用户继续后同原通道恢复。隔离入口PATH/CRLF/同名包扫描错误均保留并修正；未修改生产保护。规划地图更新来源、整条尾队列否决和长期无进展退出仍未闭环；与上轮GraphPathBridge执行进展候选均待冻结现场验收，不commit、不计20轮。
- 补充证据ZIP `complete_route_review.zip`：254载荷，901296字节，SHA72c8e9b813553241b41f6d743574ea5beca171c667074b99273b9b5b76d7d211，本机CRC及逐载荷SHA通过。旧封存包未修改。

### 2026-10-17 同目标重试准入的三个缺口补修

- 详docs/RETRY_ADMISSION_REPAIR_20261017.md；证据test_artifacts/retry_admission_repair_20261017。沿已有执行进展修复继续检查，复现失败后changed_route不计重试、同探索ID部分末端移出4m不走阻塞准入、定时器耗尽只返回不请求退出三个缺口；不是每段现场停驶的已证唯一根因。
- 本轮生产仅GraphPathBridge修改，相关进展测试新增8项。同ID换路线/部分末端移动仍消耗重试，新ID仍清旧预算；重复受阻/活动消息不计，transport unavailable恢复与定时器一致不计执行失败。耗尽定时器在退避结束、输入/执行器/服务就绪、无活动/待接受/隔离动作时发原精确探索拒绝，发出后状态转耗尽。显式/记忆/查询保护及实际进展记账保持，不新增障碍区域/到达ACK/强制定时退出。
- 修复前两个反例失败，后续部分末端反例也保留；最终508本机回调替身通过，服务器独立域176的26真实ROS消息回调通过。57载荷component.zip SHA f23d83dcc8d0ae9a1a2e9270a93f9ab751fb5267755a4d694b4e6d107f9a9e32，CRC及当前源/载荷逐SHA通过。首服务器旧消息overlay缺ExplorationChoice，改加载f9d冻结依赖后通过，无接口放宽。
- 现场f9d仍未替换，无新仿真/录像、不计成功、不commit；无失败回报的成功短段循环、frontier持续缺失、地图变障碍来源与长队列否决仍待解决。既有完整路线候选也尚待整车验收，不称停驶已解决。

### 2026-10-17 用户授权合并修复部署整车验收（进行中）

- 冻结400成员932ac9bf8cbe6444076f2ad702c560e0ab2eed781fdc749d7687998e75b38e44，deploy/20261017_full_vehicle_candidate；相对f9d仅10成员：完整路线C++及相关测试/fixture、Bridge执行进展与重试准入、两个回调测试和套件入口。独立/home/ly/full_vehicle_acceptance_20261017；旧部署保留。
- 预读/proc无相关残留，GPU0空闲、约43GiB磁盘，未触碰GPU1。部署新prefix实际C++21项和26真实ROS回调通过，候选及原e6逐员/基线开关/field harness预检通过。field_yaw_left90_search_20261017_172049/session_20261017_172053已启动，一次地图加载、Town原点左转90°、真实车体、目标green residential house，沿用三新增先验关闭共同链路基线。
- 运行中/proc/maps核GraphNav db52f00e/RPP bd96eb9d；重建Smac 3cd81492caf0df265fd9e635e528d1e876efa1e540e5fa94b9e05f09d2dfefd5来自本轮新install。重建路径不同故库hash不等于隔离回放旧构建，源码逐员冻结相同。现场启动早期库未加载时只读audit空集失败，加载后复核通过；非生产故障。
- 本机准备脚本一次默认GBK读UTF8失败，上传因harness尚未生成未启动；显式UTF8后继续。测试/源码门禁未放宽。当前仍运行，完整结果/停车/录像/录包封闭待核验，不计成功、不commit。

### 2026-10-17 合并修复整车172053封闭：未通过

- 详docs/FULL_VEHICLE_ACCEPTANCE_20261017.md，证据test_artifacts/full_vehicle_acceptance_20261017。932ac9bf冻结运行期间导航源/配置未改，423.841s timeout；Native记录175.699m/max离站33.864m。650完整路线决策且无>5s发布间隔、670评分、42following/28maneuver_done/3换向后重规划；mask2/visible hint/定位/确认0，三新增先验关闭。不能称完整搜索/视觉记忆或稳定性通过，不计20轮、不commit。
- 所有旁路目标窗口8474 Native样本积分：<.05m/s 194.997s/<.2 241.648s，10段>=5s最长23.799s。末公开状态近似following79.17s/map等待43.43s/退役17.71s，不当内部精确阻塞归因。与旧轮轨迹不同，不用静止和里程差推导成功率改善。
- 356.49s一次RPP bounded command arc collision拒绝→FollowPath status6，359.95s一次同目标empty_path后续探索。296.15–298.93s execution_unavailable4：source_protocol/source_expired各2。8468 applied回执均匹配有效源、错误/无有效回执正油门0；watchdog58/执行器故障与遥测异常0，源龄max.846s/回调max.452s。具体延迟和私有图碰撞真值未闭环，不归为全是物理/网络问题。
- frontier_resolved28/vehicle_cost_selection5/route_rejected1，观察ACK0。退役/候选/交接频繁仍须审查真实覆盖与运动连续性；不以完整路线或单动作完成宣称规划已好。goal停稳末速度.001026m/s、session errors=[]、独立/proc相关残留0；日常e6保留，候选独立目录保留。
- 本轮失败视频已回传并打开：captures/20261017_full_vehicle_acceptance/rviz_live.mp4，427.75s/3422帧/8fps/1920x1080全解码，133474920字节，SHA88ce3ba9b941413d82ed94fc30efa0dc154484ce0cb6fdba08ebf8524cbb7cca。右栏camera，无独立前摄MP4。核心ZIP195421240字节/128成员，SHAbcd9ed33a77f3c23e969f0f0b5f63d0303c46959e7a8fcd0d5832a3fe4998afd，本机CRC/127载荷SHA/视频一致。3.246GB原始sensor DB服务器保留SHA3752c0ecba23da71bd506b2d5552ac9466f14428e9acf8b1ae3cd9159bcafcb4，外部manifest齐全。所有SQLite分析在三关闭屏障后执行。

### 2026-10-17 用户要求一次系统彻查：统一契约审计，未继续改生产

- 详docs/SYSTEM_CONTRACT_AUDIT_20261017.md；证据test_artifacts/system_contract_audit_20261017。冻结932ac9bf全部400载荷仍逐SHA同当前源，14关键文件源快照。未新仿真/新录像/生产改动；只读封闭172053记录，地图ROS反序列化在服务器只读关闭DB。不能承诺穷尽一切运行时缺陷，报告区分实证/源码缺口/待证风险/验收空缺。
- 新实证解释194.997s静止：3900静止样本3848与applied源身份精确匹配，source reason stale_command96.541s/gear_change62.106s/execution_hold10.922s/tracking14.327s/ackermann_stop5.027s/disabled3.450s/未匹配2.624s。下发brake>.1占180.670s，正油门>.1且brake<=.01仅.495s，主要是系统主动停车，不支持“主要一直给油受困”。SDK输入/ACK不当物理力真值。
- following静止中62.106s为换挡，对应旁路最新Twist接收龄max.100s，没有>.5s收流间断；不能把79s following静止全部归为没发指令。Bridge取消日志10条。控制图/探索意图/运动段/速度环生命周期仍未完成连续性闭环，16日退役直接停车副作用的旧结论保持，不推卸为仿真随机问题。
- 上轮目标窗口118个ComputePath请求：110匹配结果(66成功/44失败)，8未匹配明确保留；44 candidate empty_path多数被比较/回退消化，顶层no_vehicle_path=1不表示规划只失败一次。往返median.217s/P951.779s/max1.819s；不把总往返当CPU时间。
- 关闭公开图：global发布间隔P951.741s/max2.482s，Bridge .75s新图门槛；local约.6s稳定。34等待仅16在旁路也> .75s；global生成到录包龄P95 .0027s，不支持单归网络延迟。Bridge回调/cache、私有地图锁/计算/负载占比仍未闭环，不冒充已证锁根因。
- 两个实际回调+ROS替身诊断复现：无header Twist晚到按新接收时刻计时；同目标局部success往返20循环无整体退出。通过表示缺口复现，不是修复。4证据校验通过，确认400源不变、三种停车分区守恒、规划失败计数一致、地图范围。生产未修改/无新整车测试，不commit、不称整体修好。
- 统一10类：连续性/退役取消/换挡成本/整体无进展与缺失退出/前半控制源龄/动态刹停/地图来源与时序/目标接近/记忆增量未验收/测试诊断盲区。修复依赖：先事务连续性交接，再联动源龄与真实车辆动态，整体目标有效性、地图和视觉链，最后记忆增量；不再只以组件数或刷420秒代替跨模块反例门禁。已修坐标/query/障碍接入/旧回调等无新反例不推翻。
- 补充审计ZIP 30载荷/133456字节，SHA1bb5fed25aaa42a0dc9290c785b296cb60570b23f734a313d93ad770197b30c0，本机CRC/逐载荷SHA通过。旧封存包未覆盖。

### 2026-10-18 全流程审计后继续交接修复与服务器验收

- 用户先要求全流程只读排查，随后授权以全局连通性和回归控制为前提修复，再要求连接服务器测试。详docs/WHOLE_FLOW_AUDIT_20261017.md，证据test_artifacts/whole_flow_audit_20261017。497源/配置身份清单、10个诊断反例复现；不是497文件逐行证明。服务器932现场400成员、日常e6的439成员和旧隔离组件58成员均一致，无相关残留。
- 审计扩展23项问题/限制，区分现场/源码反例/待证。新增确证：ExecutionExperienceStore当前成功-only生产链以全1初值+成功奖励1返回恒等先验；GraphObservationStore方向共用最大负证据仍存在；时钟重启adapter接受而速度环旧源戳单调门拒绝、延迟大回退当重启均为条件反例，未当上轮单次加载实证。语义/terrain评分未完整覆盖Nav2车辆绕行路径、未知策略、地图时序、整体无进展/持续缺失退出、动态刹停与控制器前半源身份等仍未闭环。不能把23项全部称独立已证现场bug。
- 初版未部署交接候选40实际ROS测试通过仍漏两处：旧A动作失败会阻塞/条件拒绝后台B意图；重复局部扫掠从已驶旧起点检查，漏车前新障碍或滚动图出界误停。10诊断保留为修复前证据；本次新增6回归原版全部失败，修复后528回调替身通过。现在每次裁剪实际位置前方路径；退役动作失败走取消/持续停稳隔离，不记到新意图；已收到终态仍须测量停车后释放。保留障碍、曲率、物理和速度保护。
- 401成员冻结94c468a7158a22559d4ead4fad47afaeb390a45863d7580ea8b44509f8e3f34d，deploy/20261018_motion_handoff_candidate，服务器独立/home/ly/motion_handoff_field_20261018；相对932仅continuous_path/Bridge及3测试入口文件。日常e6不替换。服务器仅重建nav2_adapter，继承932 Smac3cd81492/GraphNav db52f00e/RPP bd96eb9d；实际安装46真实ROS消息回调通过，401候选/439日常/基线/harness逐员门禁通过，非真实动作服务器/BeamNG验收。
- 本机全局检查首轮沙箱TEMP无法SQLite、严格路径解析受限及旧通用bundle不一致，保留失败日志；用工作区TEMP、宿主正常权限复跑和重新冻结通用包后全局通过，没有删断言/改生产保护。旧bundle已按hash保留。本次必要整车验收未完成，不commit、不计20轮成功。
- 服务器2026-10-18测试前无相关进程、磁盘约161GiB、GPU0空闲；GPU1有其他负载不触碰。下一轮同Town位置/左90°、真实车体/原碰撞、green residential house、420秒单目标；三个新增记忆先验仍关闭验证公共执行链，不能称完整记忆增量实验。现场关闭/日志封闭/失败录像取回和结果待补。

- session20261018_083258已封闭，详docs/WHOLE_FLOW_REPAIR_20261018.md：94c468a7冻结，421.055秒timeout，Native144.722m/max离站46.991m；48following/36maneuver_done/10换向重规划，685完整图路/无>5秒图路间断。速度<.05累计187.230秒，3743静止样本全部精确回执匹配：换挡77.731s/指令超时75.207s/hold9.338s等，实际下发制动174.051s、正油门不制动.508s。未通过、不计20轮、不commit；不能以里程/动作成功宣称整体可用。
- 新交接现场8次：2短段完成、6长前进租约全部约5秒到期停止、移动替换成功0。封闭原始录包查6租约内10条已发布合格候选，全部2–6方向段，8首前进/2首倒车；首前进7条.23–1.03m、1条8.33m。现有仅整条单向前进允许移动替换，余缓存待停，背景frontier又快速退役；长租约连续性目标**未实现**。不直接放开多段/延长超时：完整路径成本、首段动态可用性、实际速度/刹停和换挡约定必须联动，先受控物理预占/换向再整轮。366秒窗口14.14m/2段/首8.33m与7.95m/3段/首.58m共同发布，当前几何+3m/cusp比较偏好后者，动态价值缺失明确保留。
- 换向后拒绝collision_cell5/footprint_collision3/tracking_preview_would_reverse1/actual_cusp_pose_differs1，部分停车误差约.15m，地图否决和真实接入各自追查，私有地图障碍真值未闭环。1次execution_unavailable为source_owner_unconfirmed；8459旁路Native/8458匹配回执，错误匹配/无有效ACK正油门/watchdog/执行器坏/遥测坏均0，源龄P95.06583/max.18706s；不支持全归网络或BeamNG执行异常。
- 7mask/6bearinghint/1visible_target_approach、3D/confirmation0；7配对mask/LiDAR源戳差0。末段方位接近不等于到达。694评分、三个新增先验关闭/实际存储None；记忆增量未验收。goal后持续停车末速.000229807m/s、sessionerrors=[]、独立/proc相关残留0，原真实723节点/467车身/四轮32碰撞、1449.72kg保持。
- 本轮失败RViz已回传/打开：captures/20261018_motion_handoff/rviz_live.mp4，424.875秒/3399帧/8fps全解码，122141143B，SHAe855fa4642d0f0fff401578066f6ea936233c4bebf7bc25774bca5917e4ffda7；右栏camera，无独立前摄MP4。核心ZIP184819582B/125载荷，SHA71807ec5fe4fe11f9b91579abee1adc14cb3d166851f20e4b7931c09d0a377a5，本机CRC/逐员SHA全过；3.001GB原始sensorDB服务器保留SHA21a588d12762a24101b0e5c695eb626e22020c90b7b5b632ace124f7b609224b及外部manifest，不称本机全量下载。后续闭合几何/停车分析独立补充、不覆盖原包。未修复的全流程审计条目保持开放。
- 5项本轮证据一致性检验通过（401源冻结、精确静止归因守恒、未成功/多段交接范围、停车及原物理、录像归档身份），通过含义是证据自洽，不是导航成功。补充审查包review_supplement.zip 52载荷/121975B，SHAa74cb28dae214ba50c4bf454f088dce187aa0958714ecab423f6df879a087f12，CRC/逐员SHA验证。源和旧核心包未覆盖，未commit。

### 2026-10-18 控制器源时间戳修复及整车续测（进行中）

- 详 docs/CONTROLLER_SOURCE_CONTRACT_20261018.md。F15 无 header Twist 丢失生成时间，现 RPP 碰撞/曲率检查通过后发布 stamped body-frame 原 twist；物理速度环扣原500ms寿命并下传执行器 expires，重复/乱序/过期不能续期，enable/hold/release 建源戳 fence。Humble 插件外的终态 legacy 全零通道单独制动，非零不能驱动；Native v1/物理/碰撞/曲率/换挡保护不放宽。未解决全部动作 UUID 或动态多段交接。
- 旧实际插件新测试失败后，新 C++21、Linux实际ROS回调81、本机541回调通过；七阶段真实 FollowPath/RPP/DDS/速度环/合成Native receiver 通过，cancel至制动.097824s，27源/94帧/拒绝0。不是车体或搜索成功。DDS首次空plugins参数类型夹具失败保留，未改生产图。AST诊断常量解析修正和3工具回归纳入包后全局53PASS/0FAIL/6WARN。
- 403成员ca8baba0冻结到deploy/20261018_controller_source_candidate及服务器/home/ly/controller_source_field_20261018，安装59回调通过；session20261018_090814已启动，Town左90°真实SBR/原碰撞/green residential house/420秒，三新增先验关闭公共链路。预检通过，实际三库db52f00e/3cd81492/d2fd9cee，测试前无残留/148空闲、约158GiB，GPU1外部负载不碰。日常e6保留，运行中源配置不改。完整结果待封闭核验，不计20轮、不commit。
- 新补充离线反例证明source deadline检查到issue间调度暂停跨期会抛source_lifetime；尚未修复且未证明现场触发，保留deadline_race_before.log。需一致发行时间快照、过期独立制动与回归，不能延长旧需求。其它全流程审计问题继续保留。

### 2026-10-18 控制源时间戳现场090814封闭，竞态增量单独验证

- 详docs/CONTROLLER_SOURCE_CONTRACT_20261018.md。ca8baba0冻结403成员整车421.612s超时、行驶149.969m/max离站61.314m；726图路561完整/无>5s发布空窗、47following/29机动，mask1/hint定位确认0，三新增先验实际关闭。未通过、不计20轮、不commit，不能称完整WildOS或记忆增量验收。
- 8432旁路样本覆盖421.597s，<.05m/s累计206.093s；4121静止全部精确源回执匹配：stale105.328s/换挡56.502s/tracking24.137s/hold10.506s等。实际下发制动181.956s，正油门不制动9.972s，另两次5s路径无进展仅.0661/.077m，不能全归未下发驱动。公共最近状态关联地图等待/stale61.999s、following/换挡54.179s，并非私有阻塞的严格时间。
- 控制器生成至DDS接收P95.002583/max.007684s；2796正油门应用源龄P95.140545/max.197269s，无原始期限违规/错误frame/无源匹配正油门。105秒stale主要位于规划、地图、交接空窗，不是105秒网络延迟。一次RPP bounded arc碰撞/action6、换向重规划footprint2/cell4、source_owner_unconfirmed1仍保留，地图真值/动态无进展未闭环。
- 7借用段：1短段完成/1接入失败/3长租约到期，原诊断仅找后续Following误报2移动接替；补充控制器preemption+零/cancel+Native速度后实际仅1，166.51至167.62s最低速度.980m/s。另9.23s窗口已cancel/zero/停稳再启，不能算连续。原封包不改，更正verified_handoff_review.json及5证据一致性回归通过。
- 发送检查到issue跨期异常在Linux旧ca8真实安装复现；首增量0a851a22的82回调通过但实际DDS阻止全部tracking，原因是新保护错误跨域比较ROS steady和Python monotonic，原点实测差约10.8s。为本次改动引入的bug，未进车辆域，失败包/日志保留。最终8ebd782e两文件增量403成员分别在本时钟域检查、过期独立制动：本机15新/543回调、全局53PASS/0FAIL/6WARN，Linux83实际回调及DDS七阶段通过，26源90帧/拒绝0，cancel制动.011960s；仅组件，8ebd未整车。ca8现场运行中源码/配置始终未改。
- 停车末速.003439m/s，真实723节点/467车身/四轮32碰撞/1449.72kg保持，session errors=[]，独立相关进程0/display148释放。退出有SIGINT/KeyboardInterrupt堆栈，未当运行崩溃；本轮没有source_lifetime运行异常证据。
- 本轮失败RViz已取回打开captures/20261018_controller_source/rviz_live.mp4，426.625s/3413帧/8fps全解码，109689952B，SHA741a895fce7bf6fe2ccb7874c2333db926090a3e9a28e4801c77da27b60c0b65，右栏camera、未单独录front_camera.mp4。核心ZIP172698276B/132载荷SHA400d41e5b0c27e5ad16ae216dc4d0b872da7dfedb5c50a9ba07fd049cd8beb17，本机CRC/逐SHA全过；3.422GB原始sensorDB留服务器SHA6664bc6dbd802d83d81cbaac636501d4a80836bf3cccb4b2e75ad386400f5591及外部清单，不称已全量下载。
- 新增修正/验证未解决全部全流程审计条目。下一步8ebd车体边界门禁、地图等待/多段动态接替与进展，再视觉定位和记忆增量；日常e6未替换。

- 本轮补充修正证据另封controller_contract_supplement.zip，SHA847a1c42cfdbda639db5e939cf1eed4042541555a1c670bc7739cd572b7bffe9，83载荷，17335763字节；CRC/逐SHA通过，含中间失败候选与最终隔离验证、交接误计更正和5证据回归，不覆盖旧核心包。

### 2026-10-18 继续定位执行与地图等待（组件进行中）

- 详docs/EXECUTION_WAIT_AUDIT_20261018.md。仅只读封闭090814原包/独立观察器，8份有界地图/点云切片，不改旧包。300.40/413.76s两个失速5秒窗各100样本均tracking/D/发brake0；实际油门中位.1099/.3483、Native速度中位.0010/.0035、轮速.0017/.0053，确有驱动但无位移，不能全归未传指令或高速空转；接触力与具体物体尚未证实。录像临近长椅花坛，不能凭画面确定碰撞。
- 两次终点adapted/markings源戳同；按正式40m及现有滤波重放1646/1651点逐点完全一致。先误用12m诊断范围造成差异后更正，生产不改。公开图车体内无致命格不代表私有图/场景安全。独立observer TF/odom持续更新；原sensor recorder漏样看似数秒旧TF，不当运行停摆。2930路径位姿源龄P95.4304/max.5513s，16>.5s；没有复现14日持续数秒旧位姿。既有yaw/后轴修复不推翻。
- 更正先前62s地图等待关联：43个waiting标签跨度68.954s，但到首次ComputePath发送合计仅.759s，68.195s在已发送后错误沿用waiting标签。不能因此放宽.75s接收/3s源龄保护。139请求134不复用匹配/5未匹配，往返P951.789/max1.995s非CPU。
- 新增planning状态在分派后/注册回调前发布含serial与background，不改控制/规划策略。旧4回归3失败，新4/整体547回调、全局53PASS/0FAIL/6WARN；诊断曾导入TestCase重复43项计590已纠正。404成员d9d2e27b冻结，独立/home/ly/planning_phase_physical_20261018，安装实际65回调通过，原三库d2fd/3cd8/db52保持。未commit。
- 静止车体源租约启停通过，几何首次DDS入口遗漏无遥测、第二次诊断列表/x-y-z对象类型不匹配均无测量结果并停车退出；失败夹具/日志保留，未把它们当生产坐标失败。纯bbox计算用历史实测和非法维度两回归通过，第三次组件进行中，不下发目标/运动、不录视频、不计20轮。源正油门物理、多段交接、视觉及记忆完整验收仍未完成。

### 2026-10-18 控制源间歇失效复核、真实静止车体通过与整车续测

- 详 docs/EXECUTION_WAIT_AUDIT_20261018.md；新证据 test_artifacts/execution_wait_audit_20261018/startup_receipt。093826第三次组件实际因源回执门禁拒绝，未下发目标/运动；336 Native中210有效ACK/125watchdog（6未收首命令），330配对接收时源龄median .2493/P95 .3885/max .4543s，同源旁路P95 .00121s。原因仍开放，不能称始终无回执或已经修复；命令未初始化的monotonic uptime已剔除源龄统计。
- 6组隔离DDS（可靠深度1/10、best-effort1，空闲/200ms暂停）未复现持续晚到，不据此改QoS/放宽500ms期限。诊断Destroyable退出警告保留。当前生产源码与404成员d9d冻结未改。
- 真实SBR全制动151059/session151103的启动、源启停/代际ACK、5份bbox、停车/清理通过，session errors=[]/相关进程0；三只读旁路各716消息源龄P95约6ms，Native687接收源龄P95 .00665/max .02615s，无watchdog/缺回执/拒绝。一次组件通过未消除093826间歇故障，不计20轮、不commit。
- 实测后轴车体front3.282006/rear.908011/half_width1.000049m，均在当前3.4/1.05/1.05m footprint内，只核静止尺度/坐标相容；动态边界和近场接触未证明。归档56载荷1001655B SHA1007ff3287c4aa3d19cafcb598164db9157fa97693c448be9badc25b7afa2ed7，本机CRC/逐SHA通过。lifecycle无录像；driver继承video=true为元数据错误，未执行ffmpeg。
- 更正中间“090814失速没有轮胎接触快照”的假设：原包真实存在，review_contacts按采集戳去重及≤1.1s年龄，两失速窗4/3份、每份4轮，支撑力最小3149.5/2039.2N、驱动扭矩绝对值最小159.99/240.67Nm、轮胎制动扭矩0/轮速低，支持有支撑有驱动但无位移。不能据此确定具体碰撞物；clutchRatio低也不单独证明变速器故障，左右轮异号不能直接判驱动抵消。
- 同d9d/8ebd源期限修正开始首次更新后的真实运动整车验收：Town左90°/原物理/green residential house/420s，三个新增先验仍关闭，只读三订阅观察，运行中不改源/配置、不读活动SQLite。完整结果/失败录像与停车清理待封闭；不计完整成功或记忆增益、不commit，日常e6未替换。

### 2026-10-18 151920整车封闭失败：规划拒绝与Native独立接收延迟并存

- 详 docs/PLANNING_PHASE_FIELD_20261018.md，证据 test_artifacts/planning_phase_physical_20261018 及 execution_wait_audit_20261018/startup_receipt。d9d冻结/原Town左90°真实SBR/三新先验关闭，420.958s timeout，mask/定位/确认0，未通过、不计20轮、不commit；日常e6未替换，本轮无新生产修复。
- 更正分析窗口：控制台输入比goal实际started早5.4749s，原窗口/封包保留；最终用goal.started和observer received_monotonic_ns同域筛选，canonical_window_review为准，不把准备阶段当5秒传感器延迟。8417 Native覆盖420.891s、累计75.283m/max离站26.920m；稀疏goal日志73.941m另列。<.05累计294.463s：精确源stale_command221.883/换挡39.831/tracking10.141/未配对或无applied16.963s等；实际制动284.322s、正油门>.1无制动.378s。公共状态近似关联planning213.035s、等待地图2.902s，并非私有CPU/锁时长。
- 统一窗口174 ComputePath发送，当前结果153含status6=133/status4=20，20旧结果忽略；剩余不强配，不直接算总体成功率。session中no valid path/max iterations/No safe bounded continuous-start plan分别保留原文，包装层归并拒绝阶段仍需细分，不能全归碰撞。GraphNav968路线均complete/无>5s间隔但观察ACK0；图路complete不证明车辆可执行。
- Native状态加权接收时源龄P95 .2497/max .4645s，314 watchdog/缺receipt，rejection0；三旁路可靠1/可靠10/BE1 P95均约8ms。原补充窗口689晚到状态去重121源身份，同帧三旁路max<16ms，收窄到Native独立接收/执行上下文，仍未闭合DDS收前还是回调等待，不据此擅改QoS。/proc发现端口有UDP丢包累计、数据端口0，不直接归控制数据网络丢包。
- 已核安装Humble默认Python executor丢弃底层take_message的时间元信息；独立诊断实际取到source_timestamp/received_timestamp字典，此版本不能直接给普通回调加第二个MessageInfo参数。仅只读诊断，未改生产；后续需准确接收边界观测。1413正油门applied均匹配源且原500ms期限内，P95 .14965/max .44799s；source_lifetime运行异常0，不等于延迟已修好。
- 末速.0000980m/s、723节点/467车身/四轮32碰撞/1449.72kg保持，session errors=[]/独立相关进程0/display148释放，GPU1外部任务不动。真实本轮失败录像 captures/20261018_planning_phase_source/rviz_live.mp4 已回传，426.625s/3413帧/8fps全解码，SHA38153f2da1f311f8bf5514929a332d58dabd531e11bfd8b96e0dd79ec8070a46；右栏camera，无独立front_camera.mp4，打开请求UI返回queued不称已显示。
- 核心ZIP135428356B/276载荷SHA98eda34a3cbc9cb33e7bc927a103de44a0d489ca18cf759766c62484859bf706，本机CRC/逐SHA/视频一致；原2.974GBsensor库服务器保留SHA9b69c4d5864bf19481dd2b55ad1fe7afc764912e009c771592627da00c742367。补充49载荷3748343B SHA7e094c7b1713413531fa53fbb4dddf107cd8e41407c5cdb13aa1e0dccae50c7f同样验证，旧封包不覆盖。统一窗口修正、8项证据一致性检查另存并通过，不当导航成功。
- 下一步先核Native接收/调度与Smac连接-后缀拒绝阶段；多段动态交接、语义/地形约束、整体无进展/持有frontier缺失、视觉定位及记忆反馈仍开放。不得宣称规划基本正常或全由BeamNG物理导致。

### 2026-10-18 真实车体接口审计：确定 Smac 平滑边界缺陷，修复仅隔离验证

- 详 `docs/VEHICLE_INTERFACE_ROOT_CAUSE_20261018.md`，证据 `test_artifacts/vehicle_interface_audit_20261018/`。用户要求先核真实/理想载体物理及消息接口，取消默认每轮回传/展示MP4（两工作区AGENTS已更新）；保留历史证据，今后仅明确要求时提供录像。本轮无新整车搜索/视频，未操作其它GPU任务。
- 确证新增Smac包装层跨阶段约定缺陷：手动平滑后缀首航向变化约.0254～.0917rad，虽然内部曲率/碰撞合格，下游精确connector拼接拒绝；选择smooth后未回退同次搜索中可行raw。六个原锚点分别搜索的raw可行；case6/7无障碍控制旧实际插件均0poses正式失败，排除BeamNG物理、识别和实时消息作为这两个反例的必要原因。不据此解释全部133次原现场规划拒绝。
- 最小生产修复仅planner.cpp，选择smooth前要求首末位姿与raw一致，否则回退已有完整校验raw；不放宽拼接/曲率/碰撞/预算、不重搜、不改车体/转向/速度。test_planner.cpp新增实际插件双反例。此前测试遗漏“平滑合格但边界变化导致后续拼接失败”，保留失败证据，不以次数多声称覆盖充分。raw可能较不平滑，动态跟踪仍待验收。
- 新404成员6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b，deploy/20261018_smoothing_boundary_candidate及服务器/home/ly/smoothing_boundary_repair_20261018，相对d9d仅上述两文件。实际新库fc149c802333ebbea9504e175ae92050821a86a2debb31aaed276fbf1d711a33且ldd核对；日常e6未替换。通用bundle刷新同候选、旧包按hash备份，缺失一份历史标定JSON从旧冻结逐SHA恢复，无旧视频恢复。
- 修复后同无障碍case6/7分别60/40poses，精确起点/曲率/全车footprint通过；12组合recorded/free×wrapped/native对照，native行为保持，recorded case0/6的目标footprint不安全仍拒绝，未绕保护。公开图重建不是现场私有地图精确重放。实际Linux C++共22通过（11planner/7contract/4connector）；本机全局53PASS/0FAIL/6WARN，547回调替身通过。均非完整导航或真实车体动态到达验收；不计20轮，不commit。
- 静止车体尺度/后轴footprint相容的既有实测保留；真实输入转角→曲率、制动距离、低速起步仍未标定。两份闭合轨迹筛选无足够稳定转向窗口，recorded_steering_fit为null，不当参数正确证据。Native独立接收延迟仍开放，不能称同一规划修复已解决消息或全部停驶；默认Humble executor丢元信息限制保留，未擅改QoS/500ms期限。
- 新封闭planning_root_cause_evidence.zip 100载荷/426515B，SHA2d6498a9d026053773a5722476c777f542b25f5a58e12eae1102cd2005e650cb，本机CRC/逐SHA通过，含前后正式反例/trace/真实库及测试，无MP4/活动DB。后续先接收边界定位与动态组件核验，再冻结整车；多段交接、地图真值、探索整体进展、视觉定位与记忆增益仍未完成。

### 2026-10-18 Native接收迟滞调查与Smac独立部署（进行中）

- 用户要求保留已修Smac、查清Native后合并；已逐404成员核对并保留6153冻结，记录test_artifacts/native_delay_20261018/preserved_smac.json。详docs/NATIVE_DELAY_INVESTIGATION_20261018.md；本轮未改Native生产代码、未延长500ms期限、未合并BEST_EFFORT对照，不commit。
- 实际CPU模拟SDK/跨进程/150ms阻塞、真实相机LiDAR90s、地图导航及视觉负载的多组全制动组件多数未复现，不能据此宣称延迟消失。真实控制台/RViz动态161939复现1751接收中777次>200ms、P95450.594/max470.553ms，目标前23.7s即存在，90s目标失败；回调本身很短。重复静止162854在前594帧也出现12次迟到，任务就绪后排队和take调用仅毫秒级，收窄到DDS发布后/Native订阅就绪前，但内核到达与DDS内部根因仍待同故障配对，不认定是规划/物理或某个上游版本缺陷。
- Native BEST_EFFORT深度1仅隔离诊断，一轮正常不足以证明因果；独立DDS暂停400ms后两种QoS都能拿新帧，否定简单“可靠历史必然积压”的泛化。当前FastDDS2.6.11/rmw6.2.10/rclpy3.3.21，未因网上旧缺陷升级或改生产QoS。还需核控制更新间隔、应用源龄及丢失/乱序/屏障/退出，不能只看单帧源龄。
- 新观测层保存DDS元信息、ready/take前后时间；进程内只读UDP观测用SO_TIMESTAMPNS真实内核戳，载荷保持回归通过。初版ioctl戳延迟生成、一次定时器签名不符、一次诊断退出等待发布器导致租约过期均保留失败；未当生产根因，修夹具后先跑实际Humble34定时器/34订阅回归。Native生产代码未改，不声称故障修好。
- 用户随后要求部署Smac并完整调查，当前6153在独立/home/ly/smoothing_boundary_repair_20261018进入420s原Town左90°真实车体/green residential house，driver field_native_20261018_164222/session164222；Native保持原可靠1接收，仅插桩观测，无MP4。运行中核Smac实际新库fc149c80、GraphNav db52/RPP d2fd，日常e6未替换。三新增记忆先验仍关闭验公共链路，不称记忆增量；前1643Native配对无>200ms仅阶段性现象。源码配置冻结、无活动SQLite读取，完整结果与停车清理待封闭，不计20轮成功、不commit。

### 2026-10-18 新Smac6153单轮164222封闭：失败，末段重置预算缺陷已复现

- 详docs/SMAC_FULL_FIELD_20261018.md，证据test_artifacts/smac_full_field_20261018/closed_evidence。420.8397s超时未到达，Native8417样本/覆盖420.799s、105.154m/max离站69.103m、<.05静止278.845s；精确源stale201.996/换挡53.619/tracking12.081/hold7.894s等。不能与公共状态关联planning67.100/following65.826/graph_reset_failed132.249s再相加，也不当CPU耗时。未通过、不计20轮、不commit。
- 新Smac实际库fc149c80运行身份核对通过；103发送、当前60成功/37失败/6旧响应忽略，不强行配对总体成功率。233 raw回退日志含其它平滑拒绝原因，不当作233成功边界回退。1968控制器变换路径、681完整图路/最大间隔2.091s；41目标query掩码/50方向提示、3D定位及确认0，仍非视觉到达；三新增先验关闭保持共同执行基线。
- 确证末段阻塞：visible_approach.py release_visible_approach直接设reset标志，未经request_graph_reset新事务；navigation_transactions.py成功清理后attempts仍累加。同query/fence初始1次+2次释放成功后，下一次不派发，pending=true/waiting=0/response_budget_exhausted，持续拒收GraphNav路。本机reproduce_reset_budget.py三次服务全成功仍复现第四次拒绝，明确BUG PRESENT非修复通过；现场未逐服务录响应，空waiting及预算状态与反例吻合，不误称3次真实超时。相关Python d9d/6153逐字节相同，本次Smac两文件未动该处；待统一新重置事务并保留旧响应隔离，不删重试上限。本轮按用户先测试一轮未继续改生产。
- Native本轮源龄P95 .006992/max.034840s，watchdog/缺receipt0；9054全会话内核UDP/DDS/回调配对无>200ms，历史间歇问题未修复、未在迟滞时捕获内核链。1690正油门有效源ACK未超原期限；另1个gen7在途RPC完成前已接受gen8，旧receipt正确applied=false。可靠包复核载荷/原期限吻合，**不是观察器丢样，不当作有效新代际确认**，更正详positive_source_recovery.json。0源期限违规不能替代真实动作到达。
- 仍有规划失败、visible_target_route_timeout/FollowPath中止、5秒实际进展.0272m等，不全归本轮Native延迟。末速.0001146m/s、连续停稳约.55s、原723节点/467车身/四轮32碰撞/1449.72kg保持，session errors=[]，独立域进程清单空。日常e6及独立6153保留，Native生产无改动，无合并Native候选。
- 核心ZIP27823627B/110载荷，SHA037f2f6dc7bdee30bb25ebe5858246606bea8fbc8bfda6cd40434b6d535863fb，本机CRC/逐SHA及7类证据一致性通过，非导航通过。原261197824B接口DB留服务器SHA dac5d8cfeef67a2a71b72edb324b4f5f80f994749d386a01b5a7a74f33c4ad45及清单完整；无MP4，少量失败RGB非到达证据。结束后自动审批额度失败曾拦SSH读取，用户明确手动允许后经原SSH恢复读取，非网络故障/未绕审批；未新增第二轮测试。

### 2026-10-18 按用户最新要求审查整体连通性：不改生产，复核停驶的多层原因

- 详docs/SYSTEM_CONNECTIVITY_AUDIT_20261018.md，证据test_artifacts/system_connectivity_audit_20261018。用户强调先查底层、整体连通性，本轮没有修改生产或规划参数/冻结包，没有新下发搜索目标，无MP4/commit；404成员与6153逐SHA一致且审查前后未变。Smac已修保留，Native仍未修。
- 封闭164222按同monotonic重算：首次预算故障在目标开始288.284s；之前覆盖288.305s/静止146.350s/行驶105.140m，之后覆盖132.494s全静止/0.0146m。前段精确源stale70.740s、换挡53.619s、tracking12.081s、stop2.511s、hold7.399s，公共planning关联67.100s不能相加或当CPU耗时。因此只修reset不足以解决全部频繁停车。1967 stamped到旁路P95 2.772/max9.142ms，不能把本轮stale标签等同通信延迟。
- 当前生产方法6缺口探针复现：reset成功历史耗尽预算并实际拒收新鲜on_route；同事务20次局部成功往返无进展退出；ExecutionExperienceStore当前成功反馈链检索全1；GraphObservationStore方向负证据保存节点最大；sensor重启epoch跨层不一致/大回退歧义（两时钟条件离线反例，非上轮实证）。6项“通过”表示BUG仍在，非修复。37既有事务保护及547回调替身通过，覆盖重叠不相加；首夹具重复安装ROS替身启动失败已保留并修夹具，不改生产。
- 源码新增关联：GraphNav clear_goal既清explicit/退役探索，又clear全部ExplorationBackoff；同query临时靠近释放/选择ACK超时使用同服务，会删除此前退避。此为服务副作用事实，未逐次证明某次重选的现场因果。统一事务入口之外，还须区分同query恢复与新query/map清理，不能简单永久保留退避。完整Smac绕行仍无同一WildOS语义/坡度连续约束，costmap未知发布为自由；动态制动/换挡/首段可接入成本不足和全局进展仍开放。
- 上轮目标窗口295视觉评分帧/293非默认节点、score errors0；新增先验关闭/retrieve0/utility delta0，证明活动与基线条件，非因果消融或记忆增益，不能用经验链缺陷解释该轮停车。
- 另做一次有界制动Native诊断field_native_20261018_202621/session202621：就绪后60s、记录83.425s，零goal/非零速度目标/正油门、reason全disabled；1673帧源到回调P95 6.242/max26.672ms、无>200ms且1673内核UDP/DDS配对。历史间歇延迟未复现、未修好，不改QoS/500ms。Driver scope继承420s搜索文字，实际--idle/goals0已更正；未计搜索成功。
- 1649 Native最大速.001101m/s，末速.0000392m/s/持续停稳约.58s，原车体碰撞保持、errors=[]、退出域进程[]。安装源门禁复用，本轮未另存运行时maps三库快照，不虚称该项已验。制动静止为诊断条件，非新一次搜索失败。证据ZIP1069529B/16载荷SHA1c20ea28601c63518c0af2930eba98d24cf6bf60c1bd5d6fcce5060328c3bb8b，本机CRC/逐SHA通过。下一步按事务归属/服务副作用、可走空间、动态执行、任务进展与反馈共同门禁处理，完整搜索与20轮仍未通过。

### 2026-10-18 继续整体审查：确证视觉超时清物理恢复，复现高度参考缺口

- 详docs/PHYSICAL_STALL_HANDOFF_AUDIT_20261018.md，新增证据仍在test_artifacts/system_connectivity_audit_20261018。本轮只读164222封闭录包/当前源/安装Lua，无新仿真、目标、录像、生产修改或commit；6153保持。
- 上轮288.045s失速前5s共100 Native全按源身份/载荷/有效ACK配对，reason全tracking，目标v中位.71356m/s、实际油门.2345/制动全0/D100帧/引擎运行，健康正常/watchdog0；四份去重接触16轮支撑最小2015.5N、驱动扭矩绝对值221.18～915.64Nm/轮制动0。净位移.03499m/累计.12395m/沿路进展事件.0272m分别记录。后轴50odom横向误差max.07235m，需求曲率半径约57～103m，转向max.081远低于.6；本段不支持无控制、明显跑偏、原地转向或饱和解释。body pitch约3.96度/clutchRatio1约.169不单独作地形/变速器根因。具体接触物/车身阻力未知。
- 实际D/R选择与匹配发送前后挡变化均28次；此前62是低速gear_change连续采样片段，非62次实际换挡，53.619s守恒。安装0.39.4 drivetrain代理/DCT源码只读审阅及SHA保留，未证明重复D指令强制打开离合，未改API；未独立记录活动powertrain模块名，不把安装文件审阅当运行模块身份证明。
- 三时刻277.060/283.045/288.045s的公开local/global costmap、odom、已下发4.564m路径导出，实际同SHA helper裁剪当前位置后，footprint无100障碍、剩余前缀至多2m检查均通过。公开图非私有缓存逐字节真值。原261MBDB SHA未变，五导出JSON本机/服务器SHA核对；无raw/adapted/marking LiDAR，不能认定具体过滤阶段丢失现场物体。图stall_public_maps.png区分99膨胀和100障碍，初版合色图保留，不将触及99误诊漏碰撞。
- 新底层反例1：pointcloud.py先按body z min_height=.1裁剪，再按地面plane ground_clearance=.25去地面。同平面地面z0/物体高.4→mark1/occupancy100；地面z-.45/物体高.4→body z-.05被删，clearing保留/occupancy0；高.65才再次mark。是部署双高度参考会漏障碍的契约缺口，未修改生产。闭包terrain与odom三组源戳完全相同，近8m有效自由低表面271/238/257格，body z中位-.137/-.250/-.251；末聚合面[-.01745,.05539,-.21384]上构造合成30cm障碍body z.00367同样被删。**合成物体非现场碰撞物，聚合面非raw LiDAR，不能写成已证该轮根因。** 不能仅把min_height改负数使地面全进障碍图，须联动地面可信度、车体净空、坡道/悬挑/稀疏/自身点/清空射线。
- 新底层反例2：失速时visible目标已激活17.185s，follow_failed给2s退避，.2388s后视觉超时释放；release→reset→init_execution_recovery清已驶轨迹、失败计数及待脱困意图。3实际方法/ROS替身对照复现首失败11轨迹/1失败被清、第二失败待停稳escape被丢、普通探索保留；quarantine正确保留，未绕安全停车。首夹具服务未ready导致最终status覆盖的两断言失败保留，修夹具为成功初始clear后复现，不改生产。此与reset成功计数耗尽是两个缺口，单修计数仍不保住恢复。
- 后续应分别管理地图/采集epoch下的物理走廊、实际失败位置与恢复意图、query/实例下的视觉意图、clear事务与已接受动作；不能只preserve全部reset状态或复活旧epoch权限。Native间歇迟滞、私有地图、具体接触、完整搜索/记忆增益/20轮仍开放；本轮结束只读域进程清单[]。
- 本轮封存physical_stall_handoff_evidence.zip：479586B/32载荷，SHA48424d960dfbb3461ea3cf8d8c1e039718983b7e564d2bb05c6e2c50d14ac0d5；本机CRC/逐SHA、五份服务器导出JSON同一性、100帧控制/四轮接触/地图检查/源404一致性通过。证据一致不是修复通过；初封存器encoding拼写错误保留后修正，未改断言。旧核心包及原DB不覆盖。

### 2026-10-18 原生接触观测器组件通过，尚未复现受困

- 详docs/CONTACT_OBSERVER_COMPONENT_20261018.md；本机test_artifacts/contact_observer_20261018，远端同名目录braked_20261018_211038。只有诊断器/脚本/测试变化，Native/Smac/生产配置/原JBeam均未改；6153包404成员逐SHA再核一致，未merge Native、不commit。
- 已核0.39.4 main.onNodeCollision→controller事件，能得节点/法向力/slip/depth/material，不给被撞对象ID/名称。package.preload注册独立auxiliary controller，不写游戏安装/user mods，不替换原回调，不改控制/物理；按JBeam wheelID分body/wheel/unknown，1024节点/128输出上限，复制引擎复用事件，窗口序号+sim时间及SDK请求响应wall界限。
- 11项真实liblua5.3接口夹具通过，非物理。所有权新反例先1失败（安装被同名preload拒绝后cleanup误删非己模块），修诊断器独立注册归属标记；before代码/log保留，未把候选问题当历史生产故障。Python语法检查通过，真实引擎注册/采样/卸载另验。
- 全制动组件一次原Town左90°/原SBR，速度环enable与Bridge follow_path均false；无搜寻/移动目标/视觉模型/RViz/录像，未计20轮。15窗sim4.078～7.312s，实际轮胎事件6300/body0/unknown0，峰值1729.4N；全程正油门0/max速.00256m/s，1449.72kg/723节点/467车身碰撞/四轮保持，卸载ACK、停车/errors=[]/独立域进程[]。actual maps三库db52/fc149/d2fd核验。**仅轮胎支路真实触发，车身接触支路仍未现场验证，不称旧失速复现。** 原始向量位置/法向符合静态出生地面范围，移动/旋转坐标契约未闭合。
- 唯一watchdog帧保留：源龄.01242s/接收龄.01153s有receipt和SDK ACK，但telemetry_ok=false/age=null，为首遥测尚未建立时的关闭路径，不是源超期。135直接Native回调P95 5.993/max10.064ms，本轮无内核链，历史Native间歇问题仍未修。
- 封闭录包raw LiDAR62/adapter59/marking和clearing各55/contact15，TF/odom/terrain/两图/源控制等齐；**本组件未录measurement_timing及raw StateSensor，不能声称跨sim/ROS时间映射已验证**，下次受困采集须补；私有costmap版本仍缺观测。
- 核心ZIP122856B/30载荷SHA1c3193eecf13b4d95d78e7e9c36e50668537e6af52550514123420f4c9649a74，本机CRC/逐SHA/边界断言通过。原102608896B库仅服务器interfaces/interfaces_0.db3，SHA202f590531c571136b416569127d56554673d3781fc35486f77911450220968b及外部清单保留。
- recorded_section_fixture.json已从164222封闭记录冻结原33点/4.564m段+cusp_goal_checker+邻近Native/实际起点，速度.07858m/s。尚未验证出生quat还原、未运动复现；新出生不含旧变形/挡位/速度历史。下一步先校验姿态与接入，受控短段补body接触/原始过滤点云/时钟，再与视觉恢复/clear事务共同处理；不把组件通过或诊断记录齐全当完整搜索成功。

### 2026-10-19 补记18日晚受控短段：真实车身接触与早期低表面回波遗漏

- 详docs/RECORDED_STALL_ROOT_AUDIT_20261019.md，主证据test_artifacts/contact_observer_20261018/recorded_evidence_214400。只有独立诊断/测试/记录变化；6153源/归档404成员再核一致，生产Native/Smac/参数/原碰撞未改，无新整轮搜索/MP4/commit，不计20轮。
- 原33点/4.564m+cusp_goal_checker的实际FollowPath受控执行214400：一次Town加载，按历史Native原点/dir/up还原出生，实际后轴误差约.6mm/yaw约.0001rad；静态新stock车，不含旧变形/.07858m/s初速/挡位史。Bridge follow_path=false，独立客户端仅发一次原段，当前图/接入准入通过；不是全局重规划或视觉到达实验。
- 确证物理受困可在无视觉交接时发生：5.022s沿路进展.0389m，100Native全精确源/载荷/有效ACK匹配、tracking/D100、制动0、发油门中位.23449/SDK实际.23101、watchdog0，Native3D净位移.04143m。原生body946事件，启动前0，node161/f3l sbr_body持续接触峰值79020N；world向量假设下normal·forward中位-.857。回调向量独立坐标契约与物体身份仍待核验，不能凭RGB或material_id命名碰撞物。
- 真实点云六时刻重放marking/clearing逐点误差0。按原始接触坐标为world的空间假设，早期sim约4/5秒邻域raw369/366→adapter116/116→车体剔除后116/116→body z>=.1后0/0，clearing仍7/6；邻域在后轴前约4米，不是车体剔除。点body z最大.0245/.0131m、地面残差最大.1731/.1723m，单降min_height仍被.25地面余量删除。**实际回波遗漏已证；这些回波与具体碰撞实体的精确同物关联仍待独立坐标核验。** 不把此前高于.25的合成反例与此低表面场景混为一个阈值修复。
- 对应公开两图当前body/前向至多2m均允许，坐标格local0/global9；未记录私有缓存。129state源/映射/odom一致、offset等式成立/重启0，130LiDAR发布；StateSensor header是bridge锚定epoch，非objSim秒数；未得到移动中精确SDK载荷锚点，不宣称contact→raw完整时钟映射已验。历史Native间歇迟滞仍开放。
- 诊断器自身错误全部保留：可变fixture3error、环境转义/CRLF在仿真前退出、214046数组参数漏解准入前退出、214221 UUID uint8序列化失败在动作接受后关门取消。第二轮父报告motion_goals=0是默认值不权威，以事件/控制器为准；非历史生产根因。最终7真实ROS输入/参数/UUID检查及11证据一致性通过，非导航成功。分析器启动无sim样本、核对脚本语法错误修正，不改原DB。
- 收尾原物理保持，动作取消status5、持续实测停稳末速.02108、观察器卸载ACK、父子errors=[]、独立退出清单[]。封包119载荷/4001801B，SHA81f2d6d04d85c23f004d7481abc9644917791a2e194402e2bf66241cb6d30eb7，本机CRC/逐SHA通过；主要270356480B原DB仅服务器，SHA31e20bf925c85f7faaf66bd8018c60f531138cf9ef8b9238a2066a707bd2c71b。整体搜索/记忆增益/稳定性仍未通过。
- 下一步共同审查可走空间/真实底盘通过能力与恢复事务：低表面障碍/地面/坡折/clearing统一契约；独立节点坐标与objSim核验接触关联；视觉超时清恢复和成功clear预算两个既有缺口仍未修，不能只改过滤或只改计数就称整体解决。

### 2026-10-19 接触坐标独立核验、显式桥接anchor与启用边界

- 详docs/CONTACT_FRAME_CLOCK_AUDIT_20261019.md，证据test_artifacts/contact_frame_audit_20261019/closed_evidence。仅诊断/测试/记录改动；6153源/归档404成员复核一致，生产/参数/碰撞/Native QoS未改，无MP4/整轮搜索/commit。
- 最终frame084254重复原段：5.005s进展.041m，101Native源/载荷/有效ACK全匹配、D101/brake0/油门中位.2140/watchdog0，body929/wheel18111/unknown0，node161 sbr_body峰值85504.5N。取消动作、持续停稳末速.01916、卸载ACK、父子errors=[]/独立进程[]，不是导航成功。
- 在同碰撞回调另读obj车辆原点/节点相对位置/dir/up/sim；24份body位置与独立origin+relative world位置误差median .0209mm/P95 1.34mm/max4.63mm，相对坐标假设误差数百米，sim差0。位置world关联已在独立重测中核验，收紧上轮假设；运动wheel节点max差6.35cm，不声称contact点与动态节点完全重合。法向独立力API仍未核验、碰撞资产ID/名称仍缺，旧封包不覆盖。
- 用独立节点world位置再选早期邻域：raw368→adapter116→车体剔除116→height0/marking0/clearing7，五时刻正式过滤重放逐点0误差。车体受阻区域低表面真实信息遗漏再次成立；余量.25仍会删除约.17m残差，单改min_height不足。实际通过能力与地形/障碍/clearing契约仍待统一。
- 新观测直接读取已初始化VehicleNode.start_time，anchor1792370573193935406ns；bridge源header=anchor+state引擎秒，adapter另加offset。DELAY_OFFSET_SEC=10是锚定约定，不是实测10秒通信延迟。碰撞峰值8.199500389/近邻state8.186000388s，约13.5ms差明确保留，去掉旧Native wall猜原点；仍非真实LiDAR采集时刻证明。
- 新时间接口缺口：136raw消息72对相邻payload完全相同却换header，其中21对车姿已移动>1cm；实际导入五份bridge/SDK源码已SHA快照，Lidar poll最新缓存并使用车辆轮询time作header，故header skew0不能证明同次采集。world点同姿态变换可抵消坐标换基，不直接推导每次障碍平移；ray origin/近场过滤/动态目标/camera像素-3D与记忆关联需真实采集身份或有界时间。未录原camera不能声称其也已复现，不能用hash相同直接丢帧。
- 首轮frame083934接受动作约20ms后source_owner_unconfirmed保护退出，未实质运动；只轮胎67份参考读数max world差.0565mm。代码/输入反例确证独立诊断客户端可能在enable后误以启用前一致gen0缓存通过，旁路gen0→gen1/ACK-not-ready时序吻合，但私有回调顺序未录，不当精确因果。仅诊断器加强为等待本次预期新gen1/post-request/有效ACK再发动作，原3s/500ms保护不变，最终录到新代确认后动作。生产历史其它owner异常未宣称解决。
- 18Lua接口/最终13真实ROS输入时钟代际/11证据一致性通过，均非导航通过；新诊断源两个版本逐SHA封存，首次10项测试源码按原SHA57f0864a…精确重建保留。封包129载荷/2919861B SHA6a82c23299347ca85c52bfc46a8beba3353e7e81e414f9fef007505bb9bfe2c0，本机CRC/逐SHA通过；主268894208B DB仅服务器SHA0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc，前轮DB清单同包。恢复/重置/原Native间歇延迟/完整搜索与记忆增益仍开放，继续整体接口审查。

### 2026-10-19 占据/地形全链重放及台阶指标契约反例

- 详docs/TERRAIN_CONTRACT_AUDIT_20261019.md，证据test_artifacts/terrain_contract_audit_20261019/closed_evidence。本轮仅只读084254原DB和冻结源码，无新仿真、生产/参数/物理改动、录像或commit；调查前独立进程[]。
- 22份真实占据及terrain地图从同原raw/state/显式时间映射重放全部逐格一致、缺失匹配0，mapping/terrain/pointcloud安装源逐SHA等于6153。接触位置22份body occupancy=0、terrain traversability=0/unknown=0；前6份elevation_valid1、后16份0。0是自由代价，未把值方向看反；未测高度不等于安全高度。
- 早期真实3×3高度格经冻结C++terrain_metrics：slope约.146～.157(rise/run)、step .0747～.0815m、roughness .0254～.0296m，均通过源码默认.35/.10/.08；不是实际GraphNav节点/边或现场参数回放。terrain先共用occupancy高度筛选，再按0.5m格取最低点+.15内平均，表示低表面而非障碍最高包络。
- 新确证**源码**反例：17cm半平面阶跃(3×3、0.5m间隔)经拟合去平面后step仅.085m/slope.17/roughness.04007，按默认10cm台阶上限通过；平滑17%坡面step约0、30cm孤立突起正确拒绝、不足5点observed=false。残差范围不等于真实阶跃，与RobotMotionConstraints米制max_step_height比较有契约缺口。属于当前适配版地形指标链；不归咎论文/模型/记忆，也不把合成17cm断面当现场实体精确高度。
- 不能只改min_height或将step替成相邻格差：需地面斜率/突变/观测尺度/高度包络与真实车辆净空共同定义，覆盖坡道/台阶/悬挑/稀疏/噪声/clearing。实际Mapper运行库/参数、节点边physical metadata及规划选择仍缺必要回放，不宣称其现场必然接受某条图边。恢复清理/重置预算、原Native迟滞、真实采集时刻、动态车体与完整验收仍开放。
- 8份冻结源与编译器/二进制hash、五类真实C++对照、9项证据一致性检查封存；通过表示反例与证据成立，不是修复。封包18载荷/200789B，SHA6f8f85f227af50addd72ef5cf4e1d2e654c48f31eaf903eb46998231db8b76a3，本机CRC/逐SHA通过；原DB SHA0a123472…不变/仅服务器。后续实际图回放再统一选择最小生产修复。

### 2026-10-19 实际Mapper回放与清理事务首项修复

- 详docs/MAPPER_REPLAY_AUDIT_20261019.md；test_artifacts/mapper_replay_20261019/replay_20261019_090817。独立ROS域191，无车辆/控制/生产改动：084254关闭DB原header回放128去重odom/22terrain/254TF组，实际Mapper库50fe10c4及参数已取证；61图/22快照、errors=[]。接触邻域节点均feasible，近接触边confidence全0；当前Planner确实读取物理字段，但置信度不足不作硬拒绝。未运行Planner选路，随机图不等同历史原图；不能声称确切历史边选择已复现。补上部分实际Mapper输出证据，地形/过滤契约仍未修。
- 详docs/RESET_INTENT_REPAIR_20261019.md；test_artifacts/reset_intent_repair_20261019。首次实施小范围生产源码修复：visible释放调用统一request_graph_reset；同scope未完成重复请求合并，独立完成后的新清理获得新预算，保留三次上限/旧请求排空/迟到响应隔离。旧版四个新反例3失败，新版41事务/551离线回调通过。
- 服务器独立candidate_portable/ROS域192真实ROS接口回调41通过；不是DDS动作服务器或整车验收。首次candidate的5项夹具错误(真实发布方法Mock接口/替身Clock属性)及日志保留；显式注入发布器、验证真实Clock构造参数后复跑通过，没有删断言或改生产保护。本机/服务器实际导入新实现SHA da57b812/fd5cdd2e相同。
- 本机当前两份实现已偏离6153，原归档/正式服务器候选未替换，不能再宣称当前源404成员全部等于6153。before快照、两增量包、失败/通过报告分别保留；尚未整车验证，不commit、不计20轮、不录视频。结束独立项目进程[]，GPU1外部负载未触碰。
- 仍开放：视觉超时通用reset清掉物理恢复、C++ clear_goal清探索回退历史、低表面障碍遗漏/terrain台阶指标及不确定图边、真实采集时间、历史Native间歇迟滞、动态连续性与完整视觉搜索/记忆增益。只修重置预算不等于全部静止解决；下步按各状态归属及可走空间统一验收。

### 2026-10-19 视觉/物理恢复归属修复及实际C++清理反例

- 详docs/RECOVERY_VISUAL_HANDOFF_REPAIR_20261019.md；test_artifacts/physical_recovery_handoff_20261019。先以9实际回调反例复现旧版7失败，再改execution_recovery/visible_approach：视觉激活/完成/释放保留同query的实测走廊、失败时间/位置及尝试预算；已准备/执行物理恢复时，视觉激活/放弃等待，不转移旧epoch权限。新query仍撤销，60s走廊寿命/旧动作终态/实测停稳/当前图与传输门禁保持。
- 新增11项交接验证，最终52事务/562全部离线回调、服务器52真实ROS接口回调通过，非DDS动作服务器/车辆/完整搜索。扩展夹具曾遗漏新路线和新戳地图，失败日志保留，修输入前提后通过；未放宽生产保护。源SHA visible29e062b7/execution7de841fe，含上轮transactions da57b812。6153先逐员核404，再叠加独立五文件增量，服务器candidate/ROS域193；正式安装/原归档不改，不commit、不计20轮。
- 详docs/GOAL_CLEAR_CONTRACT_AUDIT_20261019.md；test_artifacts/goal_clear_contract_20261019/probe_20261019_093600。实际C++库db52f00e /proc/maps核验，隔离DDS域194同图/静止位置：初选A20m→精确拒绝后选B40m→仅clear_goal后1.549654s又选A，三个独立探索身份、errors=[]。证实clear服务会删原120s暂缓记录，不能只靠Python恢复修复；合成接口反例不是历史每次回转的唯一原因。
- 后续应联动区分新查询完整搜索重置与同查询目标释放；goal_pose不无条件清失败历史，Bridge记录操作类型且保留旧请求排空/响应归属。此C++服务修复尚未实施。地图漏低表面/terrain、采集时刻、Native历史迟滞及动态连续性/整车验收仍开放。此前“视觉超时清恢复未修”现收窄为已修组件、待实际动作/车体验收；未宣称整体静止已消失。

### 2026-10-19 C++/Bridge清理语义联动修复、迟到目标边界及实际DDS验收

- 详docs/GOAL_RELEASE_HANDOFF_REPAIR_20261019.md；test_artifacts/goal_release_repair_20261019和goal_release_fence_20261019。先审查9处request_graph_reset调用，新增release_goal保同查询探索失败暂缓；原clear_goal供on_query完整清理，goal_pose不再无条件删暂缓。Bridge记录两种操作，未完成完整清理不被释放/确认fence改变降级，提升时旧响应排空后才开放目标；缺新服务不回退，preflight新增两种Trigger服务门禁。
- 首候选实际DDS正常7阶段通过，但继续负例发现GoalPose与Trigger跨通道无排空约定：新query清完已选A，重放旧B目标又切入B显式导航。首库7bac0479及late_goal_20261019_094938反例保留；其passed仅表示缺口复现。现有目标入口无fence是既有缺口，不能把正常交接通过称完整修复。
- 后继C++新增destination_fence.hpp：clear/release记录ROS纳秒边界，旧/重复/乱序/未来及非法目标戳不更新接受序列；正常新目标仍可通过。依赖一致连续ROS时钟，不等价全程query/动作UUID，任意跨epoch重合和实际采集重启仍未解决，现场前需核时钟模式/运行源。
- 最终570离线回调、服务器60真实ROS接口回调、35实际C++(含6新时间边界)、7部署门禁单测及15阶段真实Bridge/C++/DDS均通过。实际DDS约11.18s覆盖两种清理/实际目标发布/旧拒绝ACK/新query/旧目标跨清理迟到/同阶段乱序/未来戳及其后正常目标。无FollowPath动作服务器/车辆/视觉模型，不计20轮、不录视频。首次构建PATH无colcon未编译，保留失败日志后改绝对入口通过，未放宽生产保护。
- 原6153归档/正式安装保留；先核404再覆盖独立16成员，当前408源/测试文件逐SHA核验。最终 /home/ly/goal_release_fence_20261019、ROS域195；Planner源ae33191a、实际库86fecc86，Python源transactions c818af39 /Bridge fdef3b5d /execution7de841fe /visible29e062b7，实际导入/运行映射已核。首候选及反例不覆盖，必要整车待验收所以不commit。退出独立项目进程[]，GPU1外部负载未触碰。
- 此前“C++ clear_goal清退避未修”现收窄为已修组件并通过实际DDS，仍待整车；地图漏低表面/terrain指标及未知图边、ROS时钟模式与传感器真实采集、Native历史迟滞、动态连续性、完整搜索/记忆增益/20轮仍开放。下一步统一可走空间与真实车体通过能力，不能用这些接口通过替代物理或视觉验收。
- 本轮两候选/旧目标反例/最终验证封包goal_release_handoff_evidence.zip：65载荷/375469B，SHA2eaf87fdb0ca47384a5d9e6cf7c60b6c642d28adb5673ac3daf3902455c731f7，CRC与逐载荷SHA通过；原6153及前轮封包未覆盖。独立进程[]，证据封闭不等于完整导航成功。

### 2026-10-19 SSH 不可用后的离线可通行性反例与未解决清单

- 用户说明暂不能 SSH，后续未重新连接；此前三次连接/传输超时均已退出，新增远程原始点云提取未执行。服务器当前状态未知，不把此前清理清单当当前状态。无生产修改、仿真、录像或 commit。
- 最新清单 docs/OPEN_ISSUES_20261019.md；新增离线证据 test_artifacts/passability_contract_20261019/ground_model_review.json。audit_ground_models.py 本轮重跑退出 0，当前 pointcloud.py SHA7cfc07b9 与报告一致；这是反例成立，不是修复通过。
- 新确证源码缺口：地面拟合把格内 min-Z 配到格中心 XY。已知平面 z=.24x+.24y-.45 的拟合截距误差 -10.56cm；仅诊断版保留实际 min-Z 点 XY 后恢复正确值，生产未改。连续凸面在降阈值/修配对诊断假设下仍有误标，不能单改阈值或配对就部署。
- 合成控制证明 0.5m 采样下阶跃与短坡可同值，二维清除射线也不证明低处车体体积空闲；不命名现场碰撞资产。8 份旧点云切片中 5 份同源戳重放点集一致、3 份不同源戳不用于逐点一致性结论；不是最新084254原DB提取结果。
- 尚未解决：低表面遗漏、terrain/未知与语义可通行约束、整体无进展/持续缺失退出、记忆中性反馈/方向负证据串扰、真实采集身份与时钟重启、历史Native延迟及真实动态连续性；其他旧审计风险和完整验收边界见新清单。重置预算、恢复归属、两种clear语义及旧目标fence为已改组件待整车，不再列为完全未修。

### 2026-10-19 用户授权整体修复、SSH恢复：记忆评分链与地面拟合候选

- 详 docs/RECALL_GROUND_CONTRACT_REPAIR_20261019.md；本机 test_artifacts/recall_contract_repair_20261019、passability_contract_20261019。用户恢复SSH授权后连通，预检无相关残留/约147GB可用；只读提取084254关闭原DB的22帧点云/terrain，本机14782757B NPZ SHA02ab9c2f2ef2d7fbf5c3535780d9dabfce6f655e4cd24e929b0c74420244681c0，原DB SHA不变。没有新仿真/录像/commit。
- GraphObservationStore新增按map/query/node/分辨率/bin的负证据，实际候选评分读取精确方向，旧最大汇总仅诊断、不猜测旧方向；有限覆盖容量与级联清除、重复/无效/乱序/改分辨率/查询隔离已测。无mask时的负评分与mask_threshold对齐，无效输入不学负例；弱证据不证明区域无目标，其他历史表容量仍开放。
- ExecutionExperienceStore新增明确执行乘子接口，1中性、已测成功可>1、未知方向仍1；旧惩罚接口/NULL旧记录保持语义。scene样本带yaw/frame，按当前视角旋转方向；缺方向/frame不一致不应用新乘子。仅融合已有视觉支持且保留显式抑制、物理不可行硬拒绝；不强行将动态障碍/取消等写成地形失败。
- pointcloud地面最小Z与真实XY配对已修，正负坡向/物体/打乱/缺支持验证。22帧旧重放四层图全部逐格一致；新总marking29573→29566，traversability差1格/unknown0/elevation129816/elevation_valid1，未知平面填充不提升有效性。受困邻域早期仍marking0，低表面根因未修，不能仅修拟合就宣布整车可用。
- 最终51本机记忆、19地面、570既有回调替身通过；服务器隔离v2/域196实际51记忆、19地面、60真实ROS接口回调及5阶段真实存储/候选→DDS→C++决策通过：A→方向负证据选B→新query选A→成功执行先验选B→B物理拒绝时选A。模型/FollowPath/车辆未参与，完整搜索及记忆增益仍待验收。第一诊断库名错误提前退出保留，仅修诊断后重测，未改生产保护。
- 6153原包/正式安装不替换；从404原成员+31增量核当前414成员，候选 /home/ly/recall_contract_repair_20261019/v2，库86fecc86；v2增量ZIP SHAa27a1adbf7f872390a21519d5fb56b94488e82383e4b519eb916a256db212385。服务器封闭证据106419B SHA feca3489f5c323a6ec5842c50dd18fcf97eaf50c6da934a2c39f8ead6224abe3，本机核验通过；退出清单[]，SSH正常关闭。
- 相邻返回高度突变诊断原型：合成坡面/凸面/坡顶0误标、17cm阶跃60标记，但真实受困接触邻域仍0，明确不部署。下一批联合地面形状/车体3D净空/视野与清除，不只调阈值或加台阶规则。原“记忆恒等/跨方向串扰、地面XY配对完全未修”现收窄为本批候选已验证组件、待现场；低表面、terrain/未知危险置信度、语义路径、整体无进展、采集时间/epoch、Native延迟及动态连续性仍开放，不计20轮、不commit。

### 2026-10-19 可通行证据与有向图联动修复（候选验证中）

- 详 docs/PHYSICAL_EVIDENCE_CONTRACT_REPAIR_20261019.md；证据 test_artifacts/physical_evidence_repair_20261019。SSH预检无相关残留，约147GB磁盘/73GiB可用内存，顺序编译并限制2作业；未启动车辆、录像或commit，正式安装与6153包不改。
- 实际C++ baseline复现Mapper21项4失败、Planner38项2失败：局部已测危险被其他缺测降为confidence0；跨缺测拟合出伪step/roughness；未知更新覆盖旧危险及刷新时间；历史traversed/visited绕过新鲜拒绝。
- 联动候选新增明确physical_violation_observed，按连续有效高度段和实际栅格采样评估，局部明确违例不被其他缺测冲淡，未知更新保留原危险戳，完整安全观测才清除；Planner新鲜明确违例不再被旧通行历史绕过。既有时间戳/不确定性门禁保留。
- candidate2回归发现2项Mapper失败（台阶遗漏、完整安全观测未清旧危险），未放宽断言，查到等距浮点采样重复/跳格，改逐栅格LineIterator。candidate3实际21/38组件通过，但CDR/DDS链发现更深缺口：Mapper有向边被Planner无向图折叠，反向未知边重新开放正向危险路线。旧baseline7阶段均完整路径仅表示旧行为复现；candidate3第二阶段unknown_after_steep错误complete=true，失败记录保留。
- 改Planner为有向图并新增方向/边次序/退路回归。candidate4构造初始化类型未同步导致编译失败，已统一按成员声明类型重建；当前candidate5独立编译/联调中，不能把candidate3组件通过当最终修复。根目录SSH会话58885当时承担构建，继续工作先核实其进程/输出，不因观察超时重启。
- 上游低表面丢失、terrain台阶表示、真实净空/射线、语义路径、整体无进展、采集时间/epoch、Native延迟/车辆动态和完整验收仍开放。该新反例来自当前适配层数据/图方向约定，不能归为所有历史失败的唯一原因。

### 2026-10-19 可通行证据/有向图最终候选6：组件及实际消息链通过，整车仍待验收

- 接续上节，详 docs/PHYSICAL_EVIDENCE_CONTRACT_REPAIR_20261019.md。candidate5编译后Planner40项3失败，迁移有向图时还须同步两处frontier虚拟终点方向；未放宽旧断言，修为frontier→终点并覆盖后续local-frontier连续请求。candidate6实际Mapper21/Planner40全通过，旧失败候选/反例均保留。
- 实际Mapper输出直接CDR序列化，经DDS进实际Planner的7阶段通过：部分已测危险拒绝、后续缺高度且反向未知边出现仍拒绝、完整安全重观测恢复、未知可探索、分离平面不跨缺测拟合、完整危险拒绝、过期证据按原策略退为拓扑先验。最后一项不证明旧危险消失，静态危险/动态TTL/可执行地图仍须联动；未禁用新鲜度保护。旧baseline7阶段全可规划仅是旧行为反例，candidate3组件59过而第二DDS阶段失败，不能作通过版本。
- 同最终版本，570本机接口替身、服务器51记忆/19地面/60真实ROS接口回调，以及原15阶段目标交接DDS/5阶段记忆决策DDS通过。新物理标签静态检查确认Python视觉评分不会删除；没有模型、FollowPath、Native、车辆或完整搜索参与。不计20轮、不commit、无MP4。
- 原084254关闭DB SHA0a123472…只读回放22terrain/128去重odom/254TF组，22快照完成/errors=[]。近接触950条边评估中51条confidence≥.7，41条明确超当前残差台阶指标阈值，35条新鲜/6条过5秒，max年龄7.958500378s；不是独立障碍数，随机图不等同历史原图。台阶指标仍非真实阶跃高度，上游低表面遗漏未修，不能据此称真实接触已避免。
- 6153及正式安装保留；404基线+35增量当前414成员逐SHA核验。最终 /home/ly/physical_evidence_repair_20261019/candidate6，增量ZIP SHA356c1b546f76d6d49312d4eedc641c30ff65b70949a057da99a9333509616778；实际Mapper库0ef78c09/Planner11f92359。168成员服务器证据1861522B SHA43531230f6d446a07c927e28c7594c02c5787b6f1b1c87afc30f47f8a58d2a30，本机167载荷/源/结果核验通过。退出项目进程[]；SSH会话58885已正常退出，不再有本轮活动构建。
- 修复边界收窄：局部违例被整体置信度冲淡、跨缺测拟合、未知更新刷新/覆盖旧危险、新鲜违例被旧通行历史绕过，以及有向边被无向图折叠已有联动候选通过组件/消息链。下一步优先修terrain台阶/粗糙度与车体通过能力、低表面和净空/射线；语义整路、整体无进展、采集epoch、Native延迟/车辆动态及20轮完整验收仍未完成。不得把新增physical_violation_observed当真实障碍资产或真实台阶高度的证明。
- 本机最终封包 physical_evidence_handoff.zip：64有效载荷/3258358B，SHA3031ad2727f649a08b6dfbed7ed8c12eb97bf293dec2cc6fba86fe9054a54c21；CRC及逐载荷SHA通过，包含修复前/失败候选/最终源码、差异、真实消息链及历史回放证据。封包不代表完整车辆通过，不覆盖原封包。

### 2026-10-19 candidate7：低表面障碍与亚格台阶契约修复（组件/实际DDS通过，整车待验收）

- 详 docs/PASSABILITY_LOW_SURFACE_REPAIR_20261019.md；证据 test_artifacts/passability_contract_20261019 与服务器 /home/ly/passability_repair_20261019/candidate7。用户授权继续并要求考虑全局。先备份 candidate6 源/安装：本机 deploy/20261019_passability_repair_backup（六文件 SHA 与 candidate6 一致），服务器 backup/candidate6_candidate_20261019.tar.gz SHA69dd48bb…、install SHAbb618d7d…，gzip -t 通过。正式安装/6153/Native/Smac/RPP/JBeam/碰撞参数未改，未启动BeamNG、未录像、未commit。
- 低表面漏检修复：split_costmap_points 新增 obstacle_min_height；提供时在 ground_plane_radius 内按相对局部地面高度标记，半径外/拟合失败保留旧 base-frame 路径，None 完全兼容旧行为。official mapping/terrain/node 与 nav2 cloud_filter/official launch 都接入 0.10；WildOS 两图和 Nav2 costmap 使用同一阈值。0.10 是原 min_height 的地面相对化，不是实测底盘净空。
- 台阶契约修复：terrain.py 新增 witnessed_step_by_cell（0.1m 亚格 min/max，相邻亚格高差减 max_slope=0.35 可解释上升，support>=2，0.5m 地面包络），作为 witnessed_step 层发布；elevation_min/max/height_span 仅进程内诊断不发布，避免 DDS 翻倍。mapper.cpp 加载该层，节点/边 step 取 max(原残差, witnessed_step)，无层时行为不变；elevation_valid=0 的节点也能读出 witnessed step 并形成违例。
- 验证：本机/服务器 pointcloud 18、terrain 9、official_adapter 46、cloud_filter 1、mapper 22、planner 40 全通过；Nav2 全量 777 项 6 failures/298 errors，与 candidate6 基线 304 条失败名逐项 diff 为空（既有环境/消息版本问题），新增测试通过。实际 Mapper CDR→DDS→Planner 9 阶段通过：witnessed_step_flat 边 maximum_step_height .17/violation=1/feasible=0，low_surface_occupied clearance=0/violation=1/feasible=0，Planner 均 complete=false；probe 从 /proc/maps 核验 Mapper4a0401b1…/Planner3f404850…。candidate7 源与服务器逐 SHA 一致。
- 084254 冻结22帧重放：scan00–05 旧路径 contact cell=0，candidate7 contact cell=1；116个≥.10m残差回波保留；scan06–21 接触点进入车体 footprint 后外部回波0，说明须在接近前标记。接触邻域 witnessed_step≈.029，未把平滑低表面误判为台阶；早期每帧新增约51–61 occupied格，误报率需现场确认。terrain_layers 30k点约42–48ms，向量化 witnessed 约7ms。
- 真实数据 Mapper 全链回放（ROS域200）：用 candidate7 terrain_layers 从084254的22帧 level 点云重生成 local_terrain，送实际 Mapper；completed=true/22快照/62图消息/errors=[]，见 replay_candidate7_20261019_184118/result.json。scan00–05 接触栅格 traversability=1.0，证明新标记到达 Mapper；**但最近节点距接触约0.71m、minimum_clearance=1.0>robot_minimum_clearance=0.3，physical_feasible=1/violation=0/terrain_observed=0**。图级净空0.3小于SBR半宽1.05/前伸3.4；物理栈Nav2使用 nav2_ackermann_control.yaml 的SBR footprint(3.4/1.05)和inflation1.6/3.75（official点载体的0.15/0.3不是物理车配置），图选路与Smac/costmap的最终契约仍未验证。不能因低表面已标记就宣布整车解决，也不能假设costmap一定挡住；下一优先级实际costmap回放/受控短段并考虑统一graph clearance与车辆几何，不只调大单阈值。
- 受控短段（隔离叠加 candidate7_field，未改正式安装）：早期184534缺beamngpy、184626 Smac plugin/ament索引错指、184820 source_protocol准入时序与verifier校验candidate7 Planner失败，均保留。最终 frame_20261019_185152 用 candidate7 Mapper + 未改6153 Planner/Smac/RPP，admission前等待完整source ACK；runtime_identity passed（Planner db52f00e、Smac fc149c80、RPP d2fd9cee），map_loads=1、protocol_closed=true、errors=[]、component_closed=true；但 `outcome=admission_rejected`、`admission_reason=collision_cell`、goal_sent=false、motion_goals=0、body接触0、wheel3440、max native speed≈.00284m/s、无录像，harness passed=false（execution_passed=false）。录段33点末端第31/32点距接触0.70/0.53m为致命格，接触处global cost99/local cost100，2.5m内161/256致命格，与历史接触位置一致。说明低表面标记已进实际Nav2 global/local costmap并在动车前拒绝该段；**这是安全拒绝，不是成功穿越，也不是整车通过**。下一步测不经过该接触点的路线或完整搜索，确认能绕行而非只停在起点；不得把admission/component_closed当完整成功。
- 未通过/边界：无 FollowPath/Native/视觉/车辆/完整搜索，20轮未验收；candidate7 未部署正式安装；每帧重建与车体遮挡、Nav2 clearing 对 marking 的实际覆盖、真实采集时刻/epoch、Native延迟、整体无进展、记忆容量、动态换挡/制动/换向仍开放。测试通过不等于整车通过，未commit；下一步先修车体尺寸/净空契约与短段准入，再做整车验收。

### 2026-10-19 用户新改动candidate7复核：距离与连续地面候选

- 详 docs/CANDIDATE7_REVIEW_20261019.md，证据 test_artifacts/candidate7_review_20261019。先读取最新candidate7记录和六文件备份，保留改前/中间失败/改后源码；正式安装、6153、原candidate6/7不覆盖，不commit、不录像。当前仍7类核心未闭合事项，见OPEN_ISSUES最新首节，历史旧清单保留。
- 实际复现：单平面离地判定把连续缓弯路4500点中720点当障碍；亚格固定距离将允许坡度连续平面报3.36–3.78cm台阶。后继按极值真实XY距离、同格/邻格证据和输入校验修正；地面参考只沿有实测支持的连续格扩展、不跨缺测、须双轴面支持，保留实体抬高块/缺测物体/旧None分支。两地图链一致传入ground_max_slope，单位rise/run。原型删1个接触邻域返回被拒绝，面支持后早期6帧原候选全保留。
- 本机点云23/地形13/回调替身570过，服务器curved_candidate实际ROS official_adapter51/点云23/cloud_filter1过。22帧冻结回放unknown逐格相同，早期接触格及occupied格数与candidate7一致，最大terrain约60.5ms；实际Mapper回放mapper_review_20261019_214636完成22快照/62消息/errors=[]，terrain SHA c387891dc622d41dc18b24ac946f2b0d56a68370ff4c002e3337079160c9a058。未整车/20轮，不当完整成功。
- 独立静态证据：预检无残留/约140GB，GPU1不动；单次Town真实SBR制动后暂停，零运动目标/导航/模型/传感器栈。按安装引擎既有API采3721点，接触处static比Terrain高.125471776m，比历史接触Z高.003908634m，中心差分坡度约.970 rise/run；车体/四轮原碰撞及1449.72kg保持，测量前后<1mm。首包124591B SHA b5652863c62e6a821f12ae94780e2c32e7234d95a019a0502532e54ba17fcac9，本机21载荷校验过。不能识别具体资产或证明历史变形；更正旧“平滑低表面”，稀疏witnessed_step小不证明真实表面平滑可行。
- 最终合成曲面720→1，剩余点在8m外沿用旧门槛，不称远场已修。地形处理/3D净空/新分割真实误报漏报仍待现场。另核candidate7_field有意用旧6153 Planner，仅能证明该轮costmap准入，不能算最新release/fence/有向图完整栈联通；下一次真实执行前必须冻结全源/安装并核运行身份，不能为通过旧hash校验换回旧实现。

### 2026-10-19 继续车辆几何/消息契约修复（编译待验证）

- 详docs/VEHICLE_PROFILE_CONTRACT_REPAIR_20261019.md；test_artifacts/vehicle_profile_repair_20261019已保存9份改前源/测试。最早违约是物理launch未把footprint几何交给Mapper，默认横向净空.3m，且profile把横向净空误填竖向ground_clearance、未知量发布0；typed边坡度raw rise/run与消息弧度约定不一致。
- 当前候选从实际Nav2 local/global矩形与padding统一推导1.06m必要内切圆净空，launch/Mapper/运行门禁联动；发布后轴非对称footprint，未测高度/底盘净空/动态上限显式-1。保留default profile ID，因为视觉评分/执行反馈也使用它，不能局部改名导致物理评估丢失。内部坡度阈值不变，typed边发布atan。未改物理、速度、曲率、Native。
- 本机runtime27/回调替身570过；新增5项C++回归尚待实际编译结果。服务器/home/ly/vehicle_profile_repair_20261019/candidate从完整415成员父源逐SHA校验后叠加9文件，SSH62011正在顺序/2作业构建Mapper、Planner、两Python包；未启动仿真/目标。父源ZIP49cd7731e124beca49a24e78c69fe738d443c1ba67e88f05b4282859004c3750保持，正式安装/6153不改，不commit。
- 内切圆只证明必要条件，不等于长车体转向扫掠安全；必须保留Smac完整footprint。Mapper类布局变化，fixture/exporter须用新header重编，不能旧二进制混链新库。下步先读构建/实际C++和入口参数/DDS，再验真实短段；完整验收未通过。

### 2026-10-19 几何候选v3全链组件通过；有界真实执行进行中

- 接上节，详VEHICLE_PROFILE_CONTRACT_REPAIR_20261019.md后继记录。v1真实physical入口→launch→Mapper参数→DDS通过；追加ROS服务反例220727证明参数可改.3而内部仍1.06，非导航通过。现启动只读参数（几何/物理、Mapper/Planner profile ID、occupancy订阅选择）防止假配置；v2重编27 Mapper/40 Planner通过，实际221108参数服务拒绝净空/两ID修改，保持1.06/default，正常启动覆盖可用。
- 自助安装器又核出既有遗漏：未选择编译Mapper、库校验无Mapper、Python路径硬编码旧源。已补齐8包/四库/本次bundle依赖路径并更新测试；作用是防止修源后仍混装旧Mapper，不将其推断为全部历史现场的因果。当前14改动文件有改前快照；415完整源逐SHA，实际安装Python/config及四库108项一致，视觉包另行安装。
- 同v3源，真实ROS runtime27/official51/pointcloud23/cloud_filter1/console28、安装器fixture、原事务60/记忆51过；实际目标交接15阶段过，记忆决策5阶段过，Mapper CDR/DDS→Planner9阶段过。初次记忆探针缺少本轮构建身份result.json在0阶段退出已保留，补实际XML/库SHA组件报告后重测，不放松生产。9阶段fixture用新Mapper头重编，不能混ABI。仅组件验证，不计20轮。
- 连续地面后继服务器包88成员/3411918B SHA4f4e63457ae593a0cc974402545d7e5873dfc32aa40c1cdc54d98cbf821ac5b3，本机87载荷核验通过。记录中terrain SHA已按本机文件纠正PTY折行重复字符；不得手工拼接折行hash。
- 当时SSH62011运行单挡短段探针；后继结果见下节。不能将启动失败轮、规划准入拒绝轮或组件通过混为真实执行成功。

### 2026-10-20 继续短段诊断：两轮失败阶段更正

- 20261019 `short_20261019_224809` 在启动阶段失败：诊断脚本使用SDK解释器启动ROS launch，缺少lark。没有进入运动；父进程完成所属进程清理，但该轮不能证明物理停车验收。测试工具已将SDK/ROS launch解释器职责分开，并在加载地图前分别检查依赖。
- 后继 `short_20261019_225630` 启动成功，实际运行Mapper/Planner/Smac/RPP四库身份已核对。逆向3m请求status=4，52点/4.3368m，三挡段[0,32]/[32,45]/[45,51]；正向3m请求status=6。单挡短段准入拒绝，motion_goals=0、SDK平移0、odom位移约0.0000103m、stop_gate_closed=true。不能将此轮称为启动失败、车辆FollowPath失败或完成验收。
- 最新继续预检无项目残留、约114GB磁盘；GPU1外部任务不动。当前先用关闭DB比较直接倒车和已返回路线，在公共地图重放不等于证明搜索时私有costmap版本一致；多余换挡原因尚待核实。服务器证据原路径/home/ly/vehicle_profile_repair_20261019/short_20261019_225630，新增诊断test_artifacts/continuous_goal_repair_20261020。正式安装不替换，无录像、不commit、不计20轮。

### 2026-10-20 连续目标候选：实际C++及一条真实SBR倒车闭环通过

- 详docs/CONTINUOUS_GOAL_REPAIR_20261020.md。225630关闭bag的6全局/7局部公共图中直接倒车3m通过既有检查，且自由地图实际Smac插件复现多余栅格换向。包装层只枚举连续起点→中间栅格→目标，未比较已有可行的完整连续连接；不是物理引擎导致该自由图反例。
- 复用原Reeds-Shepp增加真实起终点完整候选，保留原A*和全部几何/footprint/时限保护；同一当前地图版本比较，几何换挡成本及积分inflation风险均不劣才取代原搜索候选，不靠新阈值放宽。不改变生产50m analytic长度、速度、物理、Native、控制器或目标容差。
- 实际15 planner/4 connector/7 path_contract全通过，旧平滑/预算/拒绝恢复/参数生命周期均保留。初稿风险对照误假定native必绕高代价条带而失败，保留证据；仅该对照测试缩短analytic扩展形成实际低风险替代，断言仍要求保留绕行。原生analytic扩展不是语义不可通行硬约束，第4类未关闭。
- 新416源冻结，/proc核四库；新Smac a0b4453f728e27944dac68587524b51ff59370d102afdd71a57ce14cad286cc6。实际物理short_20261020_164936：map_loads1、motion_goals1、FollowPath status4，单挡约3.000003m规划；SDK平移2.178889m/odom2.181171m，距目标.818844m符合当前1m容差，停车门关闭/13次SDK停车样本、父子errors[]、后检项目进程[]。原723节点/467车身碰撞节点/四轮各32/1449.72kg，非理想载体。
- 仅一条直线倒车组件闭环；GraphPathBridge动作所有者被隔离，模型/目标搜索未运行。弧线、换挡、多段交接、完整视觉到达、记忆收益、20轮≥16成功仍未通过；7类问题未全部关闭。不录像、不正式替换、不commit。
- 证据已本机封存核验：profile_component_evidence.zip 3168431B/625载荷 SHAe234841f6f5305a7c64f8ca4591642be0f9c493d0ac7f1d796a776c27a58a2ff8；continuous_goal_evidence.zip 6006928B/551载荷 SHAe96756302a923e7761811ee14fdb49a98dbe6dc60b4be8cdab5fe24126a87cff5。CRC、全部载荷、26项实际C++ XML及416冻结源与本机一致。原bag留服务器含SHA。42运动采样最大横偏.004257m/短时Native RPC P95约8.46ms，不关闭历史延迟。

### 2026-10-20 持续缺失frontier退役与动作交接（F12组件/实际协议通过）

- 详docs/FRONTIER_ABSENCE_REPAIR_20261020.md；本机test_artifacts/frontier_absence_repair_20261020，服务器同名目录。先核前次416冻结源、保存9文件改前字节。实际旧C++42项2失败证实frontier_resolved(nullopt)无限保留；新增退役原因在旧桥上的前瞻联动反例122项8失败/1错误，不能据此声称旧Planner已发送该原因。首次构建验证PATH缺colcon保留，修脚本后再复现生产反例。
- 按同frame/来源戳更新/非未来/未过age的图证据累计持续缺失，来源间隔超过新鲜度窗口重新计，重复/旧/未来包不推进，节点回来重置。默认只读missing_frontier_timeout_seconds=10，实际launch及runtime门禁同步；存在但属性/位置不可分类保守持有。不是墙钟TTL取消车辆，也不是证明传感器真实采集/物体永久消失；epoch第6类仍开放。
- 新frontier_missing仅退役探索意图，不当观察成功、不写visited/地形失败；桥复用frontier_resolved的既有动作租约、后台意图取消及恢复保护，查询切换仍撤销旧动作。健康动作与队列尾部所有权分开；未改Smac、速度、Native、物理或记忆学习。
- 本机590既有回调替身/本批123通过；实际C++43/真实ROS类型123回调通过。DDS首次171235在4阶段因默认禁用桥没有ActionClient退出，未接受动作，保存失败；仅修夹具显式创建测试客户端，未迁就修改生产。
- dds_20261020_171520实际Planner→DDS→Bridge→已接受FollowPath句柄6阶段通过：临时缺失/来源断档/重复包均保留原意图，持续缺失只退役一次并选B，旧动作取消0；查询切换后取消1、终态5(CANCELED)，旧退役消息被fence。C++运行库782102b16baa528ce5de76915d87acd9aa76151da385b857171c9e252129b470。输入/已接受路径/健康状态为合成，动作服务器不驱动车辆，不是全局局部规划/整车搜索验收。
- entry_20261020_171927实际physical入口图栈验证default profile/10s窗口、108安装源和四库核对通过。末次进程清单[]，无BeamNG/录像、正式安装不改、不commit。F11局部成功但整体无探索收益仍未解决；7类未全部闭合。
- 封闭证据已回本机：frontier_absence_repair_20261020/sealed/evidence.zip，8740523B/480载荷，SHA21f39276a22e1d0557e6ab04c2e57714807133dea2eea665c2d74faae9614822；CRC、全载荷、416冻结运行源/本机、9改前源和通过/失败XML及DDS/入口JSON核验通过。local_verification.json记录验证边界，verified/为紧凑证据；SSH会话23712随后退出，无活跃测试可续读。

### 2026-10-20 全局进展先查真实证据；当前416版本完整链路试验启动

- 详docs/EXPLORATION_PROGRESS_INVESTIGATION_20261020.md。前轮属于有证据进展；目标保持全链路/7类开放事项，不把协议通过当完整成功。F11先查Oct18完整失败原DB：681条路线均complete=true、route_done0，否定“该轮partial前缀已到达反复等待”初步怀疑；17maneuver_done/4segment_done中前半程换向后仍推进，不能笼统用距离单调/时限看门狗阻止合法绕行。398graph_reset_failed与已修预算故障混合，旧局部成功反例不证明全部现场成因。
- 无新生产修改；新增只读闭包审计及完整试验脚本，来源上一轮416冻结源，全部默认同进程记忆启用、原真实SBR/同Town历史左转90°朝向、绿色居民楼420s。显示和低频只读观测、原Native时序观测，不录MP4；核模型实际源/内存库/四库身份后解锁。
- 初次full/trial_20261020_174256已终止，goals0、控制未解锁：新增模型身份观测直接JSON编码sqlite3.Row失败，原模型已经初始化。保留失败日志，不认成识别/规划失败。仅修诊断序列化并在本机/服务器用真实SQLite行3项测试；后继复用同416源。先核driver1922368已无/proc、result.closed=true及项目进程[]后重试；setsid父1922367结束本身没有被当作试验终止。SSH25067可读，下一步须重核真实新driver/console和日志，不因观察超时重启、不读活动DB；未计20轮、不commit，7类仍未闭合。

### 2026-10-20 完整搜索仍超时；实际图捕获定位到物理隔离起点

- 详`EXPLORATION_PROGRESS_INVESTIGATION_20261020.md`后继。同416源完整`trial_20261020_174700`源/四库/模型身份通过、默认记忆开、真实SBR，420s搜索最终421.07s超时。约56.64s图路线不可用退役，58.47s后不再下发动作；driver.closed=true/session errors=[]/真实停车及独立后检进程[]。失败RGB已取回本机同名证据目录，不是到达证据；无视频，不计20轮，不commit。
- 未改生产，新增只读完整图观测，有界150s `trial_20261020_180816`超时后清理完成/无项目进程。保存131组评分、101精确匹配raw/scored/odom，观测错误0；物理准入镜像不含历史拒绝/backoff/实际接收时序，实际C++仍待复核。首个隔离反例source1792491005743982662：图当前点(730.2374,-12.6775)距odom约1.825m，拓扑可达124/物理可达1；minimum_clearance=1.0<profile1.06，置信度.8/明确违例，12条关联边拒绝。后续持续隔离，末帧仍有19候选但物理不可达。不是仅凭可通行heading数归为视觉模型问题。
- Mapper几何最近切换当前点未证明实际位置连接/物理可行，源码存在visited/traversed未经完整几何连接的接口缺口；但真实1m净空来源、准确起点与terrain/全局costmap对应关系尚待核实，不能宣布唯一根因或直接绕过危险。下一步实际C++反例与封闭terrain对照。SSH25067因交互shell继承set-e后ps目标已退出而关闭，非网络故障；现88255可读，读取配置后已set+e，不误重启已结束车辆。7类和完整验收仍开放。

### 2026-10-20 图起点代理错误已复现；最小接入修复组件通过，现场验证中

- 详`GRAPH_START_ATTACHMENT_REPAIR_20261020.md`。同隔离快照实际当前C++ Planner重放：原起点30个frontier均空；部分相邻起点30均非空/21 complete，另一个近点仍全空，不能直接替换任意节点上线。terrain与严格同戳TF证明旁侧图点距occupied格中心约.961m，而实际车位距同格约2.758m；末帧.963/2.761。不是完整footprint或世界障碍身份验证，不再仅猜“图片与物理冲突”。
- Mapper新增候选接入必要条件：通行类存在、有限free_radius覆盖`距离+robot_minimum_clearance`，有效置信度下明确物理违例不接入。保留未知探索、已有边/危险保护和Nav2完整footprint，不改车辆/速度/Native/记忆。只改实现和相关测试两文件，改前字节本机保存，416源冻结。现有当前节点后来新判危险的恢复及完整转向扫掠仍开放。
- 实际旧Mapper30项28过/2新增拒绝反例失败，新Mapper30全过；实际库c43576750cd8caee407c7b6d073fa85e0a46f54ba7d6256bd1b8c1efc2038645。服务器新候选/home/ly/graph_start_attachment_repair_20261020，正式/旧安装不替换。完整420s绿色居民楼复测driver2121093已启动（需以实际/proc再核），逐源/四库及模型身份后解锁、真实SBR/默认记忆/同Town朝向、只读图观测/不录视频。必要验收未通过不commit，不计20轮，整体目标保持。
- 后继状态更正：driver2121093对应214730已终止，Town场景请求超时、goals0，未进入规划，不认成新Mapper失败；没有停车采样，独立后检项目进程[]。同源同配置重试215010/driver2122937：场景/四库/源/模型通过，搜索已越过旧停驶区，中途graph物理可达约199节点、车辆约52m净位移并开始视觉候选靠近；这只是进行中证据，最终结果/目标正确性/停车仍待核。根因封闭证据已回本机ZIP16449674B/553载荷SHA9795dd1f59dc5bd70d8b4660922e689c054582aba79b506a6f2728d6846b36df，CRC/全SHA/416改前源核验通过。
- 215010终态：420.87s超时/完整到达false，真实odom净87.584m/累计145.884m，171组精确图快照0起点隔离；不是整体通过。约341/366s两次RPP有界控制弧预测碰撞中止，随后旧确认接近点8候选7empty_path/1footprint_collision；实例不匹配拒绝及确认更新延后并存，尚未分清不同建筑/同建筑立面与目标可达姿态/交接问题，不关闭碰撞或放宽实例门禁。最终SDK停车证据用goal_001/stop.json（根stop为启动采样），6样本、末速.000144m/s/原碰撞保留，driver/session errors[]、后检项目进程[]。
- 本机修复证据ZIP60492616B/1350载荷SHAa3b2e1231f361a4ee3b38e2cc804245d889791b0d0d168e3e751b0e75f0310c0已核CRC/逐SHA/416源/30旧2失败与新30过/四库/171图/真实停车；详同修复文档与local_verification.json。必要完整验收失败不commit、不计20轮。
- 用户随后明确要求重跑录像。220547来源命令ACK启动门禁未通过、未解锁/goals0、无录像；220930导航预检过但新录像工具把两个Qt子面板误认成主窗口，发目标前退出/goals0，仅修工具并加3项真实窗口树/子面板/歧义选择测试，本机服务器通过。后继仍同导航源/物理/预算，须核真实录制进程/预览/容器和本机文件才交付；不把之前无视频轮伪装成有录像。新录像授权仅本次，不自动恢复每轮录像默认。
- 221250录像轮已终止：控制租约过期/closing导致Native actuator unhealthy>.4s，搜索44.11s安全中止；与上一轮215010的420s规划/接近失败分开。driver.closed=true/session保留错误，后检项目进程[]。实际RViz MP447.6s/1920×1080/10fps/476帧已取回`captures/20261020_green_house_rviz_trial221250/rviz_live.mp4`，SHAbf3e172100ac182e7e61f053c5ba92875127d488837a4547de1ecc5b5f60da1f；容器/帧读取/布局/右侧图像变化已核，不是完整轮成功或上一轮7分钟视频。原Native时序/UDP/关闭bag在该轮服务器目录，后继查租约/SDK时延/归属，不先放宽安全保护。完整验收及7类仍开放，不commit。

### 2026-10-20 用户指定优先修复接近失败交接；组件通过，整车仍未验收

- 用户明确暂不修 Native，优先修复旧接近点反复重试/替代观察/恢复探索。详 docs/CONFIRMED_APPROACH_RECOVERY_20261020.md，证据 test_artifacts/confirmed_approach_recovery_20261020；服务器独立 v2=/home/ly/confirmed_approach_recovery_20261020/v2。原安装、物理/车速、Smac/RPP 参数、检测模型/实例阈值/到达标准均保持；不 commit。
- 旧生产 9 个回调反例全部失败：显式目标候选耗尽/预算耗尽没有自己的失败出口，不能调用 frontier 拒绝清理；失败后也到不了只在到点后消费的 deferred 观察。修复五个生产文件 confirmed_approach/target_observation/graph_path_bridge/visible_approach/goal_standoff：失败基线暂缓，同实例非径向观察可在失败后试；仍逐级做真实车体终点、GraphNav、Smac/本地路径检查；至多4个替代派发，耗尽后经旧动作终态/持续停稳和实例 release ACK、GraphNav release_goal 恢复探索。10秒无图路线出口、物理恢复归属保护保留，不将失败写成目标不存在。
- 追加上游反例证实释放后可重新锁同一区域；新增可选 navigation_failed 请求字段，仅释放匹配当前实例时施加60秒同查询导航退避，最多16区域，重复请求不续期、查询切换清除。Bridge与producer须一起部署。历史记忆候选仍由 RecallCandidatePolicy 管理，不声称所有 ConfirmedViewpointCache 失败分支已闭合。
- 当前18新增/194相关/608全部离线回调通过；服务器136真实ROS接口回调通过。初次192项扩展ROS运行11失败/3错误为旧夹具SimpleNamespace/Mock接口不兼容，日志保留；相关真实类型套件独立选择，旧全部断言仍参加本机608回归。DDS真实C++/Bridge/GoalStandoff/两个action server13阶段通过：旧点失败→更远同实例替代→健康动作保持→三次执行失败→替代耗尽→release ACK→frontier40m。输入/路径/action结果合成，无Smac/RPP/车，不当整车通过。
- 215010关闭原始输入重建交接状态：旧保留741.716,-99.280，新在现有footprint下无可派发替代则进入release_pending。不是原私有异步队列完整重放，不证明视觉类别正确。首诊断误要求存在瞬时object_standoff状态而失败，保留日志；实际waypoint先占位时可直接锁定而不发该状态，后继从原确认重建anchor并说明范围，没有捏造缺失消息。
- 本轮整车224538：冻结417源/5生产变更，108安装来源/四库核验，真实SBR/同Town/默认同进程记忆；421.88秒超时、累计166.95m/净42.27m，4实际FollowPath aborted及多次>5s停驶，非稳定成功。query已收到并编码，模型持续处理但mask/localization/confirmation均0，因此未触发已确认目标接近恢复；不误诊为query丢失，也不能说场景无目标或模型一定错。原车停车末速.006906m/s、连续样本<.03、driver/session errors=[]、后检进程[]；未复现上轮Native租约致命退出，但不关闭历史风险。
- 本机封闭证据ZIP7299866B/510载荷/571引用，SHA7f38b123092d8544bc996ee079c8c8396b5e9d922809a6fdf7ad7dba7309144c；CRC/全SHA/417本机源/旧新重放/测试/108来源/原车停车核验通过；local_verification.json明确complete_navigation_passed=false。完整视觉到达、记忆收益、20轮仍未通过。
- 用户随后要求再跑一次并录视频（本次授权）：225836场景加载SDK超时，map_loads0/goals0/无录像，已退出/无残留。真实窗口选择3项通过，原导航源/物理/参数不变重试225946；须待本轮实际录像关闭、容器/画面/哈希和本机文件核验后交付。不得用221250或224538冒充新录像。

### 2026-10-20 本轮新版本RViz录像225946已交付准备；重复换向及来源保护仍开放

- 用户要求再跑并录视频。225836场景SDK超时未加载/未发目标/无视频；确认退出无残留后同417冻结源/参数重试225946。目标420.86秒超时，driver/session errors=[]，已停稳退出、后检进程[]。未进入mask/localization/confirmation（全部0），不能当confirmed approach修复现场通过。
- 实际RViz录像426.1秒/1920×1080/10fps/4261帧已回本机 captures/20261020_confirmed_recovery_trial225946/rviz_live.mp4，265970877B SHA f76e5f98b3c0e14cd48722d4643b68e129aa393716c704f45d86f8399be58ff6。TAR268349440B SHA cc17ae521e95fe9e22200e60c703eff381ba248bd5384d4cc4859cb109c576bf，18载荷SHA及本机容器/抽样解码通过；5/60/120/421.1秒右camera图样差54.98/51.72/42.88，非整段固定图，不宣称逐帧时钟验证。
- 用户质疑是不是旧版本：运行中实际PID2174791 GraphPathBridge和2174771 GoalStandoff入口均指向本轮v2/install；5生产源当前/冻结/启动108来源检查SHA全匹配，主桥 f8bf0a8da4be4c1a94469382305d2b617dcbe163c5b7e9d80c6d50bd031dae20。live_version_check.json随视频保存在本机，不靠本机工作区推断服务器运行来源。
- 录像轮累计72.14m/净14.42m/max离站16.39m，同一frontier有2～8运动段，探索执行仍反复换向。原关闭bag明确9次换向后续接重规划，5 footprint_collision/4 collision_cell；约.117m位置误差/很小朝向误差样本仍触发当前地图检查。须核实际被拒段、两图及地图变化，不用“真实车辆必然需要换向”或“物理直接撞了”带过。
- 同轮还出现控制来源保护：去重状态戳后source_protocol77/source_expired32/source_owner_unconfirmed8条状态，不等于117独立故障；执行反馈source_expired1/source_protocol5、恢复8。会暂停撤销再重算，根源须按原始receipt/ACK/过期时间核验，不凭标签说旧版/只归因规划器；与前轮Native Lua租约致命退出分开。原DB/Native时序及execution_replan_review.json留服务器轮次。本次Native未改、无后续新整轮、无commit，20轮稳定性/完整到达和记忆收益仍未通过。

### 2026-10-20 用户指出录像车辆没动：SDK 原始位置复核，撤回累计里程代表有效运行的说法

- 详 docs/PHYSICAL_MOTION_REVIEW_20261020.md；仅只读复核关闭225946原bag，SHA201b4af5b906ffa1cd499560aab7f3775f66f53d538a4f0968671f127c480f227，与原交付清单一致；无生产改动、新仿真、录像或commit。运行源核验仍有效，不是旧版已被证实。
- 8452 Native样本、有效观测422.605s中近乎静止(speed<.1)310.426s/73.46%，其中实际制动292.391s；第5分钟末距出生点仅1.712m，终点14.503m/max16.389m。视频3:14～4:05实际连续停51.249s，首末位移5.721mm。累计约72m包含往返，不能当有效探索；camera抽样像素变化也不证明车辆实际推进。
- 近乎静止时Control stale_command214.152s/gear_change49.332s；Plan planning164.421s/following75.396s、来源保护约50.021s；公开状态异步且不能据此断言Smac计算耗时或把所有stale归网络。9次换向续接拒绝仍需实际后续队列/地图证据；原bag未录私有motion_queue，不伪造精确历史重放。
- 发布命令间隔P99 52.530ms/max66.916ms，源到录包P99 3.799ms；配对Native receipt/command_receipt_age推算回调延迟P99 293.254ms/max450.580ms，首次应用源年龄max497.970ms接近500ms源期限。尚未区分DDS/调度/锁等待，缺receipt序号不是丢包证明。Native RPC P95 18.510ms/max134.965ms，actuator坏/车辆Lua租约触发样本均0；不拿此关闭历史风险。source_protocol可因receipt=None产生，不能解释成旧协议版本。
- 本机test_artifacts/physical_motion_audit_20261020封包570560B，SHAc49038d22543d476cdbccd20c773e180a4008a0608b9f58354523f2f57a4b22d；归档及载荷SHA、原bag身份、时钟接收差(.124～7.397ms)、分段/总体积分一致性、源生命周期整数范围、单发布者验证通过。后续先核最早续接拒绝的真实输入和Native接收时序，不盲目放宽保护；完整验收仍失败，7类及20轮未关闭。

### 2026-10-20 用户要求保留进度，暂停至明天

- 当前停在225946录像的真实运动复核之后；诊断及逐载荷核验已保存本机，详 PHYSICAL_MOTION_REVIEW_20261020.md。本轮未新改生产代码、未启动新测试、未commit；confirmed recovery v2的417冻结源和既有组件通过记录保留，完整整车搜索仍失败。
- 明天先读本节及上述诊断，再核工作区/服务器运行来源和残留进程；先补真实换向后被拒队列、对应地图/姿态/epoch/action身份，查最早续接检查失败，再分开核Native命令接收延迟。原关闭bag缺私有后续motion_queue，禁止把重建当精确历史重放。
- 车辆73.46%有效时段近乎静止，不能再用约72米累计往返里程或camera像素变化作为探索正常证据。已有约292秒实际制动；stale_command包含上游等待，不能统一叫网络慢；SDK RPC样本正常也不证明回调链正常。
- 后续仍按最小可回退、上下游契约、正常/失败/恢复/交接验证推进；不先放宽源期限/碰撞检查，不盲目重跑420秒搜索。用户本次明确暂停，等待其恢复后继续。
- 暂停前只读预检wall1792510027.578661，相关项目进程[]、display148无锁；结果存服务器motion_audit/pause_process_check.json和本机physical_motion_audit_20261020/pause_process_check.json。SSH会话88255已退出；其他GPU任务未操作。

### 2026-10-21 用户恢复检查：对照87米轮，区分探索交接与Native接收迟滞

- 用户指出有420多秒净位移80多米的轮次，已确认215010。详 docs/PHYSICAL_TRIAL_COMPARISON_20261021.md；三轮关闭bag只读对照215010/224538/225946，原bag SHA均匹配旧证据。此次仅诊断/记录，无生产、参数、物理、QoS改动，无新仿真/录像/commit；用户“继续检查”覆盖前晚暂停。
- 三轮同真实SBR/出生朝向/绿色居民楼/预算，实际Mapper c43576750c、Planner782102b16b、Smaca0b4453f72、RPPd2fd9cee5e相同，Native/source/速度/continuous_path/motion_path/graph_route及模型源相同。前416→后417冻结源只有5生产+测试/入口7项差异，后两轮108安装源完全一致；本机当前417源核验一致。但采样图/首frontier/执行时序不同，不当逐帧相同A/B，也不凭一次录像定因果。
- 整轮odom净87.58/42.27/14.42m；统一query后420秒SDK有效观测420/418.179/420s，近乎静止54.50/49.73/73.35%，前90秒距起点33.74/24.32/3.84m。87米轮也非稳定；新版未录像仍可运动，不是升级后必然不动。224538缺1.821s显式列缺测；同窗SDK35.54m与终态odom42.27m是时窗/参考点不同，最终SDK停车位置697.524,-24.218，不先误诊坐标漂移。
- 首次控制段长度9.84/9.91/1.21m、运动段1/1/3；225946早期短距离往返/意图交接早于88.70s首次source_expired，63.69s已有换向续接拒绝。两图/图路/第一控制段CDR已保存并核对应关系；私有完整motion_queue仍缺，不能精确重放未录几何。整体进展须查候选姿态/完整车路/退役时机，不放宽碰撞或仅延长租约。
- 本次原始UDP/DDS/ready/take回调配对确证Native迟滞层：215010/224538回调P99 14.46/18.63ms，225946达450.83ms/max459.68ms；后者首内核到达P99 .42ms、received→ready P99 449.90ms、ready→take P99 12.50ms、回调本身P99 .30ms。8589唯一指令对应60598数据报，P95每序号48份，其他轮基本1份；不是网络到内核慢或450ms SDK调用。sequence4487原始边界保留。缩小到Native进程DDS接收可用性/唤醒至订阅被发现就绪前，具体reader/reliability状态及触发仍未闭合；不擅改QoS或500ms源期限，保留18日400ms暂停未复现简单可靠历史积压的反证。
- 后两轮mask/localization/confirmation均0且新增接近失败分支未进入，不能拿其作停车原因或现场通过；215010首次mask114.87s、定位169.30/确认320.73s，最终接近失败。失败阶段不同，不能归一或宣布记忆原理失败。诊断首版阶段名/缺测/旧trace清单范围错误均留存，更正后6证据校验通过，不当修复/整车成功。
- 本机test_artifacts/motion_comparison_20261021/comparison_evidence.zip 3041873B/91载荷，SHA67c356ccccc5ce25596b8607ac2ffbefecd9b9b4abd7d9886f04e6d08e24635d；CRC/逐载荷SHA/实际版本/原始时序与积分校验通过。225946 trace首次在本批哈希，与其原bag旧SHA验证分别说明。服务器/home/ly/motion_comparison_20261021；预后检无项目进程/无display锁，磁盘约25GB，其余GPU任务不动。下一步组件级核完整路径交接和DDS接收就绪，不盲目新整轮，不计20轮，不commit。
- 用户追问三轮共同长静止：新增同Native静止间隔control/plan交叉统计，stale_command119.92/93.29/214.15s、gear_change71.87/66.10/49.33s，合占静止83.79/76.65/85.53%；其中同时stale+planning75.95/74.63/159.45s，不与类别总量再相加。当前代码候选串行尝试/无有效速度需求时制动；实际候选失败耗时中位1.745/1.737/1.641s（非CPU耗时），持续失败累积停车。
- 默认换挡确认2.0s，实际physical launch未覆盖；记录36/34/33段gear_change，中位2.047/2.008/2.000s，整段含减速所以不能乘次数当纯静止。频繁短段/换向叠加等待是共同开销；87米轮也不稳定。旧轮另有约29.34s同时stale+instance_mismatch状态。Native独立迟滞会放大最新轮，不能解释前两轮所有停车，也不能只修Native就宣称连续性解决。候选为何频繁不可达/换向必要性仍待完整路径与地图闭包，不删保护或直接改物理。
- 追加证据shared_stop_evidence.tar 61440B，SHAbb22c39a9d9032856b4de547998482d6cecd58bfe03097f31d32efbca87360a8；三轮交叉积分/类别守恒、原bag身份/段数核验通过，详同对照文档追加节。首次未source ROS的只读import退出，后用独立bash加载ROS运行，无生产/现场动作；本次无新仿真、生产改动或commit。

### 2026-10-21 用户授权修复：胜出车辆路径交接与严格坐标类型

- 详 docs/EXECUTION_CONTINUITY_REPAIR_20261021.md。原 frontier 比较丢弃胜出完整路径，GraphNav 提交后再次规划；现在只重用匹配更新的已提交选择，核 query/epoch/UUID/目标/朝向/位姿/地图和传输，并经过原执行检查，失败回退。额外修正派发失败仍引用旧 frontier 候选的归属边界。不降低换挡/碰撞/控制源保护。
- 真实 C++ GraphNav DDS 探针暴露既有 JSON int→ROS float 严格类型异常，在 try_candidate 统一 float 转换；保留两条修复前失败反例，不修改上游夹具掩盖。当前 623 本机回调替身通过；第一候选 26 实际ROS类型通过而 DDS 失败，第二418成员候选待实际链验证。不能当整车解决。
- 证据 test_artifacts/execution_continuity_repair_20261021，服务器同名目录；第一封包/失败 DDS 保留。第二ZIP SHAb6dea302e5ebaca8cb64cc093314bbdd7c7fc776dcb75737f4ca0c621d43d1ed。Native 进程内单序号丢弃650ms对照未复现历史450ms旧回调，不改生产QoS，不声称根因闭合。整车/完整搜索/20轮待验，不commit、无新录像。
- 后继实际C++消息链第二候选仍发第3次ComputePath：GraphNav先发vehicle_cost_selection退役旧意图，通用reset清掉胜出路径。新增真实顺序/重复退役反例两条先失败，修为仅匹配未过期且无动作/查询/记忆干扰的pending choice跨这一次退役保存，并绑定reset后的epoch；仍须新提交路线授权。其他退役/旧戳/过期/活动动作/查询切换均不保留或不执行。测试夹具曾只手动置query_reset_pending后要求退役清cache而失败；实际协议会先清路，改用真实on_query介入退役与提交之间的序列，无放宽生产。
- 第三候选418成员/4增量ZIP SHAe19f01f7caee12e08cd940c3fa5c24306e82d5f0324472c2b670dfd339123559，本机630回调替身通过，实际ROS类型35通过；真实C++→DDS→Bridge→FollowPath三阶段通过，两候选只发2次ComputePath、1动作、取消0，旧退役不再强制重算。合成地图/路径/里程计，不当Smac或车辆通过。v1/v2失败证据保留，服务器v3隔离安装。
- 进入120秒有界整车诊断，无录像；增加只读motion_trace采集实际完整路径、私有后续队列、当前两图/姿态/epoch/action serial。该观察器先通过同真实DDS组件；Native原生产QoS/期限/制动/物理不改。预检无项目残留/display锁，约25GB，其余GPU作业不动。需等实际关闭结果与采集身份核验，不先宣布连续性已解决。
- v3真实短测084227在120.84秒预算超时，模型目标未完成，driver/session errors=[]，已停稳退出、后检进程[]。Native样本有效123.00s、近停72.77s(59.16%)、实际制动近停60.15s、首末净21.95m/max22.52m；包含少量查询边界时间，不与旧420秒百分比当控制变量改善结论。胜出路径真实复用1次，13动作/11换向结束/3次换向后重算；整车连续性未通过。
- 新实际队列CDR闭包复现3次collision_cell拒绝，第一处距车14.19/15.23m栅格当前99、前一派发图2/3；当前局部2m检查通过。不是缺路径、旧动作回调或单纯.15m起点偏差，也不能因局部通畅关整段检查。是否新测真实障碍、膨胀或清图变化尚需地图观测源复核；全局与执行风险仍开放。证据v3/full/trial_20261021_084227及admission_replay.json。
- Native独立改进有控制反例：进程内仅丢序号25及其650ms内重传，原RELIABLE三轮新回调空档550.26/549.84/557.97ms；诊断BEST_EFFORT三轮99.86/113.44/94.35ms。历史450ms旧命令回调未在此复现，故仅证明可靠顺序交付在丢包条件下阻塞新需求，不当历史完整触发根因。
- v4生产仅Native命令订阅变为BEST_EFFORT/KEEP_LAST1，原发布器兼容；源序号/500ms绝对有效期/ACK制动屏障/新generation/停车租约均不变，丢失屏障不许drive重启。新增丢中间需求与丢generation屏障两条回归，本机相关72/全部回调630过；实际ROS70+13过。初次合并运行13项缺rclpy.init失败已保留，按测试类生命周期分组显式初始化后全部原断言通过。
- v4无诊断QoS覆盖的实际Native/DDS+模拟BeamNG I/O三轮丢包回归通过，actual订阅depth1/reliability2、实际安装SHA794dfb692d6129462bf6b152a298e55b04a9f4b10bf290bd74dc330df76111a6；最大回调空档100.06/106.51/99.87ms，零油门，cleanup errors=[]。C++选择交接DDS三阶段再过。418冻结源/6增量 ZIP SHAcf2bc81043f26fd7cf0017da0faaf249f5b662e7bc6e1fc6e9e66b38a5ed8b3a。独立v4开始同120秒真实车体验证，尚待关闭/核验，未commit、无录像。
- v4同版本重试085212正常启动并运行122.01秒至短测预算超时，未到绿色居民楼，driver/session errors=[]、停车门关闭、真实SBR723节点/467车身碰撞节点/四轮各32/1449.72kg保留，后检项目进程[]。统一query后120秒原bag：有效120.0秒、近停43.0446%、净37.2645m，Native源→回调P99 20.8218ms/max48.5946ms、source_rejection全空、actuator_bad0、采集错误0。不是相同图采样/初始frontier的A/B，不能把与v3的位移差全归新QoS。
- v4真实复用胜出路径3次、motion_trace证实14次派发请求；following状态84条不等于84个动作，严禁拿状态记录数冒充动作次数。v3 13派发/1复用。v3原bag统一120秒统计更正为近停58.2583%、净21.9455m、源回调P99 12.7395ms/max28.0529ms，无source_rejection；前述59.16%是123秒文件样本窗口，均保留各自范围。两新轮未复现历史450ms Native旧回调，受控丢包只证明所修可靠顺序阻塞场景。
- 遗留：v4 2次换向拒绝(collision_cell/footprint_collision)、native_telemetry_stale保护、handoff_actual_pose_cannot_join、cusp_position_outside_tolerance仍出现；并非因此关闭保护。原Native样本没有telemetry_ok=false/age>.5，不足以断言Bridge接收端或executor具体根因；execution_transport_issue还累计接收龄，须补消费者时序。新原队列/两图重放保留v4/replay.log。完整视觉到达、连续性、20轮仍未通过，不commit、不录像。当前隔离验证版本v4，正式安装不替换。
- 本轮证据已取回本机并核验：execution_evidence.zip，20909609B/952载荷，SHA183f9de737fced3fac2a2a33bca5510479ac785c92aa85a1ac5526ffb48c917e；CRC/逐载荷SHA/418当前源/两轮108安装源与四库身份/83实际ROS回归/3轮无覆盖丢包回归/两轮原bag120秒积分和停车后检均通过，local_verification.json明确complete_navigation_passed=false。无新录像，无Git提交；v4保留供后续处理动态地图与交接/健康消息消费时序，未把保护关闭。
- 本机追加运行身份验证器初版把verified_sources中的4个绝对库路径当源相对路径而KeyError，保留verification_schema_failure.log；修为逐一匹配实际/proc库路径+SHA，其他安装源匹配冻结成员，原条件全部通过。不是生产/现场新故障，也未降低断言。SSH已退出，无活动测试。

### 2026-10-21 用户明确授权本次阶段性 Git 提交

- 用户在完整验收未通过的说明后明确要求“可以先提交一次git”。本次作为一次性授权的阶段快照，不永久更改“必要验证全部通过后再提交”的规则，不将提交等同稳定性或完整到达通过。
- 以v4已核验418项源/依赖保存可追踪运行版本，附本轮实现、回归、来源清单、证据摘要及未解决项。只提交这些明确路径；已有其余7项暂存内容保留，不提交当前20MB原始证据包及录像。
- 现场边界仍是120秒诊断超时、43.04%近停、净37.26m；动态地图后路径重选、换向与交接/健康消息消费时序未闭合，20轮验收未执行。本次无新增服务器测试或部署。
- 已按用户本次授权完成阶段提交 affb478704b13a4190542bddedb8a6a78bdd444c（411个实际变更文件；418源码清单有部分原已入库），提交后核验源与证据不变，7项无关暂存条目原样保留。用户随后要求继续确认剩余长静止原因，后续先按停车时间段和最早违约点排查，不把checkpoint当完整成功。

### 2026-10-21 阶段提交后继续确认剩余停车：已复现新意图交接丢失，待修

- 用户要求提交后再确定剩余长停原因；详 docs/RESIDUAL_STOP_CAUSES_20261021.md。本次只读关闭bag与源码、增加诊断和预期失败反例，无生产修改、新仿真、录像、部署或新commit。阶段快照affb478保持。
- 原bag按Native/Control/Plan来源时间戳联合积分：v4静止51.6535s，其中stale_command28.9907、gear_change11.8354、tracking4.2126、ackermann_stop3.4869、execution_hold2.0774、unknown/disabled1.0504秒；stale同时planning18.4151、exploration_retired5.0343、execution_ready3.2540秒。不同表不可相加，状态关联不当Smac CPU或网络耗时。6次换挡整段约2秒，含减速；确认等待是必要保护但换向次数仍可优化。
- 明确现场链：98.8158s开始旧owner:10的4候选比较；100.0442s C++因frontier_resolved退役；100.0558s新owner:11的route_status和4点Path已记录；100.1481s桥仍发旧owner选择、C++无ACK；100.1504s桥旧owner退役reset；同新owner直到104.9952s再次发Path、104.9999s恢复规划，空等4.8495s。已排除“C++必须等新图戳才给fresh route”的初猜，源码路径本来使用own now。
- 两条生产回调反例已复现(2fail/0error)：新owner路线在choice等待期间被on_route提前忽略，或比较pending期间先存latest_route后被旧owner reset清掉。旧选择与新意图缓冲没有分开，导致等待下一次图发布。原bag无桥端回调入口/出口，不能声称百分百还原订阅处理顺序；源码/反例与现场新Path已发但等重发吻合。测试 counterexample_route_retirement.py、residual_owner_counterexamples.log；待修，不算回归通过。
- 其他长停：首次0–8.9s含启用/握手及4串行候选，两个empty_path约各1.7s；48.45–54.70s、68.76–74.25s也有串行规划/恢复等待。新地图使旧尾段99/100的两次安全拒绝仍保留，不删碰撞检查。下一步先保留新owner替代路线、只撤销旧owner，停车屏障后重新准入；再用现有评分和可证明成本界限减少无效比较，不能第一条可行就跳过语义收益。
- 41.8493s native_telemetry_stale重复两份算一次事件，已有最新Native源龄.0897s、加采样总龄.3935s、telemetry_ok=true；消费端缓存/调度戳缺失，不能直接说SDK或Native命令DDS又450ms。需补Bridge消费缓存与长回调时间。hold全部仅2.077s，不能当全部停车主因。
- 新只读证据residual_evidence.zip 21462B/12载荷 SHA03435d2bf2eca3670e8600f4837db9995032fb699d66feaa07b3ea1f593dd957。CRC/全SHA/两bag旧身份/积分守恒及与旧120s汇总一致、2个预期失败已本机核验，residual_verification.json明确production_changed=false/fixed=false。首次加载诊断环境的shell引号错误未进入分析，后复用现有环境完成；非生产新故障。原运行组件/完整验收边界不变。
- 用户随后要求修复并重跑。隔离候选v5仅改GraphPathBridge的新owner替代路线缓存/退役交接，保留动作终态、停车屏障及地图、姿态、新鲜度门槛；636离线回调、定向30项、服务器实际ROS76项均过，安装源码SHA已核对，但未替换正式部署。先一次Town Scene RPC启动超时且未开始查询，清理检查无残留后重跑。
- v5现场重试 `trial_20261021_093254` 搜索 green residential house 120.953s超时，未到达；关闭正常，结束后项目进程为空，原SBR真实车体/碰撞保持。候选选择后约.160s启动新路线，frontier_resolved后约.64s越过原有动作终态/停车屏障并启动新owner规划；本轮未再观察到旧handoff丢新路线、等重复GraphNav发布约4.85s的情况，仅证明交接等待缩短，不代表完整流程通过。
- v5近停74.772s/120s (62.31%)；最长17.543s段中16.205s为stale_command，规划态16.153s，串行ComputePath候选连续约1.7s返回empty_path后才有可用路线，是本轮直接观察到的剩余长停来源。Native源拒绝空、源样本最大间隔.190s，无本轮证据指向Native/DDS延迟。该结果与v4不是严格相同frontier的A/B，不将比例变差因果归于修复。
- 本轮本机证据在 `test_artifacts/execution_continuity_repair_20261021/v5/trial_20261021_093254`；bag SHA256 E1D7E18F68CFE7D7B77591E47552FDA135DFB52FA6EC9FA5D085F47136E71B0F，失败RGB在 `.../goal_001/failure_last_rgb.png`，是非到达证据，距源戳.660s。详 `docs/RESIDUAL_STOP_CAUSES_20261021.md`。整轮未通过，未commit；20轮稳定性验收仍未开始。后续优先审计empty_path最早请求与地图/姿态/epoch输入及候选枚举，不能放松碰撞约束。
- 用户要求另跑并录像。v5来源核验通过，预检无进程/显示锁、约25GB空间；第一次Town Scene RPC超时且map_loads=0，清理确认后重试。第二次 `trial_20261021_094550` 搜索green residential house约42.46秒后退出，原因链记录为先 `native_telemetry_stale`、再 `planning_only`，终止错误为BeamNG `Vehicle control lease expired or closing`；不是到达/超时。运行后无残留项目进程。
- 该轮关闭bag窗口42.297s、853 Native样本、最大采样间隔.058s，速度<.1近停约26.30s（62.2%）；0–9.053s净位移.0013m，18.618–27.810s近停9.192s/净.0069m。多次串行ComputePath `empty_path` 约1.7秒后才出现可用候选：开头两个，18.6秒长停内两个；故长停和反复规划本轮仍存在。Lease故障的上游触发未定，不直接归罪于BeamNG或规划器。
- RViz录像本机 `test_artifacts/execution_continuity_repair_20261021/trial_20261021_094550/rviz_live.mp4`：1920×1080、10fps、107.1s、H.264，SHA256 `183353f7aec6a41a6b5504a4b779f22a9542cea9f5085778616cb0bc267b19c2`；查询约42.46s，其余含启动/退出。诊断包SHA256 `24e2c4973cce15f7d04fe496b9a860b044b6fd439cbf952756620f2e0f297665`，bag SHA256 `8a7a37c5ecd076e80fb056c65ad746524ae6533777a054224b815c9bfc38be50`。详细原因与路径见 `docs/RESIDUAL_STOP_CAUSES_20261021.md`。未通过现场验收，未commit；下一步核串行empty_path拒绝链与Native/Lua控制租约过期链。

### 2026-10-21 GraphNav 端点车身可行性过滤（组件通过，现场待验）

- 复核094550/093254关闭bag与栈日志：串行`empty_path`不只是规划器无解，前几个GraphNav端点/路线样本的**中心格自由，但真实车体姿态已与100格重叠**。094550比较选项1/2的4.47×2.12m车身与lethal格重叠，且1.0m内无任何自由车身姿态；选项3/4自由。093254比较选项1/2/4同样无自由姿态，仅选项3自由；普通路线样本0–4同样无自由姿态，样本5在1m内存在自由姿态，规划器随后从该样本生成62位姿路径。不能只用“端点占用”过滤，否则会误删样本5。
- 修改 `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`：新增 `vehicle_pose_feasible`（按规划器1.0m终点容差、0.2m邻域搜索自由车身姿态），在 `route_candidates` 循环中分别过滤切向候选和 approach-bearing 变体；保留观察/确认候选及memory选择（memory需自行尝试被阻塞视角）。`exploration_cost.py`的vehicle_options比较同样先过滤无自由姿态的选项，少于2项则回退普通候选。不扩大规划总预算，不跳碰撞/曲率/新鲜度检查。
- 本机 `tools/offline_callback_check.py` **640项通过**（新增4项：无自由姿态端点跳过、邻域自由姿态保留、比较少于2项回退、路线样本过滤）。记录bag只读复核：094550 6个候选判定约50.2ms；093254 6个路线样本仅样本2/5保留（样本5为实际成功样本，样本2仍被规划器拒绝，属保守过滤的假阳性）。服务器实际ROS/DDS、部署和整车均未执行，不能记作现场通过。
- 同一094550轮的控制退出链另记：约67.7s `poll_tick`与`control_tick`同时阻塞812.656/813.702ms，`telemetry_age=.864s`触发bridge `native_telemetry_stale`并取消FollowPath；随后control RPC返回lease expired（.8s租约），`actuator_ok=false`导致会话退出。该轮控制RPC p99 51ms，813ms为单点；引擎侧阻塞来源未闭合，未改租约/健康保护。
- 服务器隔离v6：复制v5 candidate后只替换本次四文件，正式安装未动；构建成功，实际ROS 78项、source control 13项、服务器离线回调640项通过；418成员冻结清单校验0失败。源码SHA `graph_path_bridge.py` b4c29fab406aa46fbd64516bc7b3d745910849d6076bb3175c1ca47531608a93、`exploration_cost.py` 3bedccc585f15e0b36294420c1d9f2d6e8f08554e13661133cd79c7dbe759e8b。
- v6现场 `trial_20261021_101912`（120s）：120.87s正常超时，errors=[]，无lease退出；近停44.11s/119.75s=36.83%，最长连续2.114s（v5 093254为74.772s/62.31%、最长17.543s）；净位移40.559m；窗口内7次`empty_path`（全轮9次），v5 093254全轮30次。图采样/frontier不同，不是严格A/B，不能把全部改善因果归于过滤。末段115.9s `visible_target_reobserving`后转`planning_only`，未到达、未确认。跑后无项目进程/显示锁，磁盘22G。
- v6现场 `trial_20261021_102500`（180s）：181.62s正常超时，errors=[]，无lease退出；近停73.74s/179.80s=41.01%，最长2.158s；净位移仅15.083m，30次`empty_path`，未进入visible target approach，结束后`planning_only`。跑后无项目进程/显示锁。两轮v6均无>2.2s长停，但探索进展差异大：120s轮接近房屋后在重观测阶段超时，180s轮未接近目标。不能宣布路径执行/完整搜寻已修好。
- 102500的30次`empty_path`对应19个唯一目标，最高重复4次（720.36,-13.19；734.56,-27.04），说明同一图版本内会重复尝试无效端点。已记录为下一候选方向（有界失败目标缓存或GraphNav端点可达性接口），未实施；需先证明地图/epoch变化会使缓存失效，不能用固定时间或永久黑名单绕过。
- 证据本机 `test_artifacts/execution_continuity_repair_20261021/v6/`：101912 trial包SHA `26b344eec3aeb5ce07fd5e44dde19b5ea0c26970b36c737f688b233e120d77c3`；102500 trial包SHA `7ed445b9cdf848ea3b2051c7e9a75533c115f6f6d94a054d24afca207ff593ee`；组件包SHA `332d36091add1d41b52c7013870c6f5817d73c90863beaa78462279515b19b17`。
- 未提交Git（仍未完成视觉到达/完整验收）；后续在同一v6版本上继续查末段visible target reobservation/确认、探索进展差异，以及lease在更长轮次是否复现。

### 2026-10-21 用户取消20轮统计目标

- 用户明确要求“不做20轮统计，可以把这个目标删除掉”。当前验收不再以“20轮中至少16轮成功”为门槛；该目标已从 `AGENTS.md` 的活动要求移除。
- 历史20轮相关记录和旧证据保留，不改写过去结论；完整视觉到达与真实车体执行仍须分别核验，完整绿色房屋到达仍未通过。

### 2026-10-21 用户目标：180秒内完成；v7/v8 重测与录像

- 用户新目标：“修复并重测，直到180秒内能够正常完成任务”；不再做20轮统计。后续按此目标迭代，不把单次接近/长停下降当作完成。
- v7（v6+失败目标缓存，`graph_path_bridge.py` b85bccb…、`exploration_cost.py` c30b311…）：服务器实际ROS79、source control13、离线643通过。180s `trial_20261021_103718` 近停33.85%、最长2.149s、净49.825m、11次empty_path，无到达；120s `104450` 近停38.96%、净26.76m、离房屋最近13.84m。缓存未明显命中（exploration_id频繁变化），不能单独归因改善。
- v8（v7+可见观察2帧/15s，官方goal_standoff源码从运行v2复制并只改参数/launch；实际ROS100、source control13、离线644通过）：180s `105336` 近停36.82%、最长2.149s、净59.4m；`105905` 近停31.63%、最长2.103s、净57.79m，115.58s距房屋1.86m。两轮均无到达；2帧/15s在约150s触发，但目标是后方另一绿色物体，178s才再次触发，预算耗尽。
- 关键几何证据：105905在115.58s车辆位置(731.02,-51.47)、heading -121.05°、房屋bearing 150.36°，相对角 **-88.6°**；前相机视场看不到左侧房屋。视觉mask/定位在约150s对应的是(738,-79)附近另一绿色物体，不是该房屋。故当前瓶颈是**检测视角/实例关联**，不是长停或路径执行。v6 180s重跑 `110450` 最近3.89m/151.66s，`110945` 最近25.48m，均无到达；路线采样差异大。
- 录像本机：`test_artifacts/execution_continuity_repair_20261021/v8/trial_20261021_105905/rviz_live.mp4`（159MB，SHA `d6f29591…acc09bc`）；`.../v6/trial_20261021_110450/rviz_live.mp4`（166MB，SHA `86e16980…c6cca12`）；另有v8 105336 143MB、v6 110945 149MB保存在服务器。视频为RViz左2D/右camera，包含启动/退出；未把录像当作到达证据。
- 未commit；完整180s完成仍未通过。下一步需针对“房屋在侧面、前相机不可见”实现有界 look-around/覆盖观察，或改进实例/视角选择；不能继续靠重复整轮碰运气，也不能放宽到达确认。
- v9（v8+可见观察 GraphNav 无候选时局部 Smac 回退；当图无候选仍尝试到已接受观察点）：实际ROS100、source13、离线645通过。180s `111840` 近停37.92%、最长2.696s、净58.85m、最近房屋1.98m/132.03s；局部回退在179.3/183.4s触发，预算已尽，未到达。1帧/15s版 `112347` 近停33.9%、净59.63m、最近1.94m/129.03s，回退179-184s，未到达。
- v10（v9+观察步5m、1帧/15s；确认仍5帧/1.5m）：实际ROS101、source13、离线646通过。180s `113035` 近停58.99s、净40.73m、最近4.88m/162.43s，**本轮没有任何 Visible target bearing/mask**，未到达。说明模型对远处/小目标漏检同样是主因，不能只靠参数提前。
- v6 `110450` 的 155s RViz帧已保存：`test_artifacts/execution_continuity_repair_20261021/v6/trial_20261021_110450/frames/f_155.jpg`，路口前方、画面中左的浅绿色房子清晰可见，但该时刻没有对应mask/定位；当前是**检测模型/视角**瓶颈，不是执行或长停。
- v10录像本机 `test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_113035/rviz_live.mp4`（156MB，SHA `4c83f5ac…d206c0`）；v9 111840/112347录像留在服务器。
- 未commit；180s完成仍失败。下一步建议针对前相机中心/路口区域做高分辨率裁剪/tile推理或更好的实例关联，而不是继续放宽时序阈值；需先离线验证裁剪能否在 f_155 帧检出该房屋。

### 2026-10-21 高分辨率 tile 检测（现场已触发，目标可达性仍待解）

- 离线验证 `f_155`：完整前相机 960×640 推理 **0 像素**；房屋区域裁剪放大后 **7246 像素**，左上 tile 3172、center crop 8590，三者落点一致；叠加图正好覆盖浅绿色房屋。入口 `test_artifacts/execution_continuity_repair_20261021/v6/trial_20261021_110450/crop_probe/house_crop_overlay.jpg`。
- 已在运行中的 `visual_navigation` 节点加入 tile 回退：完整画面无 mask 时，对前相机左/中两个 62%×72% 重叠区域放大到 960×640 推理，1s 限频，mask 以 4D 形状合并回全图；源码同时同步到本机 `wildos_port/.../nav.py` 和 v6/v10 候选。原文件备份 `nav.py.before_tiles`。
- v10 `114347`/`115409` 与 v6 `115925` 均出现 `Tiled object search recovered 1000–1840 pixels`，并在约 160s 触发 `visible_target_approach`；不再出现“完全无检测”的轮次。
- 但未到达：检测/定位落在 **(738,-79)** 附近另一绿色物体；该物体在道路外，v10 第二轮 5m 观察步 `no_vehicle_path`，v6 轮在 175.9s `footprint_collision` 重规划；object_localization 仅 1 条、views 2，`object_confirmation` 0。另一侧 (729,-50) 房屋虽在 1.86–2.71m 被车身掠过，但相机相对角约 89°，前相机看不到。
- v10 `115409`：近停未再恶化，净56.68m，最近房屋2.48m/132.39s；v6 `115925` 净53.91m、最近2.71m/129.48s；均 timeout、无到达。
- 录像本机 `test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_114347/rviz_live.mp4`（131MB，SHA `ad53f020…39d47c`）与 `.../v6/trial_20261021_115925/rviz_live.mp4`（142MB，SHA `e12ac620…66b4e4`）。
- 当前结论：tile 检测解决了“小/远房屋漏检”；剩余是**选择可达观察点/侧向 look-back** 与 (738,-79) 物体的道路外可达性。不能通过降低确认门槛或直接朝不可达物体规划来“完成”。
- 共享 vision install 已被临时打补丁（有备份），v6/v10 候选源同步且清单已刷新；正式 commit 仍未做，需在隔离安装/重启后复核。

### 2026-10-21 规划/反复倒车重规划现状（用户追问）

- 统计近四轮180s：v10 `115409` 55次ComputePath/11 empty_path/4 no_vehicle_path/13 cusp/2 follow_failed/12段D-R；v6 `115925` 67/17/0/17 cusp/5 replanning_after_maneuver/3 footprint_collision/16段D-R；v10 `113035` 49/7/0/19 cusp/2 replan/4 footprint/17段D-R；v7 `103718` 60/11/0/20 cusp/0 replan/1 footprint/17段D-R。
- 说明：串行empty_path长停已明显缩短（最长连续近停约2–3s，旧v5为17.5s），但**局部反复前进/倒车、短段cusp和重规划仍存在**；每轮约5–8组D/R换向，cusp 13–20次，planned路径最多4–6个motion section。`replanning_after_maneuver`/`footprint_collision`仍会在地图/端点变化或不可达观察点附近出现。
- 当前主要剩余：可达观察点选择（(738,-79)物体在道路外，5m观察步no_vehicle_path；v6轮footprint_collision）、侧向look-back（(729,-50)房屋在1.86–2.71m处但前相机相对角约89°）、以及整体无进展/短段换向成本。不能把“长停变短”写成“反复倒车已解决”。
- tile检测已解决小/远房屋漏检；仍需选择车辆可达的观察点，否则会朝不可达物体反复接近/重规划。180s完成目标仍未通过；未commit。
- 确认时间窗（默认30s，隔离候选65s；4帧/1.5m空间要求不变）已生效：v6 `121345` 有17条object_localization、12条object_confirmation，`search_confirmed=true`、inliers4/5、radius≈0.055m、standoff(737.28,-75.32)；但车辆距standoff 4.66m>0.6m，桥报`confirmation_before_standoff`，不产生`object_found`。v6 `121904` 同样在179.97s确认、4.66m>0.6m；最近房屋3.89m/109.28s。
- 结论更新：确认窗不是主要缺口；当前阻断是**(738,-79)物体在道路外、standoff不可达**，以及(729,-50)可达房屋没有被前相机看到。下一步必须做侧向look-back/覆盖观察，或选择可达standoff；不能把standoff容差从0.6m放宽到4.66m来“完成”。
- 65s窗改动已同步v2源/launch与v6候选源/清单；默认30s保留。tile/确认窗均为现场实验候选，正式commit未做。
- 侧向look-back候选已实现（648离线通过）：查询≥90s、近旁1–3m大障碍簇≥25格、目标向后6/5m、15s硬超时自动释放；只做观察、不确认、不改变到达门槛。
- 现场边界：v10 `123010` 在73s过早触发（当时阈值前），7段U-turn/重规划占满整轮，净14.63m、无tile，说明未约束的look-back会反向伤害。加约束后 v10 `123640` 未出现`side_lookback`事件；tile在约60s即触发可见接近（目标730.9,-36.9等），可见接近/重观测连续占用运动权，最近房屋3.14m/112.6s、净55.79m、无到达。look-back尚未在现场证明有效。
- 当前更明确的结论：tile+确认窗已能检测/确认绿色物体，但被检测物（738,-79）standoff道路外不可达；(729,-50)可达房屋需要侧向/回看视野。look-back在正常探索中才有机会，但tile可见接近会更早抢占；下一步需在“选择可达standoff”或“检测后先评估可达性、不可达则回看”之间做取舍，不能同时无界追两个目标。
- look-back仍为未验收候选；v10/v6 180s均未完成，未commit。
- 进一步现场反例：v10 `124354` 无look-back，tile约123s确认(739.86,-75.49) standoff距车6.49m>0.6m，最近房屋3.39m/69.96s、净60.64m；v10 `124921`（允许抢占）在106.9s才在起点附近触发look-back，目标(738.09,-0.36)，121.9s timeout，净8.76m；v10 `125420`（离起点≥20m）无tile、净6.23m；v10 `125948`（路线过滤全拒回退原始候选）仍无tile、最近房屋31.31m、净10.14m。
- 新结论：当前剩余首要故障已转为**起点/早期路线采样导致整车长时间不推进**（净6–10m、无tile），而非仅目标不可达。路线过滤全拒回退未解决；需单独审计最早路线候选/GraphNav route/起点姿态与候选枚举，不能继续叠加目标侧参数。
- 650项离线通过（新增可见接近寿命、look-back、路线过滤回退测试）；最新v10录像 `test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_125948/rviz_live.mp4`（145MB，SHA `d35e3585…68ad5`）。180s仍未完成，未commit。
- 最早60s审计（125948）：首个GraphNav goal(733.78,-23.09)两次empty_path；候选1/2成功但比较选了cost更高、`motion_sections_remaining=5`的路径；第一段goal仅到(734.78,-11.06)，约1.2m后`exploration_finishing_segment`，紧接着`exploration_handoff_stopped reason=handoff_actual_pose_cannot_join`；随后新route(726.47,-26.86)继续，反复短段/交接。整轮34 ComputePath、15 empty_path、10条skipped候选，净10.14m。
- 这说明剩余首要故障是**短第一段后的实际姿态交接不能加入下一段**，不是目标侧 standoff/检测。下一步应审计`motion_sections`第一段长度、`send_motion_segment`交接用的实际姿态与下一段起点契约，以及候选比较为何偏好多段短路径；不能仅靠重试/放宽碰撞。
- 修复：`continuous_path.py` 的退休段交接仅在车辆实际停稳（速度≤0.05m/s）时允许`trim_moving_path`接受反向/换向起点；运动中的0.4rad航向保护不变。650离线通过。
- 现场三轮验证：v10 `130635` handoff=0、tile25、最近房屋2.72m/162.74s、净50.42m；`131117` handoff=0、tile3、lookback82/155s、最近4.74m/126.82s、净28.13m；`131614` handoff=0、tile0、lookback184.9s（超时后）、最近16.39m、净22.63m。三轮均timeout、无到达。
- 结论更新：起点短段交接停滞已明显改善（handoff=0、净位移22–50m），但**房屋正面进入前相机的时机仍不稳定**（tile 0–25次，最早接近162s/126s；可见目标/确认仍在180s后或未发生）。180s完成仍未通过，未commit。

### 2026-10-21 按用户要求取回130635轮录像

- 已从服务器 `/home/ly/execution_continuity_repair_20261021/v10/full/trial_20261021_130635/rviz_live.mp4` 取回本机 `test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_130635/rviz_live.mp4`；177218230字节，SHA256 `e1a99f5b8523ea86932e71e0bc6ec4c5ccd381b853e4eab3ba6314893f427f4b`。
- 本机可打开，10fps/2574帧/257.4秒，第90秒图像可解码且非全黑；此为媒体可读性检查，未逐帧验收。本轮在最新三轮中距记录房屋最近（2.72m/查询后162.74s），仍超时、无到达；录像含启动退出，视频时间不直接等于查询时间。未修改生产代码或创建commit。

### 2026-10-21 130635实际走停复核（关闭bag，只读诊断）

- 用户明确“不流畅”指实际车辆走走停停。按查询起始源时间，对该轮关闭bag的Native实际速度、control_status与plan_status合并时间区间，观察前180s；Native相邻采样覆盖179.9506s。近停阈值speed<0.1m/s，累计60.0376s（33.35%）；合并标签时间积分最长8.575s（仅Native相邻样本为8.557s），17段近停≥1s。初始8.575s含约6.014s planning/2.063s execution_ready；不是录像帧率或纯识别问题。
- 近停按控制状态互斥积分：gear_change28.181s、stale_command15.350s、tracking8.915s、ackermann_stop4.151s、execution_hold3.184s、其余约0.256s。换挡等待14次，每次2.000–2.051s；Native下发gear有14次方向切换。cmd_vel_to_control.py的gear_change_confirmation_time默认2.0s，planning.update_direction_shift维持制动确认。28.161s换挡近停期间上层plan仍为following，说明following不能当车辆持续前进。
- stale_command与planning相交11.830s，支持部分停车源于规划间隔内无新鲜速度命令；该标签不单独证明网络故障。全轮stack中48次ComputePath发送/7条reason=empty_path；计数范围为完整日志，不能与180s窗口请求一一强配。按事件去重有12次maneuver_done、1次follow_aborted、1次replanning_after_maneuver/collision_cell；约查询35.370s有一次handoff_prefix_blocked（bag重复三条是同源事件），造成交接制动与重新规划。
- 更正先前缩写“handoff=0”的范围：仅handoff_actual_pose_cannot_join为0，并非全部交接拒绝为0；最新轮仍有handoff_prefix_blocked。旧“最长2–3s”仅适用于早先指定轮次，不能套用130635。此次近停主要由多段换向及固定确认等待、规划间隔和交接/执行失败共同组成；每次换向是否必要、代价比较是否合理、前缀拒绝是否地图真实更新仍未闭合。
- 180s内3599条Native source_rejection均None、telemetry_ok均true、watchdog_stop均false、execution_unavailable转换0；不支持把本轮主要走停归因于旧813ms租约中断，亦不证明系统完全无时延。
- 证据本机 `test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_130635/fluency_audit/{summary.json,residual_stops.json}`，后者含原bag路径及SHA。复用既有只读分析器，将窗口120改180在诊断进程中运行；原分析脚本、生产源码、车辆和保护参数未修改。更正为基于同轮原始证据；未运行新仿真/未commit。

### 2026-10-21 当前只修实际走停：离线修复候选与执行段数约定不一致

- 用户收窄任务为实际走走停停，明确服务器不可连接。本次没有SSH、部署或新仿真，先读CONTEXT_EXPORT及当前代码，保留用户未提交改动；仅围绕已有六段限制的数据流实施修改。
- 新反例：候选比较未执行已有max_motion_sections，导致超限路径可获胜、派发时才拒绝并浪费已找到的可执行frontier候选。四个生产回调反例在改前均断言失败、零错误；改后全部通过。该问题属于近期段数限制的选择/执行衔接缺口，不当作此前所有走停的根因。
- 修改graph_path_bridge.py/exploration_cost.py：共用exploration_motion_issue，在普通航向候选缓存、frontier比较和执行阶段采用同一已有规则；不新增阈值，不改变语义收益、Smac/connector、换挡/物理/速度/碰撞及对象/记忆/恢复流程。全比较失败恢复原普通候选，正常可行航向保留并执行。
- 七组相关回调382项，381通过、1失败、0错误；唯一lookback旧失败在改前源重载后复现，未处理无关功能。新增4项均通过；首次测试类名和视觉导入路径错误另存诊断日志，修正入口后重跑，未修改生产去迎合夹具。
- 证据test_artifacts/stop_go_candidate_admission_20261021含改前字节/SHA、独立delta、失败反例、回归及baseline_lookback.log。源码SHA：graph_path_bridge 8394bf653cb3c8f35c9c3cd6230e5405ee42424103c0871eb01ae389c5f33a93，exploration_cost e147f31dfcbaa6888ef063c09979e8efe80129224209321e3159de9abeeaefad。详docs/STOP_GO_CANDIDATE_ADMISSION_20261021.md。
- 仅离线回调验证，不是ROS/DDS/车辆效果；connector反复换向、路线与地图冲突及整车近停改善仍待验证。原C++诊断改动保持未编译，未commit。后续不因这项修复宣布走停已解决。

### 2026-10-21 离线核实“目标一侧、下发另一侧”

- 只读130635/181516/183339本机events，按planned状态actual_start_pose、同消息exploration_goal和已下发goal配对planned_path（来源elapsed差≤0.1s），计算段端位移对目标向量的投影并检查路径首边相对车辆yaw的方向。证据test_artifacts/stop_go_candidate_admission_20261021/away_sections_audit.json含原事件文件SHA、身份和逐段坐标。这里目标是GraphNav探索目标，不是已识别的房屋位置。
- 183339相同探索目标(727.5067,-26.5425)：查询43.09s起点(735.236,-3.132)下发段终点(735.602,-1.603)，反向段1.695m/余4段；56.62s起点(735.530,-2.109)下发(735.762,-0.587)，反向1.688m/余4段；70.64s起点(735.735,-0.761)下发(735.901,+0.196)，反向0.971m/余2段。对应实际起点距同目标24.65→25.72→27.06m，预期段终点距目标26.22→27.24→28.02m。表明下发路径本身反复背离，不只是车辆控制器反向跟踪；不能据此证明底层坐标轴取反或所有背离机动非法。
- 原130635也存在少量短背离准备段，不能把朝目标距离单调下降作为硬条件。上述已下发分段和事件不包含完整connector边界，仍不能判定反向源于connector还是Smac后缀；完整路径/源码诊断未编译部署。前一项离线候选六段准入一致修复不处理这些2–4段背离路径，不将其当作此问题修复。当前继续优先定位路线生成及重新选择，不能靠再降换挡等待掩盖。

### 2026-10-21 反向原因再分析：确认重复机动与交接拒绝相邻

- 用户要求只思考反向成因，本次仅源码/关闭事件读取，未改导航代码或连接服务器。新证据reverse_replan_timeline.json位于stop_go_candidate_admission_20261021目录。
- 183339同owner:5/同探索目标，查询49.56s maneuver_done后立即replanning_after_maneuver/collision_cell，误差.16452m/.02548rad、join_failure_reason=null；56.62s再派发北向倒车。62.75s再次replanning_after_maneuver/footprint_collision，误差.17342m/.06444rad、join_failure_reason=null；70.64s再派发北向倒车。事件支持“多段机动中后续分支被地图/扫掠检查拒绝后重算，再执行背离段”的重复链；不证明相关碰撞观测虚假，也不能确定该段就是connector。
- 当前源码事实：connector.search不接收目标，只按局部连接长度+3*cusps保留最多6个锚点，再由planner比较完整连接+后缀；continuous起点与栅格中心/yaw近乎精确相同才跳过connector。GraphNav observation_heading进入目标姿态，规划半径12m/控制8m为既有刻意留跟踪余量。三者可能增加反向机动，但没有同图完整路线反例证明任一项为首要根因，不能直接改成半径8或禁倒车。
- 本轮更有证据的归纳：需要区分“首次反向路径如何生成/选中”和“为什么执行后反复回到反向准备段”；后者已有交接拒绝→重算的现场链，前者仍待connector/后缀分界诊断。没有证据优先把问题判为整体坐标符号反转、速度环或视觉记忆检索错误。

### 2026-10-21 反向惩罚 A/B 与“确认后无可达 standoff”链路复核

- 现场 A/B：`210104`（penalty 0）反向首段 21/37（56.8%）、近停39.3%、净41.5m；`210759`（penalty 3.0）22/41（53.7%）、近停48.2%、净59.8m、11次可见接近；同参数重跑 `211315` 仅净14.6m、无tile、无可见接近后返回起点附近(732.9,-7.6)。比例未实质下降，59.8m未复现，判为路线采样差异；已将候选/安装share改回 `reverse_start_penalty: 0.0` 并同步 frozen manifest，C++ 参数默认0保留、未重编译。证据入口 docs/CONNECTOR_DIAGNOSTIC_20261021.md。
- `210759` 末段：56.6–135.5s 沿固定方位以约4.25m观察步推进到(735.1,-67.7)，之后对(735.9,-71.7)反复规划出现 waiting_for_costmap/`replanning_after_maneuver footprint_collision`，30s观察截止于165.7s触发 `visible_target_approach_timeout`，目标被 blocked 15s，180s内未恢复。该段是“观察点不可达后仍占用整个时限”的现场反例，不是检测丢失（tile 持续到轮末）。
- 三角化日志对照：`v6 121345` 有17次 `Object LOCK using LiDAR`、17条 object_localization、17次发布，`Validated multi-view` 为0；`210759` 两者均为0，`Received callback 162`、`too close 124`、`Not enough views 15`、`No triangulated position 38`。即确认实际只走 LiDAR，多视角从未成功；`too close 0.00` 与约5s一次的 `Not enough views` 显示观测步停车等待超过5s视图窗口。不能据单轮断言超时是首因，也不能把多视角当作可用确认路径。
- 确认后链路：`v10 124354` 在127.6s对standoff(739.9,-75.5)立即 `confirmed_approach_failed reason=no_vehicle_path`，随后 `unconfirmed_target_release_pending`；`v6 121345/121904` 为确认时距standoff 4.66m>0.6m 的 `confirmation_before_standoff`。多次现场证据一致指向“确认可发生，但standoff对当前Ackermann不可达/未到达”。未放宽0.6m位置到达、0.35rad终点朝向或确认门槛。
- 边界：以上为只读日志/录像帧复核（failure_last_rgb 与轮末RViz帧已取回 test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_210759/），未修改导航语义；反向惩罚回退后未再跑完整验收，180s完成仍未通过，未commit。

### 2026-10-21 用户更正：规划仍含三类问题，优先可达观察位姿（源码核查）

- 用户强调212436规划/执行的末段可达性、走停效率和路线结果波动均未解决，不能把完整失败只归确认；优先评估可达standoff/观察点选择。本次仅本机源码和文档核查，未连接服务器或改导航代码。
- 源码事实：official_adapter.visible_approach.observation_step只沿5条窄锥射线调用safe_travel_distance；GoalStandoff.publish_standoff已有reachable_ring_candidate（固定目标距离、24个角度），同样以局部OccupancyGrid中心射线与距离余量判断。没有在这些选点函数中验证非对称真实车身姿态、转向扫掠或Nav2实际规划可达性。当前“生成的观察点”不能称作Ackermann可达观察位姿，后级Smac才承担最终准入；这支持优先审查该接口，但不证明212436具体终点拒绝的唯一原因。
- 后续方案应复用现有候选生成，统一到车辆地图/姿态约定，中心格连通性仅做粗筛，真实footprint和既有Smac给最终可达结论；同一目标/实例失败后比较替代观察位姿，避免只等同一点截止。没有实施新的环形距离阈值或放宽碰撞/到达。
- 口径更正：CONTEXT_EXPORT_20261021B第1节/CONNECTOR_DIAGNOSTIC标211315 penalty3.0、212436 penalty0.0，因此这两轮不是同配置对照。可用同为3.0的210759净59.8m与211315净14.6m说明结果波动，但不能仅凭此锁定GraphNav而排除地图/模型更新和调度。53.8m/180s是净推进速率，不等于行驶中的平均车速；planning40.5s是状态驻留时间，不是独占CPU计算耗时。

### 2026-10-21 用户授权开始：可达观察位姿选择与失败反馈（离线候选）

- 当前工作区基于新B上下文（桥改前SHA162bb30696685d3e…），保留用户未提交工作；详docs/REACHABLE_OBSERVATION_REPAIR_20261021.md。没有SSH、部署、新仿真或commit。
- 新反例5fail/0error证明中心射线/环形选点不检查Nav2车身与连通，bearing-only耗尽候选仍等超时。现GoalStandoff订阅global_costmap，复用既有射线/环形候选，使用新增observation_pose.py检查0–98中心连通和既有SBR footprint；射线端点不合适时按分辨率缩短，最终回退也不绕过。车辆图过期时停止新提案；尚未收到车辆图/ideal_point保持原提案，最终Smac准入不变。
- bridge明确空/几何规划失败后发布visible_target_no_vehicle_path及时释放，复用现有reset/停车屏障，传输/协议失败不当作位姿不可达。选点端按当前查询、来源戳和当前位置匹配该状态及confirmed_approach_failed/no_vehicle_path，15s内跳过原位置，复用approach_refresh和既有同实例恢复；原对象/instance_id/观测戳和到达判据不改。start_live_observation补同样的车辆图连通必要条件，未新增动作客户端。
- 最终16项定向全过：含生产回调失败→新提案→既有路线准入和迟到结果隔离、确认失败→同实例替代视点、旋转/未知/旧地图、重复/跨查询反馈。完整ROS替身回调678项为677过/1旧lookback失败/0error；旧失败用本次改前源重载复现；官方适配器纯函数32过。初稿“不发布”夹具错误已修正并保留日志。不得据此称真实Smac/DDS或整车通过。
- 单次320×320全自由连通计算本机47.25ms，同消息/车辆中心格缓存；服务器负载与地图误筛风险待验。连通+静态车身只作粗筛，不能证明Ackermann转弯可达；实际规划、执行扫掠检查保留。
- 证据test_artifacts/reachable_observation_repair_20261021：before字节/SHA、before_tests.log、after_final.log、callback_report.json、baseline_lookback.log、candidate.patch、candidate_delta.zip和delta_manifest.json。新测试已接入offline_callback_check。现场前须完整增量含新增模块、重建两个Python包并刷新运行源清单，不能单文件混装旧版本。末段可达性、走停改善、180s到达仍待现场验证。

### 2026-10-21 观察点候选部署失败与核心探索走停复核（221131）

- 用户恢复服务器授权。原v10保留，新隔离reachable_observation_v11；421冻结成员、实际安装/现场Python SHA、4个实际C++库已核对。Smac89ae42c4…、reverse penalty0、原真实SBR/速度/转弯/碰撞/到达判据保持。测试前后进程清单为空、X148锁清除；GPU0另有他人作业，未干预。没有视频或commit。
- 部署前审查修复新反馈缺口：GoalStandoff每0.4s更新提案，但bridge保留原接受点，失败按最新提案匹配会漏掉。现单独记录新鲜同query的visible_target_approach接受点，按原0.6m匹配失败、拒绝未接受点，重复/迟到不覆盖、query切换清理。20定向通过。真实ROS初次2处Mock夹具错误已修正，仅改测试；两包构建及真实ROS20+22通过。本机682项681过、1旧look-back失败，非全绿。
- 221131完整真实车体181.02秒timeout，0 mask/visible approach/localization/confirmation，新增观察选点分支没有现场覆盖，不能称有效。用户指出再次走停/低净推进；继续只读审计同轮，没有启动随机重复测试或追加生产修改。详docs/REACHABLE_OBSERVATION_FIELD_20261021.md。
- 同轮前180s Native覆盖179.994s/3600样本：近停91.433s=50.8%，最长7.303s；采样里程70.57m、净14.48m、最大离起点24.00m。gear_change近停36.887s，stale_command29.463s（其中planning24.346s），tracking10.519s、ackermann_stop7.170s、execution_hold7.266s；planning驻留34.815s，following122.857s但其间近停47.834s。47 ComputePath/11 empty_path。telemetry_ok全true/source_rejection全None，最大源样本间隔0.159s；不支持把本轮主因直接判为传输中断。驻留不是CPU，里程不是净位移，不和缺原bag的212436做同口径near-stop A/B。
- 23 planned为9个owner首派+13续段+1碰撞重算；18实际换向为12队列cusp+5新owner首段翻转+1碰撞重算。11派发段<1m。owner:2完整D1.209→R0.938→D10.469m；9.940s上游frontier_resolved触发既有退休规则，最后10.469m推进段未派发，5.047–14.080s累计1.747m、净0.462m。owner:18也在短R/D后161.112s frontier_resolved丢弃19.56m尾段。证明动态frontier生命周期会让整车先支付机动成本再丢弃推进收益，非动作回调崩溃；不证明frontier=false本身错误，不能简单删除退休保护。
- 源码核查connector空间预算偏置：每位置遍历72 heading再换点，.25m图6000次至多覆盖前4圈+少量第5圈（81*72=5832），允许6m范围大部搜索不到；本轮0.2s截止时千余evaluations。需要同地图/起姿受控对照，尚未修改/证明是全部换向根因。完整Smac成本已参与且多数日志比较5–6解。bridge精确frontier observation_heading可行解跳过normal_best比较，普通前缀只比同端点航向；3–5段符合6段上限。这些探索路径生成/选择与退休交接问题仍未修复。
- 本机test_artifacts/reachable_observation_repair_20261021/server_v11/full/trial_20261021_221131含紧凑证据与fluency_audit/{closed_trial_summary,segment_chains,representative_retirement_chains,path_geometry}.json。原bag留服务器SHA000bbd18…c99dc9。当前优先级回到探索执行连续性，观察点修复仅组件通过、现场未覆盖，整体目标仍未完成。

### 2026-10-22 frontier交接尾段再准入与首段退离软代价（离线）

- 基于221131短段交接后frontier退休证据实施本地改动；保留原退休停车、动作终态与新owner规则，不直接续跑旧尾段。
- `continuous_path.py` 在旧owner动作真实终止时，仅快照最多3段、总长≤20m且段间连续的未执行尾段。`graph_path_bridge.py` 只有在新owner路线新鲜、仍有效、非查询/记忆目标时，才以车辆当前位置裁切候选；它需对新goal取得≥1m净进展，并通过当前global/local costmap、路线走廊、段数和初始跟踪检查，然后走原`execute_checked_path`再次验证。拒绝则回落原`on_route`规划；旧尾段从不直接派发。
- Ackermann候选代价增加第一运动段远离当前探索目标的距离增量×0.25软代价，应用于同端点航向比较及frontier比较，只改变已准入候选排序；不硬禁反向，ideal_point不变。
- 新增尾段连续性、新owner净进展、costmap拒绝与软代价几何回归；修正look-back回调测试隔离条件，使其专测调度而不依赖车体足迹几何。全量离线ROS接口替身回调685项通过，`py_compile`通过。证据：`test_artifacts/offline_navigation/callback_report.json`及`offline_callbacks_test_graph_path_bridge_routing.log`。
- 未运行真实ROS/DDS、Smac构建或BeamNG/服务器整车；软代价系数和尾段再准入对真实走停、路线稳定性的收益待现场验证。相关代码未commit；按项目规则需现场验收后再提交。

### 2026-10-22 审查修正：空路径缓存与 streak 作用域（离线，测试未运行）

- 审查 2026-10-22 改动时发现三处作用域缺口，按最小改动修正：
  1. `graph_path_bridge.py` 的 `route_filter_fallback` 原先把 `cached_skipped` 与 `route_skipped` 一起放回候选，等于重新规划已在缓存中的空路径三元组（owner/起点区/目标）。现只回填车身姿态不可行的候选；缓存失败保持排除，全部为缓存失败时直接走 `no_free_graph_candidate` 退休路径。原因：该回填只应用于“姿态预筛过严”的场景，不能抵消缓存本身要避免的搜索。代价：同一 owner/起点区/目标下，缓存失败不再因“全部候选被筛”而重试；地图更新后的重试改由新 owner/新路线或车辆移动出 2 m 起点区触发，此行为变化待现场判断。
  2. `failed_goals` 原只在 exploration identity 变化时清空，scope 为 None 的非探索路线（visible/legacy）会跨 query 残留。现 `on_query` 显式清空缓存并把 scope 复位；不改变 2 m 起点区与 0.5 m 目标桶键。
  3. `empty_path_streak` 原只绑定 epoch；`on_route` 换路线不递增 epoch，可能继承上一条路线的最多 3 次失败，使新路线首次空路径即触发退休。现 streak 同时绑定当前路线 `(frame_id, stamp_ns)`；同一路线/同一 epoch 内累计语义与阈值 4 不变。
- 新增 3 项回归：`test_query_change_clears_cached_planner_failures`、`test_route_filter_fallback_does_not_readd_cached_planner_failures`、`test_empty_path_streak_resets_on_a_new_route_identity`（`nav2_adapter/test/test_graph_path_bridge_routing.py`）。
- 验证边界：本会话 shell 工具持续不可用，**上述 3 项及离线回调全集均未执行**；不得记为组件通过。恢复后须先跑 `python -m pytest -q nav2_adapter/test/test_graph_path_bridge_routing.py` 与既有离线回调全集（含 `test_reachable_observation_ros`），再判断是否保留。真实 ROS/DDS、Smac 构建、BeamNG/服务器整车与 180 s 到达均未验证；未 commit。
- 同时审查但未修改：`nav.py::_tiled_object_mask` 返回 batch=1 的掩码，与 `official_adapter/config/beamng_wildos.yaml` 的 `num_cameras: 1` 一致；若改回 3 相机，`visible_direction` 会因 `cam_infos`/batch 不符而静默丢弃 tile 结果（不得用复制掩码伪造多视角）。

### 2026-10-22 用户改动审查：新 owner 尾段交接与空路径缓存条件

- 用户要求先审查其未提交改动后继续修复，并授权服务器；随后提出当天完成一轮完整连续测试的目标。详 `docs/EXECUTION_CONTINUITY_REVIEW_20261022.md`。本次改前 688 项回调通过，补上此前新增三项未运行的验证缺口。
- 新反例 9 项中 5fail/0error：on_route 在旧动作期间只更新 latest_route，而倒车段不能后台替换，终态时 route 仍旧 owner，导致用户新增尾段再准入无法进入。现仅在原动作终态/实到通过后按原替代路线契约接纳新 owner，再用既有全部检查重新准入尾段；无新 owner/新障碍/不向新目标推进仍回落规划。
- 空路径缓存遗漏起始航向与地图条件；现绑定原 heading bins 和全局地图几何/内容（非发布戳），请求记录自己的起姿，地图在请求中改变不归属失败。同内容重发保留缓存，原 owner/query/2m 区域及128条上限不变。
- 最终 699 项离线回调、55项完整离线检查通过（6个既有现场边界WARN）；服务器三个包构建、Smac29项与真实ROS420项通过。真实ROS初跑9个确认测试失败为测试时钟返回普通对象，修正为ROS Time后保留原断言通过；初跑日志保留。不能把真实ROS回调称DDS/整车通过。
- 服务器新隔离 `/home/ly/execution_continuity_review_20261022/v1`，相对v11更新16文件、423冻结成员，重建用户已有Smac源码及两个Python包；预检无项目残留/X148锁，GPU1其他作业不动。完整真实车体420秒绿色居民楼搜索 `trial_20261022_155227` 已开始，无录像，关闭证据与连续性/到达待核验；未commit。

### 2026-10-22 155227关闭证据：旧current图节点变危险导致实际起点断图

- 155227已关闭，420.782秒timeout/errors=[]，无到达。420秒窗口Native覆盖418.933秒，近停274.183秒=65.45%，最长160.835秒；采样里程127.99m、净32.36m、最大离起点58.07m。近停按plan状态execution_recovered158.80秒/planning41.23秒；换挡近停41.54秒。不是CPU时间，也不按following判运动。详docs/EXECUTION_CONTINUITY_REVIEW_20261022.md。
- 约261秒bounded command arc碰撞后263.54秒graph_path_unavailable退休，264.28秒bridge终态停车恢复完成，随后图无路线反复backoff到超时。闭包Native source_rejection全None、telemetry_ok全true、最大源间隔.208s；此长停不支持传输断线主因。尾段再准入事件本轮为0，不能称其整车收益已验证。
- 精确身份配对raw/scored/odom CDR：259.76秒可达331节点/23frontier；264.16秒后至409.86秒可达只有自身。current(736.259,-46.763)距实际后轴(735.906,-44.898)约1.9m。旧节点新观测净空.70m/fresh confidence .8/measured violation，21邻边全被Planner正确过滤；同轮265秒terrain+采集TF在实车位置净空约2.50m。Mapper已有准入保护只作用于新候选，旧current新变危险后仍因距离<4m继续当起点，是此段最早确认的接口缺口。
- 最小修复mapper.cpp：旧代理点5秒内高置信实测危险且距实车>既有minimum_clearance时，优先按原free-ball准入重绑近邻；没有合格近邻则在实际odom建节点。不制造旧危险中心到新起点的traversed捷径，原地形边评估、危险节点、车体/碰撞/Smac规则保留。陈旧观测、微动和实际位置危险不会无限新建起点。
- 改前新反例2fail/0error；新增正常/危险/静止/陈旧/地图重接回归，真实Mapper34项通过。同组现场图/terrain/TF/odom、禁随机采样的C++Mapper→Planner受控回放：旧版更新地形前后图路线均0；新版绑定后仍0、原地形验边后恢复图路线（47次历史frontier目标调用均非空，不是47次整车到达）。仅组件，不能称DDS或整车通过。
- v2隔离相对v1只更新Mapper实现/测试两文件，423源码冻结，新库e8168739…89b5e；测试前无项目残留/X148锁。完整420秒真实车体重测进行中，无视频，未commit。证据test_artifacts/review_user_changes_20261022（图拓扑明细、紧凑包SHA、改前源码、回放器）；原bag与CDR保留服务器。换挡/规划间隔/首次碰撞和完整目标到达仍待验收。

### 2026-10-22 执行状态回调饿死假设与三分钟轮次

- v3完整搜索 173435 原生状态源约20Hz且数据新鲜，但单线程 `rclpy.spin` 可能令GraphNav工作阻塞桥接状态回调；这是待验证假设。v4在MultiThreadedExecutor下把native/control两个状态订阅移入独立互斥回调组并以锁读写快照。真实ROS阻塞测试覆盖两个状态流，离线回调699项、真实ROS421项、Smac31项均通过。
- v4第一次整车轮175048发生在用户提出3分钟上限之前并运行272秒，后按用户要求中断；未到达，近停与source-owner故障仍在。该轮不作新时限验收。
- 用户2026-10-22最新要求：每轮最长3分钟，包含启动预算；截止后有序停车/关闭时间另行统计。AGENTS.md及v4隔离驱动已更新，不录视频。下一轮180054实际在全局180秒截止后收尾，目标搜索145.33秒；净位移62.54m/采样路程69.35m，近停速度样本48.3%、近停段累计约63.50秒、最长14.75秒。无FollowPath失败，但一次 `source_owner_unconfirmed`，曾有approach timeout，field_acceptance=false；完整目标和连续运行验收均未通过。
- 关闭后的rosbag中native/control状态消息分别5441/5561条（272秒），两话题约20Hz；遥测新鲜不代表桥接ACK/所有者契约正确。隔离回调后仍有一次source_owner_unconfirmed；须针对该时刻对齐control status、native source_receipt的sender/generation/sequence与ACK，不能把回调饿死作为唯一原因。
- 关键日志取回本机 `test_artifacts/review_user_changes_20261022/v4_trial_20261022_180054/`；现场原始目录 `/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180054`。详见 `docs/EXECUTION_CONTINUITY_REVIEW_20261022.md`。改动未commit，因整车完整验收未通过。
- 后续关闭bag逐消息复核把180054唯一`source_owner_unconfirmed`定位到具体契约：Native已确认并应用gen7/seq1385；同owner、同sequence的control_status提前约4.8ms仍`source_ack_ready=false`，下一seq于约8ms后才更新true。这里不是topic缺席。修复仅允许同一已应用receipt且sender/generation/sequence完全相同的一帧ACK滞后；序号或owner不匹配仍拒绝。现场证据及测试见EXECUTION_CONTINUITY_REVIEW_20261022.md。
- 修复候选后现场轮180945在180秒预算内运行，目标阶段148.78秒；没有source_owner_unconfirmed或FollowPath失败，但没有到达目标。近停样本40.3%、低速区间累计54.70秒/最长6.66秒，visible target route timeout及no_vehicle_path仍出现。此单轮不证明故障稳定消失；到达可达性和走停仍未验收。日志存于`test_artifacts/review_user_changes_20261022/v4_trial_20261022_180945/`，未commit。

### 2026-10-22 异步结果归属与观察期限（离线通过，服务器待验）

- 详`docs/EXECUTION_CONTINUITY_REVIEW_20261022.md`末节。180945关闭日志117秒处成功ComputePath处理与`visible_target_route_timeout/reset`交错，之后仍下发planned/following且graph_goal=null；旧结果不能算正常观察推进。MultiThreadedExecutor的Future done callback不自动受ActionClient默认互斥组保护，不能只隔离Native订阅就假设导航状态串行。
- 候选将GraphPathBridge及NavigationTransactions的Action/Service结果排队到默认组GuardCondition，保留原epoch/serial/quarantine检查；Native独立接收不变。10秒路由等待仅在无route时触发，已接纳路线仍受原30秒截止；恢复占有控制权时截止检查不虚报已释放而吞monitor tick，恢复后原截止生效。
- 旧规则4项反例全部失败、0error；本次703项离线ROS替身回调全过、py_compile通过。证据`test_artifacts/review_user_changes_20261022/serialized_results_counterexamples.log`及`serialized_results_callbacks_verified.log`。新增真实ROS两线程/旧epoch回归未执行，此前422项真实ROS与31项Smac不覆盖本次新改动。
- 更正对话中的0.45m：这是Native原始车体参考点距离，后轴按既有偏移变换后距观察点2.41m（拒绝附近2.35m）；现有源码偏移正确，未改坐标或到达门槛。证据`v4_trial_20261022_180945/rear_axle_distance_audit.json`。request59有6个连续起点候选，具体几何/碰撞拒绝仍待关闭bag复现。
- SSH可连且无相关残留；未开新仿真。上传只读分析脚本被自动审批服务连接中断拦截，已请求手动允许，故候选未部署、未现场复测；后续每轮180秒限制不变。完整流程仍未通过，未commit。

### 2026-10-22 v5/v6现场与v7解除保持确认（最新活动入口）

- 详`docs/CONTINUOUS_EXECUTION_FIELD_20261022.md`，覆盖之前“服务器待验”状态：审批恢复后v5真实ROS427项通过，v6为435项；两轮都实际运行、已关闭、收尾无残留，均未完成目标，无视频/commit。
- v5 212511有效Native121.32s，近停49.92s=41.15%，最长16.29s，净61.48m；控制归因stale_command25.48s（planning12.70/reobserving9.48）、换挡12.02s。本轮末尾接纳16个方向段，暴露探索上限豁免了目标/记忆/续段的缺口。
- v6统一原方向段准入并增加前进中目标接续：新鲜向前提案提前请求GraphNav，匹配后经原后台规划及FollowPath接纳交接，不reset当前动作；沿用接纳事件更新GoalStandoff失败归属，确认/记忆/过期/后退/恢复保持隔离。711离线、435真实ROS通过。215520现场3次continuous推进真实触发，但一处后台规划仍赶不上旧段终态。
- v6有效Native105.69s，近停42.00s=39.74%，最长6.30s，净60.00m；stale_command13.94s、换挡12.19s、execution_hold6.62s。不同长度/路线单轮不能宣称稳定改善。现场目标未到达，仍有交接停顿。
- v6两次源门错误逐消息查明是解除execution_hold后generation交接：control已6→7或10→11、Native新回执晚几十毫秒；旧gen10有applied=false，不能放行旧代际。证据`v6_trial_20261022_215520/source_event_audit.json`。v7增加独立follow_release确认阶段，确认前不派发、不按正常过渡硬reset；原owner/lease/applied/新鲜度门不变，超过原响应时限仍故障停车。715离线、452真实ROS（含SourceControl13项）通过；已部署并开始现场，收尾待核验。
- 用户最新明确180秒从目标下发开始，启动/收尾另计，覆盖之前包含启动的解释；两份AGENTS已同步并通过3项规则测试。v7驱动计时已按此更新并保存SHA记录。历史v5/v6均仍按包含启动记录，不能改写有效搜索时长。
- 本机核心证据位于`test_artifacts/review_user_changes_20261022/v5_trial_20261022_212511/`与`v6_trial_20261022_215520/`；Smac公开地图回放显示端点安全但直接曲线5个采样碰撞，细节见新文档，不等于精确私有地图复现。当前全部改动未commit，完整目标/连续执行仍未通过。


### 2026-10-22 v7关闭与前70秒连续性复核

- v7 `trial_20261022_223039`已关闭，启动55.20s、目标搜索180.18s、关闭19.62s；未到达目标，field_acceptance=false，收尾无项目残留。没有录像/commit。此前“收尾待核验”现由关闭证据覆盖。
- 按query_sent对齐，严格前70.00s：采样路程60.26m、净位移58.42m；速度<0.1m/s累计15.61s=22.29%，最长连续近停7.30s。后110.00s：路程41.26m、阶段净位移9.60m；近停60.53s=55.02%，最长26.25s。前段相对顺畅但仍非连续无停，不能称完整验收通过。先前实时“约70秒、近停16秒/23%”为未严格query对齐快照，以上关闭数据更准确。
- 完整采样覆盖181.21s（略含搜索前后），近停77.34s=42.68%；其中plan状态planning近停40.35s、reobserving9.65s。5次no_vehicle_path、1次follow_failed，后段可达性/观察交接及规划间隔仍未解决。两次awaiting_control_release共近停0.44s，均有execution_recovered；本轮未记录execution_transport_unavailable，但单轮不能证明稳定消失。
- 证据：`test_artifacts/review_user_changes_20261022/v7_trial_20261022_223039/{fluency_summary,phase_fluency_summary}.json`及同目录events/native样本。以上是现场关闭日志复核，未改生产代码。


### 2026-10-23 v7长停反例与v8候选采样修复（整车待验）

- 先复核v7闭包：77.43s观察步完成后新鲜提示缺席，10s重新观察超时后恢复探索；88.35--101.83s三个frontier共有折返前缀，前四个相邻远点/航向失败即退休，近点被有限预算排除。不同owner重复相同首段是本轮事实，不是跨owner回调误归属。
- 车后中间路点原只用图切线/方位作车头航向，未表达保持车头朝向倒车到同点的可选姿态。公开地图+同后轴+实际Smac受控回放28项：原前4项全拒；最近反向航向返回5.58m/2方向段/等效成本8.58m，采样无碰撞、曲率合规。公开图非精确私有地图，不能等同整车复现。
- `graph_path_bridge.py`为普通Ackermann探索车后中间点增加反向姿态优先，原航向留作fallback，两种都通过现有cache和footprint筛；`graph_route.py`按远/近端点交替，保留同端点航向比较。显式目标/记忆/观察航向与ideal规则不变，4次连续空路径预算不变。新反例3项改前2fail/0error；720离线回调、27几何、457真实ROS通过。生产callback同输入回放短路径通过全路/段数/跟踪检查。
- 运行版本冻结v8（从v7复制，3文件增量、424成员），预检无相关残留/X148锁，180秒目标搜寻已启动。完整效果待关闭验证，无录像/commit。证据与数据流影响详`docs/CONTINUOUS_EXECUTION_FIELD_20261022.md`新增2026-10-23节、`test_artifacts/review_user_changes_20261022/candidate_span*`及v7/route_recovery_audit。


### 2026-10-23 v8两轮未改善，撤回全局重排，v9待现场（最新活动入口）

- 用户更新目标：每个问题至少跑两遍确定根因。v8同冻结版本/相同出生配置完整搜索两轮，085412近停43.38%/净32.23m、085909近停41.84%/净60.48m，均180.18s超时未到达、收尾无残留；无录像/commit。平均约42%近停仍未解决，不能宣布整体改善。
- 同输入新旧callback对照重复两遍确认v8全局重排扰动正常探索：8.014s原第三项10m变4m，现场执行短段，有切短推进风险，但未证明旧10m一定可达；18.087s前三项新旧相同，四方向段换向为原有路径。已撤回v8全局重排/普遍航向转换，graph_route回到v7。旧v8源码另存candidate_span_v8，不丢历史证据。
- v9仅在普通Ackermann探索连续空路径接近原4次上限或原候选耗尽且仍有预算时，补一次最短未尝试点/车后兼容航向，仍完整cache/footprint/Smac/执行检查，不增加失败限额；正常start_segment候选顺序与v7保持，两个录包输入各两遍复核前三项相同。真实Smac同公开地图近点反例又重复两遍，原4项全拒、最近反向航向两遍均5.58m/2方向段/cost8.58259/碰撞0/曲率合规；不是精确私有图复现。
- 另定位pending extension退休契约：following期间后台ComputePath未完，frontier_resolved导致finish_current_exploration_section拒绝复用当前健康动作，随后reset刹停。085412 11.98following→12.16background planning→13.56 retired；accepted/unaccepted规划回应两种反例各重复两遍，改前均失败，障碍控制例通过。v9在原扫掠/新鲜度/动作身份检查通过后只取消未来ComputePath并清serial，当前已接纳动作进入原有界交接；替代FollowPath ACK空窗仍全部隔离，新障碍/陈旧输入仍停车。
- 728离线回调连续两遍通过、真实ROS465通过，v9独立冻结424成员/5文件增量。首次091127在目标下发前控制节点enable参数服务超时、0目标、已收尾；第二次091350启动正常并开始180秒搜索。该间歇启动故障尚未根因定位，未改超时/控制门，不算搜索失败复现。两轮修复后搜索效果待关闭核验。
- 细节docs/CONTINUOUS_EXECUTION_FIELD_20261022.md v8/v9节；证据test_artifacts/review_user_changes_20261022下v8_trial_20261023_085412、v8_trial_20261023_085909、v7_v8/v7_v9_recorded_candidate_ab.json、bounded_short_probe_callback_replay.json、pending_extension_retirement_before_1/2.log、bounded_probe_handoff_callbacks_verified/repeat.log。完整连续执行未验收，保持未提交。


### 2026-10-23 v9两轮关闭：走停仍未解决（最新结论，覆盖待验状态）

- v9真正搜索091350/092847均180秒预算timeout，无完整到达，最后无仿真/ROS/显示残留。完整Native样本近停44.62%/47.62%，最长6.75/13.36s，净60.10/63.35m；严格query_sent对齐有效区间为44.24%/47.37%，与v8同口径42.97%/41.67%均无稳定改善。不能凭轨迹随机单轮证明因果退化，也不能宣布连续性修复。
- 两轮控制归因：换挡28.56/33.04s，无新鲜速度命令23.23/30.51s（planning15.50/16.90），执行保持12.33/7.73s，tracking11.02/7.08s，ackermann_stop4.41/5.19s。近点恢复在第二轮11.21s真实触发并进入FollowPath，但仍有多段机动；新pending-extension退休分支暂无明确独有现场覆盖，保持组件通过边界。
- 三次额外启动尝试091127/091817/092324均0目标，分别SetParameters cmd_vel/enable超时、初始node registry缺适配器、bridge/follow_path超时，未混算搜索轮数。缺节点报告后续可读camera_offset且相机/odom/native数据健康，提示判定时点问题，根因未闭合。正确domain88两个只读GetParameters窗口14/66次ready调用全响应；没复现先前SetParameters超时，不算启动修复。无活跃进程/环境未对齐的旧被动发现全0记录作废。
- 同公开图、同后轴与3个早期航向两遍实际Smac+直接RS+纯前进Dubins：12.95--12.97m前进曲线曲率合规但19--21个采样车身碰撞，min_section .4→.001不改变；不支持用降低方向段阈值/半径或简单禁止倒车解决该例。Native替代一遍成功一遍无路，受控搜索方差仍在。尚未分清挡住前进曲线的格子是真实障碍还是点云/地图更新误标，公开图非私有地图精确复现；未更改物理/碰撞/换挡等待。
- 728离线回调两遍与465真实ROS通过不代表整车通过；当前v9只作待验候选，本机增量SHA仍一致，保留v7/v8回退证据和用户原有dirty工作，无Git commit。详docs/CONTINUOUS_EXECUTION_FIELD_20261022.md末节；本机test_artifacts/review_user_changes_20261022下v9两轮、startup_failure_evidence.tar.gz、query_aligned_fluency_comparison.json、parameter_service_observation.json及early_shunts_probe_1/2.log。

### 2026-10-23 前进曲线障碍格隔离对照（非整车、未改运行代码）

- 继续目标前服务器prestart确认无项目进程/X148锁。读取已关闭v8 085412录包，18.087s前最新公开global costmap，定位三种前进曲线车体内部的lethal格：world (737.125,-26.625)、(737.375,-26.625)、(737.625,-26.625)。这是公开地图近似回放，非请求时私有地图精确快照；边界光栅碰撞尚未全枚举。
- 同期terrain与其源时间TF对齐（TF年龄0、bag年龄.38977s），这三个格子的对应terrain中心unknown=1、elevation_valid=0；周边存在已知与未知混合。unknown不能解释为可通行，不能据此判定Nav2误标。录包32话题没有/wildos/points或/wildos/clearing_points，无法追溯mark/clear来源；下轮诊断需保存这两个来源及对应TF，不能用反复搜索替代来源证据。
- 只在隔离测试内存地图临时清除上述三个lethal格，随后恢复，不影响真实地图/生产代码。两遍同输入结果一致：前进yaw0、yaw2的碰撞采样21/20降至0；yaw1从19降至9（仍有其他边界/中心约束，不能称三格解释全部曲线）。两遍地形定位JSON的SHA256均efdfb381da3843172195a428b33fcbeb05febef0f4fcef8773e4bcf62b7ac9c3。实际Smac同输入yaw1仍一遍成功、一遍拒绝，限时搜索方差未解决。
- 本机证据test_artifacts/review_user_changes_20261022/{blocked_forward_cells.json,blocked_cells_repeat_1.json,blocked_cells_repeat_2.json,three_cell_control_1.log,three_cell_control_2.log}；脚本audit_blocked_forward_cells.py、诊断replay_candidate_span.cpp。未启动新整车、未改物理/半径/障碍/换挡等待，无commit；完整连续执行仍未通过。

### 2026-10-23 点云来源两轮现场与原始点云回放（已定位分类缺口，未修生产）

- v9_point_evidence独立包装器引用原v9冻结424成员，导航/物理/换挡参数不变；只为rosbag增加原始LiDAR、适配后点云、/wildos/points、/wildos/clearing_points、/tf_static及两张costmap_raw。包装器两遍调用边界测试通过；首次入口在启动仿真前因缺少候选目录链接失败，补齐只读symlink后进入两轮。未把该入口失败记为整车轮次。无默认录像、无commit。
- 095734启动52.46s/搜索180.03s/关闭23.24s，100251启动50.88s/搜索180.20s/关闭22.03s；均timeout、未到达、session errors=[]、console_exit=0，结束后prestart processes=[]、X148锁不存在。记录文件Native覆盖180.03/179.95s，近停84.43/84.09s=46.90%/46.73%，最长11.35/10.30s，净61.26/58.42m、采样路程81.61/80.22m。不是严格query窗口比较，也不是修复后效果测试，增加采集负载不能当资源受控性能A/B。
- 近停控制归因：换挡27.87/18.64s；stale_command32.81/40.65s，其中planning16.26/32.69s，首轮reobserving9.60s；execution_hold7.24/8.34s；tracking7.77/8.90s；ackermann_stop6.80/4.57s；其余before_control/disabled1.94/3.00s。tracking是状态标签，不证明正常运动；stale_command不直接等于网络延迟。
- 老v8三个精确格子本轮均未重现100/lethal；第一轮仍反复99高代价，第二轮约15秒短暂99后清除。不得把旧三格失效与本次邻域分类反例混为同一条现场因果链。第一轮前40s点云各282--284条缺TF1帧；第二轮适配点云280条无缺TF、mark/clear296条各缺TF15帧（均须排除，来源未查）；两例受控回放均源TF/后轴TF年龄0。
- 已定位pointcloud.py::split_costmap_points的远距分类缺口：8m内按拟合地面残差，8m外退回body-Z>=.1m；相同低起伏表面随车体姿态可能进入mark。095734源35.467485s邻域11标记、100251源14.793895s邻域9标记；实际原始点云+对应TF+原过滤参数各重放两遍，输出与录包完全相同（最大坐标差0）。标记点世界Z跨度均约.000343m，距近处地面拟合延伸面的高度分别.0509--.0562m/.0372--.0411m，低于.1m。仅诊断改为level-frame分类两例均0标记，表明非TF传输延迟造成该分类反例；不能由此证明全部环境格误标，不能直接外推平面或把所有水平表面当路。
- 当前未改生产过滤器：通用修复需要用实测地面连续支撑处理远距返回，保留不连续台阶/凸起/未知区域；后续须验证坡道/断层/真实障碍、mark-clear一致性及处理耗时。不能简单半径8改40或将世界高度阈值替代道路判断。走停还有规划等待、路线交接等部分未闭合。
- 本机紧凑证据test_artifacts/review_user_changes_20261022/point_trial_20261023_{095734,100251}/{result,fluency_summary,control_stop_attribution,point_provenance,point_provenance_nearby,marked_surface_replay}.json；服务器完整bag留v9_point_evidence/full/trial_*。诊断脚本prepare_point_evidence.py、trace_point_provenance.py、replay_marked_surface.py；本机py_compile通过，回放每轮2次通过。导航代码保持v9候选，完整目标/连续执行未通过。

### 2026-10-23 实测远距表面支撑修复候选（v10，现场进行中）

- 没有直接把拟合范围8改40、降低障碍高度或改用水平高度阈值。诊断两遍确认10cm网格连续支撑即使扩至40m仍无法连接到远处稀疏表面；.25/.5m网格对照仍无支撑。远处8m局部拟合会混入不同高度区域，不能直接代表目标表面；这些反例说明仅靠水平坐标消除标记不是有效修复证据。
- pointcloud.py增加supported_far_ground_returns：先按原body-Z取得远处mark候选，只有近处参考高度差<原障碍阈值且独立4m局部实测拟合通过，局部/近处法向坡度差受原max_slope约束、拟合高度也一致、点位在实际patch范围且距局部表面<min(.05m,障碍阈值/2)，才移除mark。不创造clear点，不将未知变成free。局部拟合保留原>=100回波/full-rank/鲁棒残差等检查，最小独立.5m格12（每平面系数4个格）；近处拟合仍原30格，近处连续支撑及无支持远处原gate不变。地形/occupancy共用split函数，未单改Nav2配置。
- 新增7项回归：远距连续低坡、真实.16m抬高面、盲区后的孤立抬高面、稀疏远点、无拟合、clear逐点不变、乱序不变。改前新增前5项两遍均2fail/0error；改后7项连续两遍通过。已有点云23、地形13、适配器32项及导航回调728通过（最初点云模块测试因缺PYTHONPATH产生1error，补正确包路径后通过；不存在的test_mapping.py发现0项不计通过）。
- 095734/100251真实原始帧各两遍新旧回放，精确TF与源时间不变：区域mark11/9→0/0，整帧总mark1461→1384及1418→1356，clear数组逐点一致；两次分类总耗时旧约30--32ms/新约35--37ms，即单次约增加2--3ms。没有用这两帧证明所有被移除点都是真路面，仍须真实障碍与整轮回归。原实现保存pointcloud_before_support.py；新旧证据supported_far_ground_replay.json。
- v10_point_support隔离冻结425成员，仅修改pointcloud.py和新增相关测试；两Python包构建成功、495项真实ROS回归通过，源码SHA9f34efce86ef9fe8def64b7f925308e61bb14de6a737d40d4b568ce2bfd3beb9。v9与原用户dirty工作保留、不提交。第一轮trial_20261023_102730已启动，搜索180s、启动另计、无录像，现场结果待关闭。本轮启动前其他GPU作业显著高于v9两轮，不能把不同资源/路线的指标变化直接归为修复收益或退化。

### 2026-10-23 v10两轮关闭与用户5米到达新规则（最新活动入口）

- v10修复后102730/103241两轮已关闭，分别启动57.43/49.79s、搜索180.20/180.16s、关闭22.63/38.24s，均timeout、console_exit=0、session errors=[]，完整目标验收失败；收尾prestart无残留。Native有效覆盖177.81/178.39s，近停74.77/94.61s=42.05%/53.04%，最长7.60/24.60s，净61.26/61.23m、采样84.97/75.73m。不能把前段31%快照当整轮改善；两轮没有稳定收益，不将v10升为已验收基线，不commit。不同轨迹及GPU其他作业负载变化，尚不能证明差值由点云修复单独造成，亦未满足不回退验收。
- v10控制归因：换挡27.69/23.37s，stale_command20.69/51.03s，其中planning13.35/16.59s、第二轮path_changed_before_execution19.66s/reobserving6.89s；execution_hold5.97/4.93s，tracking9.27/8.07s，ackermann_stop8.53/3.95s。两轮靠近后都有接近超时、车体footprint/collision_cell拒绝和visible_target_no_vehicle_path，点云组件修复不能宣布全链路修好。
- 第二轮71.917s执行前footprint_collision触发path_changed_before_execution，直到92.106s恢复planning，相隔20.189s。代码execute_checked_path拒绝后block_current_route默认backoff=20s，on_route在期限前不因地图改变解除；这是该长停直接来源。一轮现场反例，独立重复/恢复测试尚未补，未修此状态逻辑。不能靠放宽到达标准替代该恢复缺口。
- 用户最新要求目标5m内算到达，已进行本机连通修改（尚未部署）：GraphPathBridge默认object_arrival_distance=5并将有效半径上限固定5；新鲜/有效/身份匹配的三维确认在半径内不再要求standoff近至.6m，允许无可执行standoff；超过5m不因map-clamped位姿放宽。GoalStandoff对半径内有效几何可发布search_confirmed=true且approach_goal_ready=false；质量、目标query/instance、记忆区域及原确认新鲜度不变。携带旧位置的approach_refresh仍不能用于成功。
- 新object_found事件arrival_basis=confirmed_object_radius，验收arrival_status_matches在该模式核对vehicle_position与本次confirmation.object_position的实际水平距离、报告距离一致且<=5m；standoff为诊断，可为空。旧无basis历史事件仍按旧规则解释，避免改写旧证据；旧OBJECT_SURFACE_ARRIVAL_DISTANCE_M=4.5与clamped6m helper仅供该历史解释，当前新模式不使用该放宽。
- 新增3个桥接边界反例（精确5m/5.01m/半径内无standoff）修改前两遍均3fail/0error。补2项上游确认及1项验收来源回归，更新4个旧阈值/终态语义断言；734项离线回调连续两遍通过、arrival_criteria10项通过。未运行本次更改的真实ROS/服务器/整车，未commit。最新离线日志arrival_radius_callbacks_verified.log与arrival_radius_callbacks_repeat.log。
- 同步只读核查v10两轮object_confirmation与odom，均没有带object_position的可用确认条目，不能据此说“只改半径就能让上两轮成功”；当前还需追查三维确认形成及接近路径恢复。本机现场证据support_trial_20261023_{102730,103241}，包含result/fluency_summary/control_stop_attribution；原始bag在服务器v10_point_support/full/trial_*。本次5米修改仅本机，v10服务器冻结仍旧规则。

### 2026-10-23 无新鲜命令：起点阻塞恢复与剩余停顿来源（v17，未验收）

- 复核v16_trial_20261023_184334闭包：最大Stamped Twist间隔24.683s，车辆footprint先报`start_footprint_blocked`并进入20s退避；随后新鲜全局、局部costmap均显示当前位置footprint清空。停止看门狗判断正确，不应复用旧速度或放宽新鲜度门。
- v17从冻结v16隔离复制，仅改`graph_path_bridge.py`与`test_graph_path_bridge_routing.py`。新全局地图到达后，只有同一navigation epoch/query/route goal仍有效、真实车体全局和局部footprint均清空、odom/costmap/本地costmap新鲜、控制传输与规划/FollowPath action可用、无执行/复位/恢复所有权时，才清除20s块锁并重新调用`start_segment()`，沿用全部规划和执行检查。碰撞、陈旧输入及动作租约不变仍停车。
- 离线回调754项通过；冻结v17构建两包成功，真实ROS 513项通过。用v16闭包Ackermann状态、odom和全局/局部costmap回放，事件后约0.412s两张地图均无车身footprint blockers；v17 `on_map`调用一次`start_segment`，发布`blocked_start_cleared`/`fresh_costmap_footprint_clear`并清除blocked_until。精确回放证明这个旧反例会及时重新规划，不代表现场路径已完成。
- 两轮v17目标搜寻均180s timeout、未找到绿色房屋、bag正常关闭，收尾prestart均无ROS/BeamNG/RViz/rosbag进程或`:148`锁；未录视频。trial 185826：启动36.41s、搜索180.154s、收尾22.83s，near-stop27.42%、最长6.99s、净位移46.12m、stale samples352/3686、最大Stamped命令间隔5.570s、最高命令源龄5.563s。trial 190719：启动37.55s、搜索180.169s、收尾38.24s，near-stop46.19%、最长14.30s、净位移59.04m、stale samples607/3683、最大命令间隔7.467s、最高源龄7.419s。两轮均未出现`start_footprint_blocked`，现场没有触发新恢复分支；轨迹差异大，不能宣称稳定性改善。
- 剩余新鲜命令空窗来自路径结束后的串行GraphNav/frontier交接和多次ComputePath成本比较。trial 185826最大5.57s为`follow_aborted`、可见目标`no_vehicle_path`后转回GraphNav再规划；trial 190719最大7.467s从`exploration_segment_finished`开始，经过costmap等待、`empty_path`候选拒绝和frontier选择ACK后才FollowPath。不能放宽控制新鲜度；下一步应评估提前准备可执行续段或缩短串行候选比较，同时保留WildOS选择、车身与障碍校验。
- v17候选根目录`/home/ly/execution_continuity_review_20261022/v17_costmap_unblock_retry`；服务器闭包`full/trial_20261023_{185826,190719}`。本机审计在`test_artifacts/fresh_command_retry_20261023/{v17_trial1,v17_trial2}`，回放清单在该目录根部，754项离线报告在`test_artifacts/offline_navigation/callback_report.json`。全流程未通过，修改未commit；后续先核实冻结候选与当前用户工作区差异，再处理路段结束处的串行规划空窗。

### 2026-10-23 v18候选比较提前完成与桥接odom排队证据（未验收）

- 详`docs/COMMAND_GAP_COMPARISON_20261023.md`。两轮v17关闭录包各分析两遍，5个比较中剩余候选的理论最高分已无法胜出，却继续耗时0.71–2.58s。v18只改exploration_cost及相关测试：成本下界包含原起点.05m/终点1m容差，严格低于当前可行路线才停止剩余比较；仍重复当前地图几何验证并等待GraphNav新路线提交，评分、碰撞及命令新鲜度门不变。先比较上界，再作必要几何验证，避免无收益的额外计算。
- 反例改前两遍2fail/0error；759离线回调通过，服务器518真实ROS通过、两包构建成功。5个历史实际获选路径连同原GraphNav身份、起点、odom、地图回放两遍：选择一致、几何通过、无需继续候选规划。两个旧过滤测试调整备用benefit以保留竞争性，仍验证footprint/失败缓存过滤。完整运动效果不能由这些组件结果替代。
- 冻结v18源自v17，425成员只改两文件，保留旧版本和本机其他dirty改动。现场193041/193614均目标搜索180s timeout未到达、关闭正常，复核无项目进程/X148锁；无视频。Native覆盖178.082/180.517s，近停42.20%/35.34%、最长9.998/4.653s，净71.034/59.746m，采样88.303/105.428m；stale samples502/3678、226/3729，最大命令间隔6.525/3.205s。新分支3/1次触发；整体仍不流畅，不宣称稳定改善/不回退已验收，不commit。
- 第一轮134.30s与179.50s桥接报stale_odom为1.03/1.14s；关闭bag逐消息各复核两遍，之前1s仍有10/9条odom，最新源龄.096/.310s。134.30s从722姿态ComputePath结果到planned约.94s，之后.005s即stale_odom并取消FollowPath。支持查桥接同步计算和odom回调排队，尚待核验帧/姿态/递增戳准入及耗时归因，不先改接收/调度。第二轮没有stale_odom，仍有3.2s左右动作/规划交接空窗。
- 候选根`/home/ly/execution_continuity_review_20261022/v18_comparison_bound`，本机`test_artifacts/fresh_command_retry_20261023/{v18_trial1,v18_trial2,v18_recorded_replay_1.json,v18_recorded_replay_2.json,v18_real_ros_report.json,candidate_delta_v18.json}`。现场其他GPU作业负载较高。审批额度曾阻断一次新上传，用户手动允许后两轮结果及收尾已读；当前无正在运行的本项目测试。

### 2026-10-23 v19减少同步派发重复检查（整车待验）

- 补完v18两次stale_odom准入核验：最新消息frame/姿态/源时间戳均可准入且比实际使用消息更新；两遍审计SHA一致。源端未断流，桥接在同互斥组处理路径约.94s后读旧odom并取消新FollowPath。两条实测路径隔离回放显示重复整条路径检查/初始跟踪占约.15–.16s；隔离总耗时.46–.51s，未原样复现现场stale，详docs/COMMAND_GAP_COMPARISON_20261023.md。
- graph_path_bridge只复用同一同步状态回调内、同一路径及同一通行范围的成功检查；保留旧航向候选和不同fallback通行范围复验、派发前时间新鲜度、执行段全局/局部碰撞、跟踪及动作/传输门。未改订阅并发、未放宽1s门限，不跨回调缓存。
- 改前两遍2fail/0error；新增5项及更新重复终态派发断言后764离线回调两遍通过、548真实ROS通过、两包构建成功。两条实测路径各两遍回放分别.334/.354s、.311/.306s，完整路径检查2→1、各一次派发，执行段检查仍在；只证明组件耗时降低。
- 冻结v19_dispatch_reuse从v18派生425成员，只改桥接实现及2测试文件，本机5m到达等其他dirty改动未混入。已预检无项目残留/X148锁并启动180s现场搜索，无视频、无commit；整体近停与完整目标验收仍待关闭结果。证据test_artifacts/fresh_command_retry_20261023下recorded_dispatch_profile.json、v19_real_ros_report.json、candidate_delta_v19.json和dispatch_reuse_*。

### 2026-10-24 v19仍失败，v20停车与规划重叠候选（组件通过，现场待验）

- v19两轮关闭，先前待验状态由本条覆盖：223800搜索180.168s超时，Native178.299s中近停42.874%/最长9.251s、净58.322m，stale_command32.053s/换挡24.260s/最大命令间隔10.850s；224340搜索126.258s因Native lease expired提前退出，Native124.047s中近停37.890%/最长9.861s、净59.301m，stale_command16.981s/换挡18.181s/最大间隔7.644s。两轮无stale_odom、收尾无项目进程/X148锁，但整体未改善验收，不commit。
- 逐消息去重审计两遍SHA一致4e5bde212ca1baf16571b8a3529139cec81ac1f91c6cbf68420b7d2fbe1f7247。首轮10.85/7.35s空窗从新视觉意图取消旧动作开始，约4.30/1.68s后才新规划；停车/控制释放与计算串行。旧路线可能远离新观察点，不能靠保留旧速度消除空窗。第二轮980.139ms控制RPC、978.859ms轮询阻塞越过.8s车端租约，与20261021的813ms链相同；引擎阻塞源仍未闭合，未放宽保护。
- v20允许新意图的ComputePath在旧动作停车隔离期间运行；仅保留一份带epoch/query/目标/模式/路线身份的结果，旧终态、持续停车和释放ACK后按实际车姿裁剪，重新检查完整路径/最新地图，再走执行段原准入。无法接入停车位姿则重新算，换路线/目标/epoch丢弃，输入陈旧/控制故障拒绝；原5s规划期限在隔离期间仍生效。
- 9反例修改前两遍均9fail/0error；新增11项后775离线回调连续两遍通过，冻结候选两包构建成功、559真实ROS通过。只改bridge/continuous_path及两测试文件，从冻结v19派生425成员，本机其他dirty改动保留。实际录包交接回放与现场A/B待验，整体近停、完整目标与不回退未通过。详docs/COMMAND_GAP_COMPARISON_20261023.md末节；证据test_artifacts/fresh_command_retry_20261023的v19_trial1/2、v20_real_ros_report.json、stop_preparation_*、v20_*.diff。

### 2026-10-24 v20 A/B失败并撤回（最新活动入口）

- 四轮A/B/A/B各180s已关闭，无完整到达、无录像；每轮均先验无项目残留/锁再读封闭bag。v19 153126近停42.05%/最长7.50s/净58.80m，153959为21.17%/5.68s/73.13m；v20 153554为39.53%/6.81s/59.72m，154411为45.16%/7.95s/62.29m。v20没有稳定收益、不回退未通过，不能推广。旧版21.17%不是稳定承诺，同代码另一轮仍42.05%；不同路线/观测及其他负载影响尚未隔离。
- 新版差轮与旧版较好轮的机动段切换15/5、换挡近停39.82/15.23s；主要差距在多换向路线/执行，不应继续只优化毫秒级派发。v20提前计算确实触发，但整体近停仍不合格。
- 审计中另复现停车计算错用新source代际ACK门：hold/release正常过渡会被start_segment当执行故障；两新反例各两遍失败，计算改用for_stop健康门、执行保留新代际ACK的本机候选778项离线两遍通过，未部署。该修正也与v20实验一起撤回，不能记成已上线修复。
- 按用户“不回退”要求，已用精确函数/文本核验撤回停车预规划四文件改动及对应测试，不执行整文件Git reset，不动用户其他dirty工作；保留5m到达候选、v19检查复用及历史修复。撤回前全源码（含未部署代际修正）保存在test_artifacts/fresh_command_retry_20261023/v20_rejected_local；v20_withdrawal_receipt.json记录前后SHA。恢复后764项离线回调连续两遍通过；正式服务器未替换，此刻无活动仿真，无commit。
- A/B证据本机test_artifacts/fresh_command_retry_20261023/stop_plan_ab_results.json；服务器execution_continuity_review_20261022/stop_plan_ab_20261024/results.json。冻结v19/v20与原始bag全部保留。后续先固定旧版较好/较差轮的车姿、目标姿态、地图检查方向段/候选选择，至少复核两遍再改，不继续盲目整轮搜寻。
- 当前向用户提出的建议合格线：搜索阶段近停≤15%、目标≤10%，单次规划/交接2–3s内、无5s以上命令断档，至少两轮且探索不退化；启动、成功后停车及收尾不计。这是项目建议，当前无轮次达到，完整目标另行验收。

### 2026-10-24 v21输入诊断与v22 odom接收隔离：整车仍不合格（最新活动入口）

- 用户要求集中解决频繁换向和无新鲜命令，每项原因至少确认两遍，不能以组件通过替代不回退/整车改善。21.17%旧轮是真实已有成绩，后续未稳定保持，不能称已改善。v20撤回保持有效，未重引入停车预规划。
- 更正旧文“15/5次切换”：它只是maneuver_done事件数，不是全部换向。四轮关闭bag的FollowPath接纳段几何匹配审计两遍SHA一致cf6e3641f12c6714d3793a05451eaa7bc8bdef60e381fd8b2a39f1caafe0ee9d；v19两轮实际相邻段方向翻转13/8，v20为16/20，其中无法唯一匹配完整路径的分别0/0/3/1，不能把歧义部分当唯一归因。旧较好轮后半段走了不同区域，路线/观测变化仍未隔离。
- v21_planning_inputs_r2仅记录输入身份、请求goal/start、use_start与原路径源戳；use_start=false时桥接odom不是C++实际起点，实际以返回路径首点为准。768离线回调两遍、552真实ROS通过。现场162748近停44.06%、最长11.15s、净52.69m，180s超时；收尾清空，无视频/commit。闭包源身份审计两遍SHA一致1be4da0972037ae91a477bcf67c7dbec2a1351438a7d24384ca089e1554c8ebf；177状态中41条消费源戳落后已录到且可准入最新odom≥.15s；70.584s planned位置差.596m。143.940s换向后footprint拒绝时消费odom已是最新，不能把此拒绝归因odom队列。
- 真实ROS导航互斥组阻塞.65s反例两次均2fail/0error：旧版前进源龄约.65s、倒车停姿误差约.385m。缺自定义消息的第一次环境错误单列保留，不计复现。v22独立组只接收/校验odom入有界32帧队列，原导航互斥组GuardCondition及规划、派发、终态/观察边界消费；接收不得直接改导航状态，不延长源戳寿命，不放宽epoch/ACK/碰撞/转弯半径/控制看门狗。reset前消费旧epoch轨迹，保留反馈归属。
- v22从冻结v21r2派生，5文件增量（bridge、visible_approach、target_observation及两测试），未混本机其他dirty改动或5m到达候选。775离线回调两遍、561真实ROS与两包构建通过；上述真实ROS两反例修后各两遍通过，且断言阻塞期间导航odom不被并发改写。只是调度组件验证，不代表全链路问题解决。
- **v22两轮已关闭且失败**：165941启动55.80s/搜索180.169s/收尾28.84s，近停38.42%、最长8.487s、净60.93m，换挡24.69s、stale_command22.94s；170417启动37.56s/搜索180.173s，近停36.82%、最长5.813s、净53.47m，换挡27.18s、stale_command19.14s。均timeout、无目标到达、console_exit=0，session errors=[]，外层超时错误保留。两轮后prestart进程列表为空、X148锁不存在，其他GPU作业未动；未录像、未commit、不推广为验收基线。与旧21.17%相比没有守住最好表现，不能用测试数宣称推进了整车近停。
- 两轮封闭bag再次各审计两遍，SHA分别db157a3bc75be42e5b463b4e9af5bd2c83dbf5d6acd5f3ebf70fb7e2c1d11388、8534785ceebc240750b1f806eb18d070acb135657573d76bd247a1d7b0dcf121。202/203状态仍有31/21条消费源戳落后≥.15s，其中位置差>.2m为11/9条；状态发布时源身份对比不是DDS接收跟踪，也不一定代表派发前检查时同等落后，不能称新鲜度已全面修好。换向后footprint拒绝1/5次中5次消费已是最新，另1次源差.088s/位置差.0064m；这些具体拒绝仍需地图/几何复现，不能再用odom旧值解释。
- 当前未解决：短观察点/航向约束为何产生多方向路径、换向后最新地图对续段footprint拒绝、规划/候选/控制交接导致数秒命令空窗。下一步应固定旧好/差轮的相同输入做规划与几何回放，确认两遍再改，不能继续盲目整轮叠加交接分支。
- 本机证据：test_artifacts/fresh_command_retry_20261023/{direction_choice_audit.json,v21_trial1/,v22_closed_results.json,v22_165941_consumed_odometry_audit.json,v22_170417_consumed_odometry_audit.json,v22_real_ros_report.json,candidate_delta_v22.json,odometry_before_1.log,odometry_before_2.log,odometry_ingress_callbacks_2.log,odometry_ingress_callbacks_3.log}。服务器冻结候选v21_planning_inputs_r2/v22_odometry_ingress及其full/trial_*保留；汇总odometry_ingress_trials_20261024/results.json。无活动测试，用户原有未提交工作保留。

### 2026-10-24 固定输入两遍：位置目标航向导致的短折返，v23候选待整车验收

- 上一目标轮完成了两轮关闭结果与两次一致审计，属于新增证据；整体近停仍失败。继续按频繁换向/命令空窗处理，未降格目标。
- 从v21r2/v22三轮关闭bag提取9组实际完整路径起姿、请求目标与按源戳匹配的公开global costmap；manifest连续提取两遍SHA一致6178e6f8e96c51bcd73a6b3e68e3b412802b3eb9c79c9eaff7596516757cd421。公开地图逆量化不是C++私有锁定快照，且候选比较可能保留更早回应，不能声称9组均精确重放原现场路径；不改地图、不删障碍格。v4与v22的Smac/connector源码逐文件一致才复用构建库，实际半径8m，不复用旧12m/三格清除诊断条件。
- 明确同输入反例case3（v22 165941，request52/candidate0）：后轴(734.5633,-66.3248,-1.34225)，同目标(737.09164,-71.60229)，旧yaw=-1.19555。实际Smac在同公开地图两遍均为2次换向/cost14.287；仅改终点yaw为单圆弧到达yaw=-.905797（半径13.51485m），两遍均0换向/cost5.8795、blocked0/曲率通过。空地图控制旧yaw也可单段，说明航向与地图共同作用，不是所有换向都由航向造成；其他案例存在地图主导、改yaw反而变差，不能普遍替换原航向。
- 改动仅graph_route增加forward_arc_arrival_heading；bridge在原普通候选完成比较、最佳路径仍多方向且为前方位置目标时，至多增试同点一个满足既有最小转弯半径的单弧到达yaw。保留原可行路径/原评分，失败或较差回原路径并按最新图重验；已有近似候选(.1rad)不重复，单方向成功路径不增调用，moving replacement、memory、confirmed及显式frontier观察航向不变。仍通过Smac、全/局部地图、跟踪、动作/新鲜度等原门，不直接生成/派发捷径。
- 两新回调反例修改前各两遍2fail/0error；补正常单方向不延迟、优选单段、失败回退/地图改变拒绝、显式朝向保留、重复/受阻候选及运动替换隔离共9测试后784离线回调连续两遍通过，29几何测试通过。一次新增记忆夹具漏expected导致2error，补齐合法夹具后重跑，未以生产改动绕过。冻结v23_arc_heading从v22派生425成员只改实现2+测试2文件；两包构建、570真实ROS通过。
- case3两组原插件实际返回路径，经真实bridge回调各回放两遍：原3方向段和新1方向段均过全路、局部与初始跟踪；保留旧候选、选择新路径、一次FollowPath派发且queue1段。动作传输隔离，不是DDS/车辆到达，地图/odom身份与原戳保留；回放手工送入已计算结果，不代表现场请求/地图等待时延已消失。证据arc_bridge_contract_replay.json。
- 两轮180s现场测试已启动（无录像）；首轮172932已关闭、超时未到达，近停36.18%，不能称稳定改善。第二轮173348进行中；关闭后检查进程/锁再读bag，不commit、不升为已验收版本。
- 命令空窗v22两轮各复核两遍SHA分别b6bbe5b7b4bdaf2a61aae4f3dde4066e381dbaf10b50eb3d7d48694cb5390327、bd0664731fb57ac42a14194f1aa45addb6d71656b48be5e69075a0843e5418cb。7.077s为旧段结束后接入拒绝→串行空路径候选/评分→GraphNav选择→新动作；5.653s与4.025s均含换向后footprint拒绝→空路径→释放视觉接近→GraphNav再选，另有4.188s新视觉意图取消旧动作/停车释放ACK/规划。不是源网络断流，不放宽看门狗。
- 复核历史20261021已记录2s换挡确认，在当前v22仍存在：两轮14/14段gear_change，其中低速≤.15m/s等待共24.246/26.350s；多数完整段1.95–2.00s、方向请求符号稳定、命令剩余寿命约.4s且随后tracking。审计两遍SHA均0c6dace8a6b0cc950b16605a77823444a9ca21c6ed41b3577661596165824988。这不是新发现的历史原因，而是确认当前软件等待仍占主要成本；目前未修改2s参数，须与路径改动分开验证。
- 证据目录test_artifacts/fresh_command_retry_20261023：fixed_heading_manifest.json、arc_bridge_contract_replay.json、arc_before_1/2.log、arc_callbacks_3/4.log、v23_delta.zip/v23_real_ros_report.json、shift_dwell_audit.json；服务器fixed_heading_replay_20261024保存全部输入、各两遍插件日志、路径和bridge回放，arc_heading_trials_20261024保存整车汇总。完整目标、近停≤15%及不回退尚未通过。

### 2026-10-24 用户要求优先对照原仓库（只读分析，覆盖v23进行中状态）

- 用户给出 https://github.com/nasa-jpl/nebula2-wildos ，说明此前同用BeamNG可流畅运行，要求先找差异与原因。按此停止继续功能修改/参数试验，完成源码和已有闭包对照；未作新服务器试验。详情docs/UPSTREAM_EXECUTION_COMPARISON_20261024.md。
- v23两轮已结束且失败：172932近停36.18%/最长8.30s/净52.35m，gear26.60s/stale17.69s/hold6.50s，arc probe2次；173348近停42.14%/最长见本机闭包/净57.21m，gear26.71s/stale23.94s/hold9.60s，probe0。均180s超时、无到达、console_exit0、session errors=[]，外层超时保留；postcheck进程空/X148锁无。v23_closed_results.json已保存，未录像/commit、不推广，不能称稳定收益。
- 前一阶段拟将换向确认2→.3秒的整组工具调用因自动审批额度耗尽未执行，physical launch实读仍未覆盖默认2秒，未部署v24。此前新增4个参数候选测试保留，其中两项改前反例各两遍2fail/0error；当前不能沿用早先784项通过宣称全工作区已绿。本次只读分析没有实施该参数更改。
- 参照8文件字节核对既有provenance（源提交ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572）全部一致；Git对象当前受限，未重新独立校验blob。在线官方follower/launch/planner_node与本机对应逻辑核实。source_and_evidence.json两遍SHA一致23fe1d2ad67d1c0ffbb7e238b7203a58510d151b8d5a54692881101d5c346a66；逐行diff保存在test_artifacts/upstream_execution_comparison_20261024。
- **关键接口差异**：原path_follower按odom发前视Pose、官方launch前视5m；当前仍启动它发/spot1/goal_pose，但实际GraphPathBridge订阅/spot1/graphnav_planner/path，没有订阅该前视目标。本项目adapter该话题只发现health_check订阅，实际控制由自写候选ComputePath→分方向FollowPath→交接闭环接管。不得再说“原文执行完全不动，只加记忆”。
- 原planner_node168→605行、planner399→635、mapper771→1165、视觉nav638→1594；基础goalagnostic_scoring与GoalNavigator对照一致。行数仅说明范围，不证明缺陷。新增主动探索owner/退休/车辆成本选择改变行为，记忆影响排序，但关闭记忆不会消除车辆分段、2秒确认与交接。原仓库外部依赖Nav2，不含其完整机器人/BeamNG执行配置；还缺用户原先流畅轮次的精确版本与车体/控制配置，不能称已完成同条件新旧整车对照。
- 与近停对应的主要机制已对齐：非车辆可达性证明的图/观察点+终点yaw约束导致部分短折返；每个换向末端准确停车后另有2秒软件确认；车辆候选比较明确在无活动FollowPath且speed≤.15时串行运行；frontier退休/视觉意图改变在无法连续接续时触发reset/停车/释放ACK。7.077s/5.653s/4.025s/4.188s旧空窗与此对应；既有两遍重放不能泛化为全部换向因果。原follower数学复核两遍仅为规则复现，不是整车；粗节点间距也可使前视点暂时不推进，原代码并非无条件流畅保证。
- 当前最有证据的总体原因是新执行闭环把图探索/观察意图转换为大量短车姿任务，再由停车事务串行交接；ROS2排队和BeamNG RPC阻塞仍有独立反例，但不是这些软件等待的统一解释。工作区名义规划12m/执行8m与历史冻结现场报告需分开，未现场核对，不据此认定退化根因。完整目标和不回退仍未通过；活动目标保持，后续先按本次对照继续定位，不盲目追加保护分支。

### 2026-10-24 持续探索替代短任务执行：可行性与控制器契约（分析）

- 用户询问能否把持续探索作为底层机制以提升流畅度。源码核实后方向可行：长期搜索会话、探索意图与运动所有权分离、后台准备路径、当前实车/最新图再准入并连续接续，保留WildOS图评分和记忆对目的区域/候选的作用；无需为此替换图搜索算法或删除记忆。frontier退休不应无条件变成制动，普通中间点不应都作为停靠终点。
- 新确认的硬边界：ackermann_rpp/src/controller.cpp::setPlan明确拒绝跨换向多方向路径并锁定单段方向。不能仅删bridge.motion_queue就将整条多方向路径直接送现有RPP；必须成组协调控制器、方向保持、cusp/真实停稳与动作交接契约。可先保留必要换向分段，把同方向路径更新改为连续接续。当前ContinuousPath仅为受条件限制的局部前进接续，不是完整持续探索机制。
- 理想目标为正常探索持续产生新鲜命令，普通路径/前沿更新不中断控制；当前安全路径余量不足、新障碍、必要倒车或控制故障仍停车。不同地图/旧起姿的后台结果不能直接执行，也不能重发过期速度掩盖空窗。详UPSTREAM_EXECUTION_COMPARISON_20261024.md新增接口边界节。此轮仅分析和记录，未修改运行代码、未部署或新增整车测试，完整目标仍活动。

### 2026-10-24 用户授权保留基线，持续探索v24隔离试验（现场进行中）

- 已保留本机当前未提交代码、配置、测试与HEAD/index/worktree差异：test_artifacts/continuous_exploration_20261024/baseline_source.zip，1883文件/约105MB原始内容，SHA142b2f90b193c4b2fbd3dfa9aedf0b5546549130565e6afcd1fdfaa8dd4a6859，ZIP/所有成员/源文件逐项核验，HEAD affb478704b13a4190542bddedb8a6a78bdd444c。另补9份测试夹具归档与SHA清单。模型、录像、历史运行产物未删；主目录运行源码未覆盖，无Git commit。
- 详细机制、备份范围、验证和回退边界见docs/CONTINUOUS_EXPLORATION_TRIAL_20261024.md。本机候选在同名test_artifacts目录/candidate，独立continuous_graph_path_bridge入口；服务器从冻结v23只应用7文件调度增量生成v24_continuous_exploration（427成员），原v23和其他版本保留，未混本机5m等其他未部署改动；到达、车速、2s换向确认、车体碰撞、Smac/RPP/Native配置不变。
- 意图与已接纳运动分开：新frontier/同query视觉接近可以取消旧计算、后台准备新路径而不取消健康动作；接纳新路径时沿用实际车姿/最新图/源所有权门。普通执行不再先停稳作2–4个frontier车辆路径锦标赛；WildOS/记忆图评分保留，位置目标优先兼容曲率yaw，显式观察/记忆姿态保留。真实倒车/cusp仍停稳分段，不把整条多方向路径直接送现有RPP。
- 旧动作失败有独立不可变意图/epoch/serial归属，不能拒绝新未执行frontier；旧终态即使已知也保留真实停车/ACK屏障。用户query切换/停止/消息或控制失效沿用原停车。仅有限已检查当前段可继续，旧尾段不继承新意图权限。
- 改前旧代码两反例两遍均2fail/0error；新增20项持续探索回调。本机完整804离线回调两遍通过，服务器590真实ROS组件与两包构建通过。最初7个夹具缺失错误后补既有数据重跑，失败日志保留。未实施0.3s参数的4测试在候选另存pending_shift_profile_tests.txt，主目录和原归档不删；本次只测持续探索，保留2s配置，不宣称修好原pending调参。
- SSH恢复且预检无残留，A/B/A/B驱动已启动（旧v23→新v24→旧v23→新v24），目标后各180s，启动/收尾另计、无录像；每轮先关闭/确认进程与锁清空再读bag。v24日志必须含continuous_exploration_v1以核实实际执行版本。运行exec会话75542；服务器汇总/home/ly/execution_continuity_review_20261022/continuous_exploration_ab_20261024/results.json。完整目标、近停与不回退仍待验，不推广、不commit。

### 2026-10-24 持续探索首轮启动检查失败，r2隔离修正（覆盖进行中状态）

- 初次A/B/A/B全部关闭，收尾进程空/X148锁无。旧v23 205019/205516近停37.13%/43.10%，gear25.80/29.63s、stale14.29/25.46s，均未到达。新v24 205430/205927均0目标、预检失败，没有进入搜寻，不能作为行驶性能证据；不是算法崩溃。预检只识别旧graph_path_bridge，漏计实际新入口continuous_graph_path_bridge，报expected1 found0/missing DDS owner。
- r2同时补预检owner与console残留冲突入口，旧/新同启仍判双owner。新增4项测试改前两遍均3fail/0error、改后全过，既有preflight7/物理策略9过。实际冻结参数CarLike半径12m，bridge/物理/控制8m；新bridge候选航向按二者较大值，launch从实际配置传入并预检核对，未改C++规划/物理半径。补边界后持续回调21项、完整离线805项连续两遍通过。
- **更正旧回放范围**：此前8m Smac是受控几何反例，非实际12m现场参数精确复现；不能以该回放替代12m现场因果证明，旧case3弧半径13.51m虽满足两者仍须分开解释。旧报告/证据保留。
- 独立v24_continuous_exploration_r2冻结428成员、相对v24为6文件增量，原v23/v24及主目录源码保留。构建、真实ROS回归和后续两轮180s现场待记；无视频/commit。本机证据test_artifacts/continuous_exploration_20261024/{failed_preflight_205430.json,entry_before_1.log,entry_before_2.log,continuous_full_r2_1.log,continuous_full_r2_2.log,continuous_r2_delta.zip}，机制/影响详docs/CONTINUOUS_EXPLORATION_TRIAL_20261024.md。

- r2前置真实ROS检查拦下新增参数在运行时保护注册后声明的初始化错误（21error）；加声明hook在回调安装前执行，运行中仍拒绝更改，新增1项回归。打包基类文件一度误用本机完整bridge，带入未部署到达接口ImportError；构建阶段发现，恢复为原冻结v24+仅初始化hook，未入现场。失败源码/日志保留，最终7文件增量/428成员。806离线两遍和596真实ROS通过后才启动两轮180s；首轮211048进行中，主目录703运行文件/备份SHA再次不变。无录像/commit；现场结果待关闭，组件通过不能宣称近停已改善。

### 2026-10-24 持续探索两轮关闭；下一步集中换向近停（最新入口）

- 新r2 211048/211456搜索180.169/180.145s，均timeout未完整到达、session errors=[]，console正常收尾，postcheck无项目进程/X148锁；未录像/commit。近停32.01%/27.00%，最长4.20/4.59s，净62.07/63.48m，采样105.35/118.49m；gear近停25.12/23.11s、stale5.66/11.39s、hold1.81/1.77s。旧v23对照37.13%/43.10%、gear25.80/29.63s、stale14.29/25.46s。平均stale近停减少约57%，但两轮路线/GPU负载未完全控制，不构成长期稳定/完整成功；也未达到历史21.17%或建议15%线。
- Native<=.5s龄运动样本匹配后台planning35/31次，持续探索在实际运动中参与。>=2s命令间隔首轮0次、第二轮3次2.223/2.366/2.938s；前两例独立序列在planned后7/28ms frontier_resolved进入旧退休/停车ACK释放/再规划，接纳空窗具体归属待查，不能绕过ACK；第三例观察步完成后重新观察/新接近/地图与规划交接。无>5s断档不等于完全无空窗。
- 原主目录703运行文件、baseline_source.zip与SHA再次不变，服务器v23/v24/r2均留存。完整闭包本机test_artifacts/continuous_exploration_20261024/{comparison.json,comparison.md,r2_results.json,r2_closed_runtime_evidence.json,continuous_r2_compact.tar.gz}；服务器continuous_exploration_r2_trials_20261024/results.json。详docs/CONTINUOUS_EXPLORATION_TRIAL_20261024.md最终表。
- 用户最新要求优先集中优化换向近停。旧版同样有25.8/29.6s；r2仍25.1/23.1s，不能说新模式已解决。固定2s确认未改，先在最新两轮核对真实挡位/停车/命令时间，再用同输入两遍反例定位；保留r2命令连续性收益及回退版本，避免混入到达/视觉等独立改动。

### 2026-10-24 换向近停根因两遍复核，v25仅方向去抖参数候选

- r2两轮13/12组gear_change，<=.15m/s低速等待25.6056/23.3014s（与near-stop<.1口径不同）；两遍审计SHA b5800e297a6c2609fbdb57b758c5678775400e84a753507b2cfcb3e8cffb8862一致。多数起速已<.15、单组1.95–2.00s，方向请求稳定/命令寿命约.4s；新gear直到约2.06–2.21s才发送，主要是软件请求前的确认等待，不是已发挡位后变速箱等2秒。示例实际D/R在sent后同次或后1–2样本可见，采样并非精确机械延迟。
- 更正“停稳后再等2秒”的简写：helper计时自相反方向请求开始，时间够且当前speed<=.15才切换；多数组本来近停导致几乎全落低速。Native selector D=2与actual gear_index=1不同，不能按整数相等判断换挡失败。
- 保留r2，从其冻结源码创建独立v25_fast_shift，429成员/仅2文件增量：physical launch方向请求确认2→.3s、新6项测试。默认controller仍2s；真实车体、低速换挡、freshness/ACK/lease、持续探索/到达/视觉均不改。.3是本物理配置去抖候选，不是通用真车参数。改前6项两遍3fail/0error、改后两遍6项通过，其他806离线通过；服务器构建/真实ROS和两轮180s待记。无录像/commit，不替换r2。
- 详情docs/SHIFT_WAIT_OPTIMIZATION_20261024.md；本机shift_profile_before/after_1/2.log、shift_full_regression.log、fast_shift_delta.zip；服务器r2/shift_wait_audit.json、v25_fast_shift。当前用户重点为换向近停，后续先完成本参数的实车执行对照。

### 2026-10-24 v25两轮关闭：单次换向等待减少，整体仍未验收（最新）

- 602真实ROS通过后现场核实confirmation=.3、speed门=.15、cmd/state timeout=.5、stamped=true。213139/213551均180.200/180.176s超时未到达、session errors=[]、console0、关闭后项目进程/X148锁无。gear近停4.28/4.88s（r2为25.12/23.11，平均减少约81%）；总近停29.04/17.57%、最长7.59/3.66s、净59.65/32.01m、采样113.81/151.53m。未录像/commit，主目录703源文件与原备份不变。
- 改后各14组换向请求，<=.15低速等待3.849/3.950s，多数单组.25–.30s；两遍审计SHA a96176d1e57e4526fd6117210c0bf09f91962f34d1f0e545bc91227333680d89一致。只证明每次等待缩短，次数未减少。Native selector与actual gear_index按正负行驶挡位解释，采样响应不是精确机械延迟。
- stale近停27.97/10.99s（r2 5.66/11.39），第一轮其中planning22.01s；5.232s间隔是靠近段完成→候选fallback/空路径→释放目标靠近→GraphNav再规划，4.519s是换向完成→footprint_collision→规划/航向探针。>=2s命令间隔7/3次、行驶中后台规划30/28次。第二轮净位移降而路程增，折返有效性未过；不同路线/GPU负载未完全控制，不能宣布整体无回退，保留r2与v25候选不推广。
- 本机fast_shift_{results.json,wait_audit.json,runtime_evidence.json,compact.tar.gz}（约1.18MB）位于test_artifacts/continuous_exploration_20261024；详docs/SHIFT_WAIT_OPTIMIZATION_20261024.md闭包表。包装审计CRLF在统计两遍成功后报错已修并补归档，不是现场失败。用户最新问题为目标靠近阶段缺口，下一步对齐这两轮目标提示、三维确认、规划与观察交接，先查证不混入换向参数改动。

### 2026-10-24 目标靠近/原文定位链路只读复核

- v25两轮mask话题106/30（不等于人工正确识别数），有效三维定位3/1、source全部lidar，最终confirmation0/0，bearing-only提议157/36含同源重发。不能说三维定位未实现；实际5帧配置要求至少4条同簇有效定位/30s，现总数即不足，首轮还有约61s定位间隔，确认不可能形成。frames不是LiDAR数量。
- 第一轮96.91s换向后footprint拒续段，114.65s no_vehicle_path释放靠近；第二轮114.15s FollowPath abort已在stack证实RPP bounded command arc碰撞检查拒绝，随后30s靠近期满释放。145.07s只完成临时观察步，重新观察并探索，155.18s再观察超时；179.82s旧frontier动作另一次同类RPP弧拒绝。不是已完成目标到达，也未证明真实物理碰撞或地图误标。
- 原文默认3相机、一个LiDAR点云入口，LiDAR掩码深度优先，不足用多视点粒子加权粗定位；当前多视图分支已经改为最小二乘bearing交会/几何准入，不是原样粒子算法。实际安装obj_mask_triangulation.py与v25冻结SHA均1241cbc9f5f21f2e3af1d9b659abe0c4f830b08f356bdf970d2cefa15273d5bc，已核部署。两轮没有有效multiview输出，其稀少原因尚待同步/视点/几何逐门追溯，不能无证据归因模型/ROS。
- 详docs/TARGET_APPROACH_DIAGNOSIS_20261024.md；证据target_approach_audit.json、stack_213551.log、triangulation_213551.log和实际goal_standoff_v25.py。本轮仅只读诊断与记录，没有改运行代码、没有新整车/commit。用户最新询问三维定位是否为原文三角定位，须明确原文/当前算法差别及定位≠确认≠车辆到达。

### 2026-10-24 逐门回放与共享定位规则（最新细查边界）

- 实际104/30 paired，正掩码106/30，配对源龄<1.11/1.90s；大多数输入已进入定位，不能归因全链路断传。实际视线/位姿回放两遍一致：太近64/6、不足2视点9/8、无基线/视差18/9、重投影6/0、半径3/3、交点相机后1/3，多视图均0；日志太近/不足视点计数一致。5→10/30s历史诊断仍0定位，固定点理想视线控制可6/5，不是目标真值或现场成功。当前每帧最大区中位像素点视线缺稳定对应，但未得到同点始终在掩码内而仅中位射线导致失败的反例，不能将全部根因定为算法过严/语义误检。
- **更正缺数据判断**：实际console包装器追加raw/filtered LiDAR、mark/clear、tf_static和costmap_raw，均在metadata存在。早先“原始点云未保存”由只看通用列表造成，不能再当阻塞原因。第二轮点云准确回放29/30因去重掩码回波<8被拒（其中20为0像素），唯一有效三维与现场坐标误差0。第一轮回放6提议vs现场3，虽2正例完全匹配，但存在1缺cloud/1缺TF及额外提议，尚未闭合，不能把所有门统计当精确现场复现；控制/视觉两解释器重复均如此。源身份审计未发现不同数据重用同一戳。
- 公开地图3个拒绝时当前footprint空闲，第二轮路径前缀.85/.25m处出现车身lethal冲突；公开图/前路径并非私有锁定图/被拒命令弧精确复现。原始点云既存在，后续应继续追mark/clear与raw map，而非撤销碰撞检查。
- 独立回波像素门实际位于共享foreground_depth→object_localization→GoalStandoff confirmation→GraphPathBridge到达链，非记忆存储/候选评分专属，首次搜索也受影响。原文有点数门，逐像素去重/稀疏8像素为后续定位修改；引入动机不能凭当前记录认定专用于记忆。应区分弱证据接近、可靠定位/到达与可信记忆，当前bearing-only仍可接近，不能说不足8即禁止移动。
- 证据服务器v25_fast_shift下bearing_geometry_audit.json（两遍SHA4ba4f82d...）、bearing_controls.json（cdc54d299...）、lidar_depth_audit*.json、sensor_identity_audit.json、approach_map_geometry.json（3d40ae039...），本机同名资料位于continuous_exploration_20261024。详情docs/TARGET_APPROACH_DIAGNOSIS_20261024.md新增细查节。无运行源码修改/commit。
- 误source完整环境入口启动了旧v9 trial215808，已精确PID SIGINT正常关闭，search38.488s，console0、closed=true、无残留/X148锁；明确排除所有统计。原703运行文件/备份仍一致，不算新验收。此事故及前述缺数据更正保留，后续环境只source删去exec行后的前缀。

### 2026-10-25 用户收窄范围：只查到达判断（最新工作入口）

- 用户明确纠正“不是让你查到达判断吗”；停止观察点/转弯规划改造。本轮未改任何运行源码、未启动整车、未部署/commit。此前扩查几何的脚本保留为诊断，不作为到达修复。主目录703运行文件及baseline ZIP SHA再次与保留版本一致。
- 先闭合上轮定位回放差异：TF录包缺帧导致跨缺口插值额外产生4条提议。1093/1182个同源戳odom/TF逐元素差0，使用完整odom补动态变换后，生产定位方法两遍准确复现现场3/1条，全部正例坐标误差0；第一轮仍1缺cloud。SHA54d3cd7fe3bbb5132c4d8ebe67d2843a0e6366f0531d696b09328aaa0feffdde。不能再把回放额外定位当实际节点漏发。
- 实际v25共享定位需8去重回波像素，GoalStandoff需30s内4条1.5m同簇有效3D；`retry_confirmation`把search_confirmed与standoff可生成耦合，bridge最终又要求距standoff≤.6m且距物体默认≤4.5m（clamped最多6m）。这不等于用户五米规则。本机5m候选仍未并入v23→v25，保持此前控制变量隔离事实，不能宣称已部署。
- 实际安装类隔离回调6例两遍一致：4m/4条可靠定位但无接近点仍search_confirmed=false；4m而距指定停车点3m仍拒；4.8m且已到停车点仍拒；4m+到停车点可完成。SHA36c04eacf7025a25565a73667758eda18ef4ee163e7617f70a6ca83af2133007。构造消息/模拟发布器，仅复现到达规则，非完整ROS或整车通过。
- 最新两轮有效定位帧的导航参考点距物体9.921/12.053/11.272和33.402m，确认均0，因此先卡在确认形成，不能说已到五米被误拒；也不能靠最终距离改动宣称旧两轮成功。这些不是全轨迹对真目标的最小距离，语义正确性未由此证明。
- 细节docs/ARRIVAL_JUDGMENT_AUDIT_20261025.md，本机test_artifacts/target_arrival_audit_20261025；服务器execution_continuity_review_20261022/target_contract_20261025。下一步限定为核对已有五米候选的上下游一致性，解耦到达与指定接近点，再单独验证；不混规划、换向、观察弧改动。
- 用户追问原文到达规则：本机原参照+官方在线代码核实，原三维定位非空即发布imgnav_waypoint，GraphNav官方launch goal_radius=3.0，planner_node将目标转odom后按三维欧式距离<3m清goal_pose；不是通用GoalNavigator的2m默认值，也没有当前独立object_found/多帧3D同簇/standoff .6m验收链。这里只能称原规划器清目标处理，不能概括论文全部成功判据或下层停车。当前到达链在后续移植/加固中确实变化，超出记忆本身；关闭记忆仍保留新增门。原source SHA见同证据目录upstream_arrival_sources.json，详细差异已追加到专项文档；未改运行源码。

### 2026-10-25 车身外缘2米到达候选v26（当前活动入口）

- 用户将阈值改为2米，并明确选“车身外缘距已确认目标2米以内”；覆盖五米导航参考点规则。保持只修改到达链，不改规划、换向、定位算法及4条3D共识门。
- 从实际v25导出429个SHA核对成员，独立v26_body_arrival为430冻结成员/12成员增量（运行4文件、其余检查和测试）。本机test_artifacts/arrival_decoupling_20261025/candidate；主目录703运行文件/原ZIP SHA再次未变，保留用户dirty源码和v25，不commit。
- 到达共享函数按当前odom位置/yaw计算至SBR车身矩形外缘的平面距离（既有3.4/1.05/1.05m，原预检与两张costmap一致）；理想点仍点距。2m边界允许1e-9浮点误差，clamped不能扩大。producer共识与接近点可生成分开；已在范围内跳过发新standoff。bridge不再要求精确standoff，仍需新鲜同实例可信确认，完成沿原动作取消/停车终态。验收器重新计算车身距离、配对同观测掩码。
- 改前16反例两遍8fail/0error；最终18项到达专测、822项离线回调、两包构建与694项真实ROS组件通过。扩大真实ROS时旧离线夹具Time/GraphNode/隐式mock/float数组类型问题已修，断言保留，失败日志留存，生产SHA未变。
- 隔离域206真实GoalStandoff→ContinuousGraphPathBridge→实际Harness的DDS两遍通过：3观测不确认、间距2.2m不完成、边界2m完成、query切换拒旧证据。输入为合成掩码/定位及静止odom，不是模型/整车成功；缺GraphNav reset服务为刻意隔离。
- 组件通过后单轮180s绿色房屋现场兼容性测试已启动，关闭证据待记，无录像/commit。详docs/BODY_ARRIVAL_REPAIR_20261025.md。原三维观测稀少/共识不足仍未解决，不能凭这次到达规则修改说完整搜索成功。

### 2026-10-25 v26现场关闭：到达规则组件通过，上游确认仍缺席（最新）

- 145355启动59.001/search180.147/shutdown22.626s，timeout、field_acceptance=false、session errors=[]、console0；全部项目进程/X148锁已清空，无录像/commit。预检实际bridge两个到达参数均2.0，物理/碰撞和3.4/1.05/1.05m车身匹配，Smac12m/RPP8m不变。
- 已关闭bag两遍数字一致：mask165、有效定位1（multiview）、confirmation0。唯一定位观测时车身至该三维点平面投影6.705m，不能组成4条共识，未触发新到达判断。因此无法用本轮证明2m最终到达现场通过或standoff继续误拒。有效观测距离不是全轨迹对真实建筑的最小距离，目标点也不是完整建筑表面模型。
- 本轮近停26.50%/最长6.44s，净72.83m、采样117.73m；stale26.03s（planning21.43）、换挡3.05s，仍有续段collision_cell/靠近超时。到达分支未触发，路线/负载有方差，不能将本轮数字当到达修改的流畅性收益或退化证据。继续按用户范围保留这些规划问题，不混入修改。
- 最终822离线、694真实ROS组件、两遍合成输入DDS链均通过；12成员增量中运行4文件的AST方法范围已核对，未改其他规划/控制函数。候选v26保留，完整验收未过不合并为已验收基线、不commit。细节及已取回证据见docs/BODY_ARRIVAL_REPAIR_20261025.md与test_artifacts/arrival_decoupling_20261025/{field_result,arrival_field_audit,field_stop,body_arrival_receipt,real_ros_report,dds_results,production_scope_verified}.json。

- 用户追问走停/迟迟不确认后的追加只读回放：v26 163组paired点云两遍均0有效LiDAR深度；160组<8独立像素，其中143组为0，另3组9/13/15回波的垂直像素跨度1/1/0被表面分布门拒绝。1683个同帧odom/TF精确相等后补TF；深度回放SHAb208398d...一致。零命中具体是量程/遮挡/掩码区域还是投影对应仍未闭合，不能直接定为雷达或模型故障。
- 多视图现场日志164 callback/110 too-close/9不足视点/1有效定位；回放仅1通过且too-close111、paired163有1帧边界差异，统计不能全部冒充逐callback精确复现。源龄.274--1.989s、163/165 paired；不支持主因是普遍时间戳断传。回放SHAcea275aa...两遍一致，本机v26_{lidar_depth_replay,bearing_replay}.json。
- 走停以无新命令26.03s（其中planning21.43）为主，换挡3.05s；本轮4次续段collision_cell与2次靠近超时。确认堵在仅1条空间定位无法满足4条共识，2m最终门未触发。追加分析未改运行源码/阈值、无新现场/commit，细节见BODY_ARRIVAL_REPAIR_20261025.md末节。

### 2026-10-25 v27/v28：新观测门、跨目标过滤与匿名障碍回看（最新活动入口）

- 用户要求修改走停/确认后，从冻结v26独立创建v27，再创建v28，主目录703运行文件与旧ZIP SHA再核不变，无commit。定位修复把1m关键帧存储门与新帧求解分开：近距离新射线与未过期且独立的历史相机射线重新求交，不复制旧解、不增加假基线/延长旧射线寿命，原几何/语义身份/4帧/2m门保持。10例改前2fail/0error，改后通过；v26配对bearing进入旧/新实际process回放1→3条，不足同簇确认，不能当现场成功。v27实际导入定位40例、822离线、694真实ROS通过。
- v27 154400现场启动46.760/search180.010/shutdown32.035s，超时、session errors=[]、console0、closed、进程/X148锁已清。近停34.925%/最长20.723s，净54.086m/采样102.349m；stale37.682s其中planning31.169，gear4.756。mask63/3D2/最大同簇2/确认0；无录像/commit，未完成绿色房屋搜索。
- 最长停顿证据：131.47s `side_lookback_preempt` 取消实际接近，生成车后6m点(720.4143,-64.6177)，随后131.75–151.21s尝试14个候选，151.65s恢复跟随。源码maybe_side_lookback仅以附近障碍格推测语义结构、填入当前时刻作为观测时间，可抢占真实目标。两个实际回调反例证实无视觉仍生成后方目标、前方10m真实目标被改为后方6m。v28删除该自动函数及调用，保留真实视觉提示/GraphNav与地图碰撞检查，避免匿名几何越权生成目标。
- 另一反例：连续执行0→12m收到新frontier14m或10m，旧4m端点过滤均将请求改成8m。v28仅在当前已确认动作epoch/serial、同query下意图变更时保留新候选，其他同目标/旧epoch/未确认状态仍原过滤；路径替换实际位姿、碰撞、方向、ACK仍走既有门。反例2fail改后5新增回归及相关306例通过，最终离线828通过。
- v28运行增量仅3文件：obj_mask_triangulation的process/helper、GraphPathBridge删除回看与扩展候选策略、ContinuousGraphPathBridge覆盖策略；AST范围和实际加载SHA已核。服务器v28_intent_handoff，候选本机test_artifacts/execution_handoff_repair_20261025/candidate；真实ROS/现场结果待补。当前绿色房屋180秒现场验证已启动，不宣称完整成功；LiDAR零命中、稀少定位/同簇确认和其余地图/执行空窗仍未闭合。详docs/FRESH_BEARING_HANDOFF_REPAIR_20261025.md。

### 2026-10-25 v28现场关闭（覆盖上文待验证状态；最新）

- 最终828离线、700真实ROS回归和40实际导入定位/几何/同步检查通过。实际install的GraphPathBridge/ContinuousGraphPathBridge SHA与候选一致，定位从v28候选visual_navigation加载；物理、地图、定位几何阈值、4帧共识和2m车身到达保持。
- 155518启动39.792/search180.149/shutdown19.228s，timeout、完整验收false、session errors=[]、console0、closed=true；全部项目进程/X148锁清空。无录像/commit，保留v26/v27/v28，不合并主目录。当前不是“已有后台测试仍在运行”。
- 按既有Native区间统计（覆盖180.496s，非新做query精确裁剪）：近停15.668%/28.280s，最长4.355s，净84.917m/采样141.666m；stale8.190s其中planning4.685，gear4.657s。对照v27 34.925%/20.723s最长/stale37.682/净54.086m，本轮明显改善；无side_lookback或preempt事件。两轮路线/负载未完全控制，不将全部差值当净因果收益，不据一轮宣布稳定。49planning/33planned/28following，仍1次靠近超时、1次no_vehicle_path。
- mask30/有效3D7（5LiDAR+2multiview）/最大同簇2/confirmation0。空间点分布从(746,-112)、(738,-102)、(735,-96)到(748--750,-130)，尚未核实是不同实例/同建筑不同面/掩码回波关联；不把这些假设写成已确认根因。观测时最小车身距离3.674m非全轨迹真实目标距离，2m终态未触发，完整绿房屋搜索未通过。
- 当前可审查交付：docs/FRESH_BEARING_HANDOFF_REPAIR_20261025.md；本机test_artifacts/execution_handoff_repair_20261025/{candidate,field_result.json,arrival_field_audit.json,source_verified.json,receipt.json,real_ros_report.json,perception_report.json,production_scope_verified.json,v26_to_v28.diff,v28_delta_from_v26.zip}。主目录703运行文件/原ZIP SHA仍不变。后续优先核实定位点/实例一致性，不通过降低4帧或伪造新观测把失败改成成功。

### 2026-10-25 v29定位几何修复（最新活动入口，现场结果待补）

- 用户要求继续修改。v28掩码30/30均只有一个连通区域，不支持“最大区域切换”解释；配对点云两遍SHA5af801d6...复现25个<8点拒绝、5个LiDAR定位，1674同戳odom/TF精确匹配。未添加无证据的实例追踪分支。
- 实际process/check_lidar回放精确复现原7条（坐标误差约1e-11）：一条LiDAR坐标中位点投影在其自身掩码外；两条multiview旧半径1.867/2.724m遗漏角误差预算。局部修复只改2运行文件：foreground_depth选当前真实支持回波；定位类仅用近期同query LiDAR参考稳定当前回波代表点；triangulate_bearings传播既有角误差预算，不再仅看残差。未降8点、4帧、1.5m确认或2m到达门；空云/旧帧/歧义不复发参考点或刷新寿命。
- 初版协方差使用逆加权精度而位置仍非加权，新增求解器微角度扰动回归复现约8%元素偏差；最终r2采用同估计器 A^-1 sum(sigma² P) A^-T 传播。初版保留，不覆盖旧冻结证据。最终几何SHAd1c984a5...、定位类2c602132...；实际运行控制仍v28/install，未改物理、规划与控制。
- 录包反例改前4方法/5断言fail、0error；最终66项定位相关与828项原离线回归通过，服务器59实际导入定位/几何/同步和700真实ROS回归通过。同v28数据最终输出5LiDAR且均在掩码内，剔除2个不确定度超限multiview；末三点跨度2.729→0.111m，仅3条同簇仍不足4，不能冒充确认或到达成功。
- 初版170429现场启动52.783/search180.194/shutdown31.433s超时、session errors=[]、console0、closed，全部项目进程/X148锁清空。mask153/定位0/确认0；近停33.382%/最长9.750s、净60.252m/采样104.147m、stale31.185其中planning28.078、gear4.271。154定位回调无新接口拒绝；关停唯一traceback是KeyboardInterrupt，非运行崩溃。v28单轮15.668%不能当稳定性保证，未将此次差值直接归为定位修改净影响。
- 最终活动候选为test_artifacts/target_identity_repair_20261025/candidate_r2，服务器v29_range_consistency_r2；首次验证shell因CRLF在入口失败（未启动仿真）已修为LF，随后回归通过并开始180秒绿房屋现场轮，关闭结果待补。主目录703运行文件/旧ZIP仍不变，无录像/commit。详docs/RANGE_CONSISTENCY_REPAIR_20261025.md，保留v28/v29初版/r2及本机before、after、差异和回放证据。

### 2026-10-25 v29r2关闭与v30确认窗口（最新活动入口）

- v29r2 171255启动53.020/search180.175/shutdown21.824秒超时，session errors=[]、console0、closed、项目进程/X148锁已清；近停33.415%/最长11.313s、净40.437m/采样109.064m，stale33.597其中planning28.576、gear4.080。执行仍未稳定，不能沿用v28单轮15.668%宣称完成。
- 107mask/4有效LiDAR/确认0。四条平面位置约(738.33,-79.53)，首末37.921秒、间隔19.901/6.584/11.436秒；30秒窗口在第四条时淘汰第一条，只剩3。最小观测车身距离5.445m，非全轨迹真实目标最近距离，2m终态未触发。
- 实际ROS GoalStandoff以同4条原始消息、受控时钟/odom、无地图动作回放：30秒 retained3/无确认，60秒 retained4/inliers4/radius0.033695m、search_confirmed=true但approach_goal_ready=false，不是到达。证据本机test_artifacts/confirmation_window_repair_20261025/{confirmation_window_replay.json,localization_messages.json}。
- 独立v30_confirmation_history从v29r2保留几何/执行，只在official_graph_stack.launch.py显式设置confirmation_window_sec=60.0。仅延长历史支持，不改变通用类默认30、最新3D≤5秒/掩码新鲜度、4支持、1.5m簇或2m到达。不添加目标名称/坐标适配。
- 新8项配置/真实消息回调回归、相关120项与原828离线通过。初次构建验证因漏候选visual根路径混载旧geometry，回归阻止进入现场；修复build验证PYTHONPATH后708真实ROS和59导入定位/几何/同步通过。今后复制build脚本必须显式候选visual_navigation根路径，避免新triangulator与旧geometry签名不一致；失败证据保留，非现场崩溃。
- 已只读查询现场参数 Double value is:60.0。v30绿色房屋180秒现场验证进行中，关闭结果待补；候选本机test_artifacts/confirmation_window_repair_20261025/candidate，无录像/commit、不覆盖dirty主目录。详docs/CONFIRMATION_HISTORY_PROFILE_20261025.md。

### 2026-10-25 v30关闭：几何/窗口组件通过，完整搜索和走停仍失败（最新）

- 173253启动49.990/search180.189/shutdown22.227s超时、field_acceptance=false、session errors=[]、console0、closed；全部项目进程/X148锁清空，没有后台仿真。无录像/commit，v28/v29/r2/v30均保留，不将v30升为已验收基线。
- 本轮Native180.234s口径：近停40.916%/73.744s、最长12.643s，净38.857m/采样90.915m；stale48.484s其中planning44.542s，gear4.820s。66planning/32planned/29following、3no_vehicle_path、6short_route_probe。26mask/定位0/确认0，60秒实际生效但无3D证据，不能宣称现场确认通过，更不能沿用v28单轮15.668%当稳定改善。
- 4个最长近停12.64/10.97/10.41/9.05s的闭包请求均在车前，通常1.4–1.9s一候选，串行位置/朝向搜索后才退到约4m短探针。证据test_artifacts/confirmation_window_repair_20261025/long_stop_candidate_audit.json；不能误归因换挡或反向首段。变化地图/失败缓存/frontier身份与重复候选关系待查，未改此处。
- `GoalStandoff.on_localization` 四帧支持前不发布物体standoff，可靠未确认3D用于接近指导仍有交接设计缺口；不是本轮零定位或全部停顿的已证根因，未临时叠加分支。下一步优先从重复候选规划/空窗及接近证据分级审查，避免用降低确认/碰撞门掩盖。
- 本轮交付对v28累计运行改动仅2定位文件+launch历史60s参数，源码范围/哈希与累计差异见test_artifacts/confirmation_window_repair_20261025/{source_scope.json,v28_to_v30.diff,v30_delta_from_v28.zip}。66定位相关/828原离线、8新配置/实际录包回调、708真实ROS和59实际导入检查通过；范围均为组件与回放，不是完整成功。原主目录703运行文件/旧ZIP SHA仍不变。最终证据field_result.json、arrival_field_audit.json、actual_confirmation_window.txt及docs/CONFIRMATION_HISTORY_PROFILE_20261025.md。

### 2026-10-25 无新鲜命令退化复核（只读代码检查）

- 用户指出旧修复后的无新鲜命令问题再次严重。核对冻结候选v28与v30的graph_path_bridge.py、continuous_graph_path_bridge.py，两文件SHA256分别相同；持续探索/保留健康动作的源码没有被撤销。v30闭包仍含continuous_exploration_v1，未证实回到旧入口。
- v28 stale近停8.190秒，其中planning4.685秒；v30为48.484秒，其中planning44.542秒。这是实际命令连续性表现退化，不能以源码相同或组件测试通过否认，也不能断言同一旧缺陷已被重新写回。
- 现有长停审计只证明多候选规划期间没有持续产生新命令，尚未逐段闭合旧动作结束/取消/拒绝、后台规划何时开始、路径余量和失败候选重试的最早违约点。定位改动对路线/意图的间接影响未隔离。当前未修改运行代码、未新现场测试或commit；v28完整保留。

### 2026-10-25 冻结v28复测与接近/到达假设（最新，只读排查）

- 原样v28再测trial_20261025_200844：搜寻180秒仍超时，近停32.033%/最长8.952秒/净90.404米/采样105.907米；stale32.097秒，其中planning22.730秒，gear4.234秒。源码清单与配置前后不变，持续探索入口实读，所有项目进程及X148锁清空。没有录像/生产代码修改/commit。
- 更正前面对后续修改的笼统归因：旧v28也可复现长停，不能把历史单轮15.668%作为稳定保证或把全部差值直接归于v29/v30；也不证明定位修改没有间接影响。v30按elapsed前90秒约15.04%、后90秒65.48%，恢复阶段明显恶化。
- 用户怀疑接近后到达判定反复失败：本次v28约154.7秒后26条search_confirmed=true，全部no_reachable_standoff、standoff=null；同帧odom到估计目标的车身距离32.24→29.32米，未证实已达2米而拒绝。on_object_confirmation该等待分支只发状态并返回，不自行取消或触发规划。7.32秒等待标签近停不等于全部由最终判断造成。
- 90秒8.95秒停顿在确认之前，来自maneuver_done→footprint_collision→候选0–6；v30零确认也长停。因此需重点追接近点可达性/失败恢复，而非直接降低最终到达门。no_reachable_standoff的具体过滤门尚未闭合。
- v30缓存后台路径延迟1.728秒后wrong_start可复现，但裁剪后最新公开全局图仍collision_cell，不可直接删起点校验作为修复。证据与边界详docs/COMMAND_CONTINUITY_RECHECK_20261025.md、本机test_artifacts/command_continuity_regression_20261025。当前无后台仿真；仍未修改运行代码或宣称完整成功。

### 2026-10-25 接近点26次失败逐门复现、原仓库到达复核（最新，只读）

- 真实GoalStandoff源码SHA与v28冻结一致；同26个地图/车姿/确认快照两遍均26次no_reachable_standoff，报告SHA e6a6cc30...。目标在全局图内、地图新鲜、配对戳差0；650条环候选/截断射线均被local_map已占用格截断。最终截断点23车身占位拒绝、3不连通。此是候选启发式拒绝，尚未由Smac证明所有绕行路线不可达。
- 8/26快照用既有短观察步可过地图过滤；3个原截断点当前车身朝向可容纳但面向目标不行。两者均未证明12米半径车辆能执行，不据此放宽碰撞。2个新鲜实际掩码/地图的受控回调反例（连续性count设3）显示有pending_confirmation会抑制本来可发的观察步；未冒充完整历史状态精确回放。
- 源码交接缺口：确认等待接近点时producer暂停观察步，consumer无standoff且未到达先return，未形成已确认目标的新导航意图。现有探索可能继续，不称全局死锁。未修改生产源码/配置/部署/commit。
- 原始参照复核：GraphNav planner_node及官方launch默认goal_radius=3m，以同帧目标与odom三维距离清goal；原三维定位直接发导航点。GoalNavigator另有2m二维路点规则，但WildOS_Nav并不继承它，不能混为原WildOS活动链。当前多帧/实例/新鲜度、standoff、车身2m均是后续扩展，非加入视觉记忆必然要求；v26已有满足2m时跳过standoff完成，当前缺口在尚未到达时的导航交接。详docs/COMMAND_CONTINUITY_RECHECK_20261025.md末两节及standoff_filters.json。

### 2026-10-25 v31确认后继续接近隔离修复（现场进行中）

- 用户授权下一步后，从冻结v28创建v31_confirmed_progress，保留v29/v30不混合；主目录703运行文件与原ZIP再次核验不变，无commit。
- 两个运行文件：GoalStandoff最终/截断点不可用时复用已有最多8米observation_step，经当前地图、连通性、车身与目标间距检查后以clamped standoff交接；无有效接近点的pending_confirmation不再一律暂停新鲜同方向mask观察，active_approach仍独占。visible_approach仅修网格减法漏掉理论2米边界的1e-9米浮点问题。四条支持/2米最终到达/物理/Smac12-RPP8/换挡及持续探索保持v28。
- 改前3个干净反例fail、浮点单例改前fail；最终9新增、118相关回调、828原离线以及709真实ROS通过。同26个旧快照8个可生成中间观察点、18个仍拒绝；两个受控弱提示抑制反例通过。中间点未当最终成功；未证明实际车辆能执行，快照/连续性受控边界保持。
- 预检无残留、冻结SHA验证后启动trial_20261025_205143，目标green residential house，搜索180秒、启动/收尾另计，无录像。当前现场结果待关闭；不宣称走停改善或完整完成。详docs/CONFIRMED_PROGRESS_REPAIR_20261025.md，证据test_artifacts/confirmed_progress_repair_20261025。

### 2026-10-25 v31关闭：低净位移、持续探索绕过重试准入（最新）

- 205143已关闭：startup47.601/search180.168/shutdown21.827秒，timeout、session errors=[]、console0、项目进程/X148锁无。近停16.022%/最长3.695秒/stale8.264其中planning6.159；净28.163/采样140.288米。mask6、定位0、确认0，新确认后接近分支未现场触发，不能宣称修复已整车验证或将近停改善全归因新代码。
- 低净位移：前60秒净48米，68.73秒最远57.16米，随后折返；最后64秒走34.14米净4.80米。两个frontier owner占112.78秒和79.99米；6次FollowPath abort均由RPP bounded command arc碰撞检查拒绝，有原始stack证据。不是车身已经2米却终态误拒。
- 明确源码违约：ContinuousGraphPathBridge.on_route在base已缓存latest_route但因退避/预算返回后，无条件start_segment，绕过基础重试准入。4项回调两遍均基础层2pass、持续层2fail/0error。现场存在4秒声明却0.794秒同owner恢复、路线偏差2.336米不满足3米提前条件；另2秒→.607秒/2.971米，2秒→1.835秒/1.932米。第一例1.108秒但偏差5.975米属于允许新路线，已排除。
- 用户最新目标：修复并测试至完整流畅；每个原因至少复核两遍，并保留不回退证据。v31与v28均保留，准备隔离修复持续层准入；当前未改v31冻结生产源码或commit。详docs/CONFIRMED_PROGRESS_REPAIR_20261025.md关闭节及test_artifacts/confirmed_progress_repair_20261025。

### 2026-10-25 v32持续探索重试准入（现场进行中）

- 用户目标已更新为修复并测试至完整流畅，每个原因至少复核两遍且保留不回退证据。v31低位移链已在现场三例和两遍受控回调确认；另证实闲置新意图规划被子类取消再发，8项改前两遍4fail/0error。
- v32_retry_admission保留v31，只改ContinuousGraphPathBridge.on_route（SHA1ab491af...）：闲置启动服从基础层；确需替换旧pending计算时重入共享准入；基础已规划当前key则不重复取消。健康动作后台接续、旧消息/ACK/物理/碰撞/四条支持与2米到达不动。
- 改后8新增两遍通过、139相关、828原离线、717真实ROS通过。预检无残留和冻结SHA成功后已启动180秒Town green residential house现场轮，启动/收尾另计，无录像。关闭结果待补，不commit、不宣称完整成功。详docs/RETRY_ADMISSION_REPAIR_20261025.md与test_artifacts/retry_admission_repair_20261025。

### 2026-10-25 v32关闭：交接触发，但旧多视图不确定度造成不可靠确认（最新）

- 211308已关闭：startup39.983/search180.159/shutdown23.634秒，超时、session errors=[]、console0、无残留进程/X148锁。近停34.732%/最长6.907秒/stale33.644其中planning27.598；净54.738/采样96.609米。完整流程未通过，不推广/commit。
- 本轮177mask、8定位均multiview、3确认approach_ready=true，确认接近已触发；0 RPP弧碰撞abort。接近点133.85秒no_vehicle_path，替代观察164.43秒route_timeout后实例释放。与v31零确认不同，不能把近停差值直接当准入修复净因果或宣布无回退。
- 原v28定位为隔离对照沿用至v31/v32；175配对掩码实际process/bearing/prune回放两遍均精确复现8点/误差0，替换既有v29r2几何后0点。旧半径1.42–2.96米，角预算传播5.15–15.50米超过原3米门，故旧确认不可靠（非语义真值已认证错误）。第二遍报告SHA1447c8a5...。
- 下一步合并已保留的v29r2定位与v30历史配置到v32，而非继续用旧不确定度试图造完整成功；同时保留中间观察和准入修复。所有旧版仍保存，详docs/RETRY_ADMISSION_REPAIR_20261025.md关闭节及当前证据目录。

### 2026-10-25 v33合并已有修复（现场进行中）

- 从v32独立创建v33_integrated_progress，合并已保留v29r2两文件定位及v30历史60秒；保留v31中间观察和v32准入。相对v32只有两定位文件与launch参数变化，无新物理/地图/速度/控制改动，原用户主代码/旧版不覆盖。
- v32同175帧旧8点精确复现、修正角预算后0点，两遍一致；新增8交点拒绝及充分基线正例。合并47项本机集成、828原离线、56本机几何、727真实ROS及59实际定位/同步通过。最初本机message_filters缺依赖的ImportError单独保留，改为在真实ROS验证，未改断言或运行逻辑。
- 预检无残留、清单SHA与实际定位来源核对后已启动180秒Town green residential house，启动/收尾另计，无录像。完整流畅目标仍未通过，当前结果待关闭，不commit。
- 只读远环端点探针：旧v28首个真实深度快照24端点全局过滤通过，但都在当时局部terrain范围外，不能作真实可达性证明，未据此放宽安全检查。详docs/INTEGRATED_PROGRESS_20261025.md及test_artifacts/integrated_progress_20261025。

### 2026-10-25 v33关闭：低净位移是局部往复与接近恢复长停，实际规划对照两遍（最新）

- 213349关闭：启动35.742/search180.167/shutdown32.237秒、超时/完整验收false、session errors=[]、console0，项目进程/X148锁清空；adapter关停SIGTERM后10秒未退被SIGKILL，非已观察运行崩溃。无录像/commit，不推广。净51.606/全样本路程116.274米；近停28.957%/52.188秒，最长10.556秒；stale31.359其中planning27.660，gear4.905秒。83mask/2有效LiDAR/0确认，0 RPP bounded-arc abort。
- 两遍Native/events独立复算SHA均6e547a7a...并与原汇总一致。≤.5秒间隔路程114.933米（两个空洞1.105秒剔除），正行82.630/倒行31.546米；不要与116.274全相邻样本口径混用。前30秒净30.65米/近停9.73%；90–120秒净4.18米/近停61.46%。净位移非覆盖率，但本轮停顿/往复与未完成共同表明接近恢复效率差。
- 约110.4秒10.556秒长停闭环：visible-target maneuver_done→下一段footprint_collision→候选0–6串行约9.4秒→planned/following。另6.238秒为观察no_vehicle_path→释放GraphNav→长候选失败→短探针；5.989秒相似。四大长停telemetry/actuator全部正常，最大遥测龄<.09秒，无source拒绝或Native watchdog，大部分采样为软件刹车；不归为这些停段的BeamNG网络/物理故障。时间线query中位对齐有.156秒源/接收偏移跨度。
- 本轮4个真实请求+公开raw全局图，以实际Smac12米两遍：请求23/29原目标均2次换向，前方自由段位置对照均0；41/49原目标均No safe bounded continuous-start plan，对照分别2/0换向。前两同点单弧半径2.05/7.18米不足物理8/规划12；不是所有绕行不可达证明。后两同点弧yaw已经等于原yaw仍失败，不能只改终点yaw。改点对照未证明目标可见性/整车执行；公开图也非现场私有锁定快照。
- 源码缺口定位到GoalStandoff/observation_step的地图准入与bridge车辆可执行性之间：视觉锥2–8米选点、直线自由/连通/端点车身检查未证明当前车姿整弧可执行；后续多次换向/规划失败及停车串行恢复耗时。本轮0确认未进入最终到达门，不以降低最终阈值解释或修复。几何可靠证据稀疏仍独立未解决。
- 本轮仅只读诊断和新增审计/记录，703主运行源码与原ZIP再次未变。保留v28–v33；完整流畅目标活动。详docs/INTEGRATED_PROGRESS_20261025.md；证据test_artifacts/integrated_progress_20261025/{displacement_audit_1.json,displacement_audit_2.json,field_phase_audit.json,observation_pose_probe,observation_probe_summary.json}。下步从车辆可执行中间观察点与接纳/恢复交接做最小成组修复，不以组件通过代替现场验收。

### 2026-10-25 v34车辆观察点及延迟交接隔离候选（现场待验）

- 从v33创建v34_vehicle_observation；7运行文件+1测试文件增量，主运行源码/旧版不覆盖。车辆观察点从当前姿态的前向圆弧/直行上选，保留原自由空间和障碍余量，逐姿态检查全局图车身；实际仍须Smac/bridge/RPP批准。视觉bearing与可选navigation_heading分离。
- 物理launch传实际max(规划/控制)半径12米，runtime预检核对GoalStandoff新几何参数；理想载体保留旧路径。消费及延迟接管重新核对当前车姿，旧点不适合则交接新鲜前方点或释放恢复探索，保留健康当前动作且不保留旧未执行尾段。不改变四条可靠支持、2米最终到达、真实碰撞或命令看门狗。
- 改前正确物理夹具两遍4fail/0error；最终17新/相关121项两遍、原828离线和真实ROS通过（数量/最终现场待补）。夹具漏物理参数及fakeROS has_parameter缺失的初次失败保留，未以放宽断言处理。
- 同v33四个真实请求、原raw图和原producer方位，实际新回调+实际Smac各两遍：23/29两换向→0；41失败→可规划但2换向；49视觉龄12.968秒无新提议。第三例未无换向；回放支持计数3复用已成立证据，非全历史状态精确重建，不冒充整车目标完成。
- 当前准备180秒绿色房屋现场验收；无录像/commit、不推广。详docs/VEHICLE_OBSERVATION_REPAIR_20261025.md及test_artifacts/vehicle_observation_repair_20261025。完整流畅目标仍活动。

### 2026-10-26续接v34关闭结果，优先确认后接近（最新入口）

- v34 222040已关闭：startup38.769/search180.171/shutdown22.023秒，超时、完整验收false、session errors=[]、console0、无项目进程/X148锁；10-26继续工作再次只读确认无残留。744真实ROS、原828离线通过；无录像/commit、不推广，主目录703运行源码未覆盖。
- 净89.479/路程113.655米；近停25.904%/46.471秒，最长5.698秒；stale22.723其中planning13.328、gear4.589秒。v33为净51.606/近停28.957%/最长10.556/stale31.359其中planning27.660。单轮路线/负载不同，不把全部差值归因新改动或宣称稳定不回退。13次弱接近/7次观察步完成；geometry_refreshed=0，延迟刷新只组件验证。
- 100mask/28有效LiDAR/24确认，11approach_ready、13no_reachable_standoff。已确认对象约(748.7,-130.1,58.1)，对应同帧odom最小车身间距29.925米，非已达2米却判定失败。170.5秒FollowPath中止有RPP bounded-command-arc碰撞原始日志；确认后standoff/中间点可执行性及恢复仍未解决。
- 两次Native/events报告SHA同2f0fcc81...，证据已取回test_artifacts/vehicle_observation_repair_20261025/{field_result.json,field_phase_audit.json,closed_arrival_audit.json,displacement_audit_1.json,displacement_audit_2.json,trial_20261025_222040_core.tar.gz}；必要failure_last_rgb显示前方浅绿色建筑，非到达证明。
- 上次追加确认后回放被自动审批额度耗尽拦下未执行，不能记作已验证。10-26只读SSH审批恢复；下一步先用关闭录包两遍复核确认后选点，再决定最小成组修复。详docs/VEHICLE_OBSERVATION_REPAIR_20261025.md关闭节。

### 2026-10-26 v35确认后车辆中间接近，接口成组修复（现场待验）

- SSH审批恢复后两遍复核v34全部24确认快照，原13失败，车辆弧可救8；三组原生Smac两遍均可规划，换向0/1/0。独立快照未重建全部拒绝/last_goal历史、公开raw图非现场私有锁定图，边界保留。另定位producer/consumer/规划候选三处面向目标航向覆盖，3个干净反例改前两遍3fail/0error。
- v35_confirmed_vehicle从v34冻结，3运行文件+1测试模块+夹具：GoalStandoff复用已验证车辆观察提议处理远处的地图截断接近，保持目标间距且至少.6米实际进展；可选standoff_navigation_heading绑定中间点，经确认/refresh、ConfirmedTargetApproach及GraphPathBridge候选完整传递，活动动作继续独占。最终位姿、可靠支持、2米车身到达、物理/地图/C++/车速/ACK/看门狗不改，旧消息兼容。
- 13新增/159相关两遍、原828离线通过；两包构建及真实ROS通过。实际新producer同24快照两遍ready11→19，仍5失败，JSON SHA同f21cdb14...；三组实际原生规划仍0/1/0换向，不宣称全部无换向或已整车到达。
- 当前准备180秒Town green residential house现场，关闭结果待补，无录像/commit、不推广；主运行源码与v28–v34保留。详docs/CONFIRMED_VEHICLE_REPAIR_20261026.md，证据test_artifacts/confirmed_vehicle_repair_20261026。

### 2026-10-26 v35关闭，v36临时观察预算候选（最新）

- v35首次084031预检通过后SetParameters(enable=False)10秒超时，goals0；SIGINT前无运行traceback，同冻结版本重测启动通过，准确服务延迟仍未定位。startup_failed_084031.json及原服务器session保留，不能称修好启动问题；包装器因无goal无法统计又抛错，未丢原结果。
- v35复测084439已关闭：startup39.993/search180.192/shutdown15.018秒、超时、errors=[]、console0、项目进程/X148锁空；净54.808/路程110.157米、近停33.294%/最长10.135/stale37.081其中planning30.284/gear3.607。114mask、2定位（1LiDAR+1multiview）、0确认，确认后修复未现场触发，不推广/commit。两次轨迹审计SHA0dc48f93...；比v34差，不宣称无回退。
- 同4个旧输入弱提示producer输出v34/v35逐值相同，仅证明这些快照等价；本轮临时任务仍出现换向后footprint_collision与cusp_position_outside_tolerance，非RPP bounded-arc abort。v33/v34模拟/墙钟比约.993，速度积分与位移一致，不支持这两轮因BeamNG整体慢时钟导致低位移，仍不排除局部RPC/服务延迟。
- 新受控接口反例（非本轮实测，未修改）：到最终观察点.348米但body gap2.273米、朝向差45度，两个版本都只报object_visible_not_arrived且无新意图；两遍SHA286dd25f...。须后续核查位置已近与最终整车位姿的交接，不以此解释本轮零确认超时。见confirmed_vehicle_repair_20261026/final_pose_audit_1/2.json及v34对照。
- 独立v36_weak_observation只改2运行文件：对未定位的车辆bearing-only中间点，空路径或成功路径仍含倒行时通过原事务恢复探索；健康当前动作保留。确认/记忆/全局探索仍可必要倒车，传输/旧消息/物理/2米到达不改。真实录包严格匹配导出1条已接纳临时换向完整路径54姿态/2段/7.609米，两个回调反例改前各两遍2fail/0error。
- v36新增9项/相关168两遍通过，原离线与真实ROS/现场待补；主运行源码和v28–v35保留，无commit/录像。详docs/WEAK_OBSERVATION_BUDGET_20261026.md与test_artifacts/weak_observation_budget_20261026。完整流畅任务仍未实现。

### 2026-10-26 v36关闭；改用整条路径持续控制（当前入口）

- 更正v36待验：090939已关闭，启动38.162/search180.142/收尾24.033秒，超时、errors=[]、console0、无残留/X148锁。净58.250/路程100.739m，近停36.241%/64.874秒，最长15.053秒，stale42.635其中planning38.726、gear3.166秒。112mask/9LiDAR/5确认，同帧最小车身间距7.090m，未达2m；策略退化，不推广。
- v35/v36按实际path_source_stamp与odom两遍复核：v36一次[.75146,1.11402,.005713]m末尾触发join_target_behind_motion_direction，平移.01696/yaw.02548已近停；只解释一次接续失败。v35另两次.65m+6–7m后续footprint_collision无毫米段，v36另有正常长路径collision_cell，不可混同。服务器motion_section_audit_1/2.json保留。
- 用户指出原文连续执行未落实：v24仅分段外滚动更新，实际仍自写Bridge/Smac/单方向RPP，原follower前视输出未接管。此前称已恢复原文连续执行不准确，已纠正；不能将自写分段限制推给记忆或物理必然。
- 按新指令从v35创建v37_full_path，不沿用v36：Bridge整路一个FollowPath；控制器内部维护方向进度/真实停稳换向，普通前向路径可滚动替换；新增进度回传及完成门，最近点不跨cusp。保持12m规划/8m实际控制、车体/轮胎碰撞、命令时间及ACK、可靠确认和2m车身到达。主703运行文件/备份未覆盖。
- 26项实际C++插件两遍、相关70项两遍、原828离线两遍、772真实ROS组件通过。Humble节点API和插件库链接工厂问题均在整车前发现并修正，失败源码/日志保留，组件通过不是整车成功。
- 预检无残留、冻结新RPP SHA64b95d15...后启动绿色房屋180秒现场，启动另计、无录像、录进度话题；结果待关闭，不推广/commit。详docs/FULL_PATH_CONTROL_20261026.md；本机test_artifacts/full_path_control_20261026，服务器execution_continuity_review_20261022/v37_full_path。完整流畅任务未完成。

### 2026-10-26 v37两轮关闭：整路控制已现场生效，完整流畅目标未通过（最新）

- 165843/170640均驱动180.195/180.194秒超时，启动48.774/43.590、收尾15.819/15.016秒，console0/session errors=[]，全部项目进程/X148锁空。主目录703运行源码和原备份未覆盖，无录像/commit、不推广。
- 两轮均加载新RPP SHA64b95d15...；整路动作28/19，其中含换向7/8，内部方向切换各8次，普通队列均1，未出现maneuver_done/replanning_after_maneuver。1500/1379条真实源命令全满足8m半径，无原地旋转；1500/1378条进度消息最大接收龄13.95/7.61ms。已证明换向不再依赖外层分段重发，不等于持续探索全流程通过。
- 历史口径近停22.110%/30.977%，最长7.199/13.771秒，路程140.129/115.175、净49.591/62.129m；stale20.600/31.037其中planning15.790/26.475秒，gear3.282/4.776秒。第二轮不稳定，不可把首轮改善当稳定/无回退。19/59mask、1/2有效LiDAR、0/0确认，均未到达。
- 两轮轨迹/事件各两遍一致，SHA5a4ab994.../6d1986db...。第一轮7.2秒为观察整路结束→无可用目标路径→GraphNav→串行候选/短探针；第二轮7.0秒同类，13.77秒为最新图handoff_prefix_blocked→停车/ACK→候选0–7串行。对应Native遥测正常、无watchdog/source拒绝，不归为这些长停的BeamNG网络故障。另有一次motion_lease_exhausted，碰撞弧中止2/1次仍需源图追溯，未关闭检查。
- 统计边界更正：旧汇总含下发前少量样本；按query近似裁剪近停21.03%/30.58%，覆盖177.04/178.67秒。驱动从控制台输入起计180秒，ROS query之前仍有约1–2.4秒握手，后续须从实际query_sent计时再严格验收；本轮不冒充精确180秒完整成功。
- 证据已取回test_artifacts/full_path_control_20261026/{field,field_repeat}，详docs/FULL_PATH_CONTROL_20261026.md。当前保留v37整路候选和全部旧版，后续重点为观察/探索意图接替与规划失败恢复等待，不能再把尚存长停全部归因外层cusp分段。

### 2026-10-26 流畅性与搜寻完成原因分层复核（只读分析）

- v37解决的是同一车辆路径内的换向动作拆分。观察意图仍有任务边界：visible_approach.finish_visible_approach明确reset、清expected、尝试新提示，否则请求GraphNav reset进入重新观察/探索；与两轮“观察结束→串行规划长停”的事件吻合。不能称上层持续搜寻已整体改好。
- 两轮19/59掩码仅1/2可靠LiDAR定位、无有效多视图定位、0确认；最新失败在可靠定位/确认形成之前，尚未进入车身2米最终验收。第二轮triangulation.log有连续回调且反复No triangulated position，不能诊断为定位节点完全没收到消息；每个掩码为什么深度/多视图失败仍须逐帧回放，不能仅凭输出少归因模型错误、时间戳或门过严。
- 改进方向为：观察/探索共用连续执行会话、基于剩余可执行余量提前准备替代路径；失败恢复优先验证短可行曲线，减少停稳后远候选0–7串行计算；同一疑似目标关联下根据深度/视差/遮挡拒绝原因选下一观察视点。此为方案分析，未更改运行代码/部署或启动新测试，不宣称已解决。

### 2026-10-26 按用户“不回退”要求，联合候选v38（现场进行中）

- v37真实同步掩码/点云/odom验证TF的生产回放两遍一致，报告SHA866477c6...。19/59掩码、18/59配对，16/54帧去重回波<8（其中5/34帧0回波）；多视图有视差/重投影/不确定度等拒绝。现场1/2个有效LiDAR正例全部复现，位置误差0或1.42e-14m。第二轮两点约16m分离，不能合为同簇，也尚不能据此认定语义误检。证据full_path_control_20261026/perception_replay_1/2.json。
- 从v37创建v38_rolling_observation独立候选：整路模式的弱提示增加距离门4→1m，仍限实际2m进展/单在途规划；旧观察动作已终止且有新鲜前向点时保持会话/代际续接；停驶普通探索第一次empty_path就尝试原有短探针，去掉重复候选；定位/确认/到达、实际RPP库、车体/碰撞/源新鲜度不改。测试事件增加实际query_sent的steady戳，驱动从该事件计180秒。
- 改前三个接口反例两遍3fail/0error；10新增/相关206项两遍、17harness、原828离线两遍、782真实ROS与原26实际C++插件两遍通过。RPP SHA64b95d15...、定位SHAd1c984a5...未变；主703源码/备份未覆盖。详细影响/验证边界docs/ROLLING_OBSERVATION_20261026.md。
- 当前已预检无残留、冻结source/config、从实际query_sent后180秒绿色房屋现场进行中。用户“不允许效果回退”作为验收门，不以承诺代替证据；未达成不回退与完整目标之前不覆盖v37、不commit、不宣称修好。证据test_artifacts/rolling_observation_20261026，服务器execution_continuity_review_20261022/v38_rolling_observation。

### 2026-10-26 v38/v39关闭未通过；v40恢复观察点航向归属（现场待验）

- v38 181053关闭：近停26.088%/最长17.002秒、净63.411/路程136.083m，128mask/1LiDAR/0确认、超时。最长停为上一动作结束后候选0–16串行加短探针交错，Native120样本均软件刹车/遥测正常，不归因物理受困。另180444 query后monotonic_ns出口缺失失败保留；修正实际console事件出口后才有181053。
- v39从v37隔离，保留弱观察续接和query计时、排除提前短探针；修复expected弱目标时exploration_metadata拒绝退休动作后台接续。改前2回调两遍失败；相关207两遍/原828离线两遍/783真实ROS/26实际C++两遍/29console通过。182715关闭近停36.379%/最长11.301秒，净92.289/路程107.633m；116mask/33LiDAR/11确认，确认帧最小车身间距20.386m，超时。11.3秒停在中间点后等待确认、拒绝instance_mismatch、释放再接近；遥测正常。两候选均未通过不回退，不替换v37、不commit。
- 对77真实同步配对重建旧采样点云误差<1.91e-6m，完整扫描同生产质量门两遍回放只把有效定位由1/2改成2/2，报告SHAe329d5b8...；未据此增加点云/队列负载或放宽8回波、可靠确认、2m到达门。原始回波少仍非唯一原因。
- v40仅target_observation运行文件：原恢复deferred中间点时丢其navigation_heading且旧confirmed_route_heading泄漏给新点；改前4项两遍3fail/0error。现在坐标/航向成对接纳，普通径向替代点清旧航向，无接纳不修改；99相关两遍、787真实ROS、26C++两遍、29console通过。现场实际query后180秒待关闭；不宣称完整或无回退。703主源码及基线ZIP保留，详docs/OBSERVATION_SESSION_20261026.md，证据test_artifacts/observation_heading_20261026。

### 2026-10-26 v40关闭：近停20.6%，确认后修复仍待现场触发（最新）

- 184213关闭：启动44.516/搜索180.071/收尾22.824秒，超时、session errors=[]、console0，无项目进程/X148锁。全Native历史口径近停20.581%/37.658秒、最长6.293秒；净50.261/路程146.216m，stale14.386其中planning7.930、gear4.049秒。本轮比v37两轮停顿指标较好，但不能证明稳定或归因本次修复。
- 53mask/1有效LiDAR/0确认，3次RPP实际弧碰撞中止；22完整路径动作/8含换向，命令无曲率越界或原地旋转。确认后恢复分支根本没有触发，不能称v40航向修复已整车验收。v39与v40其余代码一致，单轮33→1定位差值不构成该修复的因果证据，也不能忽略共同链路路线/时序波动。
- 828原离线回归补跑通过，99相关两遍/787真实ROS/26C++两遍仍通过；Native/events独立两遍一致，703主源码与原ZIP未覆盖。未推广/commit、完整搜寻仍未通过。机器可读不回退验收拒绝原因test_artifacts/observation_heading_20261026/validation_summary.json，详docs/OBSERVATION_SESSION_20261026.md末节。下一步仍须定位稀疏时的可执行观察/持续接近与失败交接，不能以单轮近停改善结案。

### 2026-10-26 v41弱观察一般规划准入（现场待验）

- v40四快照、精确odom与此前公开raw图实际12米Smac两遍：两个无路点三种yaw均失败；99.907s旧点拟合单圆弧R11.977<12直接被拒，但实际Smac可规划1/2次换向。fallback快照三yaw均可规划且均1次换向，不能以该快照证明新航向更快。不是所有no_vehicle_path都有同一原因；证据observation_heading_20261026/weak_admission_summary.json。
- 从v40创建v41_general_observation，2运行文件+1测试：仅已接纳弱点的单圆弧失败且仍前方2–10米/车身地图通过时允许实际Smac判断；新提示激活、后方/阻挡点拒绝、任务上限不变。局部fallback保留有效圆弧终点航向。实际路径/碰撞/ACK、12m规划/8m控制、可靠确认与车身2m到达门不改。更多一般路径可能增加换向，必须现场验收，不承诺用组件覆盖整车风险。
- 干净6项反例两遍2fail/0error，改后101相关两遍/原828离线通过。首次793真实ROS中旧Native测试出现.09317m>.06m滞后；原失败保留，未改断言或odom生产代码。v40/v41交替各三组5项均通过，再两遍完整793通过，原26C++两遍/29console通过；偶发延迟根因尚未复现，不称已解决。最初探针系统无python及夹具缺map receipt错误单独保留。
- 预检无残留和冻结SHA通过后启动180秒green residential house现场，关闭待补。703主源码/ZIP及v37/v40保持不动，无录像/commit、不推广。详docs/GENERAL_OBSERVATION_REPAIR_20261026.md，证据test_artifacts/general_observation_20261026。

### 2026-10-26 v41关闭未通过，v42整路退役权限成组修复（现场进行中）

- v41 190520关闭：startup39.348/search180.045/shutdown21.826秒，超时、errors=[]、console0、进程/X148锁空。近停29.087%/最长15.029秒，净35.185/路程125.156m，39mask/0定位/0确认。比v40差，未推广；末段4次empty后预算正常退休，新owner成功路径本地车身拒绝+退避，不称无限同owner重试。
- 共同整路接口根因已在六轮日志及两个独立受控插件反例复核：队列第一项已是完整多方向Path，但意图退役仍只queue[:1]，未撤销控制器内后续方向。v37复测剩.526m仍两换向，v41剩1.251m仍两换向直至租约耗尽。不是把全部停顿归因此一处；详docs/RETIREMENT_BOUNDARY_20261026.md。
- v42从v40隔离，不混入v41一般规划实验；只改Bridge/RPP运行文件，Path身份+当前方向终点限制真正控制器权限，到边界发新鲜刹车/真实停稳后终止旧动作，沿原停稳/ACK交接；新路径和非退役整路必要换向保持。旧消息不能限制新path，重复不能扩大权限，保持物理/碰撞/可靠确认/2米车身到达。
- 改前Python3入口两遍均3fail；新C++6项在旧真实插件两遍均3fail，其余3对照通过。改后50相关两遍、原828离线、32实际C++两遍、795真实ROS两遍通过；新RPP SHA217609ca...。主703运行文件/基线ZIP/全部旧版未覆盖，无commit。
- 预检空进程/X148锁及冻结SHA通过后，已启动实际query后180秒绿色房屋搜索，无录像，关闭待补。完整流畅仍未通过，候选不推广；证据test_artifacts/retirement_boundary_20261026。

### 2026-10-26 v42两轮关闭，整路退休边界生效但探索仍循环（最新）

- 215243/215959均180.022/180.171秒超时、session errors=[]、console0、进程/X148锁空。实际新RPP来源/SHA已核对；12/1条退役消息以后旧方向后缀越界0/0，第一轮两次边界停稳受控终止，正常整路必要换向保留。相关具体修复已现场验证，整体仍未通过、不推广/commit。
- 近停23.910%/16.737%、最长4.360/10.248秒，净27.905/5.342、路程132.971/158.675米；不能只报较好占比忽略第二轮低净位移/长停。0/4mask、0/0可靠定位确认；视觉query正确、322/220有效TF处理帧、评分持续，非节点停摆。两次各自Native/events复算一致，详docs/RETIREMENT_BOUNDARY_20261026.md关闭节。
- 第二轮同探索owner约175秒/155米局部运动，三次真实RPP弧碰撞后第三次已8秒恢复档，仍继续旧owner；源码重复失败换frontier仅覆盖execution_stalled/tracking_deviation/局部预检碰撞，未覆盖实际STATUS_ABORTED。下一步须用受控回调重复确认，再限定在未退休普通探索owner修复；不扩大至记忆/目标/旧异步动作，不把临时路径成功当作探索已完成。诊断repeated_abort_budget_audit.json保留；当前尚未修改此处。

### 2026-10-26 v43普通探索Action中止恢复（现场进行中）

- 该恢复缺口以一个关闭现场原始RPP栈+7条回调两遍复核，改前均2fail/0error；不能写成两个独立现场反例。原目标进展信用按固定owner真实新1米重置，正确逻辑保留。
- 从v42隔离v43，唯一运行文件graph_path_bridge.py：当前未退休普通探索、follow_aborted/status=6、原恢复次数≥2时复用既有相关拒绝协议暂时退避frontier，status5取消/新owner/目标/记忆不改。C++/物理/地图/可靠定位/2m到达不变；仍用v42 RPP SHA217609ca...。
- 169相关两遍/802真实ROS两遍/32原实际C++两遍/console通过。fakeROS补齐标准abort枚举的首次错误保留；本机828最初因复制候选漏历史夹具失败，补齐原夹具后最终重跑待补。启动现场时本机全量尚未完成，是验证顺序偏差，不记为全部先通过；服务器真实ROS含原夹具通过。
- 预检无残留/锁、冻结来源后已启动180秒绿色房屋现场，无录像，待关闭，不推广/commit。详docs/ABORT_RECOVERY_20261026.md，证据test_artifacts/abort_recovery_20261026；主源码与旧版保留。

### 2026-10-26 v43关闭：确认点绑定蓝房屋，靠近链不能只归因规划（最新）

- 221716搜索180.144秒超时、startup41.646/shutdown24.030，session errors=[]、console0、进程/X148锁空。近停31.896%/最长11.447秒、净87.875/路程121.764m；145mask/51LiDAR/16程序确认，确认帧最小车身间距23.358m，未达2m。4次RPP实际弧碰撞、33instance_mismatch/两次观察路线失败；不推广/commit。v43新普通探索abort预算分支未现场触发，不能把更远路线归因它。
- 本机漏历史夹具的828回归最终补齐原夹具通过；169相关两遍/802真实ROS两遍/32C++两遍仍通过。13次退役限制旧后缀越界0，实际源命令无曲率越界或原地旋转。主703源码/旧版保留。
- 新投影证据两遍完全一致：首/末旧确认点投失败RGB蓝房屋区域(430.66,184.07)/(408.10,306.28)，最新定位点投浅绿房屋(591.45,269.87)，仅新点在最新mask内。已补偿RGB与mask .475秒间.198m/.02196rad位姿差，固定相机外参末5帧一致到数值精度。不是仅凭19米坐标差猜两个物体，也不是宣布初始每一帧mask真值已重建；原始4维mask诊断错误保存，最终审计已纠正。
- 几何质量/程序确认不等于语义正确：本轮旧实例绑定蓝房屋区域，后续绿房屋观测被旧实例门拒绝。仍待确认最早错在语义mask或mask→LiDAR表面关联，不能直接放宽实例门/合并表面，也不能只改到达阈值。foreground_depth已有充分支持多表面歧义拒绝，勿重复误诊。详docs/ABORT_RECOVERY_20261026.md关闭与投影节，证据test_artifacts/abort_recovery_20261026/field。

### 2026-10-27 恢复：v44关闭、v45接近接口候选（最新）

- 用户要求先修目标绑定/接近路线/失败交接。v43蓝房屋几何方法隔离两遍：50条可配对LiDAR坐标全复现、最大2.84e-14m；早期蓝房屋回波确在当时mask内。首帧缺点云使初版顺序回放偏差，报告保留；不能声称完整异步/语义真值复现，不放宽定位或实例门。
- v44从v43隔离：原instance_mismatch每次清空观测、旧锁保留，持续新表面无退出依据。两遍干净3fail/0error后，新增独立最多10条替代定位历史，沿原簇质量门，只触发旧实例停车/释放/ACK；新鲜观测重新确认后才可导航，旧区域沿原60秒退避。单帧/散乱/旧消息、记忆区域和执行脱困不被强行覆盖。3运行文件，无新直接控制通道。106相关两遍/828离线/812真实ROS两遍/32C++两遍通过。
- v44 224834关闭：启动41.776/search180.022/shutdown22.623秒超时、errors=[]、console0，进程/X148锁空。近停20.800%/最长3.209秒，净38.566/路程140.946m；13mask、0可靠定位/确认，新增交接未现场触发，不能报修好。11退役限制后旧方向越界0，无命令曲率越界或原地旋转。未推广/commit。
- v45独立基于v44，仅补goal_standoff失效地图过滤函数的navigation_heading参数：调用2参数而旧失效分支只接1会TypeError。真实夹具改时间戳两遍2fail/0error，修后58相关两遍、828离线通过；真实ROS验证进行中，非已证现场根因。主703源码/旧ZIP保留。
- 上次自动审批额度耗尽导致文档未写入，恢复后补记真实日志；详docs/INSTANCE_HANDOFF_20261026.md及instance_handoff_20261026、approach_contract_20261026，当前优先利用v43录包闭合交接，再验证整车接近和最终2m。

### 2026-10-27 聚焦目标接近：v45关闭，v46失败反馈关联修复

- 用户最新要求专注接近阶段。v45 100928关闭：启动41.739/search180.015/shutdown23.435秒超时、errors=[]、console0、进程/X148锁空；近停28.140%/最长11.207秒、净78.200/路程125.224m，156mask/13定位/5确认，确认帧最小车身间距26.306米，未到2米。4条不匹配分属不同簇，未触发替代释放，不放宽门。旧确认仍投蓝房屋、新定位投浅绿房屋，.121s位姿差已补偿，两遍投影一致；不把程序确认当语义正确。
- 接近核心违约：Bridge正在执行(736.976,-87.918)，Producer已发布(736.716,-90.931)新候选且消费者只缓存；失败反馈却与最新last_goal比较，3.02m差异使真实no_vehicle_path被丢弃。当前地图又拒绝旧缓存点，随后选24.702m远的径向观察点，最终30秒超时。源码/关闭日志及两遍真实回调对照定位，改前5项两遍4fail/0error。
- 独立v46基于v45，仅confirmed_approach.py携带当前instance_id、goal_standoff.py按新鲜同query同实例接纳实际失败点并当前地图重算；其他实例/旧消息不污染新路线。相关125两遍/828离线通过；初次全量真实ROS发现旧夹具手动active_approach漏实例锚点，补齐生产同时建立的状态并保留原失败日志，真实ROS重跑中。
- 同v45地图回调两遍：旧恢复24.702m，新生成车体检查通过的(736.015,-92.794)、自有yaw -1.77885、距离7.963m。真实Smac和整车待补；回放非完整动作历史、公开图非现场私有图，不算完整成功。不覆盖703主运行文件/旧基线、不commit/推广。详docs/APPROACH_FEEDBACK_20261027.md。

### 2026-10-27 v46接近恢复对照通过，现场进行中

- 819真实ROS两遍、32保留实际C++两遍/console、125相关两遍与828原离线最终通过；首次漏confirmed_instance_anchor夹具失败已保留，补齐生产与active_approach同时建立的状态，未放宽运行身份或测试断言。
- 关闭v45同一车姿/raw全局图，真实12m Smac两遍：旧24.702m径向观察点路径7次换向、maneuverCost99.4252；新7.963m中间接近点路径0换向、cost7.9815，两者车身碰撞采样0/曲率通过。代价99.4不是路径米数。实际反馈重算/航向交接前后两遍一致，证据approach_feedback_20261027/replay/{before,after}。
- 主703源码/旧ZIP未覆盖，控制库仍v42 SHA217609ca...。冻结/预检后启动v46 trial103053绿房屋实际query后180秒，无录像，关闭待补；不将组件规划当整车成功，不推广/commit。用户当前优先接近阶段，不继续增加语义/记忆规则。

### 2026-10-27 v46现场关闭：接近反馈组件改善，整车未验收（最新）

- 103053关闭：启动42.374/search180.188/shutdown23.035秒超时、session errors=[]、console0、进程/X148锁空。全Native181.904秒口径近停28.411%/最长5.095秒、净58.184/路程114.243m；stale18.816其中planning11.706，gear6.778。不能把最长停顿缩短归因新分支，也不能称稳定无回退。
- 129mask、0可靠3D/0确认，新确认后失败反馈分支未现场触发；仍有13弱目标靠近、5visible_target_no_vehicle_path、11fallback、6观察步结束/再观察。用户当前要求专注接近阶段，后续继续沿此链路定位，不扩展语义/记忆/到达规则。
- 新接近失败交接仍有实证：125相关两遍/828离线/819真实ROS两遍通过；同v45关闭地图对照两遍旧恢复24.702m/7换向/cost99.4252→新中间7.963m/0换向/cost7.9815，车体和曲率检查通过。组件/录包范围不等于完整车体成功。
- 实际运行库/模型SHA匹配，12退役限制后旧方向越界0，无曲率越界或原地旋转；Native/events独立两遍bc79c6ef...一致，主703及ZIP未覆盖。无录像/commit、不推广。全部关闭证据approach_feedback_20261027/field，详docs/APPROACH_FEEDBACK_20261027.md；目标完整流畅尚未完成。

### 2026-10-27 三维定位v47（现场进行中，最新）

- 用户要求修复3D。v46零定位真实方法两遍复现：129配对/128可读云（1录包缺），127不足8独立像素，另1被投影横/纵均需展开的门拒绝；多视图视差/病态/误差半径拒绝合理，不降3米误差门。原始扫描重建误差<1.91e-6m，先过滤后限点和全密度两遍仍0，5/15/30秒历史探针仍0，未据此改运行配置。
- 独立v47基于v46，唯一运行文件localization_geometry.py：直接LiDAR位置不再要求二维平面像素展开，仍≥8去重支持/深度质量/多表面歧义；法向可靠性单独按三维第二轴与既有.1m精度下限标记，法向不可靠且低于相机条带保持道路保守拒绝。query/5秒新鲜度/四支持确认/2m到达/接近及车体不变。
- 7新增最终改前两遍6fail/0error，82相关两遍/828原离线/826真实ROS两遍/32实际C++两遍通过。初始测试KeyError及窄投影=无法向、遮挡共享像素的错误夹具假设保留并更正，未降运行门。
- 最新v46回放两遍0→1真实LiDAR点（当前回波在mask内，不等于语义正确）；旧v43深度回放50可配对原输出48坐标不变，另2因8+15/9+18支持的双深度层拒绝，新增输出合计55。最初all-50断言失败保留，不能说50全不变；现场在该额外审计失败后仍启动是顺序偏差，搜索早期已复核是合理歧义，后续应阻断并列命令掩盖前序失败。
- 主703源码/旧ZIP保留、控制库217609ca...未变，当前geometry826e3040...。v47 trial110900实际query180秒现场进行中，无录像，关闭待补，不推广/commit。多数mask无足够回波仍为独立限制，详docs/DEPTH_GEOMETRY_20261027.md及depth_geometry_20261027。

### 2026-10-27 v47三维定位现场关闭：测距误拒绝修复，确认仍受输入限制（最新）

- trial110900已关闭：启动39.603/search180.200/shutdown14.818秒超时，errors=[]、console0、项目进程/X148锁空；88mask/85配对/6LiDAR/0确认。近停21.844%/最长3.956秒、净47.196/累计128.925m。路线不同，不将近停变化全归因定位修复；完整绿房屋到达未通过。
- 同一v47关闭录包旧/新真实几何两遍4→6有效定位，新增10/9独立像素条带跨度[15,1]/[13,1]原被平面展开门误拒；6新输出与现场坐标完全相同，报告7484c5f7.../旧9e33c2a0...。v46旧录包0→1仍是独立组件范围，不能用跨路线0→6虚报增益。
- 实际on_localization两遍复核：运行launch窗口60秒（非构造默认30秒）、半径1.5m/4支持，6点分约21m的两个空间簇，同簇窗口最多3帧；接收年龄设0的上界仍不确认，四帧同点正对照可确认。未见此处状态机异常；不合并两个位置、不降支持门、不宣称语义正确或已找到绿房屋。
- 85配对中76不足8像素、3无合格深度簇，可靠观测覆盖仍是独立限制，尚不能称整体三维定位完成；不盲目增加点密度/延长历史（旧探针均无效）。826真实ROS两遍/82相关两遍/828离线/32C++两遍通过，703主源码/ZIP保持，未推广/commit。19条退休限制后旧方向越界0。详docs/DEPTH_GEOMETRY_20261027.md关闭节，证据depth_geometry_20261027/field/diagnostics。

### 2026-10-27 继续：v48测距到靠近观察候选（现场待验收）

- 用户暂停时服务器项目进程/X148锁空；恢复后重查仍空，v48仅本机未部署状态起步。没有将暂停期间候选当作已通过版本。
- 固定6记录视点×16/32/64线×两遍静态对照，原几何下各12采样定位6/2/3次；增加线数会丢原有效位置，未修改运行rig或地图。服务器证据depth_coverage_probe_20261027，本地range_candidate_20261027/diagnostics/static_coverage_report.json。
- 新确定接口缺口：当前已合格3D在未满4支持时不参与观察选点，仍只按bearing；新7项两遍2fail/0error。v48从v47隔离，只改GoalStandoff和VisibleTargetApproach：同一新鲜mask的合格测距约束原车体观察步，新mask不继承旧距离；range_candidate仍显式未确认，经原GraphNav/Smac/整路控制，不改4帧/2米门、不禁用碰撞/物理。
- 61相关两遍/原828离线通过。真实ROS首次批量进程在旧retry用例附近segfault（未断言失败），原日志保留，原因未定；随后v47原826通过、v48候选833两遍及32C++两遍/console通过，未为未复现段错误改运行代码。不能写成首次回归一次全过或崩溃根因已查明。
- 同v47关闭录包真实Producer和地图/姿态两遍：旧124 bearing_only→新123 bearing_only+1 range_candidate，0确认不变；只证明当前范围进入实际观察点计算，不等于完整动作/语义/到达成功。现场待补，未推广/commit，主703和v47保留。详docs/RANGE_CANDIDATE_20261027.md。

### 2026-10-27 v48现场关闭，继续定位输入侧（最新）

- trial162318已关闭：启动41.334/search180.012/shutdown21.825秒超时，session errors=[]、console0、项目进程/X148锁空。80mask/0LiDAR/1multiview/0确认，87条提示全部bearing_only，新range_candidate未触发，无整车修复成功证据。
- 近停26.668%/最长6.148秒、净60.353/累计109.047m，比v47近停更差；路线不同且新测距分支未触发，不宣称因果或无回退，v48不推广/commit。65planning/38路径，stale17.084中planning12.423、gear4.627；不是单凭状态归属认定根因。
- Native/events两遍一致，20退休限制后旧方向越界0，命令曲率/原地旋转违约0、控制器弧碰撞0，实际库和模型SHA不变。此前61相关两遍/828离线/833真实ROS两遍/32C++两遍通过，仅组件与旧录包测距交接有增量。
- 当前继续固定视点40/80m静态量程对照，查mask零回波来源，未更改运行rig/量程/地图，结果待补。证据range_candidate_20261027/field；详docs/RANGE_CANDIDATE_20261027.md。

### 2026-10-27 40米量程根因两遍对照，v49分流候选（真实ROS进行中）

- v48零LiDAR后固定6原相机视点/mask，16线/density30不变，两次40米0/12、80米12/12定位（22–36支持，约747–749,-130房屋区域），非语义真值或完整车辆验证。静态证据depth_range_probe_20261027及localization_range_20261027/diagnostics；此前增线对照会回退，未改线数/密度。
- 最早输入限制：rig和定位/建图共用过滤云均40m，远目标回波根本未进入定位。v49从v48隔离：rig仅量程80m；Adapter按40m导航与80m定位分别限30k，occupancy/terrain严格同40m样本、各自预算不被远点挤占；新points_for_localization沿原虚拟TF/state测量戳，Triangulator订阅切换，costmap/控制仍近场旧topic。新增记录该流，未改门限/模型/物理/连续控制。
- 7最终反例两遍6fail/0error后实现，61相关两遍/828原离线/rig4通过；初始离线fake缺PointCloud2/PointField及name工厂冲突日志保留、按标准消息修正，非运行问题。真实ROS/C++待补，未现场/推广/commit。
- 采样分布和额外DDS负载会变化，不能假称地图结果逐点相同或无回退，后续须核验频率/时间戳/障碍数据与完整流程。详docs/LOCALIZATION_RANGE_20261027.md。

### 2026-10-27 v49双范围回归通过，现场进行中

- 840真实ROS两遍、原实际C++32两遍、console/rig4、61相关两遍/828离线全部通过，主703/ZIP及旧版保留；RPP仍217609ca...，node247b3c26.../pointcloud41e4a935.../tri configef5e554d.../rig6bd41bd4...冻结。
- 预检项目进程/X148锁空后启动v49实际query后180秒green residential house，无录像，关闭待补。须验证新80m定位DDS有实际远点、40m地图隔离/更新频率和定位确认接近整链；不得用静态12/12宣称整车成功。不推广/commit，详LOCALIZATION_RANGE_20261027.md。

### 2026-10-27 v49关闭：定位恢复，确认后短点串行执行未解决（最新）

- trial164340启动40.389/search180.195/shutdown25.633秒超时，session errors=[]、console0、项目进程/X148锁空。117mask/98LiDAR几何定位/70程序确认和接近消息，最小确认帧车身间距52.780m，未到2m。不是70个目标成功，语义真值/完整流程仍未验收。
- 80m定位云1052份/近场云1060份，均≤30k，最大半径79.99996/39.99999m，越界0；1051共同测量戳一致，录包近场独有9/定位独有1不称全收录。定位接收P95 .392/最大.479秒，局部map最大更新间隔.391秒，新范围未串入地图；不能据此认证全部地形分类。
- 全Native185.741秒近停37.305%/最长11.813秒、净64.320/累计92.982m，含收尾非精确180裁切；整体不合格，不推广/commit。Native/events两遍一致，13退休限制旧后缀越界0，原RPP/Smac/模型保持。
- 接近后续实证：126.663s切换2.327m中间点，129.2s新点已存在因accepted_route_active延迟；129.701s follow_aborted后仍重试旧点，137.379s结束等观察，142.812s释放。172.55s生成约(736–737,-82)接续点却持续等旧(737.219,-74.692)动作，182.95s才交接，超过搜索时限。目标位置仍约(747–750,-130)。不是3D一直无效，也不能把所有停顿一概归于此门。
- 源码confirmed_route_busy统一等动作完；弱目标有连续advance，confirmed接近缺移动接续入口。下步须按真实控制器当前路径/几何可行窗口设计连续接续，保留异步、恢复和旧动作权限保护；不能直接删busy门。本轮未对此追加未验证改动。详docs/LOCALIZATION_RANGE_20261027.md关闭节，证据localization_range_20261027/field。


### 2026-10-27 v50确认目标连续接近（现场进行中）

- 用户要求避免另建复杂防御机制，借鉴持续执行策略。只改confirmed_approach和continuous_graph_path_bridge，复用既有移动意图交接/GraphNav/Smac/FollowPath更新，保留目标身份和原执行动作，未改模型、记忆评分、2m到达或车体约束。
- v49同录包两次复核真实controller进度/odom/地图：125.425、129.196、172.553s为前向有效窗口，174.485/176.427s为倒车。最初full_planned_path源戳匹配动作戳错误已更正为planned_path，未据错误结果修改运行链路。
- 改前12新用例两遍3fail/0error，改后102相关两遍/828离线/853真实ROS两遍/原32实际C++两遍/console/rig通过。生产方法同录包受控Action状态回放两版本各两遍：125.425/172.553s可提前更新路线且保留原Action/epoch；另三点不抢占，object_found均false。组件范围非整车成功。
- 主703/旧ZIP/v49保留，无commit/推广。已预检并启动v50实际query后180秒绿房屋，收尾待补。详docs/CONFIRMED_CONTINUATION_20261027.md与test_artifacts/confirmed_continuation_20261027。


### 2026-10-27 v50关闭未到达；v51修复动作未ACK误退役

- v50 trial171954启动39.304/search180.068/收尾16.020秒，超时；session errors=[]、console0、项目进程/X148锁空。145mask/125LiDAR/41程序确认，确认帧最小车身57.616m。Native182.896秒近停27.757%/最长7.197秒、净53.020/累计125.217m；不宣称稳定或完整成功，未推广/commit，无录像。
- 63/66/70秒连续更新已现场触发。73.712秒旧segment_done，73.814秒新planned提交未ACK，73.820秒误报retired_controller_progress_unavailable并取消新任务；following在提交即真，旧_motion_intent尚存，on_route误当已接受旧动作。源码与受控状态反例两遍复核。97.725秒才出现no_vehicle_path，不能把它当最早原因或把后续真实几何失败全部归为同一原因。
- v51从v50隔离，仅修on_route旧动作归属判断，使用既有follow_active/serial/epoch，不新建状态或防御框架，不关碰撞和换向。最终新3项改前两遍2fail/0error（初版缺真实动作戳的夹具错误保留），改后84相关两遍；828离线及ROS验证待补。证据confirmed_action_handoff_20261027，详docs/CONFIRMED_ACTION_HANDOFF_20261027.md。

- v51补充验证：828离线、856真实ROS两遍、原32C++两遍、console/rig通过；主703/旧ZIP保留。预检清场后trial173048实际query后180秒现场进行中，收尾待补，无录像/commit/推广。


### 2026-10-27 v51未到达；v52对齐接近选点与局部地图（现场中）

- v51 trial173048搜索180.115秒超时、session errors=[]/console0/进程X148空。109mask/93LiDAR/46程序确认，最小确认帧车身54.397m。Native180.547秒近停32.633%/最长13.910秒、净91.231/累计105.113m；独立轨迹复算两遍一致，不称稳定/完整成功，无commit推广。
- v50接近阶段的未ACK误退役本轮未见；探索10.896秒仍有同名进度缺失，不可全称消失。120.704/123.881秒实际local_footprint_collision后128.390秒失败交接。两遍精确odom/map/path复核首点误差仅3.6e-5/3.5e-6m，前2m/裁剪均可，实际4m连接扫掠在目标前端碰撞；旧路径前缀假设不成立，未改其检查。
- 提出姿态时全局允许，局部图已在车头3.41–3.49m记录lethal、图龄.251/.469秒。GoalStandoff只接全局，未用执行局部图。v52只在该运行文件订阅已有local_costmap，近处完整车身覆盖时同时满足两图，远处仍走全局，实际Smac/控制/碰撞/身份/2m不改，不新建状态机。
- 新9项初版旧API错误保留，更正旧版输入恢复后最终改前两遍4fail/0error；改后96相关两遍/828离线/865真实ROS两遍/32实际C++两遍/console/rig通过。同录包两版本各两遍旧2/2放行、新2/2提前排除；仅证明排除已知无效姿态，非完整替代路线成功。主703/ZIP保留。预检清场后v52实际query180秒现场进行中，无录像/commit/推广。详docs/APPROACH_LOCAL_MAP_20261027.md。


### 2026-10-27 用户两张末帧质疑与确认目标保持（v53关闭/v54现场中）

- 用户第一张为v51 trial173048末帧，左侧确有绿色房屋，不能归为目标未出现；第二张为不同的v52 trial175000末帧。v52 115.680s临时点结束等待，121.059/121.060s释放房屋实例，恢复普通frontier；151.720s执行(716.375,-62.625)、frontier约(710.735,-59.560)朝加油站区域，查询仍green residential house。不是同轮连续两帧，不是到达后离开或已证实把加油站当绿房屋，而是目标保持/失败交接导致偏离。
- v52搜索180.150秒超时，session errors=[]/console0/进程X148空；Native182.243秒近停27.894%/最长15.949、净54.933/累计115.073m，独立轨迹两遍一致，不推广。
- 核对当前GraphNav plan_to_goal已有未达远目标的frontier分支。v53用单个实验bool graph_destination_approach（默认false，physical launch显式true，现场参数只读确认true）把远确认目标交给GraphNav，近处保留姿态筛选。原GraphNav/Smac/RPP不改。新6项最终改前两遍4fail/0error，79相关两遍/828离线/871真实ROS两遍/32C++两遍通过。
- v53 trial180515搜索180.116秒仍超时、session errors=[]/console0/进程X148空；Native183.113秒近停30.190%/最长13.017、净43.777/累计124.760m，已提出远目标约(747.640,-125.848)，不宣称完整修复/无回退，未推广commit。
- v54只修交接：去掉远目的地同时发裸Pose的旧入口，仅走带query/instance的确认；已确认目标的partial graph前缀结束不进入最终观察释放计时，保留目标等下一条GraphNav路径。新4项改前两遍2fail/0error，298相关两遍/828离线/875真实ROS两遍/32C++两遍/console/rig通过；主703/ZIP/控制库保留。预检清场后v54实际query180秒现场中，无视频/commit/推广。详docs/CONFIRMED_GRAPH_HANDOFF_20261027.md，关闭结果待补。


### 2026-10-27 v54关闭未通过（最新）

- trial181924启动50.983/search150.958/收尾20.837秒，不是完整180秒。150.663秒Native actuator unhealthy>.4s，车辆Lua报Vehicle control lease expired or closing，goal session_exit失败/console1；所有项目进程与X148锁空。准确feed间隙或closing来源未查明，不能直接归因本次目标逻辑，也未放宽/自动重置过期租约。
- 47mask/37LiDAR/14程序确认，确认帧最小车身50.354m。Native149.590秒近停27.585%/最长9.995、净81.757/累计106.124m，独立轨迹两遍一致。目标接近仍未通过，之前planning/执行失败不能全归于末尾Native异常。875真实ROS两遍、298相关两遍/828离线/32C++两遍是组件范围。主703/ZIP保留，未推广commit。
- 新机制保持远确认目标、复用GraphNav并消除远目标裸Pose双入口；partial前缀结束不进入最终观察释放。实际触发范围及未覆盖分支数见confirmed_graph_handoff_20261027/validation_summary.json。后续继续排查接近路径可执行性与推进，同时独立核对租约feed/closing后再180秒完整验收。详docs/CONFIRMED_GRAPH_HANDOFF_20261027.md末节。


### v54释放事件更精确复核（更正，不与v52混同）

v54确有一次target_instance_released，但原因不是v52的中间点后5秒观察超时。137.554s先发生target_instance_superseded，替代簇约(750.620,-129.983,58.180)、4/5支持，随后137.555/137.560s按现有实例替换协议释放旧实例；旧目标60.186s约(747.196,-130.085,54.793)，两者XY相差约3.43m、Z相差约3.39m。144.654s重新进入接近时又确认约(761.999,-130.031,58.783)。尚不能仅凭这些坐标判为同一房屋不同立面或另一个物体，须结合对应掩码/RGB/投影复核；不擅自扩大关联半径、取消已修复的跨实例隔离或把重绑定认作正确。

普通partial前缀等待分支本轮未触发，不能声称已现场验收。先前看到released计数时只统计，不能据该计数认定仍是相同的5秒释放故障。目标接近路径/稳定绑定仍未通过完整验收，Native异常发生更晚，不能拿它解释之前的规划停顿和实例变化。当前所有进程已退出，所有候选及证据保留，未commit。


### 2026-10-27 用户授权清理过时录包（已完成）

服务器execution_continuity_review_20261022内删除46个已关闭旧轮次录包（v2–v27，排除v25）及85个同目录字节相同的重复日志，共104,459,074,791字节（104.46GB/97.29GiB）。保留35个录包：v1/v25/v28及之后全部，包括v43蓝房屋对照、v51/v52图像轮次、v54当前故障与投影审计。旧结果、事件/控制JSONL、关键图像、诊断、代码配置保留；未删除本机录包和用户改动。预检无项目进程/X148，删除后精确核验，保留录包存在且大小一致；已删旧轮次不能再完整传感器回放，目录有REMOVED_RECORDINGS_20261027.json。详docs/LOG_CLEANUP_20261027.md及test_artifacts/log_cleanup_20261027/server_cleanup_receipt.json。此次仅清理，不表示目标接近或Native租约故障已修，未推广/commit。


### 2026-10-28 目标接近继续：端点评分契约修复，前向物理过滤仍待解决

隔离候选frontier_endpoint_contract_20261027修复Planner::plan_to_goal用最小frontier距离评分却输出所有出口均值的问题，改为复用评分出口。实际C++新6项+既有后半段23项改前两遍5fail、改后两遍29通过；原完整test_route_mechanism前半段未闭合，不称全套通过。v54旁路graph_pairs保存完整拓扑/评分，更正此前仅看bag未发现快照的判断。22份严格配对实际图两版本各两遍：15终点欧氏距离改善/2稍增/5同，远离目标仍6份；取消视觉先验诊断仍5份，不改运行评分，不称解决所有反向。pair156前向19/53原图可达但真实Planner物理过滤后不可达，净空.5–1m是主要拒绝指标；同帧实际raw costmap+odom+12m Smac六姿态两遍5可生成安全曲线/1失败，有2–8换向且原1m容差，非精确姿态或整车验收。未知区域误作障碍尚未证实，未取消物理过滤。v54租约首错RPC约1114ms，具体延迟来源未查明。v55隔离部署构建中、未现场/推广/commit。详docs/FRONTIER_ENDPOINT_CONTRACT_20261028.md及test_artifacts/frontier_endpoint_contract_20261027/diagnostics；主703及原ZIP保留。


### 2026-10-28 v55关闭未到达；v56局部连续接近候选

v55 trial083200启动62.095/search180.031/收尾22.034秒超时，session errors=[]/console0/项目进程X148空。实际新GraphNav库f573bcd1，Smac/RPP旧库保持；Native181.460秒近停24.747%/最长9.200、净46.307/累计128.728m，49mask/39LiDAR/36程序确认，最小确认帧车身56.851m，无实例释放/Native故障但仍折返，完整流程失败。原场景与新路线不同，不夸大无回退。

v56仅3运行文件：physical launch关闭实验远目标模式，复用既有近距观察姿态和移动整路交接；TargetObservation先消费排队接近，再判断partial等待；新增明确standoff_intermediate区分临时点和仅车身安全距离调整；GoalStandoff确认/刷新不同时发无身份Pose，保留带query/instance唯一入口。不是换模型或取消物理约束。原两项反例2fail两遍、重复Pose1fail；204相关两遍通过。旧三个测试依赖裸Pose/mocking签名导致首次真实ROS2fail+1error，已更新确认JSON身份/几何/时间戳约定且保留失败报告；882真实ROS/32控制通过，追加最终字段版真实ROS仍在验证。

同v55关闭录包真实producer旧/新两遍：36个远点最大68.441m→36局部点最大8.321m/裸Pose0，历史车轨迹保留，非新整车成功。诊断缺to_msg首次错误保留。详docs/CONFIRMED_LOCAL_CONTINUITY_20261028.md及confirmed_local_continuity_20261028，主703/ZIP保留、未推广commit。


### 2026-10-28 v56主动中止与用户纠偏（当前最新方向）

最终883真实ROS通过后，检查实际graph_path_bridge::on_object_confirmation发现active_object_confirmation只保留几何子集，未保存新standoff_intermediate/standoff。v56 TargetObservation读取该容器的方案实际不连通；手工设置容器的新测试未覆盖producer→真实消费者→完成，不能称端到端修复。向精确v56 driver正常结束信号，由/quit停车清场：trial085117启动56.228/search80.057/收尾13.219秒主动中止，console0/session errors=[]/进程X148空；Native81.508秒近停16.306%/最长3.562、净61.535/累计68.669m，非完整180秒，不与完整轮次排名。未推广commit。

用户强调纯WildOS原文真实车体已可直接接近目标，反对擅自叠加机制。当前停止新增接近补丁，先对照原文并以用户跑通链路为基线。原文参考在Beamng/wildos_controlled_experiment/repository/reference/nebula2-wildos：三维定位直接发导航goal；Planner按trav_cost建无向图；PathFollower随odom更新前视目标。当前新增物理硬剪图/确认接近点队列/观察等待释放恢复不是原文机制，不能笼统说真实车体必需。原坐标/时间戳/消息修复、记忆候选与执行适配须分开核验，不擅自清理用户dirty源码或关碰撞。confirmed_route_metadata_20261028仅基线副本，未改运行代码/部署v57。详docs/CONFIRMED_LOCAL_CONTINUITY_20261028.md末节。


### 2026-10-28 用户收窄范围：仅原文目标接近入口对照（当前）

用户明确“只替换为原文的目标接近模块，其他可以保留”，不做整套GraphNav回退。官方HEAD ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572已Git核验并获取定位原文；本机参考的直接发布函数一致。当前Triangulator已保留world XYZ原样发布/raw_object_waypoint。v57_upstream_target_approach从v55派生，仅桥接器订阅回该现成输出，确认只作到达证据、不生成standoff队列，绕过bearing观察/接近释放，记忆比较与区域匹配交接保留；默认模式false，本轮launch true。仅4运行文件+1测试改变，GraphNav/Mapper、Smac/RPP、模型/记忆/传感器/2m标准均保持父版，不称整个纯原版。13目标接口用例通过，172相关两遍通过，真实ROS构建中；现场待验收、不commit。详docs/UPSTREAM_TARGET_APPROACH_20261028.md及test_artifacts/upstream_target_goal_20261028。

### 2026-10-28 v57现场关闭与反向图快照复核

更正上节待验收：888真实ROS、29相关GraphNav C++、32 RPP组件通过；现场trial091404目标后180.009秒超时，未到达，不推广/commit。74.308秒定位原始目标进入规划，157条关联图路径均frontier/未完成；Native覆盖181.828秒近停27.790%、最长10.467秒、净55.886/累计124.775米，含收尾边界；3次follow_aborted，3条RPP预测碰撞弧日志，不等同3次实际碰撞。无旧standoff/观察等待/释放状态。真实运行仍是改动版GraphNav，不是整套原版基线。

本轮真实配对图快照单帧重放：91.944秒与168.175秒的终点分别距目标73.198和80.452米，和实际graph_goal吻合，起点分别66.293/66.298米；关视觉评分后仍67.891/86.881米，因此这两个例子不是由视觉评分单独造成。重放清空平滑历史，87.731秒重放向前而实际89.453秒向后，未证明单一根因；旧v54剪边实证不能直接套用v57。继续查pair235/325图物理过滤及候选成本、现场历史，并分别对齐follow中止与地图。证据和完整边界见docs/UPSTREAM_TARGET_APPROACH_20261028.md及test_artifacts/upstream_target_goal_20261028/field。未修改其他运行模块。

### 2026-10-28 v57本轮图连通性诊断（仅分析）

使用现场配对真实图和现有边准入规则离线复算；前向定义节点中心到目标比当前近至少5米。pair235前向节点有限原始图可达18、过滤后10；按节点中心前向的frontier有限图可达0，不能将这帧全部归因物理过滤。pair325前向节点有限图可达56、过滤后14；前向frontier有限图可达5、过滤后0。前向frontier370/30样例有限路径经253→103，该边记录净空0.5米、物理不可行/实测违规；这是直线图边拒绝，不证明车辆曲线必然不可达。诊断不复现运行时rejected_edges、候选评分或路径平滑状态。

约76–80秒Smac对前向图终点及附近候选返回No safe bounded continuous-start plan/empty_path；约118.97、143.01、164.93秒follow中止前均有RPP预测碰撞弧，具体局部图证据和预测正确性待复核。无运行代码改动、不宣称修复。详docs/UPSTREAM_TARGET_APPROACH_20261028.md，证据upstream_target_goal_20261028/field/v57_connectivity.json和同轮stack.log。

### 2026-10-28 v57前向Smac与中止地图对照（仅诊断）

pair325对应实际全局costmap/odom接收较图快照早0.729/0.121秒。被过滤前向frontier370/30节点及出口、不同目标朝向共6姿态，用实际12米Smac/原1米容差两遍一致：3个生成blocked=0、geometry=1曲线路径，5/6/8次换向；3个无安全连续起步路径。说明单条直线边被剪不代表邻域车辆曲线均不可行，但路径多次换向且未经过现场桥接/车辆执行，不称可流畅执行或无碰撞。精确实际地图仍随时间变化。

三次follow中止对齐最近局部图/odom，frame相同、当前足迹检查均无阻碍，地图接收距中止0.580/0.021/0.336秒；RPP检测的是未来命令弧，录包未保存该弧，因此无法判断误报；最近命令均为零可能属于停止后。证据upstream_target_goal_20261028/field/v57_forward_smac_scope.json、v57_forward_smac_{1,2}.log、v57_follow_maps.json，详docs/UPSTREAM_TARGET_APPROACH_20261028.md。未改运行代码/配置、未现场验收或commit。

### 2026-10-28 v58首次确认接管候选与执行碰撞复核

v57第二轮trial093504发现临时raw点约(746,-112)在following期间阻挡多帧确认约(748,-130)，直至执行失败才换目标。v58隔离修graph_path_bridge：首次有效确认可接管未确认临时点，复用取消和目标发布，其他实例仍受已有门限制。新反例改前失败，16目标接口/891真实ROS通过；构建安装哈希一致。测试环境路径缺失、一次拼写错误及colcon PATH失败均保留，非现场代码故障。

v58 trial094705搜索180.037秒超时未到达、正常收尾且无项目残留/X148；odom按180秒接收截取净54.473/累计145.394米，不等同Native近停统计。本轮首raw已与首次确认相近，未验收远距离确认接管分支。记录现有lookahead_collision_arc；两次碰撞中止时车到最近路径采样点0.027/0.172米、预测弧也接近路径，同一路径在当时局部/全局图均有障碍；按已下发路径接收时刻取最近global图则无这些障碍，彼时位置超出局部图。支持执行地图更新导致旧路径受阻，不能认定精确规划器内部地图相同或RPP误报。弧消息orientation统一w=1，禁止直接据此复算车身方向；近似用切向估计，非精确控制器重放。详docs/CONFIRMED_RAW_HANDOFF_20261028.md及upstream_confirmed_handoff_20261028/field。未推广commit，继续查已有路径接续与目标失败交接，不叠加防御分支。

### 2026-10-28 v59受阻同目标重算入口（现场待验收）

新增ContinuousGraphPathBridge的active_window要求旧路前2米通行，maybe_extend_path依赖该窗口，导致旧路受阻不能启动后台重算。明确属于新增适配代码缺口，不归因WildOS原文。v59隔离仅在规划入口允许读取旧路受阻但状态有效的窗口，受阻时跳过剩余距离/2米位移触发门，复用既有start_segment连续规划、结果验证和交接；不增状态/计时器，不改碰撞判断。仍仅支持原有前进接续，倒车受阻不在补丁覆盖范围。整路实际进度/地图反例旧版失败，新版894真实ROS通过，两包构建/部署哈希一致。现场前清场通过，绿房屋目标后180秒现场中，完整成功待验收，未推广commit。详docs/MAP_UPDATE_REPLAN_20261028.md及test_artifacts/map_update_replan_20261028。

v59关闭补充：trial214226搜索180.151秒超时，正常收尾、无项目进程/X148，odom180秒截取净39.915/累计143.205米，未到达，不称无回退。两次follow碰撞弧中止前均已background规划，首次替代tracking_preview_would_reverse拒绝，第二次计算未完成便中止。入口代码反例已消除，但现场可执行替代/及时性交接仍失败；后台日志本身不能独立证明是本补丁触发，不能称整链修复。详MAP_UPDATE_REPLAN_20261028.md关闭节，未推广commit。

v59时间线更正：首处RPP碰撞先于preview拒绝约0.28秒，计算结果到达仅比碰撞早约2毫秒，因此不能把preview拒绝当作碰撞首因。tracking_preview是新增整段几何预测，拒绝名不证明真实车即时要求反向。用户再次要求停止外围防御性分支、集中规划结果到真实控制的接口；未据此盲删预测或取消车体碰撞。详MAP_UPDATE_REPLAN_20261028.md更正。

### 2026-10-28 v60重算时机修正（现场中）

同v59关闭录包当前控制器方向路径与odom/global地图逐帧复核，前2米首次报障碍距两次碰撞仅0.730/0.116秒，已有接续8米范围可在3.917/3.261秒前检出。属于同轨迹诊断，非实际提前改道结果。v60仅将后台重算的global检查覆盖已有8米接续距离，local仍2米，不加状态/计时器，不放宽下发碰撞或方向判断；倒车接续不在覆盖范围。旧版新反例失败，新版895真实ROS通过，两包构建/475来源冻结/清场通过，正在目标后180秒现场绿房屋，不推广commit。详docs/REPLAN_HORIZON_20261028.md，test_artifacts/replan_horizon_20261028。

v60关闭：trial215532搜索180.045秒超时未到达，正常收尾/无项目残留X148，2次碰撞弧中止仍在。用户再次强调流畅性；以同口径180秒odom接收截取比较v59/v60：近停18.460→17.581%、最长5.593→5.305秒、累计143.205→150.405米、净39.915→45.643米，测得前后运动方向切换44→46（非变速箱次数）。仅单轮小幅指标变化，不能称稳定改善/无回退；v59有两处>0.5秒接收间隔，路径也不同。planning状态时长含后台行驶，不能当停车等待。完整目标与流畅性均未验收，未推广commit。详REPLAN_HORIZON_20261028.md及replan_horizon_20261028/field；后续两项一起报告。

v60目标路线复核：59.3秒目的地约(748.467,-130.087)，最初确认约(749.249,-130.087)，62.7秒前向终点(731.824,-70.905)，77.1/101.3秒转向(712.097,-63.147)/(750.131,-38.974)；114.7秒目的地改为(746.331,-110.506)，约19.7米跳变。目标已识别而未持续接近，非仅末端2米验收问题。upstream模式on_waypoint仍允许空闲时无身份raw覆盖expected，已确认目标持续归属有缺口；本次改写来源须对齐raw/confirmation验证，不凭坐标宣称不同物体或语义错误。详REPLAN_HORIZON_20261028.md新增节，待修复。

### 2026-10-28 v61已确认目标入口归属修复（现场中）

v60跳变来源已核实：原始Pose在更新前0.000726秒发出新点，随后0.003266秒确认话题判instance_mismatch/evidence_rejected，raw入口确实绕过确认。v61隔离仅桥接器：确认后无身份raw不能覆盖目标，有效确认按已有instance_id更新，同实例修正和首次确认接管保留；query reset继续负责切换清理。不加状态/计时器。旧版新增用例2fail，新版首次完整ROS进程段错误（confirmed_progress测试，未改模式，原因未定位），重跑898通过；失败日志保留。两包构建/475来源冻结/预检清场通过，现场目标后180秒绿房屋测试中，到达及流畅性待核实，未推广commit。详docs/CONFIRMED_GOAL_OWNER_20261028.md及test_artifacts/confirmed_goal_owner_20261028。

v61关闭trial221644搜索180.028秒超时、正常收尾/无项目进程X148。约74.3秒首次确认接管临时点至(749.941,-130.083)，随后raw及拒绝消息未再改写目的地，目标归属在本轮有现场支持，完整到达失败。odom口径近停22.632%/最长8.244秒、累计128.384/净40.628米、测得方向切换49，四处接收间隔>0.5秒；比v60差，不能称整体改善/无回退，不推广commit。确认后图目标仍转后方(711.746,-68.941)/(711.429,-59.419)，车辆规划又多次缩短graph_goal，持续推进仍缺失。详CONFIRMED_GOAL_OWNER_20261028.md关闭节及对应field审计。下一步聚焦图可达出口与车辆候选缩短，不能把入口修复称到达已解决。

### 2026-10-28 v62在途规划路线归属（现场中）

更正v61“缩短终点”的部分解读：旧FollowPath在新ComputePath未返回时完成，on_follow_result先采用latest_route再检查pending，把在途计算的图路线换掉；旧候选(733,-69)结果返回后日志却标新graph_goal(742,-77)。这是新增适配真实归属错误，不能将全部差异当主动缩短，也不能断言其他短候选都正确。v62仅把pending检查移至采用latest_route之前，不增状态/计时器。新增2项旧版1fail；全套单进程又段错误，保留日志；逐既有模块独立进程900项全部通过，非生产段错误修复结论。两包构建/475冻结/预检清场通过，现场目标后180秒绿房屋中，目标与流畅性待验收，不推广commit。详docs/PENDING_ROUTE_OWNER_20261028.md及test_artifacts/pending_route_owner_20261028。

v62关闭trial222746搜索180.124秒超时、正常收尾/无项目残留X148。odom近停28.910%/最长7.801秒、累计124.463/净62.946米、方向切换49（非齿轮换挡），3处>0.5秒接收间隔；净位移增加但近停更差，整体不通过不推广commit。确认后目的地未再覆盖。planned.route_stamp与GraphNav同戳终点审计v61的22份/v62的18份都0不一致，该指标不足以辨别请求上下文被换，因为旧版换route后的日志本身可以自洽；不能把它当现场归属修复证据。反例证明代码缺口，整车效果仍未证明。详PENDING_ROUTE_OWNER_20261028.md关闭节及pending_route_owner_20261028/field。

### 2026-10-29 v63关闭与道路地面误标（已证实局部原因，v64待验收）

v63显式路线拒绝反馈候选trial20261028_230923启动42.886/search180.096/收尾22.627秒，超时、正常关闭，完整目标未通过。180秒odom口径近停26.154%、最长17.105秒、累计128.833/净42.618米，运动方向切换43次（非换挡计数）。组件902真实ROS/31 C++通过不代表完整到达；未推广commit。

沿请求24/34复核同帧原始点云、odom变换、/wildos/points与实际地图：目标前道路出现横向/弯曲lethal条带。独立BeamNG静态castRayStatic地表采样4225点，网格x726..742/y-84..-68、间距0.25m；车体/轮胎碰撞保持、车辆未移动。两帧关注区域x728..740/y-82..-71分别107/163个障碍标记与平滑静态表面高度相差不到5cm，证明局部地面误标，非仅凭图像猜测。请求时地图和odom接收均新鲜；不据此排除所有其他时间问题。证据服务器v63_explicit_route_feedback/request_audit与static_20261029_150013，本机test_artifacts/explicit_route_feedback_20261028内诊断脚本。

已定位pointcloud.py近处单平面外推、远处先按该平面高度筛选，加上稀疏扫描环无法通过密集相邻单元连通，导致缓弯路面被当作抬高障碍。误标进入地图并膨胀封路，Smac无安全路径/缩短路径，属于本轮接近失败的直接阻塞因素；尚未证明它解释全部走停、换向和目标失败。

隔离v64_ground_surface基于v62，仅修改运行pointcloud.py，不包含v63新增反馈，不改GraphNav/目标身份/到达标准/控制器/物理参数。采用当前帧实测点支持的局部地面片连接，保留路沿和抬高回波；中间会删孤立抬高表面的候选已淘汰。最终同录包两次复算，平滑地面误标107→8、163→0，关注区各4个高于地表至少0.1m的标记均保留，clearing点完全相同。仅覆盖两帧，不能泛称所有障碍均正确；每帧耗时由约8–12ms增加至100–124ms，现场新鲜度及整车效果待验证。本机报告test_artifacts/ground_surface_repair_20261029/segmentation_replay.json。

本机35项相关点云测试通过；服务器构建通过、新增5项及既有点云测试通过，但完整独立ROS模块套件有test_scanline_depth未通过，不能称全套通过或进入已修复终态。未做v64整车验收、未修改主运行源码、未commit。后续先处理验证环境/失败原因，检查历史真实障碍保留及处理延迟，再做一次目标后180秒验收，不能通过再次叠加导航重试分支替代地面数据修正。

### 2026-10-29 v64现场回退；用户要求原文目标流程对照

v64 trial152928目标后180.004秒超时，启动47.971/收尾22.825秒，console0/session errors=[]/项目进程及X148空，未到达、不推广不commit。180秒odom接收口径覆盖178.868秒：近停31.819%、最长8.077秒、累计110.947/净52.354米、运动方向切换61（非换挡）。相对v62近停28.910%、累计124.463/净62.946米发生回退，不能称地面修正带来整车收益。

同源header点云接收延迟中位42.5→176.4ms，P95 78.6→205.7ms；上游适配点云频率5.72→2.86Hz。node.process_lidar内点云发布后同步调用occupancy，按周期调用terrain，两者共享分割函数。新增局部地面拟合不仅消耗CloudFilter时间；每帧100–124ms基准没有充分覆盖同步回调总预算。不能仅据不同路线的两轮数据把全部近停定量归因CPU，但已足以拒绝推广。905项组件/51适配器/22历史帧早期障碍保留通过，现场仍失败。证据本机Beamng/test_artifacts/ground_surface_validation_20261029/REPORT.md及field/*.json；路面误标的既有独立实证仍保留。

用户明确停止v64，按原文看到目标后接近/到达策略。v65_upstream_object_flow隔离基于v62，仅桥接器与arrival_criteria独立验收适配及相关测试改变，未改定位器、记忆、模型、图规划/车辆控制或主源码。raw三维目标更新不取消当前FollowPath，当前里程计按车身外缘2米终态，额外确认仅供记忆不再控制导航；query清目标，记忆视点不能假冒当前三维目标。独立完整验收仍核对新鲜确认/掩码与真实2米距离，防止把导航日志当完整成功。原文PlannerNode实际默认3米三维清目标；通用GoalNavigator二维2米不是WildOS主链，不能混同。v65保留用户2米车体标准，因此不是逐字节原版。

新3项在v62反例3fail/0error；首版896真实ROS通过，最终验收连通性回归及现场待补。恢复原文raw更新意味着不保留v61确认后拒绝raw修正的导航所有权策略，属于本次用户收窄的对照，不宣称多实例/绑定问题已解决。原定位器quality/query/TF修正及记忆确认记录仍保留。详Beamng/test_artifacts/upstream_object_flow_20261029/REPORT.md。主源码未覆盖，未commit。

### 2026-10-29 v65原文策略对照关闭：仍失败，不推广

最终897项独立真实ROS、51适配器回归通过；v65 trial155726启动45.800/search180.179/收尾25.230秒，超时、console0/session errors=[]/项目进程及X148空，未到达。原文策略仅目标直发与距离完成，保留已有三维定位器及用户2米车体度量，不称整套原版。完整视觉验收仍是独立新鲜确认/掩码匹配，新增真实距离事件到验收接口的回归。

180秒odom接收口径覆盖179.764秒：近停35.668%、最长13.953秒、累计101.591/净78.512米、方向切换39；planning121次，其中76次后台。净位移增加，但近停和累计行驶比v62回退，不能称整体改善；路线不同，不将全部差值武断归因两文件。无object_found，未推广commit。主运行源码未覆盖，v64/v65均仅隔离候选。

重要时序：最长近停query+46.127至60.080秒，首三维raw目标query+73.099秒才出现。期间FollowPath中止后四次motion_section_invalid detail=local_footprint_collision。因此该长停不由目标到达确认造成，车辆执行适配层在目标出现前已存在独立问题；尚需地图/扫掠复核，不能称真实撞车或误报。retired_controller_progress_unavailable及后半程follow_aborted仍存在。

v65点云约6.06Hz、同源raw→filtered接收延迟中位38.5/P95 56.7ms，v64新增拟合耗时已撤出；不能据此替代控制命令新鲜度验收。首两个raw目标XY跳变45.392米（718.128,-95.291→747.277,-130.087），仅为位置变化事实，不凭坐标判语义错误。以最后确认点(750.654,-130.083,56.994)为参考，最小车身距离50.682米，未到2米范围；该程序确认点非独立语义真值，故本轮不能现场验收到达分支。证据及改动边界见Beamng/test_artifacts/upstream_object_flow_20261029/REPORT.md、field/*.json和failure_last_rgb.png。不要继续把所有走停推到末端确认，不再向接近流程叠加分支掩盖执行故障。

### 2026-10-29 用户追问v60前五版：统一历史近停统计（只读回放，无新仿真）

重新读取v55–v59已关闭录包，以与v61–v65相同的odom接收时间、速度<0.1m/s口径统计目标下发后180秒；v56严格截取搜索80.057秒，不能与完整轮排名。v55 trial083200近停22.144%/最长9.452秒；v56 trial085117搜索中止，79.912秒有效覆盖近停13.594%/最长3.110秒；v57首轮091404近停27.239%/最长8.979秒，复测093504近停17.569%/最长3.293秒；v58 trial094705近停17.050%/最长3.982秒；v59 trial214226近停18.460%/最长5.593秒。所有轮次完整目标验收仍未通过。旧记忆v55 24.747%、v56 16.306%、v57 27.790%来自Native及不同边界，保留旧值不覆盖，但不能混作同口径排名；v57同配置两轮也有较大方差，不把单轮版本差异全部归因修复。此次证明历史完整单轮已有17%–18%水平，不能只以较差v62作流畅性比较基线。证据Beamng/test_artifacts/upstream_object_flow_20261029/historical_v55_v59_comparison.json，服务器各轮fluency_odom_audit.json及v56/fluency_odom_search_window_audit.json。

### 2026-10-29 新目标时限120秒；原文目标尾端对照v66（现场中）

用户目标为保持流畅、参考原文目标接近、不擅自新增模块、完成完整搜索；每轮目标后120秒，启动/收尾另计，覆盖180秒。此前只回答版本数字的目标轮属于无进展，本轮重新核查当前源码、进程及证据后继续工作。服务器预检无项目进程/X148锁。

复核v58→v65冻结清单：58→59/59→60只改变连续桥接文件，60→61/61→62改变目标/请求归属桥接，62→65为桥接+独立验收。控制链没有整体换版；不能将单轮波动全归因某行补丁。v65真实图7帧中最近图节点到目标平面仍32–47米，6米内节点0（首临时点最近31米），三维/二维均不能直接接图，所以目标Z虽不同，不能认作本轮远处失败首因，未据此改高度。

新确认的原文差异：原文Planner在frontier路径后保留virtual goal，当前适配主动删掉该尾端。v66仅在PlannerNode显式目标且非空/未complete的图前缀之后恢复目标点；自主探索不加尾端，Smac/控制/碰撞/记忆不改，无新增运行模块。控制保留v60已验证的连续重算方式与v61/62必要归属修复，相对v62新的运行源码只有planner_node.cpp三行条件行为，未混入v63/64/65新机制；不把v62较差单轮作为流畅性门槛。

实际ROS PlannerNode同一真实CDR图：改前远目标(750.654,-130.083,56.994)的发布尾点(741.063,-94.669,52.949)，complete=false；改后尾点为目标XYZ且仍complete=false。已知目标路径两版complete=true且尾点正确。原文参考及实际节点对照而非仅静态函数测试。900独立真实ROS与6端点评分C++通过；实际新节点路径→原route_candidates+录包global图仍输出6–24米内候选，不把整条未知尾端直接作为执行指令；后续真实Smac/整车待验收。图快照对应地图接收差1.292秒，因此该接口检查不当作同步地图真值或可行路径证明。

120秒历史同odom口径：v57复测21.105%、v58 17.689%、v59 14.635%、v60 17.865%、v62 37.087%、v65 34.347%，均为旧完整录包前120秒，非新现场稳定性统计。证据服务器v66_original_approach_120s/baseline_120s_metrics.json。v66来源475成员冻结、运行GraphNav库bf4df4461ae967fbc90ba5aaa884630c2aaf40373adcaaf418ad3f3c0a52477d；driver和config均120秒。已预检清场后开始首轮120秒绿色房屋搜索，无录像、未改主运行源码、未推广commit，现场结果待补。

### 2026-10-29 v66首个120秒轮次关闭：低近停恢复，完整目标仍未通过

trial164525启动43.653/search120.185/收尾23.024秒，超时、console0/session errors=[]/项目进程与X148清空。120秒odom有效119.930秒：近停14.230%、最长3.612秒、累计108.129/净41.092米、方向切换36次；与旧同窗口v58 17.689%/v59 14.635%相比，本轮低近停恢复，但只有单轮、路线不同，不称统计稳定或归因全部改善。程序目标约query+53.676s首次进入导航。未到达、不推广commit，保留v66冻结版本及原文显式目标尾端修复。绝不因低近停便将完整搜索目标标为完成。

v66真实配对图按无向有限基图与现有物理边规则复核（不含运行时rejected_edges/平滑历史）：60秒前向frontier基图11/过滤后7；75/90/105秒前向frontier基图即为0；119秒为21/21。不能靠直接删除物理边过滤解释或解决全部前向探索缺失，未删除过滤。目标前道路点云请求40/46：与平滑静态表面吻合标记135/58；进一步要求静态Z与Terrain Z相差<5cm后为122/58，前一135中13个平坦平台不能直接称路面，应保留该更精确边界。新优化候选的Terrain吻合误标122→5、58→0；其他两历史帧107→8、160→0。检查区域原抬高标记4/4/8/5全保留，四帧与原正确分类候选marking/clearing逐点一致。

当前仅分析副本pointcloud_fast_review.py（服务器v66根目录，不在安装包）：对先前正确但昂贵的地面分类计算做数值优化，共用一次SVD计算拟合和leverage、固定点收敛后停止、共享排序后的线性分位插值；阈值/分类规则不变，不新增导航或记忆运行模块。35点云测试通过、22历史接触帧早期障碍保留。服务器分割中位40–44ms（旧v64约98–127ms），occupancy+terrain纯数学总约138–148ms（旧运行约76–87ms）。terrain仍原有0.5秒周期；数据处理尚有额外开销，尚未验证实际适配节点DDS/队列/心跳开销与完整车辆近停，不能将其认作已解决v64回退或直接推广。v66低近停运行安装包未替换。拟合缓存无收益、粗网格简化未通过真实录包，均未采用。

证据本机Beamng/test_artifacts/original_approach_120s_20261029/REPORT.md、field/{result,fluency_odom_120s_audit,arc_trial_audit,static_comparison}.json，服务器同名v66目录ground_fast_final_review.json、request_audit、v66_connectivity.json与tail_before/tail_after_export。下一步先验证优化计算在真实ROS适配处理/发布预算内，保持v66对照，再决定是否做下一轮120秒整车搜索；不改速度掩盖停顿、不把仍远离目标的失败推给到达判定，也不继续叠加接近状态机。主工作区运行源码未覆盖，无commit。


### 2026-10-29 用户收窄到目标到达；v67接近推进改善但未到达

用户明确当前流畅性已够，停止继续优化近停，集中解决目标接近/到达，不擅自添加组件。停止性能扩展，保留v66冻结对照；未覆盖主工作区运行源码。

隔离v67_target_road_120s相对v66仅pointcloud.py运行差异，采用已验证的弯路地面分类修正及分类结果一致的计算优化，不改目标绑定、GraphNav、连续控制、记忆、真实车体/轮胎或2米车身标准。478成员冻结（475原成员+新曲面测试/两夹具），源码/安装pointcloud SHA613caaf0740fc72a85765ccc899935cd8f1909f3b44943b3b430c39145110415，GraphNav库仍bf4df446。905独立ROS/51适配通过，不代表完整验收。真实Adapter+DDS回放6Hz/48帧：优化版完成延迟P95约210ms，无重版3.3秒持续积压，但辅助回调间隔仍可达.49秒、旧版.18秒；不称延迟完全保持，用户收窄后未继续调性能。

trial212751启动55.580/search120.031/收尾23.832秒，超时、console0/session errors=[]/项目进程及X148清空，无录像。120秒同odom口径近停15.876%/最长4.861秒、累计109.222/净91.010米、运动方向变化14（非换挡）。v66为14.230%/累计108.129/净41.092米，流畅性略差1.65个百分点，净推进改善但单轮路线不同，不宣称统计稳定/无回退。以各轮最终程序确认点作参考（非独立语义真值），最小车身距离v66 58.945米、v67 30.557米；均未到2米、没有object_found，完整目标仍失败、不推广commit。

v67约52.094秒首次程序确认，55/70/82秒下发终点连续靠近目标。95.751秒RPP碰撞预测中止，98.232秒改为西侧(718.625,-102.125)，离导航目标由33.808增至40.398米。中止时车(735.208,-98.587,yaw-1.122)，路径横向差.052米/航向差-.017rad；近处路径车身在局部图有lethal，原全局图未命中lethal。首阻碍约(738.095,-101.700)，当前marking三点Z约52.76，局部cost100/全局cost99，图接收龄.221/.488秒，分辨率.2/.25米。已证实际地图表示/时序差异，未隔离分辨率、清除、时间与真实物体/地面分类，不能直接认定唯一根因。未放宽碰撞、改分辨率或新增恢复机制。

诊断边界：tracking_collision_audit的at_dispatch.local=(3.41,1.06)是超出局部窗口的角点哨兵，不能说当时已有障碍；预测弧orientation全w=1，不把切向近似当精确控制器重放；本轮无RGB入包，仅末帧，不能恢复中止RGB。分析脚本相对路径as_uri及缺RGB两次失败保留，更正后完成，未影响运行。

证据Beamng/test_artifacts/target_road_120s_20261029/REPORT.md及result/target_approach_audit/fluency_odom_120s_audit/tracking_collision_audit/collision_arc_audit/abort_obstacle_source/final_prestart JSON；原始闭合录包在服务器v67/full/trial_20261029_212751。下一步专查约96秒这一反例的同源障碍写入/清除及规划-执行栅格检查，确认最早差异后修改已有实现；不再优化流畅性或叠加组件。


### 2026-10-30 上下文导出与最新原文三维3米要求（尚未部署完成）

用户最新明确：直接采用原文车辆里程计XYZ到定位目标XYZ的欧氏距离严格小于3米结束导航，覆盖旧车身外缘2米要求；保留目标后120秒、不默认录视频、不新增组件、集中接近/到达。完整搜索仍未通过。

v68全局/局部1秒观测保留实验trial221238：120.118秒超时、近停23.540%、累计89.610/净77.316米、最近旧车身口径44.719米，回退未采用。v69仅全局保留实验trial221844启动控制门/enable参数更新超时，goals=0、console1，不能算搜索或近停验收；已清场。原生Nav2缓冲器确实两次复现默认最新帧漏掉中间障碍，1秒保留可检出，但不等于整车修复成功。用户纠偏后缓冲实验全部撤回；主config/test_runtime_contract恢复改前完整字节，config SHA5a7df9ef2dae4c5e0f4adc9453977079a80fd93c32bb929f3b27cedd43959dc3，测试SHA7d9ff35a6d78918c6fd80254d7f9ca5bbbd04b6c4115e23692c61e801d99015b。

v70_upstream_arrival_120s基于v67复用v65原文式距离完成的既有桥接/独立验收实现，但本次只读核对其candidate_scope仍为旧车身2米口径。三维3米修改脚本仅确认本机存在：Beamng/test_artifacts/target_road_120s_20261029/set_upstream_3d_arrival.py；未发现服务器该脚本或after_3d_related_tests.log，上次长工具调用结果未确认，不能宣称三米已部署或测试通过。此次服务器预检项目进程为空、X148锁空，不启动新测试。没有commit。

交接文件：C:\Users\Yu\Desktop\拟真\docs\CONTEXT_EXPORT_20261030.md；辅助副本C:\Users\Yu\Desktop\Beamng\CONTEXT_HANDOFF_20261030.md，包含目标/最新约定、原文依据、v57-v70证据、失败机制与验证边界、文件/服务器入口和下一步。继续时先核实v70实际状态，再贯通三维3米的实现、参数、严格边界和独立验收，不能沿用旧2米通过结论。

### 2026-10-30 原文3D3米已实施；目标坐标细化误作新任务已修，完整验收仍失败

用户再次明确绝不擅自新增模块，集中目标确定，依照原文。已同步主/辅助AGENTS入口：当前导航完成为里程计XYZ与定位XYZ欧氏距离严格小于3米，旧车身2米不再是当前标准；120秒、无默认录像维持。最新状态覆盖上节“3米脚本仅在本机”的待执行状态，但保留该历史说明。

v70候选已实际应用原文距离完成与现有运行契约3米配置，现场参数查询确认3.0。目标直接保存/更新不reset健康FollowPath，确认仅供记忆与独立验收。18项真实ROS目标链路改前4fail/0error、改后通过。早期测试缺ROS初始化及运行后修改冻结参数被拒绝属于测试入口/夹具问题，已按启动参数修正，没有放宽生产规则。

再定位到ContinuousGraphPathBridge.intent_key把原文raw目标的精确XY作为任务身份；同一查询目标从(20,0)细化到(20.04,.02)会实际调用discard_intent_plan。仅在原文目标模式改为既有query_generation约束的object身份，坐标仍持续参与规划，查询/recall切换保持原约定，无新增状态/距离阈值/队列/模块。实际回调反例旧1fail/0error，相关34项通过，最终907独立ROS模块与37纯契约通过。原始Pose没有语义实例ID，本修复不宣称所有多实例问题已解决。

现场第一轮trial20261030_094101（三米已改、身份未改）搜索120.053秒超时，近停25.077%、累计86.709/净67.486m、planning85/后台74、至最终程序确认点最小XYZ54.471m。身份修正后trial094938启动36.474/search120.095/收尾16.421秒，超时、console0/session errors=[]/项目进程和X148空；近停18.908%/最长6.904秒、累计100.277/净90.226m、planning39/后台28、最小XYZ35.201m。目标约77.865秒首次确认，末段仍离目标很远，无object_found，不能称整车通过或优于历史低近停基准。单轮不同路线/检测时刻不能将差值全部归因一处修改。

本次仅四个已有运行文件：graph_path_bridge.py、arrival_criteria.py、runtime_contract.py、continuous_graph_path_bridge.py，两个已有测试文件；没有新运行组件。基于v67保留点云和v66目标尾端、连续控制，不含撤回的缓冲实验，不改模型/记忆存储/物理/Smac/RPP。服务器候选/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s；bridge源码/安装SHA1f02171987294f2cbb831cdda5eae2c19b19058ec35950a8ebce740d9a1b4ae6，continuous SHA23537e712006c6777c89b1d1c95711283723bf0b31075266314a0d73bd5a200e。

本机可审查源码/回归/两轮证据：C:\Users\Yu\Desktop\Beamng\test_artifacts\upstream_3d_20261030\REPORT.md及nav2_adapter/、field_after_identity/。主工作区用户源码未整仓覆盖，候选未推广、未commit。下一步只用闭合录包继续查目标确定后的实际图路线和车辆终点，不能将35米外未到达全部归为末端确认或擅自增加机制。

### 2026-10-30 v70同版本复测095616走向树林；冻结运行代码待只读定位

本轮无运行代码变化。启动34.723/search120.060/收尾18.420秒超时，console0/session errors=[]/项目进程X148空。120秒odom近停26.943%/最长5.371秒、累计78.956/净33.834m，planning32/后台22。视觉query ACK已接受，闭合录包直接raw_object_waypoint计数0、确认0、无object_found，未进入目标接近。末帧密集树林；约39秒终点向西(718.375,-14.125)，后续至(713.769,-15.951)/(705.375,-26.375)/(703.625,-31.625)。同版本和上轮路线/推进差异大，完整探索稳定性未解决，不能以目标身份局部修复宣称全局可靠。37秒左右follow_aborted，58秒左右handoff_actual_pose_cannot_join，110秒左右handoff_prefix_blocked；精确最早根因未查清，不擅自归因模型、坐标或目标三米标准。

用户再次质疑擅自修改。当前已明确冻结运行版本，查清转向树林前frontier选择与路径下发的原文差异/证据前，不再改运行代码或叠加模块。失败证据C:\Users\Yu\Desktop\Beamng\test_artifacts\upstream_3d_20261030\field_repeat_095616\及REPORT.md末节，服务器闭合原包v70/full/trial_20261030_095616。没有视频、没有commit。
### 2026-10-30 v71方向禁行有效期不同步：已修组件，完整验收仍未通过

用户“快修复”后按v70 095616闭合录包定位：pair181道路frontier45视觉评分约0.646，物理评估已过期约9.8秒（既有TTL5秒），但16个缓存heading均禁行。既有物理过滤忽略过期评估，新增heading消费入口却仍硬拒绝，转选西侧48（分数约0.3）。该帧32/32 frontier均有评分，缺评分假设已排除。不能把这一反例泛称为所有树林/路线方差的唯一根因。

隔离v71_heading_freshness仅修planner.cpp和planner_node.cpp两个已有消费入口，让heading禁行和效用随已有源评估有效期同步失效；新鲜禁行含低置信度显式heading禁行仍保留。没有新模块、状态、参数、阈值，不改真实物理/Smac/RPP碰撞或XYZ严格小于3米到达。旧反例1fail，新版8项原生端点/heading及15项相关原有路线/目标/效用通过；不是全历史C++套件通过。真实PlannerNode控制拓扑/完整录包拓扑均旧版过期仍选西侧，新版选道路；改为新鲜禁行则仍拒绝道路。重放没有恢复backoff/访问历史，不替代整车。

首轮trial20261030_101938启动37.539/search120.167/收尾23.435秒超时，console0/session errors=[]/清场空，无录像。120秒odom近停16.311%、最长6.293秒、累计102.969/净72.361m、planning28/后台17，1.029秒odom接收间隔。首次确认query+89.392s，以最终程序确认点为参考最小XYZ33.233m，无object_found；末帧在道路上，不能称成功或稳定改善。复测102415启动控制门follow_path参数更新超时，goals0，未进搜索，清场空；原因未定位，不统计搜索近停，未改门或叠加恢复。随后保持同版本再次复测中，结果待补。

服务器候选/home/ly/execution_continuity_review_20261022/v71_heading_freshness，运行实际GraphNav库SHA857549906bcaebd6ff32f0f72a91993a066b79b8bc9759d2b8b5a0497a794510。478成员冻结，仅2源码+1测试区别v70。首轮8项测试一度误source旧overlay，加载正确v71库后通过，入口错误日志保留。主仓库用户未提交工作未覆盖；完整目标失败不推广、不commit。证据C:\Users\Yu\Desktop\Beamng\test_artifacts\heading_freshness_20261030\REPORT.md、graphnav_planner/、field_101938/及startup_failure_102415/。

v71复测关闭补充：trial20261030_102622启动36.829/search120.019/收尾22.236秒超时，console0/session errors=[]/清场空。近停25.738%（首轮16.311%，不能称稳定改善）、最长6.315秒、累计86.604/净74.950m，planning37/后台25，无大于0.5秒odom接收间隔。首次确认query+92.580秒、共24确认，最终程序目标(748.266,-129.610,57.958)，最小XYZ49.277m，无object_found。确认后约27秒仍沿道路推进约28米，时限先到，未进入3米区；本轮不支持“靠近后到达门反复拒绝”解释，也不证明语义/高度正确。已复核末帧：仍在道路/路口，可见红砖建筑，不能仅凭末帧断言整条路径无问题；未出现前轮深入树林的末景，但全部完整目标均未通过。没有后续测试运行，无推广/commit。证据heading_freshness_20261030/field_repeat_102622/及REPORT.md。

### 2026-10-30 用户追问未到达：接近时间线与距离口径更正（只读，无代码修改）

复核本机两轮events：v71 101938在query+82.40秒首次planned/following远目标(746.243,-111.618)，95.83/108.37秒改至(747.324,-130.087)/(749.644,-129.983)，后两段仍following；复测102622在query+90.86秒首次该类目标planned，103.66秒后台接续，120秒结束。不能把首次额外confirmation的89.39/92.58秒说成导航等待确认门的起点；raw目标已经先参与导航。

重要口径更正：第一轮33.233米是至最后一次额外程序确认点(746.239,-112.243,54.978)的XYZ距离，该点之后raw又更新导航目标。末车至最新planned.graph_goal的XY已50.855米，不能把33米当作当前导航目标距离。首次和末次远目标XY差18.678米，来源/是否同一物体合理位置更新未查清，不仅凭坐标认定实例误绑。两轮未进入当前目标3米范围，时限先结束，不能归因末端距离判断拒绝。此前heading修复只处理已证过期禁行，早期探索/执行效率及目标坐标变化仍需独立定位。本次仅分析记录，没有修改代码或开启新测试。

### 2026-10-30 v72局部朝向对照不足以支持整体修改：现场失败，拒绝采用

用户要求最快修复、不加模块。本次先只读定位v71目标跳变：约95秒上游raw从y=-112改至-130，同帧额外确认判instance_mismatch。尚不能凭该判定证明换错物体（大房屋不同墙面也可能被误分），未新增锁点/绑定。按相同实际掩码/定位专用点云/odom验证TF重放，39条有效定位逐点一致；最早掩码约45秒仅1–4回波，原文20点中值比当前稀疏8点更晚输出，恢复该阈值不能解释或解决本轮迟到。最初复用旧脚本硬编码points_filtered产生错误对照，改为配置points_for_localization才与39条录制定位匹配；错误报告另存，不作结论依据。

随后将v71约40.828秒同地图Smac对照中观察yaw1次换向、现有圆弧yaw0次换向，错误扩大为整车修改依据；另两处3种yaw均无路，原本已有证据说明并非整体解。v72只在ContinuousGraphPathBridge.begin_exploration_comparison删除普通frontier观察yaw的优先约束，保留原候选fallback，记忆/确认朝向不改；1个运行文件+1个测试，无新组件。旧反例失败、67项相关真实ROS和构建通过，但不能证明探索策略修改必要且整体正确。首次构建colcon未在PATH，改用venv绝对路径通过，此入口错误保留。

v72 trial20261030_111116启动40.949/search120.126/收尾16.832秒超时，console0/session errors=[]/项目进程及X148锁空。近停39.151%、最长8.238秒、累计64.846/净51.526m，planning47/后台21，无大于0.5秒odom接收间隔；不采用、不推广、不commit。独立三维审计工具在无合格确认时退回二维goal，math.dist维数异常，不能编造三维距离，该工具错误不等于生产异常。服务器存在其他GPU负载、路线不同，不能严格将全部指标差值归因补丁；不利现场结果足以拒绝推广。

用户“他妈的在这瞎给老子改是吧”后立即停止本方向和后续运行代码修改/测试，承认以局部对照支撑整体探索修改证据不足。v72保留失败源/测试/证据并有REJECTED.txt；v71基线保留但也未完整验收，用户主仓库未覆盖。当前没有测试运行。证据C:\Users\Yu\Desktop\Beamng\test_artifacts\vehicle_heading_20261030\REPORT.md和field/；目标流与原文对照在heading_freshness_20261030/field_101938/target_flow_audit.json、original_depth_comparison.json。后续不得再把局部可行性或组件通过升级为全流程修复依据；先围绕用户目标接近核实真实输入输出，不新增或重启此候选。

### 2026-10-30 用户要求再试v71：已知目标后图前缀转西（新现场反例，仅诊断）

用户“先继续试一下”后仅复测保留v71，无运行源码/配置改动，未加载弃用v72。prestart无进程/X148，实际GraphNav库857549906bcaebd6ff32f0f72a91993a066b79b8bc9759d2b8b5a0497a794510，连续桥接器仍v70安装23537e712006c6777c89b1d1c95711283723bf0b31075266314a0d73bd5a200e。trial20261030_174941启动59.967/search120.012/收尾23.437秒超时，console0/session errors=[]/关闭清场空，无录像。120秒odom近停21.510%、最长6.905秒、累计96.049/净76.575m，planning51/后台38，无大于0.5秒接收间隔。41raw/41定位/35确认，首raw69.945秒、首额外确认74.12秒。末车至末rawXYZ60.253m，raw不是独立语义真值，未到达。

73.767秒向南接近；87.966秒目标仍(748.884,-130.084)，车(734.935,-68.323)，车辆候选却至(718.625,-64.375)。真实route_stamp=1793353927647101131精确对齐，GraphNav输入路线本身先向西经(725.568,-64.201)/(717.768,-65.335)，再附目标尾点，route_status mode=frontier/complete=false；转西来源已定位为图规划frontier前缀，不能归因车辆桥接坐标反转。88.459秒FollowPath aborted，89.636秒随后改西南前缀。原文距离3米终态未进入，之前仅用晚接近/120秒解释不能覆盖本轮反例。

只读源码确认plan_to_goal先找已知图到目标路径，失败再用frontier虚拟目标代价（距离/视觉及新增物理/heading）选择。仍待对该帧图和地图核实前方候选何种过滤/评分/真实受阻；不能据转西本身断言必然错误或必要绕行，更不能再用单一局部对照改探索机制。用户追问“为什么有目标反方向”时继续只读，无运行修改。证据C:\Users\Yu\Desktop\Beamng\test_artifacts\heading_freshness_20261030\field_repeat_174941\target_path_interface_audit.json、target_flow_audit.json、events.jsonl、failure_last_rgb.png及REPORT.md。新增audit_target_path_interface.py仅为闭包诊断，不是运行组件。没有commit或后续测试运行。

### 2026-10-30 按用户上传方案实施v73（候选现场中，未commit）

用户明确“按照这个修改”，授权视觉目标水平距离严格小于3米，覆盖旧XYZ严格3米；原始XYZ继续供定位、确认、记忆，不压平/伪造高度。方案“v70未部署”已过时，保留更正；零副作用、仅分辨率导致、只剩3个问题等推断未写成事实。弃用v72，v73基于保留v71。

按序修改7个已有运行文件、4个已有测试：bridge/arrival_criteria统一visual_goal_distance_xy；局部有效覆盖8米后台检查，拒绝新规划的局部前瞻碰撞，边界外不伪报；显式目标的背离图出口或无候选时先试前向8米及左右2米，复用Smac/运动学/失败缓存，保留原备选；Planner显式目标仅当严格图无可达且缩短目标距离的出口，才在副本对0.5米到平台净空门槛间且其他地形/动态指标合法的现有有限边赋软代价，不造边、不改原始测量，不改变自主探索严格图。不增运行节点/组件/任务状态机；速度/模型/点云/车体/观测保留时间未改。局部地图遵循原Nav2中心99/车体轮廓100语义，避免再次膨胀整车。

最终独立相关ROS912项、运行契约28项、原生端点/heading/净空12项及既有路线/目标隔离/快照18项通过。首次完整套件遗漏visual_navigation/test路径，test_scanline_depth导入失败0项，补齐路径后7项通过，原失败保留。v57 pair325原生重放两遍从后方前缀改为先向前，严格已有前向出口的近期3图保持原行为。v67控制器路径/odom/局部图两遍在91.32987秒新检查触发、旧检查通过，较95.751秒中止早4.42113秒；同帧真实12米Smac两遍各3个候选中均有1条168点路径通过局部8米校验。以上是切片反事实，不是车辆执行/整车到达成功。

105905/111840/130635历史袋存在但均无有效程序目标确认，无法作为新鲜正确目标近处到达正例，exercised=false，不把历史地面真值最近距离当新到达成功。旧断路诊断还观察到图边与候选TTL基准可能不同（31边按图时间未过期而当前时间已过期），反事实未完成且未纳入本次运行改动，仍待确认影响，不能称已修。

服务器/home/ly/execution_continuity_review_20261022/v73_document_plan，实际新GraphNav库SHA5a50fde43e92642fdc245d73db80b13be0a034159f52c32c9b130b5ce4fb4109；四个Python源/安装字节一致，478成员冻结。清场空后启动目标后120秒trial20261030_184621，无默认录像，结果待补。证据C:\Users\Yu\Desktop\Beamng\test_artifacts\document_plan_20261030\REPORT.md、changes/、candidate_scope.json、final_effective_regression.json、v67_local_ahead_replay.json、v67_smac_forward_replay.json、native_recorded_replays.log。用户主仓库未整仓覆盖，完整验收前无commit。

### 2026-10-30 用户方案v73/v74现场关闭：三步已实现，整轮仍未通过

v73 trial184621启动60.296/search120.078/收尾23.251秒超时，console0/session errors=[]/清场空。近停19.684%、最长6.481秒、累计94.480/净87.460m；首raw59.588秒，末车至末raw水平35.781m，未到达。目标前近停35.776%（21.287s）、目标后3.817%（2.303s），90.24%的近停在目标出现前。此轮向南接近但不能称稳定。

按方案检查发现遗漏单次empty_path的入口：即便图仍向前，也应尝试有界前向/左右绕行。建立v74_document_handoff，保留v73证据不覆盖；仅补已有candidate_failed，用既有failure_reasons限制每批一次；修正新局部前瞻拒绝不应写入仅按全局地图签名的失败缓存，保留原Smac empty_path缓存；理想载体不采用车体局部探路。相对v73改2运行文件+1测试，无新状态机/节点。最终914相关独立ROS、28契约通过，沿用完全相同GraphNav库5a50fde4及30原生通过；实际首轮empty_path时刻两遍原生Smac各3个前向/侧移候选均规划并通过局部检查。最终源/安装字节和478成员冻结一致。v74代码v74/changes/，scope记录全部相对v71和增量差异。

v74 trial20261030_190118启动47.167/search120.089/收尾22.227秒超时，console0/session errors=[]/清场空。近停32.934%、最长7.692秒、累计75.525/净66.857m；3处odom间隔>.5秒，最大1.226秒，整体比v73差，不推广为稳定版、不commit。首raw87.222秒，目标前近停42.411%（36.938s），目标后7.693%（2.516s），93.624%的近停在首目标前。16.504/46.607/66.695/78.746s retired_controller_progress_unavailable、58.854s follow_aborted均早于三维目标；此时新增目标接近/到达/显式软净空逻辑未激活，但局部前瞻探索期也运行，不排除其影响，不把总体差值全归因补齐入口。

前向失败入口已现场采用：103.265等请求形成约8米前向候选，103.905/105.135/106.754/108.603秒计划被接受。114.959秒仍选西南(720.625,-83.875)，夹角不到90度且略减目标距离，未触发方案“超过90度”规则；不能称已解决所有侧向低效出口。末车(733.515,-76.711,52.250)，末raw(748.455,-129.816,57.459)，水平55.167m；3米导航和完整到达均未通过，新的平面到达终态仍只有组件正例，没有本轮现场正例。

用户方案的“探索执行已稳定”前提不被现场支持。下一步明确待查探索期控制器进度交接缺失，以及不到90度仍低效的图出口，状态码不是已查清根因；本轮不擅自增加其他策略。当前服务器没有测试进程，保持v71/v73/v74来源和失败证据，用户主仓库未整仓覆盖。未默认录像，少量末帧和日志已保存。证据C:\Users\Yu\Desktop\Beamng\test_artifacts\document_plan_20261030\REPORT.md，field_184621/、v74/field_190118/、v74/search_phase_audit.json、v74/empty_path_forward_replay.json。完整验收失败，禁止commit或宣称修复完成。
