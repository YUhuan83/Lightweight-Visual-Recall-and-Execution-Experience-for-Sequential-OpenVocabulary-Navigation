# 连续执行现场续查（2026-10-22）

本轮目标仍是完整目标搜寻和连续执行，以下组件通过不表示整体通过。保留用户工作区全部未提交改动，无Git commit；没有录视频。

## v5：异步结果串行候选

- `NavigationTransactions`的Future结果转入默认组GuardCondition，Native/control状态保留独立组与快照锁；修正已接纳路线段间误触发10秒无路由超时及恢复tick被整体截止吞掉的问题。
- 离线703项、真实ROS427项通过，424冻结成员SHA核验。首次构建脚本缺少venv PATH而未找到colcon，保留环境失败日志后修正；不是生产代码编译错误。
- `trial_20261022_212511`采用当时“启动计入180秒”解释。超时关闭、console_exit=0、field_acceptance=false，预检/收尾进程为空。Native有效覆盖121.321秒，近停49.923秒=41.15%，最长16.294秒，净位移61.476米/采样路程69.324米。
- 同源时间对齐control_status：近停中stale_command25.483秒（planning12.702秒、reobserving9.476秒）、gear_change12.024秒、tracking6.053秒、execution_hold3.436秒、ackermann_stop2.119秒。不是单纯录像不流畅，也不能将stale_command直接判为网络丢包。
- 本轮末尾回看路径被接纳为16个方向段，暴露原`max_motion_sections`只限制探索、豁免可见目标/记忆/续段的不一致。16段只在轮末开始，不能称已观察到完整16次换挡。
- 本机证据`test_artifacts/review_user_changes_20261022/v5_trial_20261022_212511/`，服务器`/home/ly/execution_continuity_review_20261022/v5/full/trial_20261022_212511`。

## 关闭地图Smac回放

- 180945的request59读取真实记录的spot1/odom，后轴约(737.215,-67.701,-1.359)，观察点(737.825,-70.058,-1.316)。公开全局OccupancyGrid反量化，使用实际插件/车身；并非当时私有搜索地图的精确快照。
- 起点/终点footprint均安全，直接Reeds-Shepp连接长约2.435m、无cusp且曲率合规，但5个采样姿态碰撞。降低min_section的诊断没有改变路径，因此此反例不支持以放宽最短方向段解决。
- 6个原生后缀5个有碰撞，1个通过；组合后回放插件选择代价约43.498、7个cusp的替代路径。现场原请求返回无安全路径，公开地图回放未精确复现该拒绝。可以证明端点清晰不足以保证扫掠可行；不能据此宣称内部地图或Smac必然错误。
- 证据`test_artifacts/review_user_changes_20261022/observation_replay_fixed.log`，服务器180945/observation_replay保存公开地图及后缀转储。

## v6：目标推进与统一机动准入

- 原方向段上限覆盖探索、可见目标、记忆比较及后台续段；有界测量恢复保留其原约束。过于复杂的未来路径拒绝后继续既有有效动作，按原候选/观察失败链路选替代点；未改变碰撞、曲率、到达或速度门。
- 可见目标前进段将结束时，若新鲜同query提案在原2--10米包络内且比旧目标多4米进展，则提前发布GraphNav意图；不reset、不取消当前FollowPath。等待匹配goal和route状态后，使用原后台规划/替代动作接纳。沿用visible_target_approach接纳事件，使GoalStandoff的失败归属更新到实际接受的新点，迟到旧点失败不生效。陈旧、后退、很小抖动、确认/记忆目标、回看及恢复不走此更新。
- 711项离线、435项真实ROS通过；初稿3处测试夹具错误（复用odom位置、route使用真实时钟而fixture为100秒、缺少require_route_status对应metadata）已修正，保留失败日志。
- `trial_20261022_215520`仍为启动计入180秒，field_acceptance=false/console_exit=0，关闭后无残留。Native有效105.692秒，近停41.997秒=39.74%，最长6.301秒，净位移60.001米/采样路程63.653米。不能把不同长度/不同路线的单轮比较当统计稳定性证明。
- 82.17、85.68、89.52秒现场触发三次continuous目标推进，并接纳后续路径；97.52秒到达最后一步后再启动下一观察点。最后一次后台续段因地图等待/规划赶不上而在92.43秒完成旧段后92.83秒接续，仍有短空窗，未完全连续。
- 近停控制归因：stale_command13.940秒（planning9.949秒）、gear_change12.186秒、execution_hold6.616秒、tracking4.310秒、ackermann_stop2.401秒。仍有换向、规划空窗和保持/恢复，不能宣布走停已解决。
- 新现场反例：49.419秒source_owner_unconfirmed与68.031秒source_not_applied。逐消息闭包查明均在解除execution_hold后发生：控制gen6→7/10→11，Native新generation回执晚约几十毫秒；桥接立即当故障撤销动作所有权。第二次旧gen10回执已applied=false，而新gen11回执约事件后18ms才到，不能简单允许旧generation继续驱动。
- 本机证据`test_artifacts/review_user_changes_20261022/v6_trial_20261022_215520/`（含source_event_audit.json、control_stop_attribution.json）；服务器对应v6/full/trial_20261022_215520。当前停着是该轮180秒正常结束，现场进程为空，不据此判死锁。

## v7：解除保持确认阶段（已关闭，整体未通过）

- `NavigationTransactions.monitor_follow_transactions`在旧动作终态且测量停车后，发出held=false，进入follow_release阶段。要求控制状态晚于解除时间且不再execution_hold，并通过既有execution_transport_issue及Native applied/lease/owner门，才恢复规划/逃逸。正常generation交接不再走硬故障重置；超时仍用原follow_response_timeout_sec并报告control_release_ack_timeout，不授权动作。
- 规划重试、monitor_inputs及段派发都尊重此阶段；query/事务退休撤销等待，视觉超时不能吞掉释放阶段的monitor tick。未放宽Native ACK、generation、应用回执、过期或新鲜度检查。
- 715项完整离线回调通过；真实ROS452项通过，新增包含13项SourceControlCallbackTests。3处旧回归原来假设解除hold立即start，现显式交付解除后的控制状态/ACK；缺少源的测试加强为不调用新规划而等确认。失败日志与最终日志均保留于review_user_changes_20261022。
- v7已隔离部署，424冻结成员/运行库SHA照原约定检查。现场试验启动前进程为空、X148无锁。现场还需检查释放确认事件、近停积分、最长停顿及完整到达，当前不得称通过。

- 关闭补充：223039搜索180.18秒，未到达；启动55.20秒、关闭19.62秒。按query_sent严格对齐的前70秒路程60.26米、净58.42米、近停15.61秒=22.29%、最长7.30秒；后110秒路程41.26米、阶段净9.60米、近停60.53秒=55.02%、最长26.25秒。全部采样略含搜索前后，近停77.34秒=42.68%。
- 两次解除保持均恢复，共近停约0.44秒，本轮未记录execution_transport_unavailable。仍有5次no_vehicle_path及1次FollowPath因有界控制弧碰撞而中止，整体未通过。

## 2026-10-23：有限失败预算遗漏近处反向路点（v8候选）

- 闭包77.43秒结束一次bearing-only观察步后，10秒内无新鲜目标提示，按既有规则87.43秒恢复探索。88.35--101.83秒三个不同frontier共享同一条折返前缀；各自先试远处相邻点和航向，连续4次空路径就退休。改变frontier没有改变局部首段，近处路点反复得不到尝试。10秒重新观察仍是本轮独立停顿来源，本次未修改其计时语义。
- 近处路点位于后轴后方，但原候选只提供图切线/观察点方位作为车头航向，相当于要求汽车转身后到达。实际Reeds-Shepp支持倒车，候选却没有表达保持车头朝向、倒车到同一中间点的几何选择。
- 关闭录包公开全局图、同一后轴起姿及实际Smac插件受控回放28项：原前4项均无路径；原近处航向产生约55--68米等效机动成本，反向终点航向的最近路点产生5.58米、2方向段、等效成本8.58米的路径，全体采样无碰撞且曲率合规。公开图有反量化误差，不能冒充当时私有地图的精确重放。后续生产callback重放验证该路径通过整条路径、方向段上限及初始跟踪检查。
- 最小实现：普通Ackermann探索的中间路点在车后时，增加兼容当前车头朝向的反向航向，原方向保留为后备；正反方向分别经过失败缓存和车身姿态预筛。候选按远/近端点交替，端点内原两航向成本比较保留。明确的观察、确认、可见目标与记忆目标航向及ideal carrier不适用该自由航向规则；碰撞、曲率、动作所有权和4次连续空路径限额保持原契约。
- 改前新反例3项中2fail/0error；改后720项离线回调、27项纯几何测试、457项真实ROS通过。首次离线回归有1处新夹具错误：24米直线的十采样上限实际保留24..6米，测试误认最低4米，修正断言为真实保留范围；日志保留。
- 独立v8从已冻结v7复制，只更新graph_path_bridge.py、graph_route.py及相关回归测试3文件，424成员核验；测试前无相关残留/X148锁。目标下发后180秒真实车体测试已启动，结果待关闭复核，当前不作完整成功或稳定改善结论，未commit。
- 证据：test_artifacts/review_user_changes_20261022/{candidate_span_counterexamples.log,candidate_span.log,candidate_span_callback_replay.json,candidate_span_callbacks_final.log,candidate_span_delta_manifest.json}，v7_trial_20261022_223039/route_recovery_audit保留各请求的录包路径/地图/后轴/提示输入。

## v8两轮失败与v9收窄（2026-10-23）

- 用户要求每个问题至少跑两遍确定根因。冻结v8两轮目标搜索均180.18秒、均未到达：085412近停78.21s/43.38%、最长8.74s、净32.23m、路程91.63m；085909近停74.84s/41.84%、最长10.72s、净60.48m、路程94.84m。两轮收尾无残留。无no_vehicle_path不表示连续执行成功，following状态下仍近停33.58s和30.31s。两轮路线随机性不同，不作为严格同轨迹性能A/B；第二轮中曾运行短时组件回放，性能统计不能视为专用资源基准。
- 同输入callback旧v7/新v8对照各跑两遍：085412第8.014秒原第三项10m，新版第三项提前变4m，现场也执行该短段。证明全局重排扰动正常规划且有切短推进的风险，尚不证明旧10m必定可行。第18.087秒前三项新旧相同，现场四方向段换向不能归因于这处重排。证据v7_v8_recorded_candidate_ab.json。
- 已撤回v8的正常候选全局重排与普遍航向转换；保留冻结v8源码在candidate_span_v8。v9正常start_segment候选与v7一致，两遍同输入复核前三项相同。仅在普通Ackermann探索连续空路径接近原4次失败上限（或原候选已耗尽、仍有预算）时，补一个最短未尝试路点；车后路点可用兼容车头的反向航向，仍经过原cache/footprint/完整规划与执行门。不会增加4次连续空路径额度；显式观察目标的航向、目标/记忆、正在行驶和后台续段不走此恢复。
- v7车后近点反例已在实际Smac公开图回放重复两遍：原前四项两遍均拒，最近兼容航向两遍均47poses、5.58m/2方向段、cost8.58259；全体采样碰撞0、曲率合规。保存candidate_span_repeat_1/2.log。v9 callback两遍在第4次尝试引入同一近点，整路/段数/初始跟踪检查通过。
- 独立交接故障：旧finish_current_exploration_section把pending/extending与执行动作不确定混在一起，只要后台ComputePath未完，frontier退休就拒绝保留已接纳动作并走reset停车。085412在11.98s following、12.16s background planning、13.56s frontier_resolved后退休。accepted/unaccepted两种规划响应边界的组件反例重复两遍，均2fail/0error；原障碍停车控制例通过。
- v9只在原动作身份、新鲜度、当前剩余扫掠等既有检查全部通过后取消未来ComputePath，清其serial并保留已接纳动作进入原有界交接；替代FollowPath的ACK空窗、障碍与陈旧输入仍取消/隔离。迟到ComputePath回调被serial拒绝。旧测试中extending_path被视作“不确定动作”的预期已按此契约更新；其后测试污染随之消失。初次编辑误命中maybe_extend_path守卫，被离线回归拦截并纠正，未部署；保留失败日志。
- 728项离线回调连续两次通过，27几何此前通过；真实ROS465项通过。独立v9相对v8更新5文件（含graph_route回退）、424成员冻结，现开始两轮180s目标搜索，整车效果待验；未commit。

## v9关闭结论：连续执行仍未通过

- 两次真正搜索091350与092847均按180秒驱动预算结束、无目标到达、console_exit=0，最后独立预检无仿真/ROS/显示残留。全部Native文件口径：第一轮近停80.87s/44.62%、最长6.75s、净60.10m、路程86.72m；第二轮85.40s/47.62%、最长13.36s、净63.35m、路程79.74m。
- 进一步统一query_sent至+180秒、仅有效Native区间的严格口径：v8两轮42.97%（覆盖179.01s）/41.67%（178.32s）；v9两轮44.24%（180.00s）/47.37%（178.50s）。少量缺样或文件边界不填造静止/运动。两组轨迹并非相同随机样本，不能据此确定全局性能退化的因果，但足以确认v9没有达到稳定连续运行目标。证据query_aligned_fluency_comparison.json。
- 两轮控制归因分别：gear_change28.56/33.04s，stale_command23.23/30.51s（其中planning15.50/16.90s），execution_hold12.33/7.73s，tracking11.02/7.08s，ackermann_stop4.41/5.19s。状态following不代表车辆持续运动，主要停车仍分布在换向、规划空窗、退休保持和重新观察。
- 第二轮11.21s实际触发short_route_probe（3次空路径后），11.58s已规划、11.59s following，之后仍有maneuver_done；证明链路现场触发，不证明整车效率收益。新pending-extension退休分支未在现有日志中得到明确的现场覆盖确认（取消该类已接受ComputePath的日志两轮均0，未接受请求无法据此计数），其效果仍以组件通过为界，不能把旧exploration_finishing_segment当作该修复独有收益。
- 三次额外尝试091127、091817、092324均在目标下发前退出，0目标，不计为搜索复测：两次分别为cmd_vel enable/bridge follow_path的SetParameters响应超时，一次为初始节点枚举缺少/beamng_wildos_adapter。后一份报告后续已读到该节点camera_offset，camera/odom/state/source均通过，存在初始枚举时点不一致迹象，未改预检或超时。两次独立domain88只读GetParameters窗口共14/66次已就绪调用全部有响应，最大约0.002/0.988秒，未复现先前SetParameters故障；不能宣布DDS/启动问题修复。此前被动发现脚本未覆盖活跃仿真且环境未对齐，其全0结果作废，不作故障证据。
- 继续做同图、同后轴、同目标三航向的实际Smac/纯前进Dubins/直接Reeds-Shepp对照，重复两遍。前进曲线约12.95--12.97m且曲率合规，但同公开地图有19--21个采样车身碰撞；min_section从0.4诊断到0.001不改变该曲线。此例不支持降低方向段阈值或把所有短换向直接归因于缺少纯前进候选。Native替代项一遍成功、一遍无路，提示有界搜索/候选搜索方差仍需查；公开图反量化不能替代精确私有图。尚未判清这些阻挡格是正确物理障碍还是点云/地图更新误标，未改碰撞、转弯半径或换挡等待。
- 当前代码仍为未经整体验收的v9候选，本机SHA与冻结增量一致；旧v7/v8及用户未提交工作保留，没有Git commit。证据v9_trial_20261023_091350、v9_trial_20261023_092847、v9_startup_failures、parameter_service_observation.json、early_shunts_probe_1/2.log、bounded_handoff_final.patch。

## 最新轮次计时约定

- 用户最新明确：目标下发后搜索最多180秒，地图/模型启动另计，停车收尾另计；覆盖本日早先包含启动的解释。历史v5/v6数字保留原口径。
- v7驱动另设300秒启动就绪上限，发出目标前切换到180秒搜索截止，独立报告startup_elapsed_sec/search_elapsed_sec/shutdown_elapsed_sec。配置goal_timeout_s=180。时限修改有before/after SHA和AST校验记录search_budget_receipt.json。

## 2026-10-23 v17：costmap解除起点阻塞后重启规划（未验收）

- **根因与数据流**：v16最长新Stamped Twist间隔24.683s。序列为`start_footprint_blocked` → 固定20s blocked backoff；global costmap很快清空车辆footprint，但该状态未进入timer retry允许名单，且route重发在退避内不会解除锁。控制器的`controller_safety_stop`/`stale_command`是对旧命令的正确拒绝，不能通过降低新鲜度门或沿用旧速度解决。
- **修改**：v17从冻结v16复制，只有`graph_path_bridge.py`新增`retry_cleared_start_footprint()`和`test_graph_path_bridge_routing.py`新增4个恢复/否决测试。`on_map`在costmap更新后调用：只允许当前epoch/query/goal和route endpoint一致，FollowPath启用、fresh odom/global/local maps、当前全局/局部真实Ackermann footprint都清空、transport和action server ready、无reset/active action/quarantine/recovery/frontier selection所有权时清锁并调用标准`start_segment()`。新阻塞、旧意图、stale local map、控制/规划不可用仍不尝试。冻结delta哈希见`candidate_delta_v17.json`。
- **回归**：本机离线回调754/754通过；服务器isolated build两包通过，真实ROS 513/513通过。用v16关闭bag的原始Ackermann odom、global/local costmap回放，事件后0.412s地图均无vehicle footprint blockers，调用v17 `on_map`后`start_segment`一次且清除blocked_until。证据`v16_blocked_start_replay.json`。旧审计估计的约1.4s首次清空由精确闭包回放更正为0.412s。
- **现场复测**：每轮先过prestart，收尾进程为空且`:148`锁释放。trial `185826`启动36.41s/搜索180.154s/关闭22.825s，timeout、无目标；near-stop27.42%、最长6.985s、净46.116m、352/3686 stale control samples，最大新命令间隔5.570s。trial `190719`启动37.551s/搜索180.169s/关闭38.243s，timeout、无目标；near-stop46.195%、最长14.298s、净59.040m、607/3683 stale samples，最大命令间隔7.467s。两轮均没有现场`start_footprint_blocked`，故现场未执行这条恢复；不同轨迹指标不能证明改动收益或稳定改善。完整目标搜寻仍失败，无视频、无commit。
- **剩余新鲜命令问题**：trial 185826最大间隔发生于`follow_aborted` → 可见目标`no_vehicle_path` → GraphNav重新路由和新路径；trial 190719最大间隔发生于`exploration_segment_finished`后costmap等待、串行ComputePath候选评估（含`empty_path`）和frontier选择ACK。两者分别5.57s/7.467s，不是20s起点阻塞回退。下一步调查预先续段或减少串行候选耗时，必须保留WildOS frontier选择/vehicle-cost规则、真实车体footprint、stale输入和action lease门；不要周期发零命令来把stale状态隐藏成“命令新鲜”。
- **证据**：本机`test_artifacts/fresh_command_retry_20261023/`保存两轮`result.json`、`fluency_summary.json`、stale-command/gap audits、v17 delta、真实ROS报告及v16闭包回放。服务器候选和完整bags位于`/home/ly/execution_continuity_review_20261022/v17_costmap_unblock_retry`；完整bag未复制回本机。
