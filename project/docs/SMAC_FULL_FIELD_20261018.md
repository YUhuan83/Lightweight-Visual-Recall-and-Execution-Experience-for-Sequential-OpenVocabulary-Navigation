# 新 Smac 单轮完整搜寻调查（2026-10-18）：未通过

## 版本与条件

用户先要求保留 Smac 修复、独立调查 Native，随后明确要求部署新 Smac 并先测一轮。使用保留的 `6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b`（404成员）；相对d9d仅planner.cpp及test_planner.cpp两文件不同。Native生产代码及可靠深度1接收保持原样，没有合入BEST_EFFORT诊断或其它Native候选。

- 服务器 `/home/ly/smoothing_boundary_repair_20261018`，driver `field_native_20261018_164222`，session `session_20261018_164222`。
- 同Town加油站位置、左转90°、原真实SBR车体与全部原碰撞、green residential house、420秒预算，一次地图加载，不录像。
- 三新增记忆先验按共同链路基线关闭；仍有WildOS视觉评分和检测。这不是视觉记忆增量或多目标验收。
- `/proc/maps`实际核对 Smac `fc149c802333ebbea9504e175ae92050821a86a2debb31aaed276fbf1d711a33` 来自新独立install；GraphNav db52f00e、RPP d2fd9cee保持原版本，Python/配置源门禁通过。日常e6没有替换。
- Native保留DDS/执行器与进程UDP诊断插桩；插桩已做实际Humble接口、UDP载荷及内核戳回归。其观测开销是限制，不能把本轮当成无插桩稳定性验收。

## 结果

**420.8397秒超时，未通过目标到达验收，不计20轮成功、不commit。**

统一使用goal.started与独立observer的Python monotonic窗口；不用控制台输入或不同稳态时钟原点推算。

| 项目 | 结果 |
| --- | --- |
| Native样本/覆盖 | 8417 / 420.799秒 |
| 累计行驶/最大离站 | 105.154米 / 69.103米 |
| 速度<0.05m/s | 278.845秒，约66.3%观察窗口 |
| 精确源归因的静止 | stale_command201.996秒、换挡53.619秒、tracking12.081秒、execution_hold7.894秒等；这些分类守恒 |
| 下发控制的静止 | 制动266.764秒、低扭矩/滑行7.515秒、正油门不制动4.565秒 |
| ComputePath发送/当前结果 | 103发送；60成功、37失败、6旧结果忽略。日志缺共同完整serial，不能强行逐个配对或当总体成功率 |
| Smac raw回退诊断 | 233条；包含所有平滑拒绝理由，不等于233次仅航向问题或成功路线 |
| 控制器变换后路径 | 1968份，33 planned/189 following状态，17 maneuver_done、3换向后重规划 |
| GraphNav公开路线 | 681份，全部complete，最大发布间隔2.091秒；不等于车辆执行或目标到达 |
| 目标观测 | 41份目标query掩码、50份方向提示；无对应3D定位/最终确认。掩码不等于物体身份正确或到达 |
| Native接收源龄 | P95约6.992ms/max34.840ms，watchdog及缺receipt均0 |
| 结束停车 | 连续约0.55秒低速，末速0.0001146m/s；session errors=[]，独立ROS域进程清单为空 |

静止的公共状态近似关联：planning67.100秒、following65.826秒、**graph_reset_failed132.249秒**。这是三种不同分类轴，不能再与stale_command201.996秒相加，也不代表私有CPU/锁耗时。

末位约(740.477,-80.775)，723节点/467车身碰撞/四轮各32碰撞、1449.72kg均保持。SIGINT退出的KeyboardInterrupt/exit -2记录在日志中，不当作中途运行崩溃。

## 本轮确定的末段阻塞机制

首次graph_reset_failed位于Harness elapsed292.029秒；该elapsed从Harness构造开始，不能直接当目标已运行292秒。统一目标窗口中，这个失败状态与约132秒静止关联，持续到结束。

现场状态 reason=`response_budget_exhausted`、unresolved_requests=0，后续GraphNav继续发布新路线，但Bridge以query_reset_pending拒收。不能把该文字直接解释为“服务连续三次超时”。

源码及离线反例确认：

1. `NavigationTransactions.graph_reset_attempts`每次派发就递增，成功完成后仍保留累计值。
2. `VisibleApproach.release_visible_approach()`直接设置query_reset_pending/inflight，未用`request_graph_reset()`开启新重置事务。
3. 同query/fence内多次临时靠近结束后，之前成功的重置也累计消耗三次预算。
4. 初始清理成功 + 两次视觉释放清理成功之后，第三次释放即拒绝派发，pending保持，waiting为空，随后持续拒绝新路线。

本机 `test_artifacts/native_delay_20261018/reproduce_reset_budget.py`调用现有生产回调、模拟三次全部成功的服务响应，稳定复现上述第四次被拒绝；输出`reset_budget_counterexample.json`明确为**BUG PRESENT，非修复通过**。现场缺少逐服务响应抓包，源码反例和“空waiting但预算耗尽”的现场状态相符；不扩大为所有历史停车的唯一原因。

相关Python在d9d与6153冻结中逐字节相同；本次Smac C++修复没有改动该交接代码。此为既有交接机制在本轮暴露的新反例，仍待修复。本轮按用户“先测一轮”要求未继续修改生产。

下一步修复应通过统一入口创建新的重置事务，同时保留旧请求响应隔离、旧clear排空和目标/fence保护；不能仅删除三次上限或无条件清空所有状态。还须覆盖多次成功释放、真实失败重试、旧响应迟到/重复及新目标切换。

## 其它未解决项与本轮边界

- 仍有37个当前规划失败结果和其它日志拒绝，不代表平滑边界修复已解决所有规划条件。需用实际请求/地图/候选阶段继续区分障碍、预算及接入几何。
- 发生visible_target_route_timeout、一次follow_aborted及一次actuated_no_path_progress（5秒进展0.0272米）等；受困具体物体/私有地图真值尚未闭合，不全归消息延迟。
- Native本轮接收正常，因此不是本轮末段132秒保持的原因；历史约300～450ms间歇迟滞依然未修复。9054个全会话指令完成内核→UDP→DDS→回调配对、没有>200ms，但未在同一故障时刻捕获完整内核链，不能据此宣称根因解决。
- 正油门统计中1690份有效源ACK匹配且在原controller截止前；另1份旧generation=7的在途RPC，在generation=8已被接收后完成，因此receipt正确保持applied=false。可靠录包证明载荷/源身份吻合、原截止未超；**不是旁路丢样，也不能当作有效的新代际确认**。详positive_source_recovery.json，保留最初未确认分类与更正，不覆盖源数据。
- 一轮结果及轨迹不能证明成功率或因果改善，不把行驶更多/看到更多掩码当作Smac或记忆完整成功。

## 证据

本机 `test_artifacts/smac_full_field_20261018/closed_evidence/`；核心归档`smac_full_field_evidence_20261018.zip`，27823627字节/110载荷，SHA256 `037f2f6dc7bdee30bb25ebe5858246606bea8fbc8bfda6cd40434b6d535863fb`，CRC/逐载荷SHA及7类证据一致性校验通过。**证据自洽不代表导航成功。**

关键文件：session/full_query_review.json、udp_timing_review.json、running_identity.json、positive_source_recovery.json、goal_001/result.json和stop.json；少量失败RGB在goal_001/failure_last_rgb.png，不是到达证据。无MP4。

261197824字节原始接口DB仅服务器保存：`.../goal_001/interfaces/interfaces_0.db3`，SHA256 `dac5d8cfeef67a2a71b72edb324b4f5f80f994749d386a01b5a7a74f33c4ad45`，归档清单含完整路径。所有SQLite分析在driver/session/recorder关闭后执行。

测试已自行结束后，一次新SSH读取被自动审批服务额度失败拦截；先经原已授权会话确认进程结束，用户明确手动允许只读获取结果后通过原SSH通道继续。不是网络故障，没有绕过审批。没有新开测试或在本轮运行中改源码。
