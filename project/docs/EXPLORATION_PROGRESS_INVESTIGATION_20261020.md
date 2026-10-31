# 全局探索进展与完整链路调查（2026-10-20）

## 目标与范围

接续F12退役交接，调查F11“局部成功不保证整体探索进展”。先从实际完整失败轮次核对数据流，不按单个局部成功反例直接叠加距离/时间看门狗。整体任务仍是修复并验证WildOS、同进程记忆、真实车辆规划执行的完整连通性；7类开放问题及20轮验收要求不变。

## 实际证据对初步怀疑的更正

只读审查已关闭的`field_native_20261018_164222/.../goal_001/interfaces/interfaces_0.db3`，脚本`test_artifacts/exploration_progress_repair_20261020/audit_closed_field.py`：

- 681条`graphnav_planner/route_status`全部`complete=true`，没有`route_done`记录；不支持“该轮partial前缀耗尽后route_done反复等待”的初步怀疑。
- 17条`maneuver_done`及4条`segment_done`，18次执行反馈`maneuver_completed`（与plan_status的计数不必一一对应）；19s、36s、43s附近多次短换向后，车辆仍继续向南推进。不能把合法多段换向直接判成兜圈。
- 398条`graph_reset_failed`及末段静止与先前已定位的重置预算问题混合；此处仅核原轮事实，不重复把已改故障列成完全未修。
- 现有F11离线反例确实说明恢复次数不涵盖“局部成功无整体收益”，但它手工重建动作队列，不单独证明现场原地循环都由同一机制导致。

源码链：查询/实例目标归属→C++保持探索目的地→图路线complete/partial→Smac全路径/方向段→Bridge真实动作终态与独立到达检查→执行反馈/观察ACK→Planner退役或新图更新。设计进展协议前须区分合法绕行/换挡、可执行前缀耗尽、旧图/旧目标循环、目标重新观察，以及组件失效；欧氏距离单调下降不足以覆盖车辆规划。

## 当前完整冻结试验

- 根目录`/home/ly/exploration_progress_repair_20261020/full`，416源来自上一轮F12候选逐SHA复制，未修改生产逻辑。保留旧原包及已通过的直线短段候选；不commit。
- 原Town加油站坐标、历史授权左转90°对照朝向、真实SBR车身和四轮物理。模型/同进程三个存储启用生产默认，不事先给加油站记忆指令；一个绿色居民楼目标，420s预算，无视频。
- 四库预期：Mapper5d0ed0d2、Planner782102b1、Smaca0b4453f、RPPd2fd9cee；模型实际源和内存数据库、零初始确认视点须通过检查后才发目标。实际完整到达需再审查视觉和车辆证据，不能以driver退出或Harness状态代替。
- 只增显示/源身份/低频图决策观测、原已验证Native时序观测；不改模型返回值、物理、速度、控制或规划门禁。启动预检保持，另核实际/proc四库来源。核验期间不读活动SQLite。
- 初次trial`trial_20261020_174256`已结束，goals=0、控制未解锁：模型原构造完成后，新增身份观测将`sqlite3.Row`直接JSON序列化失败。仅诊断脚本错误；保留vision.log/session_result。实际driver1922368已缺失、结果closed=true、服务器项目进程[]后才安排重试；setsid父1922367的退出本身没有被当作试验结束。
- 已改诊断元数据为JSON安全列表，并用实际SQLite Row/tuple工厂、内存库与持久库判定3项用例验证，不修改生产模型或导航来迁就脚本。后继复用相同416源/配置，先确认进程结束及重做预检，再启动新trial；继续工作以真实/proc和结果为准，不依据PID文件推断活跃。

当前未宣布F11修复或整车成功。后继沿本轮最早实际违约点定位，避免通过新增重试/超时分支掩盖其他模块接口问题。

## 后继现场结果：图起点与物理连通性（调查中）

- `trial_20261020_174700`为同416源、四库及模型身份通过的420s完整搜索；实际行驶后，56.64s附近`graph_path_unavailable`退役、58.47s恢复结束，此后没有新动作，最终421.07s超时。真实停车成功、driver.closed=true、session errors=[]，独立后检无项目进程。没有视觉到达、没有完整验收成功。约30个frontier/300多可通行heading不证明连续路线存在；精确图配对警告为0。失败图已取回本机`test_artifacts/exploration_progress_repair_20261020/failure_last_rgb.png`，不是到达证据。
- 上轮标准bag缺少raw/scored完整图，不能从heading数量猜测边拒绝原因。未改生产代码，新增只读`graph_pair_observer.py`，独立于动作状态保存CDR原图/评分/odom及精确匹配标志。`trial_20261020_180816`为150s有界连通性诊断，同源/物理/目标，无录像，不是420s完整验收；150.79s超时、closed=true/errors=[]、后检无项目进程。
- 该轮166条观测、131组评分记录中101组满足同frame/stamp/classes/current_idx/UUID/pose、评分边为空的匹配约定，观测错误0；其余不强行合并。`analyze_graph_pairs.py`逐SHA读取已保存CDR，按当前Planner物理边准入重建连通性。该镜像尚未包含历史拒绝边、目的地backoff、虚拟frontier代价或实际接收时序，不能替代实际C++重放。
- 关键反例：source_ns=1792491005743982662，当前节点`(730.237422,-12.677480)`距odom`(731.947246,-12.038762)`约1.825m；原拓扑可达124节点，物理过滤后仅自身。当前节点minimum_clearance=1.0m，小于真实profile要求1.06m，confidence=.8/physical_violation_observed=1/feasible=false；关联12条有向边均被拒绝。此帧状态正是`graph_path_unavailable`。后续近百秒同UUID仍为起点且持续隔离，末帧原拓扑可达139、物理可达1，过滤backoff前仍有19个候选。
- 源码`Mapper::odometry_update`只按4m内几何最近切换当前节点，尚无物理可行或实际位置→节点连接证明，并将该连接标为traversed；不能用visited绕过新鲜危险修复，因为会破坏已验证安全契约。现在优先核实起点代理是否错误以及1m净空对应的真实terrain/地图位置，再用实际C++验证反例。尚未认定RGB模型、地图误标或车身物理是唯一根因，未放宽净空/碰撞门禁、未commit。

### 已完成的实际C++及地图对应复核

- 当前库782102b1实际Planner重放同合并CDR：原起点30个frontier全部空；3个其他起点均30非空/21 complete，另1个起点仍全空。这是起点敏感性的反例，不授权实际车辆到这些点的物理连接。
- 封闭terrain与严格相等采集戳TF：原起点最近occupied格中心.961m、实际odom到同格2.758m；末帧.963m/2.761m。记录elevation_valid=0，不把地面填充值当实测；也不单独证明整车扫掠安全。
- 根因证据已封存到本机`test_artifacts/exploration_progress_repair_20261020/sealed/exploration_evidence.zip`，16449674B/553载荷，SHA9795dd1f59dc5bd70d8b4660922e689c054582aba79b506a6f2728d6846b36df；CRC/逐载荷SHA/416改前源与本机备份或未变源核验通过。两轮原DB与Native大trace留服务器并保存流式SHA，不读活动DB。`local_evidence_verification.json`明确完整导航未通过。
- 后继最小接入修复及验收见`GRAPH_START_ATTACHMENT_REPAIR_20261020.md`；原轮证据不覆盖或改称成功。
