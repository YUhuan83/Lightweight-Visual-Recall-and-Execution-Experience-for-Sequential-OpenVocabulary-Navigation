# 用户改动审查与执行连续性交接修正（2026-10-22）

## 审查基线

保留用户在 affb478 之后的未提交工作。工作前读取 PROJECT_MEMORY、AGENTS、
2026-10-21B 上下文与 v11 走停证据。2026-10-22 的主要新增行为为退休尾段再准入、
首段退离软代价、空路径缓存及 streak 作用域。本次改前回调全集 688 项通过；
这补上前一节记忆中三项新增回归尚未运行的缺口。

## 两个已复现的缺口及修改

1. **新 owner 已收到，倒车准备段终止后仍重新规划。**
   `on_route` 在旧动作运行时只更新 `latest_route`；倒车段不会进入前进中的替换规划，
   因而 `route` 仍属于旧 owner。原 `finish_retired_exploration_section` 仅检查
   `route` 是否已切到新 owner，使新尾段复用辅助函数在这个实际回调顺序下无法进入。
   现在在旧动作通过原身份、终态和实际到达检查后，使用原
   `replacement_route_for_retirement` 校验已收到的新路线，接纳新 owner，再对旧尾段
   做当前路线走廊、地图、车身、跟踪及原执行入口的重新准入。
   已有比较/选择 ACK 或规划仍在进行时继续原流程；没有新 owner、旧消息、
   不再向新目标推进或新障碍均不能借旧尾段执行。没有复活旧 frontier 或删除取消保护。

2. **空路径缓存未包含规划起始朝向和地图内容。**
   同一 owner/2m 起始区/目标桶下，车辆已换朝向或地图变化后仍可能被先前失败排除。
   现在按原规划 heading bins 区分起始朝向，按全局地图的 frame、几何和内容签名
   区分地图条件。只更新时间戳而内容相同的发布保持缓存，避免每次发布重启搜索。
   请求保存自己的起姿和地图上下文；地图在请求期间改变时不把失败归给任一地图。
   每条地图消息只计算一次签名，失败缓存仍最多 128 条；未新增跨查询持久化或超时阈值。

## 验证及测试修正

- 新增 9 项回归改前为 5 失败、0 错误；原始源码字节及 SHA 已保留。
- 最终定向 11 项，包含真实回调顺序、前进/倒车终态、过期/缺席新 owner、障碍、
  地图内容/原点/时间戳变化、请求起始朝向保存；离线接口替身全集 **699 项通过**。
- 完整离线检查 **55 PASS、0 FAIL、6 WARN**；WARN 为既有真实 ROS/现场验收边界。
  首次在 Windows 沙箱内因临时目录/文件解析权限失败的日志保留；正常权限重跑通过。
- 更新原交接测试为“旧尾段不向新目标推进时应重新规划”，保留回退覆盖；新测试覆盖可复用尾段。
- 服务器三个包构建通过；Smac C++ **29 项通过**；真实 ROS 消息/节点回归
  **420 项通过**。这验证消息类型及回调，不能代替 DDS action 或整车运行。
- 真实 ROS 初跑的确认测试 9 失败均为时钟夹具返回 SimpleNamespace 而非 ROS Time；
  仅修正夹具 `to_msg` 类型，保留原断言，复跑全部通过。初跑证据保留。

## 部署与现场边界

- 服务器 `/home/ly/execution_continuity_review_20261022/v1`；从已校验 v11 独立复制，
  更新本机相对服务器的 16 个不同/新增文件，冻结 **423** 个源码成员。
- 用户先前 Smac 源码亦与 v11 不同，本轮重建 Smac 与两个 Python 包；
  安装 Python SHA 和新 Smac 库 SHA 均核对后交给完整驱动核验实际运行库。
- 预检项目残留进程为空，X148 无锁，GPU0空闲；GPU1其他作业不操作。
  SSH 一度超时后恢复；部分读/部署请求遇自动审批服务超时，按工具指引重试后执行。
- 完整测试：Town 原出生配置、真实车体、`green residential house`，预算 420 秒，
  无录像。轮次 `trial_20261022_155227` 已关闭，420.782秒timeout，无到达，errors=[]。
- 未创建 Git commit；完整现场验收尚未通过。

本机证据入口：`test_artifacts/review_user_changes_20261022/`。
包含 before_manifest/before 源码、反例、699 项日志、完整离线报告、
真实 ROS 初跑及最终报告、增量包/清单和部署脚本。旧部署包按内容 SHA 备份保留。

## 155227 长停：实际起点与图节点位置不一致

关闭 bag 的420秒窗口覆盖418.933秒；近停274.183秒（65.45%覆盖时间），
最长连续近停160.835秒。采样里程127.99米，净位移32.36米，最大离起点58.07米。
近停中换挡41.54秒、planning驻留41.23秒、execution_recovered驻留158.80秒。
这些不是CPU计时；净位移也不是车辆行驶速度。33个下发段、79个planning状态事件；
不能只看following状态认定连续行驶。前180秒与完整420秒统计分开保留。

约261秒FollowPath报告 bounded command arc collision；263.54秒GraphNav退休
`graph_path_unavailable`，264.28秒bridge完成停车并进入execution_recovered。
此后GraphNav反复无图路径、目的地进入backoff，没有新的可执行路线，直到超时。
Native样本source_rejection均None、telemetry_ok均true，最大源间隔0.208秒；
不支持把此段159秒无路等待归为通信断线。

本轮除bag外另存原始/评分图CDR及odom，使用**身份全部匹配**的快照复核：
259.76秒当前节点可达331节点、23个frontier；264.16秒起至409.86秒可达集始终只有自身。
旧current节点坐标(736.259,-46.763)，实际rear-axle位置约(735.906,-44.898)，
相距约1.9米。节点新观测净空0.70米、fresh confidence=.8、measured violation=true；
所有21条相邻边被原Planner正确按端点危险过滤。265秒原始地形在实际车位处净空约2.50米，
同一旧节点处0.70米。采用对应采集时刻的terrain TF计算，不用最新TF代替。

最早违约点在Mapper起点身份：已有保护不允许新吸附到危险采样点，
但**已被选作current的旧代理点变危险后，只要车辆未离它4米，仍保留为起点**。
Planner的障碍过滤由此把车辆并未占据的危险位置当作实车起点，持续断图。
并不说明障碍应放行，也不排除更早路线/碰撞带来的停车。

### 最小修复与受控验证

`graphnav_mapper/src/mapper.cpp`：仅当旧current具有5秒内有效、高置信实测危险，
且与实际位置距离超过既有robot_minimum_clearance时重新绑定。
优先使用原有free-ball准入的近邻（旧节点失效时允许近邻略远）；没有合格近邻则在实际
odom处建节点。**不制造从危险旧中心到新起点的traversed边**；新边仍由原地形评估建立。
危险节点、历史拓扑、真实障碍/车体/Smac/到达与查询规则不放宽。
实际位置本身危险、原地微动、旧观测均不会不断创建新起点。

- 三项最初反例在改前2失败/0错误，改后通过。
- 新增上下游回归覆盖地形建立安全连接、旧危险保留、真实位置新障碍仍判不可行；
  Mapper真实C++全量34项通过。
- 同一组现场原始图/265秒terrain/对应TF/odom受控回放，重建原始图拓扑与物理属性，
  禁用随机新节点采样：旧Mapper更新地形前后均0条图路线；修复Mapper重新绑定后、
  地形更新前仍0，原地形更新后有通向frontier的图路线（对47个历史frontier目标调用）。
  图路线不是47个目标均可由整车到达；该回放不包含DDS时序或真实执行。
- v2相对v1仅Mapper实现与测试两文件不同，423源码成员冻结。
  新Mapper库SHA `e8168739ecbbff9f4c82d6ca66410a086b09d6d21cb6af3c5a52757501f89b5e`。
  部署前项目残留为空、X148无锁；其他GPU作业未操作。420秒完整复测已开始，待验收。

证据：同目录`graph_topology_audit.json`、`audit_graph_snapshots.py`、
`extract_longstop_map.py`、`replay_mapper.cpp`；服务器v1原始bag/CDR不删。
紧凑包`trial_155227_compact.tar.gz` SHA
`b2c0b63d6a0228e2ca1363a3821e2baf0380d7129aaeecfc9491e8260bcde3cb`已核对。
完整走停与目标到达仍未通过；换挡、规划间隔与首次碰撞不是此项组件修复的已解决范围。

### 2026-10-22 v3/v4 状态回调调度复核与三分钟试跑

- v3完整轮 `trial_20261022_173435` 运行420.814秒超时、未到达。Native样本8380/418.988秒，最大源间隔0.103秒、source rejection为0，但桥接报出大量 execution unavailable/近停。结合入口使用单线程 `rclpy.spin(node)`，怀疑图/规划回调占用同一执行队列，导致状态订阅回调无法及时更新；仅有bag发布频率不足以证明订阅回调执行及时。
- v4将GraphPathBridge入口改为2线程 `MultiThreadedExecutor`；native/control状态订阅放入独立互斥组，并以锁保护快照。控制状态回调只更新安全状态，停止事件时间由主状态机消费，以免跨线程直接改动作/停滞锚点。真实ROS新增用例阻塞默认组0.8秒，同时发20Hz native及control状态，验证两个订阅仍推进且新鲜；该测试通过。`execution_feedback.py`、`execution_recovery.py`、对应测试均有本地未提交改动。
- 调度改动验证：全量离线接口回调699/699通过；服务器隔离包构建通过，Smac 31/31、真实ROS导航/调度421/421通过。它们验证组件行为，不代表BeamNG完整验收。
- v4第一整车轮 `trial_20261022_175048` 在用户提出三分钟上限前已运行272.1秒，随后按请求SIGINT并有序退出，未到达。事件有4次 `source_owner_unconfirmed`、3次FollowPath失败（2次tracking_deviation、1次cusp_not_reached）；前180秒Native近停约52.5%、最长15.75秒，净位移15.66m、采样路径67.06m，遥测最大源龄0.186秒。该轮超过用户后来设定的时限，保留为反例；没有将其算作新规则下的验收。
- 用户2026-10-22要求之后每轮最多3分钟。v4下一轮 `trial_20261022_180054` 的驱动已从轮次启动时设180秒全局截止（启动占用预算；有序停车/关闭在截止后另计），不录视频。预检无残留进程、X148无锁；Town只加载一次，真实车体及传感器启用，运行源码和Smac库SHA均经核验。
- 180秒轮最终为 `TimeoutError('Field readiness/result deadline')` 中断，进程退出码0、errors=[]于session，field_acceptance=false；目标搜索有效时长145.33秒，未形成完整到达验收。Native覆盖142.54秒/1019条，最大源龄0.186秒；起点(734.294,-11.950)→终点(738.918,-74.316)，净位移62.54m、采样路程69.35m，最高速度1.889m/s、均速0.497m/s；速度<0.1m/s占采样48.3%，记录低速段总计约63.50秒、最长连续14.75秒。规划状态59次planning、20 planned、22 following、5 maneuver_done；无FollowPath失败，但一次 `source_owner_unconfirmed`（45.93秒）。出现3次visible-target-approach、1次reobserving、1次approach timeout；不等于目标到达。故调度回调独立后source-owner告警数量在这两轮中由4降至1，但轮长不同且样本数不足，不能据此宣称已修复或统计显著改善；近停仍偏高，连续稳定运行未通过。
- 本轮关闭后rosbag元数据显示 `/wildos/native_control_status` 5441条、`/wildos/control_status` 5561条/272秒，二者均约20Hz，支持“发布端持续发消息”，但单靠bag不能确定某条owner未确认究竟是回调延迟、状态/租约代际错位还是一周期Topic skew。回调隔离后仍出现一次 `source_owner_unconfirmed`，下一步应按该时刻核对控制状态、Native receipt的sender/generation/sequence与确认ACK，不再假定调度饥饿是唯一原因。
- 本轮关键日志已取回至 `test_artifacts/review_user_changes_20261022/v4_trial_20261022_180054/`；服务器原始轮目录 `/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180054` 保留，视频关闭。第一轮证据在服务器 `trial_20261022_175048`。未创建Git commit：虽然隔离/组件验证通过，整车目标与走停验收未通过。

### 2026-10-22 同sequence ACK滞后复核与修复候选

- 关闭后的180054 rosbag已按plan_status时间戳复核唯一 `source_owner_unconfirmed`。bridge事件时间 `1792663335.400459677`；最近Native状态在事件前0.912ms，generation=7/sequence=1385、`source_receipt.applied=true`、`control_acknowledged=true`、`source_rejection=null`、telemetry age约53ms。`/wildos/control_status` 同sender/generation/sequence=1385的状态在事件前约4.8ms报告 `source_ack_ready=false`；约8ms后sequence 1386报告true。该事件是同一控制命令的ACK发布顺序错位，不能解释成消息源断流；回调隔离本身没有解决这条确定的同帧错位。
- 最小候选仅在 native receipt 已确认且 applied、control/native两份状态协议/sender/generation/sequence完全匹配时，允许 `control_status.source_ack_ready=false` 一帧；source年龄仍须≤0.5秒，receipt仍须有效。owner或sequence错位仍返回 `source_owner_unconfirmed`。新真实ROS回归验证同sequence可过、下一sequence或错误代际仍拒绝；服务器真实ROS **422项**、Smac31项通过，全量离线回调699项通过。
- 最新真实车体轮 `trial_20261022_180945` 严格受180秒总预算，目标阶段148.78秒后超时收尾；完整验收false，console_exit=0，预检复查进程为空。Native覆盖145.20秒/1050条；净位移58.10m、采样路程78.13m，速度<0.1样本40.3%，低速区间累计54.70秒、最长6.66秒，均速0.551m/s、最大1.656m/s、最大源龄0.223秒。无`execution_unavailable`和FollowPath失败，说明本轮未复现owner门错误；单轮短试跑不足以证明概率性问题消失。
- 目标仍未抵达：有4次visible target approach、2个观察步完成，117.59秒 `visible_target_route_timeout`、147.54秒 `visible_target_no_vehicle_path`。因此通行/观察点可达性仍是独立未解链路；近停占比也仍约四成，不能称完整执行已稳定。
- 本轮核心日志 `test_artifacts/review_user_changes_20261022/v4_trial_20261022_180945/`，原始服务器路径 `/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180945`，没有视频。之前180054核心日志也已取回本机。状态回调与单帧ACK修复尚未commit，完整目标/走停验收未通过。

### 2026-10-22 继续：异步结果交错与观察期限（仅离线候选）

- 关闭轮180945的stack.log：117.35秒完成段并请求epoch15/request48；117.57秒收到成功规划；117.59秒发布`visible_target_route_timeout`并reset；随后仍发布planned/following，`source=graphnav`且`graph_goal=null`。任务重置与旧结果下发交错，不能算作有效观察接近。MultiThreadedExecutor中的Future done callback是executor task，不能假设它自动遵守ActionClient默认互斥组。
- 候选在NavigationTransactions增加结果队列及默认组GuardCondition；GraphNav reset、ComputePath接纳/结果、FollowPath接纳/结果及退休动作结果通过该入口处理，原epoch/serial/quarantine检查在消费时执行。Native/control状态独立接收及原锁保留；无executor的直接回调测试仍同步执行。新增真实ROS两线程调度测试尚未执行。
- 10秒`visible_target_route_timeout`现只约束未收到GraphNav route的等待，已接纳路线的段间空隙不按任务总年龄释放；原30秒整体截止保留。回归另暴露：整体截止时物理恢复拥有控制权，release实际延期，但check原返回True令monitor_inputs提前返回、阻断恢复。现恢复占有控制时返回False使恢复继续；恢复结束后原截止生效，不重置计时。
- `serialized_results_counterexamples.py`仅在内存恢复旧规则，4个新反例全部失败、0error，日志`serialized_results_counterexamples.log`；当前完整离线ROS替身703项全过，日志`serialized_results_callbacks_verified.log`，py_compile通过。首次700项有2项旧测试依赖“有路线仍在11秒释放”，已改为明确测试原整体截止到期；随后702项暴露恢复tick被吞，失败证据保留`serialized_results_callbacks_final.log`。本次703离线通过不能沿用此前422真实ROS通过数字。
- 距离口径更正：对话中曾把Native原始position距观察点约0.45m当作规划距离，此判断错误。按既有SBR_BASE_OFFSET和完整body_rotation变换，request59后轴约(737.220,-67.724)，距观察点(737.825,-70.058)约2.410m；拒绝附近约2.350m。证据`v4_trial_20261022_180945/rear_axle_distance_audit.json`。现有代码偏移正确，未据此修改坐标或放宽到达门槛。
- request59日志有1388 evaluations/6连续起点候选，6次平滑后回退raw检查，最终`No safe bounded continuous-start plan`。具体raw几何/碰撞拒绝尚不能区分，末段可达性未解决；只读`audit_last_observation.py`已准备提取关闭bag的精确odom/TF/public costmap，未上传执行。
- SSH可连，进程检查无BeamNG/项目ROS/driver/recorder残留，未启动新仿真。首次预检脚本路径误用了不存在的full/prestart.py，之后依据实际目录及进程清单核对。上传分析脚本被自动审批服务连接中断拦截，未执行；已请求手动允许上传和读取。新候选未部署，真实ROS和180秒整车待验，未commit。
