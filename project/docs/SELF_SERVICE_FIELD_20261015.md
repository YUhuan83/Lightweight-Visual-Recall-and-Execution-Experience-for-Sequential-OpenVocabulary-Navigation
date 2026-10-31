# 自助入口现场验证（2026-10-15）

本记录与 `SELF_SERVICE_CONSOLE.md` 配合使用。本轮已结束，启停通过，完整绿色房屋搜索超时未通过。

## 运行版本与初始化

- 旧组件会话 `physical_component_session2` 的栈1840196、传感器1840081、引擎1709748仍在。
  已核实起始身份/进程组/安装路径，关闭两控制门、SDK速度0.0175m/s、暂停ACK后清理。
  遗留cloud_filter曾占约3950% CPU；该进程随旧栈退出，不以此推断历史行驶故障原因。
- 本轮独立安装 `/home/ly/wildos-console/install`，继承明确基础环境，重建五包。
  初始候选5c6e失败于非交互PATH没有colcon；实际工具在SDK虚拟环境。
  安装入口显式指定该环境bin，新增 `test_console_installer.sh`：干净PATH、路径含空格、
  五包参数/库身份和已有安装拒绝的Linux替身回归通过。17adb4ae候选实际五包构建通过。
- 之后仅tools与测试有变；已逐员确认生产Python/C++源码、安装库和参数未变化。
  当前执行候选 `ff0fa6e512f21a67e1605ea960df15148edac385db6b2a55c84cb1f5f8785a43`。
  Smac库SHA f915a4fceb44aaa07f7edc16937760d8eed671ae5494a874ee93f9585d49fc1c；
  RPP库SHA bd96eb9d6679cf4e3b04638714d540306fc83de7f37064fd310c805c17e56cfb。

## 两个入口问题与证据

1. **出生制动前未设换挡模式。** 首轮地图加载后12秒无法持续停稳，原始工具只发一次brake，
   未对齐native控制器的 `realistic_automatic` 约定。独立原流程复现114个SDK样本，
   后段速度约7.5m/s并偏移约45m；只在connect后设置该模式，其他操作相同，8次读取内
   持续停稳通过，末速0.0158m/s、位置仍在出生点。修正场景工具在首个brake前设置模式，
   继续原0.05m/s/0.5秒判据，并在失败时保留连续SDK样本。
   后续成功场景实测末速0.0150m/s。原探针中的known_brake后处理因场景已暂停，
   速度冻结，该对比无效，不能拿它来判断已有制动器失效。
2. **静默事件回调重复query关键字。** 初次完整预检通过，但/stop中接收带query的plan_status时，
   新入口用dict(query=..., **details)抛TypeError。改为与原Harness一致的字典展开覆盖，
   保留源状态字段。新增纯记录测试及真实Harness状态回调替身反例。
   27项console/16项Harness回归通过；生产导航/记忆状态机没有为此改动。

## 已通过的现场范围

- `field_lifecycle_20261015_144859` / `session_20261015_144900`：真实SBR场景、传感器、模型、
  五包安装来源/DDS/输入预检通过；/stop持续停稳、/quit退出，console返回0、errors=[]。
  独立进程检查无本轮仿真/传感器/导航/视觉残留。此项是启停验收，不是目标搜索成功。
- 保持723节点/固定0/467车身碰撞节点/四轮各32碰撞节点/质量1449.72kg；不改速度、
  转弯半径、碰撞、到达阈值或DDS保护。不录视频。
- 已封闭的启动失败、物理单变量对照、启停通过证据取回本机：
  `test_artifacts/self_service_server_20261015/closed_evidence_initial.zip`，88成员，
  SHA4bac40c29dcc0fbf43e01d96b21169a72b6cf2ef2f7c523f72268dd57c9fa16a，
  ZIP及全部成员SHA校验通过。失败启动尚未附加camera传感器，不能提供该阶段RGB。

## 完整搜索（进行中）

- `field_search_20261015_145008` / `session_20261015_145008`，绿色房屋，420秒上限，
  地图单次加载、执行中无复位；严格ACK和profile后才解锁。
- 目前只读中途快照：约81m原生累计里程，15次following、11次maneuver_done、2次segment_done。
  第141.3秒一次follow_failed；控制器原始日志为bounded command arc碰撞检查中止，随后换路。
  当前没有严格目标到达证据。中途统计不能当作封闭最终结果或成功率。
- 相关源与封闭日志：`test_artifacts/self_service_server_20261015/`。
  必须补最终结果、实际停车、证据取回与人工视觉复核。必要验证尚未全部成功，未commit。

## 完整搜索封闭结果（更正上述中途快照）

- session_20261015_145008 / goal_001：420.8641秒（含退出停车）超时，passed=false；
  Native累计169.7334m，35次following、26次maneuver_done、5次segment_done、8次follow_failed。
  单次Town加载、唯一查询/ACK、真实物理保持；无严格视觉到达，不计20轮成功。
- 末四次follow_failed均为motion_section_invalid/detail=local_footprint_collision，
  同一终点(684.5423,-17.3379)，依次2/4/8/8秒退避；最后一次后约74秒没有新执行状态。
  graph_path_bridge.follow_failed仅对重复execution_stalled/tracking_deviation通知探索拒绝；
  本类局部碰撞拒绝不进入该分支，retry_when_planner_starts/on_route又限制最多3次恢复。
  这是一处有证据支持的恢复交接缺口；尚未实施生产修复/回放验证，不能称根因全部闭环。
  此前stack.log确有一次“selecting another frontier”，不能泛称整轮拒绝反馈从未工作。
- 接口bag未录rejected_route_endpoint/traversal_feedback，不能以bag缺席证明未发布；
  后续应补这两个主题。进入绿化区域的terrain/全局代价来源仍需核查，未关闭碰撞或改道路专用规则。
  不从Harness缺少确认事件直接推断模型整轮没有mask。
- 失败末帧failure_last_rgb.png采集源龄0.44865s，显示绿化/树木，无目标到达证据。
  按用户规则无MP4。连续SDK停车通过，session_result.errors=[]；console退出后独立ps未发现本轮残留。
- 封闭归档test_artifacts/self_service_server_20261015/closed_evidence_full.zip：
  SHA5bbbf0314b33393da619c2e6fcf2e5480548fccdf3c3bee796bf790fd32b06ca，
  128文件及ZIP全部校验，解包同名目录；final_search_review.json保存统计/原始失败事件。
- 本机最终全局48PASS/0FAIL/6WARN；console27、Harness16回归通过（范围重叠）。
  服务器Smac及RPP原生测试通过，不能替代完整搜索。
  /stop与/quit现场已验证，外部--stop与SSH断连退出尚未现场验证。必要验收未全通过，不commit。

## 第二轮候选与回归（搜索进行中）

- 用户要求继续测试。独立ps无旧栈，205GiB可用、负载下降。只修复重复局部碰撞拒绝的交接：
  follow_failed在motion_section_invalid/detail=local_footprint_collision且recovery_attempts>=2时，
  复用reject_exploration_route通知GraphNav临时回避；保留早期重试、原次数/退避、碰撞与物理。
  不触发记忆失效，不改显式/记忆目标；原freshness/query/route identity门继续有效。
- 加入现场同终点反例、前两次不拒绝、达到门槛精确header/endpoint、显式/记忆/旧查询/
  query切换/停用/旧odom保护、其他几何原因不混判测试。原ff0fa6方法离线重放缺拒绝复现；
  当前374回调通过。首轮测试对fake消息做对象等号失败（fake无值比较），改字段比较后通过。
- 真实ROS118 routing测试首次暴露已有夹具SimpleNamespace Time类型断言，生产导航未改；
  夹具使用真实clock消息并更改sec/nanosec后118通过，console27通过。
  全局48PASS/0FAIL/6WARN。当前431成员SHA751218fde4ef9575a606b53e440719c6b737bb2840d54fa2ebaebe0738527f8c；
  与ff0fa6只4文件差异（Bridge、routing测试、console/测试），生产仅重建adapter，C++库保持原身份。
  deploy/20261015_local_collision_recovery_candidate冻结；ffcf中间包保留，不能当最新版本。
- 新接口bag增加rejected_route_endpoint/traversal_feedback/route_status，便于现场核验反馈闭环。
- session_20261015_151045，green residential house，420秒、同一地图单次加载，真实物理/速度配置保持。
  中途约31m、4机动/1段成功、0follow_failed，完整结果待封闭；不计成功、不commit。

## 第二轮中断与用户录像要求（不得误归为规划失败）

- session_151045在174.257秒session_exit，Native79.0526m、24following、20maneuver_done、3segment_done、0follow_failed。
  bag.log直接异常SqliteException/database is locked。运行中外部诊断review_search对SQLite做全表只读计数，
  与故障时间重合，读取事务可能阻塞rosbag写入，不能把此次中断归于车辆/模型或新规划修复。
  已改诊断工具必须同时存在goal result与session_result才可接触SQLite；活动试验立即拒绝的服务器检查通过。
  故障前未出现新增局部碰撞分支，不能当现场验证该修复。
- 用户中途明确要求传回视频，覆盖本次“不录像”约定。151045已在录像启动前结束，首个晚启动工具未收到帧，
  无可用视频；初次shell启动CRLF导致路径尾\r失败，修正LF后仍不将空录像称真实证据。
- 新session_151859同一候选，外部只读recorder先收到camera并写started.json再发green query，完整记录。
  另存45秒墙钟预览，431帧/10fps=43.1秒，逐帧可解码，360个不同源戳，新鲜帧比例100%。
  预览captures/20261015_green_house_recovery_trial3/front_camera_preview.mp4，
  SHA8e0bfe58beeeb95460604b87d3c69ff6e110aaa188c89d0d52f23eb6ae2109d4，
  对应json与validation保存时间戳。实际前摄像头，不是RViz录屏、不是到达验收。
  片段取样存在少量重复帧且播放长度短于墙钟，不能宣称精确实时播放速度。
  完整录像仍在录制，最终结果另补；console内置video=false与本次外部授权录制区分。

## 第三轮封闭结果与完整视频交付

- session_151859：420.8003秒timeout；Native199.2655m，34following、27maneuver_done、3segment_done、
  4follow_failed（110.84s动作中止、212.74s偏离1.051m、241.70s无进展、348.00s动作中止），
  2execution_recovered；末段10次no_vehicle_path、10条精确拒绝消息，GraphNav日志10次接受并选择其他frontier。
  这证明本轮有拒绝闭环，不证明换来的路径可执行；后续仍长时间无法运动，不能称稳定性修复。
  新增“重复局部碰撞”分支本轮未触发，因此只有离线/真实ROS回调验证，未获现场该分支验收。
- 封闭bag实际object_mask/localization/confirmation/visible approach均0，query1；该轮没有目标视觉确认。
  完整RGB保留供人工复核是否错过可见绿色房屋；不能只凭0掩码认定场景中无目标或唯一模型问题。
- 两门关闭、SDK持续停稳，session errors=[]，独立ps无本轮车辆/栈/录像/driver残留。
  SSH连接曾reset，但重连时driver、console、engine、recorder仍活跃；最终正常超时退出，未造成该轮中断。
- 完整H.264录像本机captures/20261015_green_house_recovery_trial3/front_camera_full.mp4，
  112806673字节，4231帧/10fps=423.1秒，全部解码通过；源戳3133个，新鲜帧比例99.905%。
  SHA67cd22e3d3cba279dcbe3beadd331620caf9914a8358a2bd82f11ea09eac2f07，
  对应json/validation保存真实起止和每帧源戳，实际墙钟424.26秒。前摄像头原生960x640，
  无RViz二维拓扑面板；完整片从query前开始，不称成功验收。源MPEG4原片留服务器。
- 两轮封闭日志/接口/版本/回归归档closed_continuation.zip，91文件，
  SHA4abdb622e62b0d243b21e44325acb08ac98720ca674aafdb3214c387a6c605a4，ZIP与全部成员核对通过。
  解包test_artifacts/self_service_server_20261015/closed_continuation，视频单独保存，不混入归档SHA。
  recorded_trial_final_review.json记录最终统计；20轮成功数未增加，正式多目标仍未开始，不commit。
