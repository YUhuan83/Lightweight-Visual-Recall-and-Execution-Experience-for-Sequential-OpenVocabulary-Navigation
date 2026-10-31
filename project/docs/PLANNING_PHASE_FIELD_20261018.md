# 2026-10-18 冻结控制源期限候选的整车复测

本轮 `field_yaw_left90_search_20261018_151916` / `session_20261018_151920` 已关闭。候选为404成员 d9d2e27b035b46486cef3b3d74a381bc73b6bbececec8c39818ade6115d4a900，运行中源与配置未改，日常e6未替换。Town原位置左转90°、真实SBR/原碰撞、green residential house、420秒；三个新增记忆先验关闭，验收公共执行链。之前真实静止组件和bbox测量见 EXECUTION_WAIT_AUDIT_20261018.md。

**完整搜索失败，未到达目标，不计20轮成功，不commit。** 本轮没有新增生产实现修复；核验了已经冻结的源期限/时钟域修正，收集后续修复所需的反例。

## 统一试验窗口后的事实

证据在 `test_artifacts/planning_phase_physical_20261018/`，采用 `canonical_window_review.json`。`goal_001/result.json.started` 与独立观察器 `received_monotonic_ns` 都来自服务器Python monotonic，直接同域筛选，避免把控制台输入时刻当搜索计时起点。控制台输入早于实际goal开始5.4749秒，是准备阶段；旧分析窗口及其日志保留，不作为最终分解。该修正是分析窗口更正，不是又发现了5秒传感器延迟。

- goal耗时420.9583秒timeout；独立8417 Native状态覆盖420.8910秒，轨迹累计约75.283米，最大离站26.920米。原稀疏goal日志积分73.941米，采样不同，不能混称两个真实里程变化。
- 低于.05m/s累计294.463秒；其中按**精确源身份**匹配，stale_command221.883秒、gear_change39.831秒、tracking10.141秒、ackermann_stop3.449秒、hold1.391秒等；未匹配或未applied16.963秒独立保留。
- 静止时实际发送制动284.322秒、正油门>.1且无制动.378秒、低油门/滑行9.763秒。因此本轮大部分停驶发生在主动制动期间，不能用真实物体卡住解释全部294秒。
- 按**最近独立观察到的公共状态**关联，planning213.035秒、following45.392秒、execution_unavailable28.779秒、waiting_for_costmap_update2.902秒。这是关联而非私有规划CPU/锁占用精确时间；不能把213秒一律归到某个搜索函数。
- 实际goal时间窗有174条ComputePath发送日志，153条仍属当前请求的结果日志中133为status6、20为status4，另20条旧结果被忽略；剩余不强配。请求/结果旧文本没有共同serial，禁止直接把这些数字做一一配对的成功率。公开following30、maneuver_done21也不是完整搜索成功。
- session日志中有“no valid path found”“exceeded maximum iterations”“No safe bounded continuous-start plan”等不同拒绝；封闭摘要保留原文。连续起点包装层将多个拒绝归为统一异常，当前无法区分每次是起点连接、后缀搜索、几何、地图变化还是预算耗尽。不能把全部empty_path解释为车身真实碰撞，也不能直接增加预算/删几何保护宣布修复。
- GraphNav公开968条路线且全部标complete、无>5秒发布间隔，但观察端点ACK为0，mask/定位/确认均0。图路complete仅表示该层连接成功，不代表Smac可执行或语义到达。此次没有进入绿色房屋接近/确认，记忆增量仍未验收。

## 控制源晚到被再次复现，但不是所有停驶的同一原因

三个只读订阅分别为 reliable depth1、reliable depth10、best-effort depth1，共用一个额外诊断participant；实际Native是独立participant。

- 统一goal窗口三旁路源龄P95分别.008063/.008267/.008044秒，max .0683/.0679/.0394秒。Native状态采样加权的“源龄减接收龄”P95 .2497秒、max .4645秒，314条watchdog且314条缺receipt；没有source_rejection。重复状态会包含同一帧，不能把状态条数冒充独立接收次数。
- 原补充窗口按控制台输入切分，得到689条晚到状态、去重121个源身份。对这些**同一源帧**三旁路max接收源龄均不到.016秒，而Native较晚，进一步把故障缩小到Native自己的接收/执行上下文；不能由同配置旁路正常证明修改QoS必解决。
- 只读/proc UDP快照：控制两进程发现端口有内核丢包累计，用户数据端口未见内核丢包；不能将发现端口计数直接当成控制数据丢包根因。DDS环境均核为既有loopback UDP profile。
- 1413条正油门applied记录均精确匹配源身份/载荷，生成至ACK源龄P95 .14965/max .44799秒；无已观察的原始500ms期限越界/错误frame/无匹配正油门。源保护有效不代表传输延迟已修好；本轮没有source_lifetime运行异常证据。
- 已只读核实安装ROS Humble：底层take_message返回正文和source_timestamp/received_timestamp字典，默认Python executor只返回正文、丢弃元信息。隔离诊断话题实际取到两个时间戳；“回调自动接收第二个MessageInfo参数”的假定在此版本不成立，未按此假定修改生产回调。下一步需要在可核验的接收边界记录元信息，以区分中间件接收前和接收后等待。

## 停车、证据及验证范围

- goal结束持续停车末速.0000980m/s；723节点、467车身碰撞节点、四轮各32碰撞节点、1449.72kg，原物理未关闭；session errors=[]。关闭后独立进程检查相关进程0、display148释放，未触碰GPU1外部任务。
- 本轮真实双栏RViz视频已回传：`C:\Users\Yu\Desktop\拟真\captures\20261018_planning_phase_source\rviz_live.mp4`，426.625秒/3413帧/8fps/1920×1080，72633082字节，SHA256 38153f2da1f311f8bf5514929a332d58dabd531e11bfd8b96e0dd79ec8070a46；全解码通过。右栏含camera，未独立录制front_camera.mp4；不是之前090814的视频。
- 核心ZIP135428356字节、276载荷，SHA256 98eda34a3cbc9cb33e7bc927a103de44a0d489ca18cf759766c62484859bf706；本机CRC、逐成员SHA/长度与视频身份通过。2.974GB原传感器库只保留服务器，外部清单SHA 9b69c4d5864bf19481dd2b55ad1fe7afc764912e009c771592627da00c742367；不称全量已下载。
- 补充ZIP49载荷/3748343字节，SHA256 7e094c7b1713413531fa53fbb4dddf107cd8e41407c5cdb13aa1e0dccae50c7f，本机CRC/逐SHA通过；原核心封包不覆盖。之后本机统一窗口更正、测试和本文单独保存，不回写旧封包。
- 8项证据一致性检查通过，覆盖失败不变、图路不冒充视觉到达、统一时域和准备阶段排除、停驶时间守恒、正油门源/期限、原物理与停稳、当前视频身份。它们验证记录自洽，不能替代整车通过。

下一步优先闭合两处：Native独立接收/调度的晚到来源；Smac连续起点到后缀规划的具体拒绝阶段及同目标重复尝试。多段动态交接、语义/地形覆盖、整体进展退出、视觉定位和记忆反馈问题继续按全流程审计保留。此次没有证据支持关闭所有物理、放宽过期/碰撞/到达保护或宣布规划基本正常。
