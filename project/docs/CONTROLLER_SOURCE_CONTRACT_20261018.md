# Controller source lifetime：2026-10-18

## 范围

全流程审计 F15：Humble RPP 内部返回 TwistStamped，但 controller_server 对外发布无 header Twist。速度环无法知道生成时间，晚到的旧速度在接收端获得新的有效期。Native 端原有 source-control 租约只能约束下半段，不能补回已丢失的上游时间。

此修复处理控制指令生成到执行的有效期，不解决全部探索、动态刹停、换挡或多段移动交接。未提供真正的 FollowPath UUID；时间戳和释放边界不能冒充完整动作身份协议。

## 实现与交接

- `ackermann_rpp/src/controller.cpp`：原曲率及碰撞检查通过后，生命周期 publisher 在 `/wildos/cmd_vel_stamped` 发布相同 twist，附计算结束时 ROS 系统时间与 costmap base frame；原返回值及 legacy topic 继续由 Humble controller_server 管理。
- `cmd_vel_to_control.py`：物理 profile 绑定 stamped topic、`spot1/nav_base_link`、system clock 和既有 source-control。未知 frame、重复、乱序、未来或过期指令不能替换有效需求；接收延迟扣减最多 500ms 的原始寿命；同时使用 steady deadline 防止墙钟回退续命。
- 启用/停用及 execution hold/release 会清理需求、建立 source timestamp fence。发布执行器帧时，tracking 的 expires 不超过原控制器期限；独立安全制动可获得自身制动租约。
- **Humble 的终态零速度在插件之外发布**。新增独立 legacy Twist 零值安全通道：全部六分量必须有限且全零才能清需求、制动并设时间戳 fence；非零 legacy 指令不能驱动车辆。取消动作不能只等 stamped 需求自然超时。
- 物理 launch/runtime contract/preflight 增加唯一 stamped producer 与 typed speed receiver 核对。console/两类 trial recorder 记录新 topic。Native v1 消息和 receiver 保持，未放宽碰撞、曲率、换挡、速度或 ACK 门禁。

## 反例与组件验证

证据：`test_artifacts/controller_source_contract_20261018/`。

- 修改前回调反例失败；旧实际 RPP 库运行新增 stamped 发布测试失败，保留 `before_tests.log` 和 `baseline_reproduction.log`。
- 本机 13 个新回归、合计 541 个真实生产回调的 ROS 替身检查通过，不含 DDS/车体。Linux 安装候选真实 ROS 消息回调 81 项通过，实际 C++ 插件 21 项通过。
- 隔离候选 b1f6be14…402 成员；RPP 库 SHA `d2fd9cee5ecabb49e8be06a24079813eb46a9e77872ef6982d1ed547bb04d846`，服务器 `/home/ly/controller_source_contract_20261018/install`。
- 初次 DDS 夹具 `plugins: []` 被 Humble 当作未设参数而启动失败；改为无障碍的合成 inflation 图后完成。未改生产地图。两次目录均保留。
- 真实 Humble FollowPath action + RPP + DDS + 速度环 + SourceControlReceiver，`dds_20261018_090644` 七阶段通过：正常驱动、350ms 延迟扣寿命、重复包不续命、新消息恢复、真实 cancel 零值停车并隔离旧需求、新 FollowPath 恢复、重新 enable 隔离缓存旧需求。取消到制动约 .097824s，27 个真实 RPP 源消息、94 个执行帧，receiver 拒绝 0。实际物理执行由合成 ACK 替代，不能当作 BeamNG 成功。
- 本机全局检查先因 AST 不接受新增 imported constant 而失败；修复检查工具为仅解析明确 literal 常量、未知名称/表达式仍报错，3 个工具回归通过；新依赖纳入包。重新冻结后全局 53 PASS/0 FAIL/6 WARN（离线范围警告）。这属于诊断工具修正，不是降低运行保护。

## 整车候选与当前状态

冻结 403 成员 `ca8baba0fe21a422b8ac958b5128238a39a00c9170fe6e34f0f4d3d8b4f8e422`，相对 94c468a7 共 19 个源码/测试/工具成员变化。本机 `deploy/20261018_controller_source_candidate`，服务器 `/home/ly/controller_source_field_20261018`，安装回调 59 项通过。日常 e6 入口未替换，旧现场版本和证据保留。

整车 session_20261018_090814，Town 原位置左 90°、真实 SBR 原碰撞、一次地图加载、green residential house、420s 预算。测试前相关进程和 display148 占用为 0，约 158GiB 可用，使用 GPU0，不触碰 GPU1 外部负载。严格预检通过；/proc/maps 三库确认 GraphNav db52f00e、Smac 3cd81492、RPP d2fd9cee。三个新增记忆先验关闭，检验公共执行链；不是 untouched WildOS 或记忆增益试验。现场运行期间冻结源/配置不变。

完整结果待 driver、goal、session 三关闭屏障后核验。未计 20 轮成功，未 commit。多段动态接替、全局无进展、地图与视觉三维接近、记忆增量等原审计条目仍开放。

## 新复查反例（尚未修复）

补充诊断 `deadline_race_reproduction.py` 证明：tracking 在剩余期限检查后、SourceControlSender.issue 前被调度暂停，跨过原始截止时间，可因 `source_lifetime` 异常中断回调。此为新 source clipping 的边界竞态；不是已证明本轮现场触发。不得在运行中修改冻结候选。需在单次发行时取一致时间快照、过期转独立制动，新增回归后另行冻结验证；不可通过扩展旧需求期限修复。

## 竞态修正的后续门禁（未进入上述整车轮次）

- 原 ca8 发送前跨期反例在 Linux 实际安装同样抛 source_lifetime。第一修正候选 0a851a22 的82项回调通过，但真实DDS始终stale_command。读取真实时钟发现 ROS steady 与 Python monotonic 原点约差10.8秒；新保护直接跨域比较，是本次修正引入的错误，独立DDS门禁拦下，未进入车辆域。保留失败安装、源包及dds_20261018_091233。
- 最终两文件增量8ebd782e876f74a91165a6118643c233af879f289f23c19c2baad05b4155c250，403成员，test_artifacts/controller_deadline_clock_20261018，服务器同名/home/ly目录。发行前分别读取wall、Python monotonic、ROS steady，一致快照仅在同域检查截止；跨期转独立制动。发行快照之后再有调度延迟，原帧期限仍由Native拒绝，不能续期。新增调度跨期和时钟原点偏移两反例。
- 本机15新/543整体回调及全局53PASS/0FAIL/6WARN通过；Linux真实83回调及DDS七阶段通过，dds_20261018_091431有26源/90执行帧/拒绝0，cancel至制动约.011960s。合成空场景/ACK，非车体效果。8ebd未做整车，ca8现场源/配置始终未改；不将这些组件结果合并当作8ebd整车成功，不commit。

## ca8 整车完整关闭结果：未通过

driver `field_yaw_left90_search_20261018_090810`，session `session_20261018_090814`：421.612s timeout，Native 行驶149.969m，最大离站61.314m，末离站58.096m。726次图路决策中561完整，无>5s图路发布空窗；47 following、29 maneuver_done，不能当作完整导航成功。本轮只有1 mask、0 visible hint、0定位、0确认。已配对 mask/LiDAR 1次源戳差0，既不证明语义正确，也不证明其它时刻深度完整。三新增先验实际关闭、记忆增量未验收。

### 时间戳链路在本轮成立，长停依然存在

- 8432个旁路 Native 样本覆盖421.597s，低于.05m/s累计206.093s。4121静止样本全部按实际 applied 回执身份精确关联源原因：stale_command105.328s、gear_change56.502s、tracking24.137s、execution_hold10.506s、ackermann_stop4.813s、disabled4.655s、opposite_motion_braking.151s。实际下发brake>.1共181.956s，低扭矩/滑行14.165s，正油门>.1且brake<=.01有9.972s。不能说所有停驶都未发驱动，也不能从SDK回执推断真实轮胎力。
- source reason精确，公共规划状态是最近值关联：waiting_for_costmap_update/stale约61.999s，following/gear_change54.179s，following/tracking23.649s，成本比较/stale10.082s。不是私有状态逐项阻塞时间证明。105s stale大多发生在没有活动指令的规划/地图/交接阶段，不能直接解释成105s网络延迟。
- 新 stamped topic 2929条，目标窗口2898条生成至DDS接收P95 .002583s/max .007684s。2796个正油门实际回执从控制器生成至应用P95 .140545s/max .197269s，无原始期限违规、错误frame或无匹配源的正油门。following静止时最新legacy Twist接收龄max .0991s（仅诊断legacy，不把它替代新源期限）。本轮证据不支持传输延迟是大部分长停原因。
- 一次真实RPP bounded command arc碰撞导致action6，另两次execution_stalled（约300.40s、413.76s）：5s路径进展分别.0661m、.077m；不是仅看掩码或图路就可宣布可执行。换向后6次重规划为footprint_collision2/collision_cell4。私有地图障碍真值、无进展物理/控制原因尚未闭环；不绕过碰撞或盲加油门。另一次source_owner_unconfirmed保留。

### 交接判定更正

7次借用旧运动段：1短段完成，1短段实际接入失败，3长段租约到期停车。原审查器仅把之后另一个ID的Following算接替，报2次，包含152.70s到161.93s的一次**取消、零速度、停稳再启动**，该统计不能当移动成功。

补充`verified_handoff_review.json`结合控制器Passing new path、取消日志、零指令及Native速度核验，实际仅1次移动接替：166.51→167.62s窗口，最低测得速度约.980m/s，有控制器preemption、无取消/终态零指令。旧误计窗口有cancel/zero、最低.000198m/s。原封包不覆盖，更正报告与反例单独保留。本次5项证据一致性回归通过，含两窗口分类；不等于车辆实验成功或证明所有交接分支。

### 停车、归档和后续边界

停车末速度.003439m/s；723节点、467车身碰撞节点、四轮各32、质量1449.72kg、kinematic=false与原碰撞保持。session errors=[]，独立/proc相关残留0、display148释放。退出阶段日志有SIGINT/KeyboardInterrupt堆栈，不能称所有日志无错误；未出现本轮运行期source_lifetime异常，已复现边界仍在8ebd修正候选中等待整车验收。

- 失败RViz已回传并打开：`captures/20261018_controller_source/rviz_live.mp4`，426.625s/3413帧/8fps/1920×1080，109689952B，SHA `741a895fce7bf6fe2ccb7874c2333db926090a3e9a28e4801c77da27b60c0b65`，全解码与本机/归档同一性通过。右栏camera，本轮未单独录front_camera.mp4。
- 核心ZIP `test_artifacts/controller_source_field_20261018/full_vehicle_field_evidence.zip`，172698276B/133成员，SHA `400d41e5b0c27e5ad16ae216dc4d0b872da7dfedb5c50a9ba07fd049cd8beb17`，本机CRC/132载荷SHA通过。原始3.422GB传感器DB仍留服务器，SHA `6664bc6dbd802d83d81cbaac636501d4a80836bf3cccb4b2e75ad386400f5591`，外部manifest含路径/长度；未声称下载全部原始库。

全流程仍未跑通，不计20轮成功，不提交。下一步先验证8ebd的实际车体边界，再沿地图等待/多方向段动态准入/整体进展契约处理主要停车，之后再验证视觉定位与记忆增量。不得将时间戳门禁通过解释为规划整体已修好。
