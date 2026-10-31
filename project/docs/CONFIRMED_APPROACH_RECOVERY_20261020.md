# 已确认目标的接近失败与观察交接（2026-10-20）

## 用户指定范围

用户要求优先修复：车辆路径失败后继续围绕旧接近点重试，未及时转入有效替代观察或 frontier 探索。此次不修改 Native 租约、车辆物理、速度、Smac/RPP 参数、检测模型、实例关联阈值或目标到达标准。

证据目录：`test_artifacts/confirmed_approach_recovery_20261020`；服务器独立候选：`/home/ly/confirmed_approach_recovery_20261020/v2`。旧安装和旧失败证据保留，不覆盖正式安装。

## 最早缺口及旧反例

- 关闭的 `trial_20261020_215010` 中，接近点 `(741.7159575234645, -99.2799169063999)` 的 8 个车辆候选最终得到 7 次 `empty_path`、1 次 `footprint_collision`。先前收到 `(737.3662952789695, -94.3376134247336)` 等新观察位置；更新因 `accepted_route_active` 或 `no_material_approach_progress` 被推迟。
- `GraphPathBridge.try_candidate()` 候选用完后仅封锁当前路线并调用 `reject_exploration_route()`；后者正确地拒绝把显式物体目标当 frontier 删除，但缺少物体目标自己的失败交接。重试预算用尽时也有同类缺口。
- `consume_confirmed_approach()` 主要从到点后的 `await_object_observation()` 被调用。失败后不一定到点，因而到不了这条交接入口。仅依据到物体的径向距离判断更新，又会排除更远但可能可达的侧向/退后观察位置。
- 已确认接近点等待 GraphNav 路线的 10 秒窗口结束后没有自己的退出；临时观察计时器还可能重置正在执行的物理恢复。
- 先编写 9 个生产回调反例，旧版 9 项失败、0 错误。初稿修复后补充上游反例：释放 ACK 后，新图像可立即重新积累同一区域的确认，重锁旧物体；该反例在加入导航退避前失败。

预测碰撞中止及空路径是导航失败证据，不证明物体不存在，也不证明实际车身已经撞到具体资产。

## 改动与数据流

生产仅修改 5 个文件：

1. `nav2_adapter/wildos_nav2_adapter/confirmed_approach.py`
   - 新增已确认接近点的失败入口及至多 8 个失败位置记录；属于当前查询，不跨进程保存。
   - 失败后撤销旧动作 epoch，保留目标实例和实测恢复走廊/失败预算；旧动作终态和持续停稳门禁继续有效。
   - 相同失败基线附近换 heading 不算新观察。新确认不能复活失败点。
   - 健康路线仍保持；把非径向前进的同实例更新保留为失败后的导航种子。
2. `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`
   - 将实际车辆候选耗尽、无自由候选、已用尽的重试预算及连续执行失败接入该出口。
   - planner 拒收、通信异常、坐标系错误不作为观察点不可达的证据。
   - 交接期间旧路线消息不能恢复旧点执行。
3. `nav2_adapter/wildos_nav2_adapter/target_observation.py`
   - 失败后先尝试保留的同实例观察种子，再尝试有界环形观察位置；既有实例边界不放宽。
   - 先做当前地图上的完整车体终点检查；派发后仍走 GraphNav → Smac 完整路径验证 → 本地 footprint 检查 → FollowPath。终点空闲不等于路线已可达，未通过规划的候选不能驱动车辆。
   - 至多 4 个替代观察派发；失败不反复重开同一基线。等待 GraphNav 无路线有 10 秒出口；临时观察沿用 30 秒期限。
   - 候选耗尽后复用现有实例 release/ACK、GraphNav `release_goal` 和时间 fence 恢复探索；不调用完整搜索重置，不删除当前地图或失败 frontier 暂缓记录。
   - 观察超时不能取消已归属于物理恢复的动作。替代派发仍等待旧动作终态/停稳；输入过期时暂停交接。
4. `official_adapter/beamng_wildos_adapter/goal_standoff.py`
   - release 请求可带可选 `navigation_failed=true`。仅在身份/fence 校验通过、确实释放对应当前 anchor 时，给该物体区域 60 秒导航退避，最多保存 16 个区域。
   - 退避用 steady time；重复 release 不延长，切换查询清除，期满可再次积累正常观测。其它区域仍可确认。
   - 这不向语义记忆写“无目标”，不修改模型输出，也不扩大蓝房子/绿房子的实例关联门限。
5. `nav2_adapter/wildos_nav2_adapter/visible_approach.py`
   - bearing-only 的临时接近点也遵守已有失败接近区域的短暂退避，避免从另一个输入入口立刻接管回旧区域。

Bridge 与 GoalStandoff 必须一起部署。旧 producer 会忽略新增字段，无法提供该上游退避保障；来源一致性门禁继续检查两个包，不能只热换一个文件。

记忆检索候选仍由既有 RecallCandidatePolicy 管理；本轮新失败事务针对实测第一阶段的 live confirmed approach，不声称历史 ConfirmedViewpointCache 所有失败分支已整体通过。

## 验证及边界

- 本机：18 项新增正常/失败/交接回调，相关 194 项回归通过；全套 608 项离线回调通过。覆盖健康路线不取消、真实空 action result 回调、侧向观察、重复失败点、无路线、预算耗尽、旧回调/查询切换、过期地图、占据候选、其它实例隔离、物理恢复与终态屏障、实例 release/ACK 及短暂退避。
- 服务器真实 ROS 消息/节点接口：136 项通过。首次把含 SimpleNamespace 替身的旧套件整体作为 ROS ABI 测试运行，192 项中 11 失败、3 错误；均保留日志。该套件仍纳入本机完整回归；真实 ROS 运行器改为明确选择使用真实接口类型的相关套件，没有放宽生产保护或删除本机断言。
- 实际 DDS：C++ GraphNav、Bridge、GoalStandoff、ComputePathToPose/FollowPath action server 联动通过。旧点失败 → 更远同实例替代点 → 接受真实 FollowPath 句柄；新旧点刷新不取消健康替代动作；三次执行失败 → 有界替代耗尽 → 实例 release ACK → C++ 再选 frontier `(40, 0)`。四个 Python 来源和 producer 来源、实际 C++ 库 SHA 已核。地图、检测、静止 odom/transport 与 action 结果是合成输入；未运行 Smac/RPP/车辆，不能当成真实可达或视觉到达。
- 关闭的旧轮真实输入重放：`closed_before_v2.json` 保留旧点；`closed_after_v2.json` 在当前完整 footprint 检查下没有派发可用替代点，转入 `unconfirmed_target_release_pending`。使用原始 odom/公开 costmap/确认与失败消息重建交接状态，不声称复原了原私有异步队列，也不把源确认当作目标类别已经正确。
- 两个 Python 包独立构建通过，冻结 417 个成员，生产改动仅上述 5 个文件。整车复测 `trial_20261020_224538` 使用同 Town、同真实 SBR、同物理/规划参数和同进程记忆；四库及 108 个安装来源门禁通过，无录像。最终 421.88 秒超时，odom 累计 166.95 米/净 42.27 米；实际执行反馈 4 次 follow_aborted，仍有多次超过 5 秒停驶。该轮 object_mask/localization/confirmation 均 0，目标已编码、推理继续报告 No object detected；不能据此断言场景无目标、模型错或本次交接已经现场通过。目标接近恢复分支未触发。
- 该轮真实原 SBR 停车末速 .006906m/s，至少 5 个连续停车样本均小于 .03m/s；无碰撞关闭/理想载体，driver/session errors 均空，后检项目进程清单空。没有复现前一录像轮的 Native 租约致命退出，历史风险仍开放。
- 封闭证据已回本机：`sealed/confirmed_recovery_evidence.zip`，7299866 字节、510 载荷、571 个服务器大文件引用，SHA `7f38b123092d8544bc996ee079c8c8396b5e9d922809a6fdf7ad7dba7309144c`；CRC、逐载荷 SHA、417 当前源、旧/新重放、608 离线/136 ROS/13 阶段 DDS、四库/108 来源及原车停车均核验通过。核验通过不是完整导航通过。
- 用户随后要求再跑并录像；`225836` 在场景加载 SDK 超时退出，map_loads=0/goals=0/无录像，退出和无残留已核。原版本/参数不变重试 `225946`，真实 RViz 录制与目标下发须以该轮运行证据为准，不能借用旧轮录像。

## 用户要求的录像复测与版本核对

- `trial_20261020_225946` 已关闭：目标搜索 420.86 秒超时；实际 RViz 窗口录制 426.1 秒、1920×1080、10fps、4261 帧，编码退出 0，driver/session errors 空，后检进程空。
- 录像本机地址：`captures/20261020_confirmed_recovery_trial225946/rviz_live.mp4`，265970877 字节，SHA `f76e5f98b3c0e14cd48722d4643b68e129aa393716c704f45d86f8399be58ff6`。传输 TAR 268349440 字节，SHA `cc17ae521e95fe9e22200e60c703eff381ba248bd5384d4cc4859cb109c576bf`，18 载荷逐 SHA 通过；本机实际解码 5/60/120/421.1 秒图像，右 camera 区域样本差异约 54.98/51.72/42.88，未出现整段固定图片，不能将抽样变化等同逐帧时钟或持续高帧率证明。
- 用户质疑重复换向及旧版本：录制期间核对实际进程 `2174791` 的 GraphPathBridge 和 `2174771` 的 GoalStandoff 均使用本轮 `v2/install` 入口，参数文件也是该安装；5 生产源的当前/冻结/启动检查 SHA 全部相同。主桥 SHA `f8bf0a8da4be4c1a94469382305d2b617dcbe163c5b7e9d80c6d50bd031dae20`。记录已随视频取回 `live_version_check.json`，没有把仅本机工作区检查当作运行版本证明。
- 该轮 odom 累计 72.14m/净 14.42m/最大离站 16.39m。同一 frontier 多次出现 2～8 个运动段，探索执行仍不稳定；目标 mask/localization/confirmation 仍全 0，新 confirmed approach 失败事务仍未进入。因此不当作修复后的完整通过，不计稳定性轮次，不提交。
- 关闭日志确认 9 次 `replanning_after_maneuver`：5 次 `footprint_collision`、4 次 `collision_cell`。位置误差约 .117m、朝向误差 .002～.051rad 的样本也会触发当前地图上的续接拒绝；不是仅凭视频猜测车辆真实撞到了障碍。需继续查当时被检查的哪一段、两张地图及占据变化，不能直接归为 Smac 原始路径错误或某个资产。
- 另有执行来源保护：去除重复相同源状态戳后，`execution_unavailable` 中 source_protocol 77/source_expired 32/source_owner_unconfirmed 8 条状态记录；不是 117 次独立故障。执行反馈记录来源过期 1/协议 5 次，恢复 8 次。它们会暂停/撤销运动，恢复后重算路线；仍需从原始 source receipt/ACK/时限定位最早违约，不把协议标签直接解释为运行了旧版本，也不把它与车辆 Lua 租约致命退出混为一谈。
- 原始 DB、Native 时序和新 `execution_replan_review.json` 保留在该服务器轮次；本轮不修改 Native。录像及日志证明完整流程仍失败，不能用组件/DDS通过掩盖探索执行和控制来源问题。

完整视觉到达、真实车体执行、记忆收益与 20 轮稳定性仍须分别报告。必要整车验收未通过前不创建 commit。
