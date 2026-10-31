# WildOS 原仓库与当前执行链对照（2026-10-24，只读分析）

本次按用户要求先找差异与原因，未修改导航、规划、控制配置或部署。原版先前在 BeamNG 流畅运行是用户提供的历史事实；本次尚未取得该轮的精确运行源码、Nav2 配置与物理模式，不能冒充已经完成了同条件原版/新版整车 A/B。

## 参照与可复核证据

- 官方仓库：https://github.com/nasa-jpl/nebula2-wildos 。官方 README 明确还需要外部 Nav2；仓库本身没有给出本项目 BeamNG 执行桥，也不能据此认定作者使用了我们现在的自定义 Smac/RPP。
- 本机原始参照：`C:\Users\Yu\nebula2-wildos`，既有 provenance 记录源提交 `ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572`。8 个参照文件原始字节均匹配既有 provenance；当前 Git 对象访问受限，本次没有重新独立校验 Git blob。在线核对了官方 follower、launch 和 planner_node 的对应逻辑。
- 当前对照：`wildos_port/repro_check/src/nebula2-wildos` 与 `nav2_adapter`、`official_adapter`。
- 两遍本机审计输出一致：`test_artifacts/upstream_execution_comparison_20261024/source_and_evidence.json`，SHA256 `23fe1d2ad67d1c0ffbb7e238b7203a58510d151b8d5a54692881101d5c346a66`。同目录保存8份逐行diff。

## 最大的差异是执行入口已经更换

官方 `graphnav_planner/src/path_follower_node.cpp` 随 odom 计算并发布前视 PoseStamped；其官方 launch 将前视配置为5米。它本身不执行逐段车身换挡事务。[原始 follower](https://github.com/nasa-jpl/nebula2-wildos/blob/main/graphnav_planner/src/path_follower_node.cpp)、[官方 launch](https://github.com/nasa-jpl/nebula2-wildos/blob/main/graphnav_planner/launch/graphnav_planner.launch.yml)。

当前 `official_adapter/launch/official_graph_stack.launch.py` 仍启动 follower，输出 `/spot1/goal_pose`。但 `GraphPathBridge` 的实际订阅是 `/spot1/graphnav_planner/path`，没有订阅 `/spot1/goal_pose`。在本项目 nav2_adapter/official_adapter 内，后一个话题的订阅者只查到 health_check。`nav2_official_control.launch.py` 的首行说明仍写原输出，实际只启动 planner_server/controller_server，没有把该 Pose 接入控制的节点。**保留了原 follower 进程，不等于执行仍走原 follower 链路。**

当前执行链是：GraphNav 整路 → 自写车辆候选/航向比较 → ComputePathToPose → 全路与地图准入 → 按方向拆队列 → 每段 FollowPath → 停车/换挡或续段 → 失败后恢复/重新选路。基础 WildOS 前视输出已不在这个闭环里。

这里的“滚动目标”不表示原算法始终保持精确5米距离或完全无跳变。两遍公式复核中，密采样直线目标随车位推进；粗疏10米节点时目标可能在一段内保持不变。原代码也有局限，不能把“原版更简单”直接当成稳定性证明。

## 与实际停顿有关的差异

| 差异 | 当前代码位置/行为 | 对停顿的含义与证据强度 |
|---|---|---|
| 位置路线变为受车姿/曲率约束的终点 | `graph_route.route_candidates`、bridge `start_segment/on_plan_result`；路点切线与目标方位被用于 Smac 终点yaw | 点与边在图上可达，不保证该终点姿态能被汽车顺畅到达。相同车姿、目标位置和公开地图的两遍真实Smac对照，仅改兼容圆弧的yaw，2次换向→0次；证明部分折返来源，不能泛化为全部路径原因。 |
| 多方向路径拆成多个停车任务 | bridge `dispatch_validated_path/send_motion_segment/on_follow_result`，`motion_sections` | 每个方向段末端需真实到位，再进入下一段。cusp checker要求0.2米精度与≤0.05m/s；这是明确的停止点，不是图像卡顿。真实车体必须安全换向，但不必要的短折返会将这项成本放大。 |
| 另加2秒软件换向确认 | `cmd_vel_to_control.py:84`、`planning.update_direction_shift` | v22两轮各14个gear_change区间，低速≤.15m/s等待24.246/26.350秒；完整段多数1.95–2秒，命令新鲜、请求方向稳定。两遍审计相同。这部分不是BeamNG必然需要的机械等待。 |
| 停稳后再串行比较多条车辆路径 | `exploration_cost.begin_exploration_comparison` 明确排除following/活动动作，要求speed≤.15，比较2–4个选项 | 原有图分数之外新增了一轮真实车辆路径成本选择，代价是停止状态下等待多次ComputePath。不是所有候选比较都无意义，但这份计算不能当作“车辆正在following”。 |
| 探索意图换主会触发执行取消/释放 | C++ `retire_exploration` → bridge `on_exploration_retired` → 未满足连续交接条件时reset → quarantine/停稳/新代际ACK | 同一任务中frontier已观测、候选重选或视觉意图变化，可以造成制动，而不仅是发现障碍。已有连续交接只覆盖部分前进单段情形，倒车、多段、未就绪回应等仍会停。 |
| 观察点提议没有完整车辆路径可达性证明 | `goal_standoff.publish_visible_step` 按可见bearing和局部栅格提议点，之后交给GraphNav/Smac | 提议点的净空和视线成立，不代表车辆能以合理转弯与朝向抵达。被拒后若再提相近点，会重复消耗规划与交接时间。这一层位于新增目标接近/执行适配，不能只归因记忆检索。 |

两轮v22命令空窗逐消息复核各重复两遍：

- 7.077秒：旧段结束/接入拒绝 → 多次候选规划与评分 → 选择交接 → 新FollowPath。
- 5.653秒与4.025秒：换向后的续段footprint拒绝 → 重规划失败 → 退出视觉接近 → GraphNav重选 → 新FollowPath。
- 4.188秒：新视觉意图取消旧动作 → 停车/释放确认 → 新规划与派发。

这些具体窗口中控制端没有新鲜速度命令属于上游执行链尚未提供下一条可执行路径，不能仅凭 `stale_command` 诊断网络断线。不得通过重发过期速度把等待隐藏成运动。

## 原核心也并非完全未动

参照与当前移植代码的行数（仅表示修改范围，不以行数判断好坏）：

| 文件 | 参照 | 当前 | 主要差异 |
|---|---:|---:|---|
| graphnav_planner/src/planner.cpp | 399 | 635 | 车辆/执行经验等边代价和过滤扩展 |
| graphnav_planner/src/planner_node.cpp | 168 | 605 | 自主探索目标、owner退休、车辆选项与选择ACK、失败退避 |
| graphnav_mapper/src/mapper.cpp | 771 | 1165 | 车辆几何、通行与图更新保护等扩展 |
| visual_navigation/wildos/nav.py | 638 | 1594 | 记忆、heading候选、目标引导和反馈等扩展 |
| visual_navigation/wildos/goalagnostic_scoring.py | 207 | 207 | 对照内容一致 |
| visual_navigation/utils/goal_navigator.py | 156 | 156 | 对照内容一致 |

因此准确说法是“保留了部分原始评分/建图规划基础，并扩展了探索策略，另写车辆执行闭环”，而不是“原文执行完全不变，只插入四类记忆”。记忆影响候选优先级和访问反馈；即使关闭记忆，当前的车辆规划、分段、2秒确认、动作交接仍在，不会自动恢复原执行方式。尚不能排除记忆反馈改变路线选择方差，不能反过来宣布记忆完全无关。

## 当前能作出的原因判断

最有证据支持的主因是：**原本用于图探索的路线/观察意图，被落实为大量受终点姿态约束的短车辆任务；这些任务又通过分段停车、换向确认、串行候选比较和意图退休事务交接。** 这同时解释了多换向与数秒指令空窗，并形成“折返→停稳→新图拒绝续段→重规划→再停”的循环。

ROS2回调排队确有反例，BeamNG原生RPC超过租约也有反例，但它们不足以解释这里逐次约2秒的软件等待与已对齐的串行规划空窗。不能把所有问题都推给仿真器。此前在同一BeamNG能流畅行驶，也应促使优先追查执行链改动，而非继续默认原方案/模型失败。

后续验证应以原先流畅轮次的确切控制配置为第一对照；区分“原评分+原目标交接+相同车体执行”与“仅启用记忆”的差别。先验证执行器保持连续运动、必要时更新剩余路径，意图更新不必然撤销安全运动；再验证记忆收益。恢复滚动目标原则不等于把每帧Pose直接转成取消/重发的NavigateToPose，也不等于移除车辆碰撞、新鲜度或真实停车保护。

## 验证边界与候选状态

- 本次是源码、配置与已有关闭日志对照，两次本机审计一致；未作新的ROS/DDS、服务器或整车试验。
- v23两轮已关闭：172932近停36.18%/净52.35m，173348近停42.14%/净57.21m，均未到达。首轮航向补充2次、第二轮0次；不能说该候选解决主导停顿，不推广/不commit。收尾核查均无项目进程/显示锁。
- 前一阶段计划的0.3秒换向配置修改、构建与部署被自动审批额度限制拦截，未执行。当前physical launch仍未覆盖2秒默认。工作区此前已新增对应控制测试，改前两遍均2fail/0error；它们是待实施参数候选的反例，不能说当前全工作区回归已经通过。本次用户转向源码对照后未继续改这些代码。
- 本机 `nav2_ackermann_control.yaml` 当前名义规划半径12米、控制/执行8米；历史冻结现场状态报告8米，须独立核对运行配置，不能把工作区文件直接当现场参数。当前对照不足以将这一差异认定为某轮退化根因。

## 持续探索改造的接口边界（用户后续询问）

方向可行：保留WildOS的图与视觉评分，将执行管理改为长生命周期的探索会话。探索frontier/记忆目的地是可更新的规划意图；在同一搜索会话中，目标更新不自动等价于取消当前安全运动。

建议的数据流是：图/视觉/记忆给出目的区域与观察需求 → 后台规划生成车辆可执行的滚动路径 → 原子接纳并接续已检查的新路径 → 控制器持续跟踪、持续产生带真实源时间的新命令。新路径尚未就绪时，只有当前路径仍安全且具有足够可执行余量才继续；余量不足、障碍或控制失效则停车。不得无限延长旧任务，不重放旧速度。

需要成组更改的接口：

1. GraphNav的探索owner管理与GraphPathBridge的执行所有权解耦；同一query内frontier退休可触发重选与后台规划，不能无条件触发物理reset。query切换、用户停止与真正控制故障仍明确撤销会话。
2. 将规划/候选比较与控制状态分开；可在行驶期间准备新路径，结果携带其起姿、地图与意图身份，接续时按当前实车位置与最新地图重新准入。原停稳多候选比较假设不能直接搬到运动中，旧起点结果不能直接执行。
3. 普通探索路点作为经过约束或区域目标；只有最终确认观察、必要姿态或换向点才要求到姿停车。前进片段边界使用连续接续，不能把每次更新都变为“成功终态→停车→再起一个任务”。
4. **当前RPP硬契约必须协调**：`ackermann_rpp/src/controller.cpp::setPlan` 明确拒绝跨换向的多方向路径，报 `Ackermann path must be split at its reversing cusp`，并锁定单段方向。因而不能只删除bridge的motion_queue后把整条多方向路径塞给现有RPP。可先保留必要换向处分段，把普通同方向滚动更新与前沿变化改为不中断运动的接续；如改成控制器内部处理换向，必须同时重做方向保持、cusp到位和真实停车契约。
5. 保留转弯半径、完整车身碰撞、地图与消息新鲜度、车端租约及用户停止；必要倒车仍先停稳。持续探索的验收是减少不必要的停车，不要求实车永不停止。

现有 `ContinuousPath.maybe_extend_path` 仅适用单队列/单方向前进、余程与新图进展满足条件的情况，是局部接续能力，尚不是上述会话机制。不能通过改名或把当前回调放进永久循环宣称替换完成。

实施验证须分离正常直行更新、frontier更换、新目标后台路径失败、延迟结果、障碍出现、必要换向、用户停止与同进程记忆目标切换；关键接口至少各两遍复现和回归，再用冻结旧版与新机制整车对照检查近停、换向、命令空窗、探索有效进展。不以路径发布或following标签代替车辆实际运动。本段为接口设计分析，未修改实现或作现场验收。
