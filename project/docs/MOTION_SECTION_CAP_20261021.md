# 探索路径多段（换向）上限（2026-10-21）

范围：针对 `181516` 在起点 9 m 内反复换向 180 s 的现象，限制单条探索路径的 motion section 数。
不改碰撞、速度、Smac/RPP 参数、地图、目标确认或到达判据。

## 1. 现场证据

- `trial_20261021_181516`：180.8 s 超时；native 近停 87.0 s（47.7%），换挡 24 次（12 组往返），
  12 次 `maneuver_done`，距起点最远 9.0 m，累计 59.8 m 全在起点区域。
- `/wildos/planned_path` 解码：85.1 s 起的一条路径有 **10 个 motion section**，逐段执行；
  各段长度 0.44–1.5 m，方向在 (736,-6.4) 与 (735.6,-7.1) 之间反复。
- 同一轮 planner 日志：52 次 `continuous_start`，每次都 `timeout=1`；远路线候选多次 `empty_path`。
- 对照 `130635`：起点逃逸路径最多 **4 段**，随后正常行驶 50 m。
- 结论：连续起点 connector + Smac 后缀在受限区域会返回多段（多 cusp）路径；
  逐段停车/换挡把一次无进展的挪动放大成整轮停走。不是碰撞检查或控制器抖动。

## 2. 修复

`nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`：

- 新增参数 `max_motion_sections`（默认 6，范围 1–64）。
- `execute_checked_path` 在切分 motion section 后、派发前检查：若属于
  真实车体（非 ideal_point）、探索路径（`expected is None`、非 memory、非 recovering），
  且段数超过上限，则记录 `Rejecting exploration path with N motion sections`，
  以 `excessive_motion_sections` 调用既有 `candidate_failed`。
- 该候选失败后沿用原有流程：尝试下一个候选；全部失败则 `block_current_route` +
  `reject_exploration_route`，由 GraphNav 重新选路/前沿。
- 记忆目标、物体目标、可见目标接近、恢复逃逸路径不受此上限约束；
  所有碰撞、曲率、新鲜度、动作所有权检查保持原样。

## 3. 验证

- 修复前反例：新增的 `test_excessive_motion_sections_rejects_the_candidate` 对旧候选代码
  失败（`candidate_failed` 调用 0 次），其余两项通过。
- 修复后：3 项新增测试通过（8 段拒绝、4 段放行、objects/memory 不受限）。
- `test_graph_path_bridge_routing.py` 共 183 项：182 通过；
  `test_side_lookback_turns_back_once_near_large_structure` 失败。该失败在**回退本修复的同一副本**上
  仍失败（同一断言），属既有夹具问题，不归因于本次改动。
- 服务器离线回调整体 651 项：650 通过，唯一失败即上述 lookback 项。
- 候选与安装的 `graph_path_bridge.py` 字节一致；418 项冻结清单校验 0 不符
  （新成员哈希 `1bd2b48a…`，测试文件 `a2d0e63e…`）。

## 4. 连续空路径上限（同一轮补充）

- 证据：`183339` 中 epoch 4 一次规划轮内连续 **19 次 `empty_path`**（日志中共 28 次失败），
  每次约 1.73 s；成功路径耗时中位 1.20 s、最长 10.79 s。失败候选的目标姿态经实测，
  在 route 切线/接近方位附近与 4.47×2.12 m 车身重叠（`footprint_blockers` 为真），
  规划器无法终止；逐候选重试只是空等。
- 修复：新增 `max_consecutive_empty_paths`（默认 4，1–32）。同一 epoch 内连续
  `empty_path` 达到上限时，探索路径立即 `block_current_route` + `reject_exploration_route`
  并发布 `no_vehicle_path`，不再把剩余候选逐个送进规划器；成功路径或新 epoch 清零计数。
  物体目标、记忆目标、可见接近不受该上限影响；所有原有检查保留。
- 修复前反例：3 项新测试在旧候选上因缺少参数直接报错；修复后 6 项新测试全部通过。
  路由套件 186 项、离线回调 654 项，均只有既有的
  `test_side_lookback_turns_back_once_near_large_structure` 失败（回退本改动仍失败）。

## 5. 离线估算：连空路径上限能省多少

用 `183339` 已记录的失败序列（同一 epoch 4 连续 19 次 `empty_path`，每次实测约 1.73 s）：

- 改前：19 × 1.73 ≈ **32.9 s** 停驶等待；
- 改后（上限 4）：4 × 1.73 ≈ **6.9 s**，该轮最多节省约 **26 s**（占 180 s 预算 14%）。
- epoch 0/2 各只有 3 次连续失败，不会触发上限，合法路线不受影响。

该估算只说明等待时间可被截断，不证明省下的 26 s 足以到达房屋。

## 6. 验证边界

- 本次只证明“超过 6 段的探索路径不再执行”，未证明 180 s 内可完成任务；
  现场效果以下一轮试跑统计为准。
- 上限 6 依据现有两轮证据（130635 ≤4 段可逃逸；181516 10 段循环），
  不是对所有地形都经过验证的通用常量。
- 未提交 Git。

## 7. 后继离线修复：比较与执行准入一致

现有六段限制在比较阶段漏用，可能先选中超限路径再在派发时拒绝。
已补普通航向和frontier候选比较的同规则检查；四个改前失败反例改后通过。
相关回调382项中仅既有lookback失败，改前副本亦复现。没有部署/仿真/commit。
详 `docs/STOP_GO_CANDIDATE_ADMISSION_20261021.md`，不能据此宣布走停消除。
