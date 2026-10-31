# 有界失败目标缓存（2026-10-21）

## 证据

`trial_20261021_102500`（v6，180s）中 30 次 `ComputePath` `empty_path`：

- 只有 19 个唯一目标，最高重复 4 次；
- 按 GraphNav `exploration_id` 分组，同一探索身份内重复 12 次；
- 例如 `720.36,-13.19` 在同一 `exploration_id` 内失败 4 次，`734.56,-27.04` 失败 4 次。

对失败目标做过代价地图连通性检查：即使把障碍膨胀 2m，多数目标仍与起点连通。因此这不是简单的“目标在另一个自由空间连通分量”问题，不能用连通性 BFS 预检替代规划；失败来自真实车体/曲率/路径几何。

## 策略

在 `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py` 增加有界失败目标缓存：

- 缓存键：目标位置 0.5m 桶、朝向量 0.5rad 桶、起点位置 2m 桶；
- 作用域：当前 GraphNav `exploration_id`；
- 只记录 `empty_path`（规划器无路径），不记录传输/协议/碰撞拒绝；
- 新 `exploration_id` 或车辆移动超过 2m 时自动重试；
- 最多 128 项，超出按插入顺序淘汰最旧项；
- memory / 确认 / 观察候选不预过滤，保留其自身拒绝/切换流程；
- `vehicle_options` 比较同样过滤缓存目标；过滤后少于 2 项则回退普通候选路径。

这不是固定时间黑名单，也不是永久禁用；同一探索身份和相近起点下才跳过。

## 验证

- 本机 `tools/offline_callback_check.py`：**643 项通过**。
  - 新增 `test_failed_goal_cache_is_scoped_to_exploration_and_start_region`
  - 新增 `test_route_candidate_with_cached_empty_path_is_skipped`
  - 新增 `test_cached_failed_frontier_is_not_replanned`
- 预计 102500 场景少 12 次串行 `empty_path`（约 20s 规划等待），但尚未在服务器/现场验证。
- 未改变规划总预算、碰撞/曲率/新鲜度检查、控制租约和 fail-closed。

## 边界

- 同一 `exploration_id`、相同 2m 起点区域内，代价地图发生局部变化时缓存仍可能跳过，直到 GraphNav 更换探索身份或车辆移动超过 2m。该延迟有界于当前探索目标，但不是地图内容级失效；若后续证明会造成漏检，再增加地图证据键，不能用固定时间替代。
- 该策略只减少重复无效候选，不解决 GraphNav 端点本身的几何可行性；完整到达仍需现场验收。
