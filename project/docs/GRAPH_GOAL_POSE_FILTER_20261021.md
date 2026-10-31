# GraphNav 端点车身可行性过滤（2026-10-21）

## 现象

- `trial_20261021_093254`：120 秒内近停 74.772 秒（62.31%），最长连续 17.543 秒；日志显示串行 `ComputePath` 候选连续返回 `empty_path`，单次约 1.7 秒。
- `trial_20261021_094550`：42.46 秒轮次近停 26.30 秒；0–9.053 秒和 18.618–27.810 秒两段长停前都出现 2 个连续 `empty_path` 比较候选。

## 根因（有现场 bag 证据）

GraphNav 提供的候选/路线样本按中心格检查为自由，但真实 SBR 车体（front 3.41 m、rear 1.06 m、半宽 1.06 m）按候选航向展开后与全局代价地图的 100（lethal）格重叠。

- 094550 比较选项 1/2：中心格 39/99，车身最大 100；1.0 m 邻域内没有任何自由车身姿态。
- 094550 比较选项 3/4：车身自由，规划成功。
- 093254 比较选项 1/2/4：1.0 m 内无自由姿态；选项 3 自由。
- 093254 普通路线样本 0–4：1.0 m 内无自由姿态；样本 5 在 1.0 m 内存在自由姿态。规划器随后从样本 5 生成了 62 位姿路径。

因此不能按“精确端点碰撞”直接删除：Nav2 规划器允许在 1.0 m 容差内提前停车，样本 5 就是可用反例。

## 修改

`nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`

- 新增 `vehicle_pose_feasible(position, heading, tolerance=1.0)`：先检查精确姿态；若碰撞，在 1.0 m、0.2 m 步长邻域搜索是否存在自由车身姿态。
- 在路线候选循环中逐个检查路线切向候选和 approach-bearing 候选；两者独立过滤，避免因切向姿态被拒而误删可行的 bearing 变体。
- memory 选择不预过滤（`memory_selecting` 或 `memory_goal` 存在时保留），保证被阻塞记忆视角仍进入原有拒绝/切换流程。
- 观察/确认端点候选保留，不改变其既有交接和恢复路径。

`nav2_adapter/wildos_nav2_adapter/exploration_cost.py`

- `vehicle_options` 比较前过滤 `vehicle_pose_feasible` 为假的选项；过滤后少于 2 项则放弃比较并回退普通候选路径。
- 保留 GraphNav 的 `exploration_id`、benefit、route endpoint 和选择 ACK 契约。

未改变：规划总预算、Smac/碰撞/曲率检查、地图/姿态/新鲜度门槛、控制租约与 fail-closed 语义。

## 验证

- 本机 `tools/offline_callback_check.py`：**640 项通过**（ROS 接口替身，无 DDS/仿真）。
  - 新增 `test_footprint_colliding_frontier_is_not_planned_or_selected`
  - 新增 `test_footprint_check_leaves_fewer_than_two_options_on_normal_path`
  - 新增 `test_route_candidates_that_collide_with_vehicle_body_are_skipped`
  - 新增 `test_clear_pose_within_goal_tolerance_keeps_colliding_endpoint`
- 记录 bag 只读复核（`audit_goal_pose_filter.py` → `goal_pose_filter_audit.json`）：094550 6 个候选约 50.2 ms 完成判定；093254 6 个路线样本仅样本 2/5 保留（样本 5 为实际成功样本，样本 2 仍被规划器拒绝，属保守过滤的假阳性）。
- 服务器隔离 v6（复制 v5 candidate，仅替换本次四文件，正式安装未动）：构建成功；**实际 ROS 78 项、source control 13 项、服务器离线回调 640 项通过**；418 成员冻结清单校验 0 失败。源码 SHA：`graph_path_bridge.py` `b4c29fab406aa46fbd64516bc7b3d745910849d6076bb3175c1ca47531608a93`，`exploration_cost.py` `3bedccc585f15e0b36294420c1d9f2d6e8f08554e13661133cd79c7dbe759e8b`。
- v6 现场 `trial_20261021_101912`（120 s）：120.87 s 正常超时，`errors=[]`，无 lease 退出；**近停 44.11 s / 119.75 s = 36.83%**，最长连续 **2.114 s**（v5 093254 为 74.772 s / 62.31%、最长 17.543 s）；净位移 40.559 m；窗口内 7 次 `empty_path`（全轮 9 次），v5 093254 全轮 30 次。图采样/frontier 不同，不是严格 A/B，不能把全部改善因果归于过滤。
- v6 现场 `trial_20261021_102500`（180 s）：181.62 s 正常超时，`errors=[]`，无 lease 退出；近停 73.74 s / 179.80 s = 41.01%，最长 2.158 s；净位移仅 15.083 m，30 次 `empty_path`，未进入 visible target approach，结束后 `planning_only`。两轮 v6 均无 >2.2 s 长停，但探索进展差异大。
- 102500 的 30 次 `empty_path` 只对应 19 个唯一目标，最高重复 4 次；说明同一图版本内仍会重复尝试无效端点。下一步候选是有界失败目标缓存或 GraphNav 端点可达性接口，但必须先证明地图/epoch 变化会使缓存失效；本次未实施。
- 101912 末段 115.9 s 进入 `visible_target_reobserving`，随后 `planning_only`；**两轮均未到达、未确认**。跑后无项目进程/显示锁，磁盘 22G。完整绿色房屋搜索和 20 轮统计仍未执行。
- v6 证据本机 `test_artifacts/execution_continuity_repair_20261021/v6/`：101912 trial 包 SHA `26b344eec3aeb5ce07fd5e44dde19b5ea0c26970b36c737f688b233e120d77c3`，102500 trial 包 SHA `7ed445b9cdf848ea3b2051c7e9a75533c115f6f6d94a054d24afca207ff593ee`，组件包 SHA `332d36091add1d41b52c7013870c6f5817d73c90863beaa78462279515b19b17`。
- 后续 v7（失败目标缓存）/v8（可见观察 2帧/15s）180s 重测仍未到达；v8 105905 在 115.58s 距房屋 1.86m，但房屋位于车体左侧约 88.6°，前相机不可见；约150s 的可见目标对应另一绿色物体。结论见 `docs/RESIDUAL_STOP_CAUSES_20261021.md` 的 v7/v8 节。长停已不是首要瓶颈；当前需解决侧向观察/实例关联。

## 复现入口

- 服务器隔离候选：`/home/ly/execution_continuity_repair_20261021/v6/`（v5 正式隔离候选保持 SHA `5305099b...` 未变）。
- 构建与组件测试：`bash /home/ly/execution_continuity_repair_20261021/v6/build.sh`（实际 ROS 78 项、source control 13 项）。
- 服务器离线回调：在 v6/candidate 下运行 `tools/offline_callback_check.py`（640 项）。
- 现场入口：`cd /home/ly/execution_continuity_repair_20261021/v6 && bash run_full.sh`；`full/config.json` 的 `goal_timeout_s` 当前为 180（120s 轮为 120）。两轮均无录像。

## 另一条未闭合链：控制租约

094550 约 67.7 秒：

| 时刻 | 事件 |
|---|---|
| 67.7s | `poll_tick` 812.656 ms、`control_tick` 813.702 ms 同时阻塞 |
| 67.8s | `telemetry_age=.864s`，bridge 报 `native_telemetry_stale` 并取消 FollowPath |
| 67.9s | control RPC 返回 `Vehicle control lease expired or closing`（0.8 s 租约） |
| 68.0s | `actuator_ok=false`，Native 健康保护退出会话 |

该轮控制 RPC 中位 6.61 ms、p99 51.48 ms，813 ms 是单点异常。引擎侧阻塞来源尚未闭合；未通过延长租约或自动重连来规避，保留 fail-closed 保护。

## 证据

- `test_artifacts/offline_navigation/callback_report.json`
- `test_artifacts/offline_navigation/offline_callbacks_test_exploration_cost_ros.log`
- `test_artifacts/offline_navigation/offline_callbacks_test_graph_path_bridge_routing.log`
- `test_artifacts/execution_continuity_repair_20261021/audit_goal_pose_filter.py`
- `test_artifacts/execution_continuity_repair_20261021/goal_pose_filter_audit.json`
- `test_artifacts/execution_continuity_repair_20261021/v5/trial_20261021_093254/`
- `test_artifacts/execution_continuity_repair_20261021/trial_20261021_094550/`
