# 连续起点 connector 诊断与反向首段惩罚（2026-10-21）

目的：确认 `183339` 等轮次里"目标在南、前段向北"的反向首段，究竟来自
直接 Reeds-Shepp 连接（`direct`）、连续起点 connector，还是 Smac 后缀。
在拿到这个归属之前不改规划行为——用 `first_section_away` 直接拒绝会误伤正常机动
（回放显示 130635 也有 36% 计划首段后退）。

## 0. 归属结论（205052 / 205615 / 210104 现场日志）

- 205052：40 次选路中 26 次为 connector + 后缀，其中绝大多数 `first_cos≈-0.9…-1.0`；
  14 次直接连接基本 `first_cos≈+0.999`。最坏一条 `cost=76.18, cusps=10, first_cos=-0.954`。
- 205615：25 次选择，23 次 connector，16 次 `first_cos<0`，最大 4 段。
- 210104：47 次选择，37 次 connector，21 次 `first_cos<0`，最大 9 段。
- **反向首段来自 connector，不是 Smac 后缀。** 直接连接更便宜却未被选，是因为规划器
  以风险（积分代价）为先；connector 路线只是"更安全的绕行"。因此修复不能简单拒绝反向，
  只能在同等安全的候选间调整排序。

## 0.1 实验改动：`reverse_start_penalty`

- `planner.cpp` 新增参数 `reverse_start_penalty`（默认 0，保持原有排序；范围 0–100）。
  `first_cos < -0.5` 的候选（connector 或 direct）加该值后再参与 `maneuverCost` 排序；
  碰撞、曲率、风险检查不变。
- 日志 `connector_candidate` 增加 `base` 与 `penalty`。
- 现场 A/B：`nav2_ackermann_control.yaml` 设 `reverse_start_penalty: 3.0`（一个换向当量）。
  跑完对比 `first_cos<0` 比例、近停、换挡与位移；无效则改回 0 并回退库。
- 构建：Smac 重新编译，27 项 gtest 通过；库 SHA `89ae42c4…` 已写入 `config.json`。

### 0.1.1 现场 A/B 结果：3.0 未获支持，已改回 0

- `210104`（penalty 0）：47 次选路、37 次 connector、21 次 `first_cos<0`（56.8%），
  近停 39.3%、12 次换向、净 41.5m，`visible_target_approach` 仅 1 次。
- `210759`（penalty 3.0）：62 次选路、41 次 connector、22 次 `first_cos<0`（53.7%），
  近停 48.2%、14 次换向、净 59.8m（终点 (734.9,-67.9)），`visible_target_approach` 11 次。
  日志确认参数生效：158 个候选 `penalty=3.000`，192 个 `penalty=0.000`；存在
  正向候选以 17.143 < 反向 14.581+3.000=17.581 胜出的实例。
- 重跑 `211315`（同为 3.0、同一起点 (735.0,-9.9)）：14.6m 后返回起点附近
  (732.9,-7.6)，无 tile、无可见接近，`errors=[]`、正常收尾。
- 判读：反向比例只从 56.8% 降到 53.7%，近停反而恶化；59.8m 与 11 次接近在重跑中
  未复现，属于路线采样差异而非惩罚效果。**结论：3.0 无可重复收益，已把
  `nav2_ackermann_control.yaml` 改回 `reverse_start_penalty: 0.0`**（候选、安装
  share 与 frozen manifest 同步，库仍为含参数默认 0 的版本，无需重编译）。
  参数与日志保留，供后续在有同图重复证据时再实验。

## 0.2 期间发现并修复的两个崩溃

- `try_candidate` 候选耗尽时 `list(self.route_endpoint)` 在 endpoint 为 `None` 时抛
  `TypeError`（205052 收尾时 bridge 死亡、停车门失败）。已加 None 保护。
- 迟到的 ComputePath 结果在 `route` 已清空后调用 `current_route_points`，触发
  `None.poses`（205615）。已让 `current_route_points` 在无 route 时返回空列表，
  并在 `on_plan_result` 丢弃无路线结果（`route_cleared`）。
- 回归测试：两项新增；210104 轮 bridge 全程存活，7 个节点均以 `-2` 正常收尾。

## 1. 录像：记录完整规划路径

`tools/wildos_console.py` 的 rosbag 话题表加入 `/wildos/full_planned_path`
（bridge 已在 `full_path_pub` 上发布每条规划结果）。此前只录分段后的
`/wildos/planned_path`，无法切开 connector 与后缀。

- 测试：`tools/test_wildos_console.py` 的录制话题断言加入该话题；本机 28 项通过。
- 不改任何阈值或行为，只多录一个 topic（约每条规划 1 条消息）。

## 2. 规划器：输出归属信息（仅日志）

`ackermann_smac/src/planner.cpp` 新增两个只读 lambda 和三条 INFO 日志：

- `cusp_count(path)`：按相邻非零边的反向检测统计换向次数（与 `maneuverCost` 同口径）。
- `first_cosine(path)`：路径第一条非零边相对"起点→目标"方向的余弦；
  `< 0` 即首段背离目标。
- `connector_candidate: cost=… poses=… connector_poses=… cusps=… first_cos=…`
  每个可行候选一行；`connector_poses` 是 connector 前缀长度。
- `continuous_goal_selected: … cusps=… first_cos=…`（直接连接被选中时）。
- `continuous_complete_selected: … cusps=… first_cos=…`（connector + 后缀被选中时）。

判读方式：若 `connector_candidate` 的 `first_cos<0` 且被选中，反向来自 connector；
若 `first_cos≥0` 而最终路径反向，则来自 Smac 后缀（或直接连接）。
配合 `/wildos/full_planned_path` 可逐点复核。

## 3. 状态与部署步骤（尚未构建）

- 本机已改：`planner.cpp` 日志、`wildos_console.py` 话题、对应测试；
  括号/注释平衡检查通过，但**本机无 ROS/编译器，未编译、未运行**。
- 服务器恢复后：
  1. 把本机 `ackermann_smac/src/planner.cpp` 同步到 `v10/candidate/ackermann_smac/`；
  2. colcon 构建 ackermann_smac，运行原有 gtest（26 项）；
  3. 用新库 SHA 更新 `v10/full/config.json` 的 `library_sha256`（否则驱动拒绝启动）；
  4. 同步 `tools/wildos_console.py` 到候选，`bash -n`/语法检查；
  5. `prestart.py` 通过后跑一轮 180s，检查日志中的 `first_cos` 与 full path。
- 验证边界：这些日志只证明"反向首段来自哪一段"，不证明修复有效；
  修复方案要等归属结果再定。
