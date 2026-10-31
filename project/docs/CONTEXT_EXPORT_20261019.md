# 项目上下文导出（2026-10-19 18:20）

本文件用于把当前代码、运行版本、已验证边界和下一步工作一次性导出。它不替代 `docs/PROJECT_MEMORY.md`、各专项审计文档和 `test_artifacts/` 原始证据；出现冲突时以原始证据和最新专项记录为准。导出本身不是修复，也不是验收。

## 1. 项目目标与硬约束

- 目标：在 BeamNG/WildOS 中完成同进程、轻量视觉记忆引导的单目标视觉到达；一次地图加载，不要求原路返回。
- 当前完整验收标准：20 轮中至少 16 轮成功后才做多目标；视觉到达与真实车体执行分别核验。
- 不录制视频（用户 2026-10-18 取消默认失败视频回传）；保留传感器、规划、控制、时间戳日志和判定到达所需的少量图像。
- 不得把掩码、路径、日志到达、组件单测通过或 DDS 接口通过单独当作完整实验成功。
- 不保存密码或 API 密钥。

## 2. 代码地图

| 路径 | 作用 |
|---|---|
| `wildos_port/repro_check/src/nebula2-wildos/visual_navigation/` | 视觉经验与检索：ExecutionExperienceStore、GraphObservationStore、ConfirmedViewpointCache、PassiveLandmarkIndex、nav.py |
| `nav2_adapter/wildos_nav2_adapter/` | GraphPathBridge、RecallCandidatePolicy、visible_approach、execution_recovery、navigation_transactions、pointcloud、path_planner_bridge、cmd_vel_to_control |
| `wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/` | 占据/地形/可通行性建图，`terrain_metrics.hpp`、`mapper.cpp` |
| `wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/` | 图上选路、frontier、目标释放/清理、destination_fence |
| `ackermann_smac/` | Smac 混合 A* 车辆规划、连续起步连接器、path_contract |
| `ackermann_rpp/` | RPP 轨迹跟踪控制 |
| `official_adapter/beamng_wildos_adapter/` | BeamNG 官方接口：传感器、车体控制、mapping、goal_standoff |
| `tools/` | 服务器预检、实验入口、离线检查、证据核验 |
| `test_artifacts/` | 离线/服务器证据、失败反例、封包 |

数据流：相机/位姿/LiDAR/模型特征 → PassiveLandmarkIndex + GraphObservationStore → 新 query 检索 ConfirmedViewpointCache/历史区域 → RecallCandidatePolicy 比较 → GraphPathBridge 规划与执行交接 → 当前观测重新定位/确认/接近 → 执行反馈更新方向先验。索引命中、路径生成、动作成功都不等于视觉到达。

组件命名以 `docs/VISUAL_RECALL_COMPONENTS.md` 为准。

## 3. 当前版本与运行状态

### 本机仓库

- 分支 `main`，HEAD `cccab30`（`fix: correct physical 360 lidar coverage and preserve scan controls`）。
- 工作区有大量未提交改动：约 46 个 modified + 19 个 staged，另有大量未跟踪目录/文件。它们不是本轮 candidate6，且未 commit；继续工作前不要 `git checkout/reset/clean` 覆盖。
- 本机未持有服务器候选6的完整安装与原始大数据库。

### 服务器

- 主机：`192.168.31.91`，用户 `ly`，密钥 `~/.ssh/wildos_192_168_31_91`（使用 `IdentitiesOnly=yes`）。2026-10-19 18:17 连通，检查时无残留仿真/ROS 进程。
- 6153 冻结基线：bundle SHA256 `6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b`，404 成员；正式安装与归档保持不替换。
- 当前最新候选6：`/home/ly/physical_evidence_repair_20261019/candidate6`。源码在 `candidate/`，构建在 `build/`、安装 `install/`，测试/证据日志在同一目录。
- candidate6 源码身份：404 原成员叠加 35 项增量文件后共 414 成员逐 SHA 核验；增量 ZIP SHA256 `356c1b546f76d6d49312d4eedc641c30ff65b70949a057da99a9333509616778`。
- candidate6 实际库：Mapper `0ef78c09…`、Planner `11f92359…`；旧 6153 现场 GraphNav `db52f00e…`、Smac `fc149c80…`、RPP `d2fd9cee…` 仍保留。
- candidate6 结论：组件和实际消息链通过，**未部署、未整车、未 20 轮、未 commit**。

## 4. 验证状态矩阵

| 项目 | 状态 | 证据入口 |
|---|---|---|
| 重置预算合并 | 组件通过，整车待验收 | `docs/RESET_INTENT_REPAIR_20261019.md` |
| 视觉/物理恢复归属 | 52 事务/562 离线 + 52 真实 ROS 回调通过，整车上机待验收 | `docs/RECOVERY_VISUAL_HANDOFF_REPAIR_20261019.md` |
| release_goal / clear_goal 语义 | 35 实际 C++、60 ROS 回调、15 阶段 DDS 通过，整车待验收 | `docs/GOAL_RELEASE_HANDOFF_REPAIR_20261019.md` |
| 旧目标时间边界 fence | 与上项同批通过；依赖连续一致 ROS 时钟 | 同上 |
| 记忆评分/方向负证据 | 51 记忆 + 19 地面 + 5 阶段 DDS 决策通过，完整搜索增益未验 | `docs/RECALL_GROUND_CONTRACT_REPAIR_20261019.md` |
| 地面拟合 XY/Z 配对 | 候选已修并重放 22 帧逐格一致，整车未验 | 同上 |
| 可通行证据/有向图 | candidate6 实际 C++ 61 项 + 7 阶段实际 DDS 通过，整车未验 | `docs/PHYSICAL_EVIDENCE_CONTRACT_REPAIR_20261019.md` |
| 低表面信息丢失 | **未修**；真实受困邻域 raw→adapter→marking 仍为 0 | `docs/CONTACT_FRAME_CLOCK_AUDIT_20261019.md` |
| terrain 台阶/粗糙度表示 | **未修**；17cm 阶跃只报 8.5cm | `docs/TERRAIN_CONTRACT_AUDIT_20261019.md` |
| 3D 净空/射线清除 | **未修**；二维清除不证明车体扫掠体积为空 | `docs/OPEN_ISSUES_20261019.md` |
| 整体无进展退出 | **未修** | `docs/WHOLE_FLOW_AUDIT_20261017.md` F11/F12 |
| 采集身份/epoch | **未修**；header 新鲜 ≠ 采集新鲜 | `docs/CONTACT_FRAME_CLOCK_AUDIT_20261019.md` |
| Native 间歇延迟 | 根因开放 | `docs/NATIVE_DELAY_INVESTIGATION_20261018.md` |
| 车辆动态闭环、完整搜索、20 轮 | **未验收** | `docs/OPEN_ISSUES_20261019.md` |

## 5. 未关闭问题（按优先级）

来源：`docs/OPEN_ISSUES_20261019.md`（保留修复前事实）＋最新记录。

1. **低表面/近地障碍未进入 marking**：084254 受困邻域 raw 368 → adapter 116 → 车体剔除后 116 → body-z 高度筛选后 0，marking 0；约 0.17m 地面残差仍被 0.25m 地面余量删除。实际接触已核验，碰撞资产身份未知。
2. **terrain 台阶/粗糙度与真实通过能力不一致**：`terrain_metrics` 以拟合平面残差范围当台阶；17cm 合成阶跃只报 8.5cm，默认 10cm 上限下通过。0.5m 采样无法区分阶跃与短坡。接触邻域节点 feasible、近接触边 confidence 0；Planner 读取字段但低置信度不硬拒绝。
3. **3D 净空/射线清除**：高射线可从低障碍上方通过，二维 free 不代表车体体积空闲；地形/语义约束未覆盖 Smac 整条绕行路径。
4. **整体无进展退出**：局部 success 往返、持续 frontier 缺失缺有界策略。
5. **采集身份与时钟**：LiDAR 缓存载荷可重复而 header 更新；源时钟重启、adapter/速度环 epoch 一致性仍有反例。
6. **记忆长会话容量、RecallCandidatePolicy 换向/尾段成本、无 mask 负证据判据**等旧审计项待复核。
7. **Native 间歇延迟、动态换挡/短段制动/cusp、完整搜索与记忆增益、20 轮验收**。

## 6. 下一步执行建议

按记录中的优先顺序：

1. 以冻结证据先复现低表面遗漏与台阶低估（只读，不改生产）。
2. 设计统一契约：地面形状（坡/台阶/粗糙度/观测尺度）＋车体 3D 净空/扫掠体积 ＋ 低表面障碍标记 ＋ 清除；不得只调 `min_height` 或把 `step` 改成相邻格差。
3. 在独立候选目录实现最小 C++/Python 修复，先跑组件和实际消息链，再做整车受控短段；失败反例和失败候选保留。
4. 通过后再谈完整搜索、记忆增益与 20 轮。所有测试通过前不 commit；deploy/正式安装/6153 包不替换。

## 7. 复现入口

```bash
# 服务器连接
ssh -i ~/.ssh/wildos_192_168_31_91 -o IdentitiesOnly=yes ly@192.168.31.91

# candidate6 组件结果
cat /home/ly/physical_evidence_repair_20261019/candidate6/result.json
cat /home/ly/physical_evidence_repair_20261019/candidate6/mapper_tests.log
cat /home/ly/physical_evidence_repair_20261019/candidate6/planner_tests.log
cat /home/ly/physical_evidence_repair_20261019/candidate6/chain_v2.log

# 历史受困原始库（只读回放，勿改）
# /home/ly/... 084254 原 DB，SHA 0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc
```

## 8. 工作约定

- 每次改动先检查全链路影响：上下游输入契约、目标/路径状态归属、计时器与取消权限、旧消息/异步响应、配置与部署依赖。
- 先保留修复前反例，再以最小变更验证正常路径和受影响的失败/交接路径。
- 组件通过、实际消息链通过、整车通过分别记录；离线通过不能替代现场验收。
- 新故障/修复同步更新 `docs/PROJECT_MEMORY.md`，写清原因、代码位置、验证边界和证据入口。
- 测试失败、未执行或必要验收待验证时不 commit；通过后再提交并提供 commit ID。
