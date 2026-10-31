# 项目上下文导出（2026-10-21 晚）

本文件导出本轮会话的工作状态，供另一个终端/下一次会话直接接手。它不替代
`docs/PROJECT_MEMORY.md`、专项文档和 `test_artifacts/` 原始证据；冲突时以原始证据为准。
导出本身不是修复，也不是验收。

## 1. 当前目标与状态

- 用户目标：**持续修复 bug，使车辆在 180 秒内流畅完成"绿色住宅"任务**；不做 20 轮统计。
- 当前结论：**180 秒内完成仍未通过**。最好一轮 130635 在 162.74 s 距房屋 2.72 m，
  其余轮次在起点附近绕圈（9–11 m）。所有现场轮次超时。
- 服务器当前不可达（SSH 8422 端口超时）；最后一轮"连空路径修复"试验结果未知。

## 2. 现场轮次与指标（native_samples 复算）

| 轮次 | 近停 | 换挡 | 距起点最远 | 结果 | 备注 |
|---|---:|---:|---:|---|---|
| 130635 | 34.0% (62.6 s) | 14 | 51.1 m | 超时 | 最近房屋 2.72 m/162.74 s |
| 181516 | 47.7% (87.0 s) | 24 | 9.0 m | 超时 | 10 段路径在 1 m 内来回 |
| 183339 | 54.8% (100.5 s) | 18 | 10.8 m | 超时 | epoch 4 连续 19 次 empty_path |

已量化的停走构成：换挡等待（每次 2.0 s）、连续 `empty_path`（每次约 1.73 s）、
多段路径（0.44–1.5 m/段，逐段停车换向）。横向加速度峰值仅 0.28 m/s²，不是控制抖动。

## 3. 已确认的根因层次

1. **规划器层（最深）**：`ackermann_smac` 对每个请求都跑连续起点 connector
   （`183339` 52 次 `continuous_start` 全部 `timeout=1`）；所有弯曲段半径恰好 12.0 m
   （规划器最小半径），受限区域只能多点换向；目标在南时前几段却向北
   （43s→(735.2,-3.2)、74.8s→(735.9,+0.2)）。
2. **路线层**：图路线与当前代价地图不一致（路线段 (723.8,-11.7)→(721,-18) 有 6 个
   lethal 格）；失败候选的目标姿态与 4.47×2.12 m 车身重叠，Smac 只能返回 `empty_path`。
3. **执行层（放大器）**：每个 cusp 段单独 FollowPath：停车→换挡→校验→重规划；
   0.14–0.18 m 姿态误差或地图新标记就让下一段失败，重规划又生成反向首段。

已排除：控制抖动、Native/DDS 延迟（这些轮 source rejection 为空）、换挡 2 s（非因果）、
检测模型（未走到 mask/定位）。

## 4. 本轮已实施改动（全部未提交 Git）

| 文件 | 改动 | 验证状态 |
|---|---|---|
| `tools/wildos_console.py` | 录包增加 `/wildos/full_planned_path` | 本机 28 项 console 测试通过 |
| `nav2_adapter/.../graph_path_bridge.py` | `max_motion_sections=6`、`max_consecutive_empty_paths=4` | 服务器离线 654 项（1 项既有失败）；6 项新测试通过 |
| `nav2_adapter/.../motion_path.py` | 新增 `first_section_away`（只报告，不接线） | 本机 4 项纯几何测试通过 |
| `ackermann_smac/src/planner.cpp` | 仅日志：`connector_candidate`/`continuous_*_selected` 输出 `cusps/first_cos/connector_poses` | **未编译**（本机无 ROS/编译器） |
| `nav2_adapter/test/test_graph_path_bridge_routing.py` | 6 项新测试 | 服务器通过（修复前反例已保存） |
| `ackermann_smac` 构建产物 | 未构建 | 服务器恢复后需构建并更新 `config.json` 的 `library_sha256` |

文件哈希（本机，前 16 位）：
`graph_path_bridge.py 5726a5a8…`、`motion_path.py 76f6b1a0…`、
`test_graph_path_bridge_routing.py 8482e867…`、`planner.cpp 21e87ff0…`、
`wildos_console.py 4b33028d…`。

已回退：`gear_change_confirmation_time=0.6` 实验；launch 恢复 `6a6791b6…`，
418 项冻结清单 0 不符。

## 5. 服务器候选/安装状态

- 候选与安装的 `graph_path_bridge.py` 已同步（字节一致）；冻结清单已更新。
- 最后一轮试验（含连空路径修复）在断连前启动，结果未取回；`trial_*` 目录与残留进程未知。
- 服务器路径：`/home/ly/execution_continuity_repair_20261021/v10/`。
- 访问方式：`ssh -i ~/.ssh/wildos_192_168_31_91 ly@192.168.31.91`（密钥认证，不保存密码）。

## 6. 证据位置

- 本机：`test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_{130635,181516,183339}/fluency_audit/`
  （native_samples、events、decisions、summary/residual_stops、video_black_audit.json、luma_timeline.csv）。
- 录像：`.../trial_20261021_130635/rviz_live.mp4`（旧，含黑屏）；
  `.../trial_20261021_183339/` 的录像在服务器，未取回。
- 专项文档：`docs/RECORDING_AND_FLUENCY_20261021.md`、`docs/MOTION_SECTION_CAP_20261021.md`、
  `docs/CONNECTOR_DIAGNOSTIC_20261021.md`。

## 7. 测试状态

- 服务器离线回调 654 项：653 通过；唯一失败 `test_side_lookback_turns_back_once_near_large_structure`
  已用回退副本复现（既有夹具问题，不归因本轮改动）。
- 路由套件 186 项：185 通过，同一失败。
- 本机：console 28 项、motion 4 项通过。
- 新桥接测试 6 项：修复前反例（缺参数/未调用）→ 修复后通过。

## 8. 恢复服务器后的执行顺序

1. `prestart.py`：查断连轮残留进程、display148 锁、未关闭的 `trial_*`。
2. 取回断连轮证据（若存在），按同口径统计连续空路径是否被截断。
3. 同步 `planner.cpp` 到候选，构建 ackermann_smac，跑 26 项 gtest；用新库 SHA 更新
   `config.json` 的 `library_sha256`。
4. 同步 `wildos_console.py`，跑一轮 180 s（full path 入包），读取
   `connector_candidate` 的 `first_cos/connector_poses` 判定反向来源。
5. 依据归属再决定：改 connector（选路/换向惩罚）还是 Smac 后缀代价；
   **不要**直接把 `first_section_away` 当拒绝规则（130635 也有 36% 命中）。
6. 验证通过前不提交 Git，不宣称修复完成。

## 9. 禁止/注意

- 不要放宽碰撞、standoff 0.6 m、确认窗、源期限；不要把日志到达、路径生成、
  组件测试通过当作到达。
- 不要降低 `max_planning_time`：成功规划耗时中位 1.2 s、最长 10.79 s。
- 段数上限 6 是依据两轮证据定的保守值，不是通用常量。
- 断连轮可能留下车辆/控制进程，重连后必须先预检再启动。
