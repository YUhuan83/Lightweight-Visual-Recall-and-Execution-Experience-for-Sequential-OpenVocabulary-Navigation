# 项目上下文导出（2026-10-21 21:35 续）

本文件续接 `docs/CONTEXT_EXPORT_20261021.md`，导出 19:53 之后这一段会话的状态，
供另一个终端/下一次会话直接接手。不替代 `docs/PROJECT_MEMORY.md`、专项文档和原始证据；
冲突时以原始证据为准。导出本身不是修复，也不是验收。

## 0. 目标与总体状态

- 目标：持续修复 bug，使真实车体在 180 秒内流畅完成"green residential house"任务；不做 20 轮统计。
- 现状：**180 秒内完成仍未通过**。所有现场轮次 timeout；最好一轮 `210759` 跑到 (734.9,-67.9)、
  净 59.8 m、11 次可见接近，仍未确认；`212436` 净 53.8 m、6 次可见接近，最后 4 m 观察点被规划拒绝。
- **本段所有改动均未 commit**（按项目规则，完整验收未通过不提交）。

## 1. 本段现场轮次

| 轮次 | 配置/目的 | 结果 | 净位移 | 备注 |
|---|---|---|---:|---|
| 205052 | 连续起点诊断（C++ 日志） | 桥崩溃 | — | `route_endpoint=None` → `TypeError`，已修 |
| 205615 | 同上 | 桥崩溃 | — | `route=None` 迟到结果 → `None.poses`，已修 |
| 210104 | penalty 0 | timeout | 41.5 m | 近停 39.3%、12 换向、可见接近 1 次、无崩溃 |
| 210759 | penalty 3.0 | timeout | 59.8 m | 近停 48.2%、14 换向、可见接近 11 次；末段 30 s 观察点不可达后 timeout |
| 211315 | penalty 3.0 重跑 | timeout | 14.6 m | 回到起点附近 (732.9,-7.6)，无 tile、无可见接近 |
| 212436 | penalty 0（已回退） | timeout | 53.8 m | 6 次可见接近；136–174 s 连续推进 22 m；176.6 s `no_vehicle_path` |

`212436` 时间构成（plan_status 事件积分）：following 111.3 s、planning 40.5 s、
no_vehicle_path 8.2 s、handoff_stopped 4.5 s、exploration_retired 4.2 s、其余 <4 s。
planned/following 22 段、planning 调用 54 次、`waiting_for_costmap_update` 11 次、
`replanning_after_maneuver` 2 次、`exploration_handoff_stopped` 3 次。

## 2. 本段改动与状态

| 文件 | 改动 | 状态 |
|---|---|---|
| `nav2_adapter/.../graph_path_bridge.py` | ① `max_motion_sections=6` 探索路径段数上限；② `max_consecutive_empty_paths=4` 连续空路径提前退休；③ 两处 `route_endpoint=None` 崩溃保护；④ `route=None` 时 `current_route_points` 返回空、迟到结果按 `route_cleared` 丢弃 | 服务器候选/安装字节一致；新增 8 项路由测试通过 |
| `nav2_adapter/.../motion_path.py` | 新增 `first_section_away()`（只报告，未接入决策） | 本机 4 项纯函数测试通过 |
| `ackermann_smac/src/planner.cpp`、`planner.hpp` | ① `cusp_count`/`first_cosine` 诊断 lambda + 三条 INFO 日志；② 参数 `reverse_start_penalty`（默认 0，范围 0–100） | 已编译；27 项 gtest 通过；库 SHA `89ae42c4…` 已写入 `config.json` |
| `nav2_adapter/config/nav2_ackermann_control.yaml` | 实验值 3.0 → **已改回 0.0**（候选 + 安装 share + frozen manifest 同步） | 已完成 |
| `tools/wildos_console.py` | rosbag 话题加入 `/wildos/full_planned_path` | 本机 28 项 console 测试通过 |
| `v10/full/run_recorded_trial.sh` | v2：RViz 非黑（YAVG>24）才开录；goal result 写入后 SIGTERM 停止；REC_FPS=20 | 现场验证：210104 无黑段；212436 仅最后 0.55 s 黑尾 |

部署身份（服务器）：
- frozen manifest 418 成员；`nav2_adapter/config/nav2_ackermann_control.yaml` 哈希已更新为回退后的 0.0 版本。
- 桥 `graph_path_bridge.py` 候选与安装 SHA 一致：`162bb30696685d3e…`。
- `libwildos_ackermann_smac.so` SHA：`89ae42c46a042a74…`（`config.json` 已匹配）。

## 3. 反向首段：归属与惩罚结论

- 归属：反向首段来自连续起点 connector，不是 Smac 后缀（205052 26/40、205615 23/25、
  210104 37/47 次选择为 connector，`first_cos≈-0.9…-1.0`；直接连接约 +0.999）。
- A/B：`210104`（0）反向 21/37=56.8%、近停 39.3%；`210759`（3.0）22/41=53.7%、近停 48.2%；
  同配置重跑 `211315` 只走 14.6 m。比例未实质下降，59.8 m 未复现 → **判定无效并回退 0.0**。
- 参数与诊断日志保留（默认 0，不改变现有排序），详见 `docs/CONNECTOR_DIAGNOSTIC_20261021.md`。

## 4. 目标确认链路（本段新增的关键证据）

1. **确认实际只走 LiDAR**：`v6 121345` 有 17 次 `Object LOCK using LiDAR`、17 条
   `object_localization`、17 次发布；`Validated multi-view` 在 121345 与 210759 均为 **0**。
   210759：`Received callback 162`、`too close 124`、`Not enough views 15`、`No triangulated 38`。
2. **多视角被视图窗口饿死**：日志里 `too close 0.00` 成片出现，`Not enough views` 约每 5 s 一次，
   与 `observation_timeout_sec=5.0` 一致；观察步停车等待本身超过视图寿命。
3. **确认后止步于 standoff**：
   - `v10 124354` 127.6 s 对 standoff (739.9,-75.5) 直接 `confirmed_approach_failed reason=no_vehicle_path`，
     随后 `unconfirmed_target_release_pending`；
   - `v6 121345/121904` 在确认时距 standoff 4.66 m > 0.6 m，桥报 `confirmation_before_standoff`。
4. `210759` 末段：56.6–135.5 s 沿固定方位推进到 (735.1,-67.7)，随后对 (735.9,-71.7) 反复
   `waiting_for_costmap_update`/`footprint_collision`，30 s 观察截止在 165.7 s 到期并把目标 block 15 s。
5. `212436` 末段：136–174 s 从 (734.4,-51.1) 连续推进到 (737.9,-73.0)，176.6 s 对最后一观察点
   `no_vehicle_path`；tile 检测持续到轮末，不是检测丢失。

## 5. 为什么录像里看不到掩码（显示层，不是检测失败）

- 本轮 `/spot1/object_mask` 回调 540 次、`No object detected` 271 次，其余约 **269 次发布了 mask**
  （tile 回退 21 次，集中在 150 s 后）；最后 40 s 每 10 s 仍有 9–13 次正检测。
- RViz 右栏 `/wildos/viz/rgb_target` 的叠加条件是 `相机帧时间 − mask 时间 ≤ 0.5 s`（静止时 2.0 s），
  见 `nav2_adapter/wildos_nav2_adapter/visualization_bridge.py:129-141` 与
  `model_visualization.py:rgb_target_image`。实测本轮相机到推理回调延迟中位 1.007 s、p90 1.26 s、
  最大 1.67 s，再加推理约 0.4–0.5 s → mask 到达时已约 1.5–2.1 s 旧，行驶中必被丢弃。
- 对整段 4200 帧每 10 帧采样，右栏掩码着色像素为 **0**，与该门限一致；这是有意设计
  （"延迟推理不能把旧目标画到新画面上"），测试 `test_visualization_bridge_ros.py:64` 锁定。
- 导航侧不受影响（mask 自带同戳相机 TF），但**录像不能用来判断掩码语义是否正确**。

## 6. 规划侧仍未解决的三类问题

1. **末段可达性**：观察点/standoff 被规划拒绝（`no_vehicle_path`、`footprint_collision`），
   即使确认完全正常，车到不了 standoff 依然完不成。
2. **效率与走停**：54 次规划、40.5 s 在 planning、22 段路径；净位移 53.8 m/180 s 折算平均 0.30 m/s。
3. **路线采样方差**：同配置同起点，`211315` 14.6 m vs `212436` 53.8 m。

## 7. 测试状态

- 服务器离线回调 189 项：1 项既有失败 `test_side_lookback_turns_back_once_near_large_structure`
  （回退副本可复现，非本段改动引入）。
- 路由测试新增 8 项通过；C++ gtest 27 项通过。
- 本机 `test_motion_path_direction.py` 4 项通过。

## 8. 证据位置

- 服务器：`/home/ly/execution_continuity_repair_20261021/v10/full/trial_20261021_*/`
  （`sessions/*/goal_001/{events.jsonl,triangulation.log,vision.log,stack.log,failure_last_rgb.png}`）。
- 本机录像：`test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_212436/rviz_live.mp4`
  （184,443,220 字节；1920×1080、20 fps、4200 帧、210.0 s；仅最后 11 帧黑）。
- 本机失败帧：同目录 `failure_last_rgb.png`。
- `210759` 的失败帧与轮末 RViz 帧：`test_artifacts/execution_continuity_repair_20261021/v10/trial_20261021_210759/`
  曾在本次会话取回，该目录随后不在本机；原始轮次目录仍在服务器 `v10/full/trial_20261021_210759/`。
- 专项文档：`docs/CONNECTOR_DIAGNOSTIC_20261021.md`、`docs/RECORDING_AND_FLUENCY_20261021.md`、
  `docs/MOTION_SECTION_CAP_20261021.md`。

## 9. 下一步建议

1. **可达 standoff/观察点选择**：在局部栅格上做膨胀后的机器人连通域检查，默认 standoff
   不连通时按 [3, 4.5, …, 10.5] m 环形外扩选点；到达判据（0.6 m / 0.35 rad）与确认门槛不动。
2. **时间对齐掩码回放（仅诊断显示）**：缓存相机帧，mask 到达时按自身时间戳取同帧着色，
   发布到单独话题（如 `/wildos/viz/rgb_target_stamped`），用来判断掩码到底覆盖了房屋还是树冠。
3. 上述改动需先离线测试，再各跑一轮 180 s；未通过前不提交、不宣称完成。

## 10. 禁止/注意

- 不放宽碰撞、standoff 0.6 m、0.35 rad 终点朝向、确认帧数/半径/时间窗、源期限。
- 不把日志到达、路径生成、组件测试通过、掩码发布当作到达或确认。
- 视频不作为到达证据；录像按用户本次明确要求才取回。
- 服务器访问：`ssh -i ~/.ssh/wildos_192_168_31_91 ly@192.168.31.91`（密钥认证，不保存密码）。
