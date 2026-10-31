# 2026-10-16 停驶与取消链路彻查

## 结论

274c3ef2 最新轮次中，主要停驶来源已经定位到新增探索交接代码：**探索意图失效直接触发执行器级 reset/cancel**。上次修正了“过时 frontier 仍被持有”，同时让正常地图推进导致的 frontier 消失频繁触发正在执行动作的取消。这是本次改动的实际副作用。

另有一个确定的接口遗漏：C++ 新增 `frontier_resolved`，Python `on_exploration_retired` 的 `same_frame` 分类未包含它。该原因落入完整 reset，不保留同坐标系的已驶退路/规划失败历史，也会取消原本独立拥有走廊的恢复动作。**历史清理及独立恢复取消已用当前生产回调复现；本轮没有实际倒车恢复，不能称它导致了本轮某次恢复失败。**

本次仅读两份已关闭现场证据及当前源码，未修改生产实现、未启动新仿真，无新录像、无 commit。日常生产仍 e6，274 候选不宜据组件通过直接推广。

## 版本和证据范围

- 当前源码相关模块逐 SHA 对上冻结 274c3ef2；现场实际库/安装来源由前轮 loaded_runtime.json 证明。
- 主要现场：session_20261016_200923；对照 236 候选的 session_20261016_194721。
- 新证据：test_artifacts/cancellation_chain_audit_20261016，包括 audit_closed.py、summary.json、timeline.json、reason_contract.json、mechanism_reproduction.json、map_review.json。
- 复用已经验证的核心归档 b76698eb3ce04edc370132345412f972928c4027beaa62a40f9c4b07d53486c5；只流式读取大型 observer JSONL，仅读已封闭 SQLite，不读取活动包。

## 一、21 次取消全部由普通 frontier 退役触发

按原探索 ID、源退役时间、Bridge 状态、取消日志和控制器停车日志关联，21 条 `Cancelling follow_path action` **全部**对应 `frontier_resolved`。不存在将换 query、用户停车或控制器碰撞中止混入这 21 条的情况。

链路为：

```text
Mapper 推进观测边界，节点不再是 frontier
  → PlannerNode 发布 frontier_resolved
  → GraphPathBridge.on_exploration_retired
  → reset：清队列、使旧 epoch 失效、取消 FollowPath
  → quarantine / execution_hold：等待旧动作终态及持续停稳
  → 重新比较 frontier、等待地图、重新计算路径
  → 下发新 FollowPath，再换挡/起步
```

| 指标 | 结果 |
|---|---:|
| 取消时实测速度中位数 | 1.139m/s |
| 取消到隔离解除中位数 / 最大 | 1.824s / 2.578s |
| 取消到下次 FollowPath 中位数 / 最大 | 6.282s / 17.047s |
| >5s 静止中，开始前 3s 内有该取消链 | 11 / 14 段 |

例如第一条：车辆仍约 1.326m/s 时，源图发出退役约 7.5ms 后 Bridge 取消；约 63ms 后控制器记录 Goal was canceled / Stopping the robot。8.568s 后才下发下一动作。期间收到一条停车速度指令，之后直到下一动作才恢复运动指令。这是软件主动停车链路。

剩余三段超过五秒的静止分别包含：

1. 9.711s：从一个换向段终点停车开始，窗口内又发生两次退役/取消及重新比较；不是“与取消无关”的纯换挡。
2. 7.404s：普通路径段结束后重新求解，多个候选和地图更新等待。
3. 6.006s：换向完成后，剩余旧路径在新地图上 footprint_collision，进入安全重规划。

## 二、检查了取消当时的旧路径

只读服务器上已关闭记录，逐条匹配最后接受目标的 `/received_global_plan` 或 `/wildos/planned_path` 末点；在取消之前最后收到的 odom/全局图/局部图上，使用原生产 trim 与碰撞函数检查。

- 21/21 找到匹配路径，21/21 可按当前记录车姿裁剪。
- 前方 2m：全局和局部检查 21/21 均通过。
- 剩余运动段：19/21 通过记录全局图检查；1 个远端 footprint_collision、1 个远端 collision_cell。
- odom 源龄最大 .515s、全局图最大 .908s、局部图最大 .596s；不是插件私有地图或精确 RPP 指令弧逐字节回放。因此不能将 19 个结果外推为真实车辆继续整段绝对安全，更不能删掉运行中的碰撞保护。

这些结果支持：**不能统一把全部 frontier 退役当成必须立即停车的几何危险**。有障碍的两例仍需安全处理；没有立即障碍证据的普通意图更新应具备受控的运动段交接机制。

## 三、停车期间是谁在刹车

14 段静止总 109.852s；按 observer 中控制状态相邻时间积分：

| 控制状态 | 静止时间 |
|---|---:|
| stale_command | 81.360s |
| gear_change | 17.531s |
| execution_hold | 8.820s |
| tracking | 1.469s |
| ackermann_stop | .672s |

Native 静止样本 809 个，其中 794 个下发全制动，油门>.1 的样本为 0。停车期间 State 接收龄最大 .129s，Native 样本龄最大 .139s；整轮 Native telemetry/actuator failure 和 watchdog_stop 均为零。

`stale_command` 在这里大量发生于动作已被取消、规划尚未重新下发的窗口，控制器缺少新速度指令后按保护要求保持刹车。不能由这个名字直接推断网络丢包或 BeamNG 物理卡住。原有 State 源戳约 11.5 秒时钟域偏移仍是单独待办，本次没有将其重诊为 11.5 秒旧状态。

## 四、遗漏的退役分类已经实际方法复现

生产回调不变，使用已有 ROS 接口替身，五组正常/异常对照：

| 退役原因 | 原有独立恢复 | 取消请求 | 保留已驶历史 | 恢复仍活跃 |
|---|---|---:|---|---|
| frontier_resolved | 否 | 1 | 否 | 否 |
| vehicle_route_rejected | 否 | 1 | 是 | 否 |
| frontier_resolved | 是 | 1 | 否 | 否 |
| vehicle_route_rejected | 是 | 0 | 是 | 是 |
| graph_frame_changed | 是 | 1 | 否 | 否 |

源端 `frontier_resolved` 实际是同 frame 的探索意图变化，消费端却按不在白名单的原因处理为完整状态重置。本轮 Bridge 处理 30 次这种退役，走了该清理流程；没有私有 trace 快照，不声称每次都有有效退路被清掉。

旧测试 `test_resolved_frontier_cancels_motion_without_success_or_obstacle_evidence` 恰好要求取消动作，因而把有问题的连续性契约固定成预期。此前 447 项通过验证了大量消息隔离/保护，却没有验证普通 frontier 失效时的连续执行和物理历史保留。这是测试设计缺口，不能以通过数量抵消。

## 五、其它层的检查结果

- 车辆仍完成了真实移动；主要长静止窗口下发的是刹车，不支持将这些窗口归为轮胎空转或油门无效。
- 全局规划共有 150 次请求，57 条 Following 状态；其中 68 次只读 frontier 代价评估、33 次代价候选拒绝。拒绝含 31 empty_path、1 footprint_collision、1 comparison_pose_mismatch。最终 no_vehicle_path=0 不等于所有规划候选都可行。
- 换向后的重规划有 footprint_collision 和 collision_cell 各一次。latest FollowPath abort=0 也不等于没有障碍引起的重新规划。
- 三个新增记忆先验实际关闭；视觉仍评分 709 次，query 已正确切换。47 个目标中，按最近视觉输出 UUID 对应 38 个有非默认分数、2 个为默认分数、7 个没有该次采样匹配；这只能证明评分链工作，不证明最终选择最优。
- mask、定位、确认均为零，整轮没有进入已定位绿色房屋的接近/到达链。没有单独核实全程 RGB 是否出现合格目标前，不能把零 mask 当成已证模型错误。
- 这仍包含新增自动选点、目标保持和桥接逻辑，不是完整原版 WildOS 对照。已证主因位于新增交接适配代码，不能据此否定视觉记忆研究原理或归咎原论文。

## 修复顺序与验收约定

1. 明确区分坐标上下文失效、探索意图失效、运动段失效，统一源/消费端契约；补齐 frontier_resolved 对同 frame 物理历史及独立恢复的语义。
2. 对普通 frontier 失效，核验当前已接受运动段；能安全衔接时停止旧任务扩展，在有界运动段终点或已验证替换处接新任务。碰撞、输入过期、未知动作终态、显式停车、query/frame 改变保留原取消与隔离保护。
3. 规划准备与动作执行分别拥有异步响应身份；新目标不能清掉仍需处理的动作终态，也不能让旧规划结果复活。只补一个原因字符串不足以修复主停驶链。
4. 用本次反例验证无障碍普通退役、真实障碍、新 query/frame、后台规划晚到、取消迟到、独立恢复等路径；再做一轮相同出生点真实车体基线。验收同时看目标到达、动作完成/取消、静止占比和探索新区域，不继续用“无 abort”代表稳定。

此次为根因审计，以上实施方案尚未写入生产代码。全部现场到达、20 轮稳定性与多目标验收仍未通过。
