# 后台规划超时与当前路径执行归属（2026-10-13，离线检测）

本轮延续理想/真实车体比较后的离线检测。未连接服务器，未修改生产代码。对当前源码的生产 Python 回调运行 ROS 接口替身；没有运行 DDS、C++ Nav2 或 BeamNG。新诊断通过说明反例成立，不说明问题已修复。

## 新反例：延长路径超时使当前 FollowPath 回调失效

`ContinuousPath.maybe_extend_path()` 在单段前进路径仍在执行、剩余长度≤8 m、图路线仍较长时后台规划下一条延长路径。设计要求：延长路径失败不能取消有效的当前动作。

相关源码：

- `nav2_adapter/wildos_nav2_adapter/continuous_path.py:27`：后台延长的触发条件。
- `nav2_adapter/wildos_nav2_adapter/continuous_path.py:67`：`extension_failed()` 保留当前执行。
- `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py:547`：`recover_timed_out_plan()` 全局 `epoch += 1`。
- 同文件 `:1636`：`on_follow_result()` 首先检查 epoch，不匹配就直接返回。

注入前提：已有有效 FollowPath、后台规划仍未完成，且输入/Native 健康仍新鲜，规划达到超时。它不要求车体受困，但当前动作必须仍在运行；不是声称每次延长规划都触发此故障。

生产回调重放顺序：

| 时点 | epoch | following | 当前 handle/队列 | 结果 |
|---|---:|---|---|---|
| 延长规划等待中 | 0 | true | 保留当前动作/1段 | 正常 |
| 延长规划超时、候选耗尽 | 1 | true | 原动作/1段仍保留 | 状态仍为 following，extension_rejected=planner_timeout |
| 原动作成功，实际pose已在端点 | 1 | true | 原动作/1段仍保留 | 旧回调携带epoch=0被忽略；未进入下一段 |

将原动作终态改成 ABORTED（6），也被忽略，未进入执行失败处理。因此问题不限于漏掉成功。

若超时同时发现新的图路线终点，分支采用新路线并调用 `start_segment()`，但当前仍 `following=True`，入口保护使其返回；`extending_path=True,pending=False` 保留，旧动作终态仍失效。诊断记录调用1次但0个新FollowPath，不能将函数被调用当作开始执行新路线。

## 正常对照与归属范围

- 同样的有效当前动作，后台规划返回普通 `empty_path` 而非超时：不改变epoch，原动作成功被处理，队列清空、following变false、下一段入口调用1次。
- 前台规划超时：旧规划接受回调仍被取消。这项已有安全需求必须保留，不能简单删除所有epoch递增来修复后台超时。
- `ideal_point` 注入同样后台超时也复现漏终态。此新问题位于公共 Bridge，不能解释为只由真实物理触发；用户历史理想模式未观察到它，与条件性反例不冲突。
- 与上轮已定位的真实 Ackermann 短段接续/恢复预算问题分别记录。此反例不是视觉目标或视觉记忆检索错误，也未证明某一历史现场停驶就是它导致。

可能后果：已完成的动作仍被桥认为执行中，下一段入口被拦住；或后续监控把遗留状态转为跟踪/停滞故障，产生额外取消、重规划。后者是代码机制推断，本轮未模拟完整自主恢复循环。

修复方向是将规划请求身份与已接受执行动作身份分开：超时只失效所属规划请求，取消它的迟到接受/结果；后台延长失败保留原FollowPath归属和终态处理；查询切换、传感器失效、执行器失效等必须终止整个导航事务的情况继续使所有所属动作失效。还须覆盖取消中的旧动作、替换接受前后的旧结果、候选等待新图、恢复中路径和目标切换，不能只针对当前成功回调放宽检查。本轮按检测范围未实施这些生产修复。

## 核实旧接续修复：343组扰动未发现新的不一致

对同一cusp3短倒车路径，起点位置偏差取0、0.05、0.1、0.15、0.19 m（非零半径12个方位），朝向偏差取-0.3到+0.3 rad的7档，共343组。比较现行0.2 m接续预测与8 m半径、零延迟endpoint-carrot自行车重放：

- 220组检查接受且重放到达。
- 123组检查拒绝且重放未到达。
- 本范围0组检查接受但重放失败，0组检查拒绝但重放到达。

该结果支持现有短段收紧修改在这条路径/扰动范围内有效；343组不是343轮仿真，不证明所有路径接续已解决。长路径仍有固定3 m预测与RPP实际3～6 m前视范围差异，未在本轮验证闭环。

## 证据与验证

- 诊断：`tools/offline_extension_ownership_audit.py`。
- 7项诊断测试：`tools/test_offline_extension_ownership_audit.py`；全部通过。
- 结果：`test_artifacts/offline_execution_audit_20261013/extension_ownership.json`，包括6个回调场景、343组统计及4个生产输入SHA256。
- 最终测试日志：同目录 `extension_ownership_tests_final.log`；此前6项版本日志保留，不相加当独立覆盖。
- 冻结生产源码未变核对：同目录 `extension_ownership_verification.json`。

Windows Python启动器本轮输出“Failed to find real location of D:\python.exe”，诊断进程仍返回0且实际运行7项测试；这是本机启动器提示，不是ROS/BeamNG故障。没有据此更改生产代码。

完整单目标/记忆返站、20轮/80%验收仍未开始或未通过；必要现场验证待执行，本轮未commit。
