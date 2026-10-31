# 2026-10-12 执行器失效与动作终态交接

## 依据与边界

上一轮 `direct_steering_short_curve1` 的原始控制请求、状态及传感器共同空档导致 actuator 永久不健康；随后仍出现 route_done，实际制动没有送达、停车验收失败。这是执行健康与动作状态没有共同约束的反例，不是视觉模型识别、记忆候选或转弯半径造成该终态的证据。

本轮修改 GraphPathBridge／ExecutionFeedback，完成离线回归及服务器隔离 DDS/action 验证。**没有车辆运动、目标到达、完整搜寻或20轮统计**。车辆端过期指令制动保护尚未实施，模拟器共同响应空档来源仍未确定。

## 原代码反例

4 个测试、5 个失败子检查：

- actuator_ok=false 时，车辆位置到端点仍进入 route_done／awaiting_visual_confirmation。
- 动作成功结果迟到，即使 actuator 已失效仍完成 segment／cusp，并可能记录成功执行。
- 重复 Native 状态更新接收年龄，给旧消息续命。
- 未确认送达的 sent.brake=1 被当成已送达制动证据。

原始失败日志 `test_artifacts/server_validation_20261012/direct_steering_repair/transport_counterexamples_before2.log` 保留。第一次测试导入旧 TestCase，使 runner 同时收集43项无关旧测试；改为模块导入后得到准确4项，前一次日志也保留，不扩大故障数量。

## 实施修改

- execution_feedback.py 增加 physical execution health gate：只对启用执行的 Ackermann 载体要求新鲜 Native 状态、actuator_ok=true、control_acknowledged=true、telemetry_ok=true；样本年龄加消息传输及接收后年龄不超过原 .5s。只读规划和 ideal carrier 保持各自约定。
- 同时间戳重复消息不续命，乱序消息拒绝；接收时计入消息自身年龄。
- native_command_braking 必须有执行器健康及 ACK，避免把客户端失败后的未送达制动意图解释为已送达输入。ACK 仍不代表实际停稳。
- GraphPathBridge 在启动、运动段分发、监控、动作接受和完成交接检查健康。收到明确 actuator failure 时直接使旧动作 epoch 失效、请求取消所属动作、清除未执行队列与替换／恢复状态；迟到结果不能推进终态。
- 发布 execution_unavailable；保留已接受目标、地图和记忆候选，不把通信故障计作路线不可达。健康恢复后可重试同一目标；保留已有几何退避及重试预算，不增加路线失败计数。
- 执行经验反馈还需健康 gate 才能学习成功，防止原生执行失败污染历史图边评分。

没有更改速度、转弯半径、真实车体／碰撞、地图、视觉记忆检索和目标到达距离。取消请求本身不能证明车辆停下，车辆端控制租约仍是下一项工作。

## 验证

- 本机完整 callback runner 299 项通过，其中相关正常／失败／恢复／交接测试195项；组间重叠，不相加。
- 服务器同相关195项通过。测试夹具显式提供健康 ACK 状态；持续运动时同步刷新该状态，缺席／失效／累计超龄／重复／迟到接受／旧成功结果／退避恢复由新9项覆盖。
- localhost域86真实 ROS DDS/action：生产 GraphPathBridge 收到人工健康状态，实际 FollowPath action接受；随后人工 actuator fault 使桥取消动作、保留目的地。测试服务器故意拒绝取消并迟到返回 SUCCEEDED，旧结果仍不能形成任何 route_done／segment_done／maneuver_done。新鲜健康恢复后原目的地保留。所有坐标、地图、状态和动作服务器是隔离测试输入，没有连接 BeamNG 执行器，不声称 Nav2真实执行或停车通过。
- DDS result1及最终版本 result2均通过；最终被冻结版本另核对本机与服务器7个源文件SHA。

开发过程失败保留：两个旧测试夹具缺健康更新／ACK，被新契约拒绝后补真实所需字段；首次扩大套件时缺official_adapter导入路径、恢复夹具没有规划服务就绪，纠正后通过。大批base64传输被工具输出省略损坏，解码在测试写入前拒绝；两个核心源码SHA一致，改SCP及先验SHA后才运行。没有运行损坏的候选或把开发工具失败当旧生产根因。

## 收尾与下一步

现场只做只读物理核对并维持暂停：723节点、0固定、467车身／四轮各32碰撞有效，SDK速度 .001495m/s；无本任务导航、控制、模型、传感器或域86测试残留，传感器空。引擎422172／独立显示422169存活；原生产目录仍干净 f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。

闭包 `test_artifacts/server_validation_20261012/execution_health_evidence.zip`：113656字节、21原始成员，SHA256 `8c0329b51138def848f4c8d4829510062cb8965d04fd5ef7320d8febec8b38a0`，本机逐员验证；无录像。当前部署一次性核对在 `execution_health_repair/local_deployment_verification.json`；历史测试只核对归档的冻结版本，不因未来修复产生虚假回归。

下一项：车辆端控制租约。要求过期后锁定制动，迟到／重复／旧拥有者命令不能续命；明确新拥有者接入、正常关停、超时清理、车辆重置和引擎冻结恢复后的处理。全部引擎回调停止时 Lua 也无法执行，不能用租约掩盖冻结根因。待相关现场组件验证、长曲线/cusp及停止链路通过后，才恢复完整绿色房屋搜寻。20轮/80%及多目标未开始；必要现场验收尚未通过，本轮不commit。
