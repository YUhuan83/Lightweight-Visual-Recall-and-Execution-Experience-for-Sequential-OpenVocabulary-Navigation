# 2026-10-12：转向输入接口与移动时共同响应空档

## 本轮结论

已连接服务器并实施直接转向输入候选；静止接口和失效注入通过，实际短曲线运动失败。不能宣布规划执行已修复，也没有开始完整搜寻、20轮统计或多目标流程。按提交规则不创建 commit。

## 已定位的接口问题

BeamNG 0.39.4 安装源码 `lua/vehicle/extensions/tech/techCore.lua` 的普通 Control handler 使用 `input.event(key, value, 1)`；`lua/common/inputFilters.lua` 定义 1 为 FILTER_PAD、2 为 FILTER_DIRECT。PAD 转向进入游戏手柄滤波及相关辅助处理。SDK 的 ACK 表示请求处理，不能证明转向输入已达到请求值。

真实原始 SBR、全制动/零油门/空挡下，`steering_input_probe2` 对照：

| 接口 | 请求方向 | 输入达到 90% 的测量时间 | ACK 时间 |
|---|---:|---:|---:|
| SDK PAD | +0.5611 | 0.4372 s | 0.0010 s |
| SDK PAD | -0.5611 | 0.4094 s | 0.0027 s |
| 候选 DIRECT | +0.5611 | 首个采样 0.0063 s | 0.0021 s |
| 候选 DIRECT | -0.5611 | 首个采样 0.0069 s | 0.0032 s |

`electrics.steering_input` 是滤波后的输入，不是实际轮角。probe1 的 rack 标签不准确，原文件保留；probe2 更正为 filtered_input，并记录转向 hydros 的 cmd/state/inRate/outRate。DIRECT 下物理 state 仍逐步追赶 cmd，速率仍为 1.25；没有取消真实转向机构、轮胎或车身物理。DIRECT 自身的辅助配置仍归 BeamNG 管理，本修复没有更改这些配置。

## 修改及影响范围

- `nav2_adapter/wildos_nav2_adapter/direct_vehicle_control.py`：只给 Native actuator 实例安装控制方法；一次同步 VE Lua 请求，转向用 DIRECT，其他输入保持 SDK PAD、挡位调用保持原逻辑。先验证输入、滤波常量，再写入，要求明确协议 ACK。错误 ACK/超时向现有锁定失败路径传播；不修改 SDK 全局类。
- `native_vehicle_control.py`：只在 actuator 接入后安装上述方法；状态增加接口版本和 steeringUnassisted。原 .5/.8/2 秒保护、8m 半径、速度设置、碰撞和换挡均保持。
- 增加输入/ACK/实例隔离测试及真实 transport tick 的错误 ACK 锁定、不重试运动、看门狗制动和挡位回归。
- 两个历史证据测试改为比较归档中的冻结源码与当时的部署 SHA；历史验收不应因当前源码升级而虚假失败。旧归档及失败结果没有改写。当前部署另由新包逐项核对。

本机、服务器相关回调替身各 52 项通过；两套历史闭包 5+7 项通过。数量存在覆盖重叠，不代表独立现场成功数。

`direct_steering_freshness1`：真实 BeamNG/DDS 五阶段通过，实际零油门/全制动，分别在真实控制 ACK 和轮询返回后注入 650ms 延迟，失效检测、制动意图与新鲜数据恢复有效。合成速度意图仍只流向无执行器订阅的私有话题；不是自然引擎冻结复现或到达验收。

## 反例与剩余故障

1. `rpp_closed_loop_replay1` 使用实际安装 RPP 插件及 pilot5 的 55 姿态、19.119m 历史段，FREE_SPACE 图及理想自行车模型。实际起点和名义起点均达到几何终点，最大偏离分别 0.614801m、0.234789m，即使曲率多次饱和。原“实际起点应失败”的断言被证据推翻，exit1/result false 保留。说明饱和本身不足以复现现场失败；不证明真实车辆或含障碍地图可通过。
2. `direct_steering_curve1/2` 起步前没有得到满足原约束的有界路径，控制门保持关闭。curve1 一条 Nav2 成功返回路径被生产校验拒绝；几何单独重放定位到 heading 窗口 16～20：2.11766m 上变化 0.370957rad，校验值约 0.17517/m，高于 0.125/m。只是返回路径契约反例，尚未判定量化、解析接续或平滑的具体来源；没有放宽校验。其余采样目标规划失败，不归因于新转向输入。
3. `direct_steering_short_curve1` 正式选择 5.26176m 安全倒车曲线，真实开始执行后 Native receipt stale。请求级见证：同一窗口 Native SensorRequest 和 QueueLuaCommandVE 分别约 2.002s 超时；独立传感器 SensorRequest 等待 4.78363s；GE 读取也延迟。模型/定位没有启动，不能把该空档归因于视觉记忆或模型负载。GPU 采样有低利用率窗口，但没有主线程堆栈，不据此宣布 GPU、渲染或物理根因。
4. Native 超时锁定后，遥测恢复但 actuator 不再发送；实际 brake 仍为 0，车辆继续运动。ROS watchdog_stop 和 sent 中的制动意图不等于制动已送达。原停车和 sustained SDK stop 验收失败，结果保留；独立新连接全制动后速度低于 .05 连续超过 .5s，最终约 .001495m/s，才完成安全收尾。
5. 同轮日志随后出现 route_done，不能作为健康执行/停车成功。需要继续核对动作终态与 actuator health 的交接约定，以及模拟器内过期指令制动保护；两项尚未实施。引擎全部回调停止期间，任何 Lua 保护也不能立即运行，必须明确恢复后的处理和验收边界。

## 收尾与证据

引擎422172、独立显示422169存活，boot_id 未变；同地图/同车体、0重载/0瞬移。723节点、0固定、467车身碰撞、四轮各32碰撞有效。全部本任务控制/导航/模型/传感器进程清理，无活动传感器；仿真暂停，原生产目录仍干净 f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0。

本机闭包：`test_artifacts/server_validation_20261012/direct_steering_evidence.zip`，3,328,064 字节、124 原始成员，SHA256 `d5ecfe949c0d29bb134ad81ee3a740ff668f6fd56ed245c47187075511872462`。包含源码前后、实际请求、少量失败图像、失败尝试、资源日志和最终安全状态；没有录像。运行时 helper 与最终 helper 只有 docstring 更正，运行时版本另存并逐项匹配三个组件 frozen.json，不用当前源码替代历史版本。

下一步先验证模拟器内控制租约、超时后制动及终态交接，再排共同响应空档、长曲线与 cusp。独立 costmap 诊断退出异常、完整视觉到达和记忆返站仍未验收。
