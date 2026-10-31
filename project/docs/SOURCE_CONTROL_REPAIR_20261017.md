# 控制源龄与回执修复候选（2026-10-17）

## 状态与证据

仅离线修改及验证，未连接服务器、未部署、没有新仿真或 MP4、未创建 commit。完整绿色房屋搜寻、20 轮统计和多目标验收仍未通过。字段 `source_receipt.applied` 表示同一源需求的 SDK 输入请求得到匹配且未过期的 ACK；它不证明轮胎已响应或车辆已刹停，`applied_wall_ns` 记录的是 RPC ACK 返回时刻。

- 基线：439 成员 `12023042037205a79a57f0e76bb1d8544e94309f8ff9b80a3fe28f554f521f2d`。
- 当前离线候选：397 成员 `a69ea0560b3008f53dbf21d3ac9b66758d261a36a1659517343b4f347a6d2f30`，保存在 `deploy/20261017_source_control_candidate/`。本机通用包已同步候选，旧 120 包及既有候选保留。服务器日常入口本轮未触碰，最后现场记录为 e6；不将本机更新说成服务器已部署。
- 与 120 比较：20 个既有载荷改动，4 个载荷加入归档，46 个旧生成文件排除；新增归档不等于文件均为本轮新建（包括原有 freshness 探针）。142 个受保护源码/配置逐字节保持：GraphNav C++、视觉评分/记忆、Smac/RPP、物理控制租约、速度数学及真实车体导航配置。
- 证据：`test_artifacts/source_control_repair_20261017/summary.json`；`before/after/diffs`、基线标识、协议/回调/总检日志、候选清单。SHA 对原字节验证，阅读差异统一换行。`collect.py` 验证两份归档 CRC、逐成员 SHA、当前源码与冻结候选一致。

## 修复针对的已证缺口

上一轮封闭证据及审计见 `OFFLINE_SYSTEM_AUDIT_20261017.md`。速度环输出到 Native 消费/下发存在至少 .274–.436 秒源需求年龄，但 Native 原 `command_age` 只从接收回调起算；无源时间的 `VehicleControl` 晚到会重新获得 .8 秒寿命。遥测及 RPC ACK 新鲜，也不能证明当前源需求新鲜。具体 DDS、调度和负载占比未确定。

本轮没有修改 QoS 或线程配置，也没有证明传输延迟已经减少。

## 数据流与约定

`RPP /wildos/cmd_vel → CmdVelToControl → source command → NativeVehicleControl → SDK input ACK → source receipt → CmdVelToControl / GraphPathBridge`

1. **源封装**：新增纯 Python `source_control.py`。真实车体改用独立 `/wildos/native_control_command`、`std_msgs/String`、`wildos_source_control_v1`。载荷含完整六项控制量、随机进程 sender ID、generation、递增 sequence、明确 `unix_ns` 时钟的生成/截止时间。期限 .5 秒，沿用上游现有超时预算，不是已实测最优延迟。单包限制 4096 字符，发送历史/接收截止历史各最多 64 项，退役进程身份最多 64 个；达到身份容量后拒绝接管，不丢弃围栏。
2. **接收**：源墙钟必须来自同机系统时钟；未来、过期、非有限或越界、错误类型、重复/倒序、旧 generation 及非法外进程指令不刷新计时器。接收时只把剩余寿命转换为 monotonic 截止，绝不重新给予完整 .5 秒。过期状态锁存，墙钟前跳后回拨不能复活旧油门。初始绑定、generation 切换、失效后恢复都先要求新鲜全制动屏障；跨进程接管还要旧需求失效与新鲜实测停稳，旧 sender 被围栏。
3. **下发**：Native 的 `command_age` 现在包含速度环输出后的已知源龄，新增 `command_receipt_age` 单独保留回调后年龄；过源截止则发制动。原 .8 秒外围 watchdog、双 socket、执行器失败锁存、物理租约、挡位、碰撞和方向保护保留。源 snapshot 与控制 RPC 配对，回调更新不能给旧 RPC 换身份；迟于源期限的 ACK 不构成有效源回执。
4. **回执**：回传源 ID/generation/sequence/生成与截止时间、实际提交控制量和 ACK 时刻。速度环只接受自己有界发送历史中的同 generation、同载荷、未过期回执；旧/未知/倒序/被钳制为不同载荷/失败回执不能解锁。允许正常独立 topic 有一轮收发错位，不要求 ACK 必须恰好等于最新发布 sequence。启用/禁用、有效 hold/release 均换 generation、立即发制动并清旧速度，恢复仍须新 controller 指令。
5. **Bridge**：真实车体默认要求有效源回执、新鲜 producer 状态及相同 sender/generation；原 Native 遥测/执行器/ACK 门禁继续执行。`for_stop` 仅用于旧动作取消隔离的实测停稳确认：仍需旧动作全部终态、新鲜输入/Native、制动 ACK、低速持续 .5 秒，但不把已经失效的源需求作为确认停稳的先决条件；再次行驶重新经过完整源门禁。
6. **启动/部署**：physical launch、runtime contract、唯一 topic 发送者/接收者与消息类型检查、旧 Native producer 检查及录包话题成套更新。原 `/control` 的 legacy 调试接口保留；不允许该无源格式指向新 Native topic 或旧 `/wildos/native_control`。实际 ROS launch 展开仍待服务器验证。
7. **归档**：旧包发现会继承 `nav2_adapter/build/lib` 等过时生成副本。本轮在旧库存及目录扫描之后统一排除 build/install/log/__pycache__/egg-info，回归验证旧 ZIP 也不能重新带入。原源码、旧归档不删除；运行路径/安装来源仍须现场核验。

## 回归结果及中间反例

| 范围 | 结果 | 边界 |
|---|---|---|
| 源协议纯测试 | 17/17 | 源龄 .4 秒计入、.5 秒失效、重复乱序、代际/进程切换、时钟跳变、精确回执、容量、不可变快照等 |
| 全部实际 Python 回调 + ROS 接口替身 | 477/477（原 464 + 新 13） | 包含速度环→Native→回执→Bridge、暂停恢复、晚 RPC、损坏消息、错误配置、取消隔离；非真实 ROS/BeamNG |
| cusp 探针自身回归 | 14/14 | 未执行实际车辆 cusp 探针 |
| 总检 | 50 PASS / 1 FAIL / 6 WARN | 剩余 DDS 套件 8 项中 4 项 `Path.resolve(strict=True)` 被 Windows 沙箱 WinError 5 拒绝，4 项通过；没有修改 DDS 路径/hash 防护 |
| 包完整性/当前源码/保护范围 | 通过 | CRC、SHA、397 成员、142 保护源不变；非运行加载证明 |

本轮全局审查及测试检出的候选副作用均保留：

- `stop_gate_before.log`：初版把源 ACK 加到 `for_stop`，发送端失效后实测停车也无法结束隔离；分清停车证据与新运动授权后，通过新回归。不是旧现场已证原因。
- `clock_step_before.log`：初版双时钟在墙钟先前跳后回拨时，曾失效的需求可再次有效；新增失效锁存及屏障恢复，通过回归。不是旧现场已证时钟回拨。
- 另补未来 Native publication 不得污染后续合法消息的序号时间水位测试，防止新去重规则引入恢复障碍。
- 初次测试的 AST 测试夹具漏 `VehicleControl` 命名空间、同刻重复 tick 令油门积分 dt=0、隔离测试旧输入计时，以及采集脚本编码名拼写问题，均只修测试/诊断并保留早期日志；不改生产保护使测试通过。
- `protocol_before.log` 是新模块尚不存在时的导入失败，不冒充旧现场故障复现。旧机制的实际行为反例、具体时间签名及轨迹证据仍引用 17 日原审计。

## 剩余工作与明确限制

- 这次源身份起点是**速度环输出**；RPP→速度环的原 `Twist` 仍无源 header，不能称全部控制链端到端时间已对齐。
- .5 秒有效窗内的有序延迟需求仍可能被下发；传输延迟没有被消除，新协议使其可观察且不能在回调处续命。需要在同版本组件上测量源→回调→RPC→物理响应，再定位 DDS/执行器调度。不要直接声称切 BEST_EFFORT 会解决。
- 本地控制 RPC 可能跨越截止时间；晚 ACK 会失去源健康资格，但这不撤销已进入 SDK/引擎的请求。仍依赖原车辆租约和下一次 watchdog 制动，不能把 Python 截止误说成引擎硬实时停止保证。
- `tracking_preview` 仍是几何模型，尚未用真实延迟/制动力验证动态刹停与 cusp 捕获；现有严格到达、反向和碰撞保护继续保留。
- 持有 frontier UUID 持续缺失及整体无进展的有界重关联/退役尚未修改。当前环境缺 ROS C++ 构建验收；本轮保持该模块不变，避免叠加未经验证的生命周期改动。
- 必要现场验收与 DDS 总检未全部通过，按 AGENTS 规定不提交 Git、不宣称规划、稳定性、视觉记忆或完整搜索修好。下次先验证 ROS 类型/配置与故障交接，再测动态控制；完整搜索及记忆增量实验仍在后续。
