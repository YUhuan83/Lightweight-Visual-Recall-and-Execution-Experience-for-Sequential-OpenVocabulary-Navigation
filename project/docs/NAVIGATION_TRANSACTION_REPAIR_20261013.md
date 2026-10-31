# 导航事务修复 S01/S02/S03（2026-10-13，仅离线）

## 实现与边界

用户要求先修当前bug。本轮未连接服务器，未运行真实ROS/DDS、C++或车辆运动。没有完成完整搜寻、20轮成功率或记忆返站；必要现场验收待验证，按AGENTS.md不commit。

| 问题 | 处理 |
| --- | --- |
| S01 接受/替换响应缺席 | 单独请求身份、默认5s接受期限，超时明确失败并保持执行 |
| S01 动作结果缺席 | 默认10s没有当前动作feedback；或已进当前位置/航向容差、测量停驶5s仍缺结果，明确失败 |
| S02 重复接受/终态 | 接受只注册一次结果；消费终态前退休active serial，旧回调不能清下一请求 |
| S03 清目标服务缺回复 | 默认5s、1s退避、最多3次；独立serial/query/fence意图及旧请求排空围栏 |

正常长路径持续有反馈且未停在终点，不按总行驶时间取消。期限用monotonic，监视和服务重试定时器用STEADY_TIME；既有传感器源戳及视觉确认围栏不改。默认期限还需现场测频率、延迟和正常最长响应间隔。

`navigation_transactions.py`管理待接受、活动和退役动作。超时、目标/模式切换、输入/执行器失效后保留已发但结果不确定的事务。`/wildos/execution_hold`有owner、单调revision及源墙钟时间：物理速度环输出制动、理想载体下发零运动，保持期间拒绝新速度；释放清速度缓存，要求新指令，保持/释放周期重发，拒绝旧消息。

迟到接受只请求取消并观察终态；取消ACK不是停稳。全部不确定动作收到SUCCEEDED/CANCELED/ABORTED，加上新鲜停稳证据连续0.5s，才经原start_segment的目标/地图/执行器保护恢复。物理要求制动ACK、速度<0.05m/s、执行反馈健康；关闭FollowPath也必须检查这份反馈的新鲜度。迟到成功只作为动作已结束的证据，不授予视觉到达或道路经验成功。

永久缺接受/终态或异步异常仍明确保持，不能安全自动恢复；现场必须核对服务端动作、控制门并协调恢复，禁止伪造ACK或删记录强行继续。当前消息传递、实际制动和停止时序尚未经过真实DDS/车辆验收。

Trigger服务端没有请求身份。仅隔离旧回复，仍可能让旧请求晚执行清掉新目的地。因此最新意图成功且所有旧请求收到完整回复，才relay目的地；传输异常不作为服务端执行完成证据。预算耗尽是graph_reset_failed，路线/目的地围栏保留；最后请求迟到且旧请求已排空可恢复。MemorySelection和TargetObservation清目标入口统一退休请求/预算。

## 回归证据

目录：`test_artifacts/navigation_transaction_repair_20261013/`。

- before保存旧运行源码、94f6候选包、原审计及修复前规范测试：5方法/两种载体共6处断言失败。原审计JSON未覆盖。
- 最新358项回调替身通过，含34新增事务方法（测试覆盖有重叠，不能当仿真轮数）。覆盖正常、延迟/缺席/重复/乱序、取消ACK、接受/结果异常、目标/模式切换、新鲜度、两种执行入口、停稳恢复与STEADY_TIME定时器接口。
- 10项审计回归通过，旧S01/S02/S03遗留状态不再复现；未修的候选/预算/曲率/前视/terrain反例仍保留。4条合成记忆目标链通过，仍供应感知特征、定位、图路线、直线规划和完美运动，不是完整感知/导航/仿真验收。
- 全局结果见full_report.json；独立解包回调、目标链和审计见candidate_verification.json及unpacked_*.log。WARN是环境/现场范围限制，不当成通过门槛。
- mode_stop_freshness_before.log保留复查中新反例：关闭FollowPath可能绕过停稳反馈新鲜度。for_stop检查修复后最新回调通过。早期7处/2处cusp回归失败来自fixture跳过接受或同步接受后仍预期planned，已按真实接受/结果流程更正，未放宽请求互斥；失败日志保留。
- after_audit.json冻结测试前后当前生产来源，changes.json与上一候选包逐成员比较变更；包逐字节匹配当前源。补齐新工具依赖的传感器配置与历史转向证据，字节保持原样。

冻结包位于`deploy/20261013_navigation_transactions_candidate/`，精确SHA/清单/验证范围见CANDIDATE.json及manifest。必须同包部署bridge、反馈、记忆交接、两个执行入口和新模块，不能只换bridge。

## 剩余与现场顺序

P01/P02恢复预算/候选归属，P03/P04/G01几何/前视/短换向，T01运动区域地形、E01/E02引擎/退出未修。车辆速度、真实物理/轮胎碰撞、8m半径、视觉模型及视觉到达判据没有修改。

现场先核残留、版本/实际导入/控制门；维持真实SBR与传感器。先验正常feedback频率，再受控测试接受/结果超时、替换乱序和清目标迟到：核对保持消息、制动ACK、全部终态、实际停稳、新指令恢复。组件通过后再恢复完整单目标；不直接进入20轮或多目标。

## 2026-10-13 服务器短物理路径段与构建缓存清理补记

继续用户授权的现场验证，并在测试后按用户要求清理服务器文件。测试前核对12个候选install相关进程、0个build相关进程。仅删除候选包可再生成的build中间目录（2,296文件，227,425,712字节）；删除后确认目录不存在、install与zip仍在、12个install进程仍在。服务器/home/ly可用空间从299,631,452,160升至299,864,813,568字节。未清除运行栈、工作区、候选源码/安装、诊断日志或其他缓存。

| 阶段 | 结果 | 证据 |
| --- | --- | --- |
| 5秒运动前接口探针 | 通过；传感器与Native控制健康字段均活跃 | test_artifacts/server_validation_20261013/physical_stack_pre_motion.json，SHA256 A0FCC8C16656EC77D1F36C50F755F750FA58591CF9AA12843B62DB95A3896F38 |
| 第一次物理段预检 | 未arm，无FollowPath动作/位移；守卫失败文本与其内嵌健康快照矛盾，作为测试预检不一致保留 | test_artifacts/server_validation_20261013/physical_follow_trial.json，SHA256 C4F8EF69E1C3922C2BC3B03E79368E50B5FFA7A6724CCA0FDF8323EC0B78F9BA |
| 重试物理段 | 通过；CarLike/Ackermann、8m半径，约14.033m到审阅waypoint，实走13.550m、终点误差约0.489m，真实制动停稳；FollowPath动作与路线终态闭合 | test_artifacts/server_validation_20261013/physical_follow_trial_retry1.json，SHA256 D2253EADE151653D721184A3595A1AE3B7A9D45DA094C331668A544D75170D11 |
| 服务器文件清理 | build中间目录删除并验证未被活动进程引用；安装、候选zip、报告保持 | test_artifacts/server_validation_20261013/server_validation_cleanup_and_followpath_20261013.json |

本轮验证的是一段审阅waypoint物理执行，不是绿色房屋识别/到达；visual_arrival_confirmed=false，不计入20轮目标搜寻成功率，也不代表视觉记忆交接或完整双目标流程成功。未录像，按当前证据规则保留少量必要日志。没有生产源代码改动，不commit。