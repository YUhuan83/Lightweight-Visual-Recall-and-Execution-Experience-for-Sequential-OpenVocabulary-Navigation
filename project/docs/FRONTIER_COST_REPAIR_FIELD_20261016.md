# Frontier 车辆代价交接修复与完整重测（2026-10-16）

## 当前结论

**车辆代价比较已接入并发生真实的图层目标提交；完整绿色房屋搜索仍失败。** 不能将8次选择确认或4个探索点观察完成当成绿色房屋到达。20轮成功计数不增加，未commit。

当前生产/冻结439成员：`e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb`，本机 `deploy/20261016_frontier_cost_candidate/`。相对原生产11af6c17共16文件变化，含上一轮恢复交接5文件、新比较器/消息/C++节点/对应测试、安装与日志入口。现场未中途修改源码、物理、速度、半径、碰撞或到达标准。

## 实现与契约

- GraphNav在初次选择时冻结候选；最多查看排序前8项，使用Planner值拷贝检查已有图路线，仅提供24m范围内最多4个可完整连接的候选。探测不改变活动Planner的frontier/debug状态。效益沿用原语义效用、物理置信度与访问惩罚，去掉旧欧氏距离项。
- `ExplorationCostComparison` 只在Ackermann、低速、健康输入、没有活动/待定/隔离动作和对象/记忆占有时启动；固定同一实际起点，以现有ComputePath交易串行比较，最多12秒。使用实际路径长度＋原3m等价换向成本，以原0.004/m系数计算效益。保留原footprint、曲率、起终点朝向和tracking检查；选择前在同一最新地图上重新验证全部保留路径。
- 比较结果不发布为可执行full_path、不发FollowPath或假执行经验。新 `ExplorationChoice.msg` 携带原route header/endpoint、探索ID、起始车姿、候选UUID和代价；GraphNav核对原路线归属、新鲜同frame odom、<=0.25m/0.1rad起点变化、低速、候选仍可从图完整连接，才确认选择。更换候选时退役旧探索交易并创建新ID；保留原候选不改ID，不计为观察完成。
- Bridge等待图层确认后的新路线；ACK本身不授权执行缓存比较路径。正常规划/双地图/执行器检查仍然重走。丢ACK最多有界重发，5秒无明确路线则经真实clear-goal barrier重新建立归属；不能猜测哪个目标生效。重复、旧query、旧事务和frame切换不能复活旧选择。每query最多2次协议故障后回退正常探索。
- 恢复保留上一轮415c的动作下发失败释放、取消隔离与持续停稳后的意图交接。新增传输epoch交接反例检出比较缓存可能覆盖新路线候选，已在部署前修正；反例日志保留。
- 安装脚本增加graphnav_msgs，今后共构建7包，避免新Python/C++搭配旧消息。接口包记录vehicle_choice与selection_ack。所有新改动均有对应回归，不关闭安全保护。

## 修复前反例和组件验证

- 新“首个可行观察路径不得直接执行”行为测试，在冻结415c原实现上确实失败，记录原代码调用execute_checked_path；新实现通过。
- 新13项覆盖真实长度/换向成本、语义效益、固定起点、共同地图重验、失败回退、目标/记忆/动作隔离、确认与超时、乱序query及传输epoch交接。完整离线50PASS/0FAIL/6WARN，441回调替身通过；不是441次仿真。
- 服务器实际ROS消息13新项＋32恢复项＋35既有交接和新增取消恢复项通过；实际C++/DDS共9项（4选择协议新项＋5既有归属）通过，C++路线27项通过。7包Linux安装合同通过。均包含合成输入/动作替身，不等于物理到达验收。
- 实际Smac记录地图路径经新生产比较器/验证器重放：输入原24/25/20/21四个候选，选择25。其27.351m/4方向段，等价代价36.351m；原24为45.834m/6方向段，等价60.834m。保留语义/物理效益参与，不等同直接选最短。该重放使用旧已发布地图和路径、合成当前时间，未实际控制车辆。

## 部署来源

- `/home/ly/frontier_cost_repair_20261016/install` 提供新消息与GraphNav；`release_install` 提供最终Python。编译源与最终冻结C++/消息逐成员相同，attestation保留。未改旧消息结构，新增类型；实际运行来源通过启动门禁和/proc/maps核验。
- GraphNav库SHA `33b48f3c20fc234a1c87b30801c2b6b30046247af24e897f6350e685ca31ea39`；Smac仍f915a4fc、RPP仍bd96eb9d。原11af bundle位于 `/home/ly/wildos-console/bundle_11af6c17_before_cost_20261016`，原console和source identity备份于修复目录。
- 场景保持Town加油站、真实SBR车体、原轮胎/碰撞/速度/转弯参数。现场配置应复用 `/home/ly/frontier_cost_repair_20261016/field/sensor_full_console.json`；不要把默认不启用RViz的自助console配置当成录制现场配置。

## 首启被拦截（未进入搜索）

driver144940/session144941被预检阻止：诊断配置误复制main console的rviz=false/普通输出路径，没有visualization/RViz生产者。goals0，控制未解锁，没有搜索MP4。保存startup_failure.json与原配置，随后恢复上一轮已验证的现场配置，只更新新setup与库身份；同版本重启通过预检。此问题是本轮测试配置错误，不称模型识别/规划故障，也没有放宽预检。

## 完整现场结果：失败

driver：`/home/ly/wildos-console/field_frontier_cost_search_20261016_145142`；session：`/home/ly/wildos-console/audit_results/session_20261016_145143`。

- 单次query `green residential house`，420.889s timeout；Native累计93.147m，最大离站54.440m、结束53.183m。mask/定位/确认均无有效目标到达，未找到绿色房屋。
- 22次有效代价评估、8个唯一选择请求、8个selected ACK，7次实际替换探索交易；无selection timeout。同ID目标位置无漂移。4次探索观察完成，第一点12.929m/22.986s、第二点13.749m/25.232s；与旧83m/172s的首目标位置/轨迹不同，不能据此报告受控成功率改善。
- 21 Following，14机动完成、4段完成。3次FollowPath action6已与控制器原日志逐时对应，原因为 `Ackermann RPP detected collision on its bounded command arc`；另外19次下发前 `local_footprint_collision`。全轮22个follow_failed、12个no_vehicle_path、2个no_free_graph_candidate。
- 5段>=5s静止，最长216.248s，自1792133759.298延续到结束。恢复started/done均0。**这是未解决的执行/恢复失败，不能称规划已整体修好。**
- 967评分、7反馈写入、840检索命中、memory_modified_scores0，objects0。没有制造负反馈或把空路径当永久障碍；记忆命中不能算视觉召回成功。
- SDK持续停稳、session errors=[]，最终独立ps无本轮仿真/规划/录包进程残留。console内部video标记false指其自身不录制；本轮由外部驱动录制真实RViz。

## 恢复为何没有发生：已核查与剩余证据缺口

闭包后按bag接收顺序/时间重建实际forward走廊，前4个no_vehicle_path均能拟合原2–4m策略的退路，最近点年龄13.6/35.6/52.8/54.1秒、距车0.458/0.735/0.736/0.736m，记录双地图验证均通过；对应local源龄0.228–0.595s、odom0.310–0.368s、global<=1.644s，速度低于0.003m/s。第二次失败距首次约21.97秒，源记录的几何/时间边界没有直接解释恢复缺席。

这不是私有回调状态快照，尚未记录try_planning_escape每项内部门禁/失败历史/动作隔离状态，不能确定是哪项实际没有满足；不能把未触发简单归因TTL、倒车历史或时间戳。反向历史的诊断候选只有1点，不支持直接改成前行逃逸。**下一步先给恢复决策增加有界逐项原因/事务快照，用该反例核实最早不满足的条件；不要再盲目放宽地图、TTL或碰撞保护。**

## 录像与证据

- 本机失败RViz：`captures/20261016_frontier_cost_retest/rviz_live.mp4`，426s/8fps/3408帧全部解码，SHA `e160ee31a949a5407134f2bce9fcb9b2c1e2229f649bc0a3924ce06d9e0c02c9`。已主动展示并提供可发送路径；无独立front_camera.mp4。220s帧右侧为树林/土路，不能称绿色房屋到达。
- `test_artifacts/frontier_cost_repair_20261016/field_evidence.zip` 75有效载荷，SHA `da55d44a01266fceed8417934826c4ccea9b2a141ec5ef2994f08edaf6e9c306`；组件30载荷SHA `5838e69b7bb1074b462859f3e3e1827aea00341ee936246a0b8ab3d0c9f9e78c`。本机ZIP CRC及逐成员SHA通过，视频与归档成员相同。
- 1个大型点云/拓扑DB仍留服务器原session的sensor_interfaces，外部manifest有原路径/长度/SHA；不声称已全部下载。接口DB在本机核心包中。恢复闭包分析在核心包生成后补充，单独保存在同证据目录下。没有读取活动SQLite。
- 比较路径本身没有发布为可执行full_path；本轮包有代价数值、源地图戳、typed choice/ACK及实际路线/执行，不能声称已保存每条内部比较路径完整几何。

## 待验收

整体搜索失败，当前生产e6候选保留、原版可回退，未commit。下一项是恢复门禁来源与局部/全局碰撞约束交接；20轮稳定性、多目标和视觉记忆完整闭环仍未通过。

## 后续审计更正：探索候选确被新增退避清空

用户质疑探索被改坏后，原版对照与同轮实际 C++ 图回放确认区域退避耗尽候选。以 bag 接收 query 为零点，240.410 秒的 51 个基本候选全部被区域退避屏蔽，其中 28 个仍有完整图路；418.288 秒 56 个全部屏蔽，其中 29 个仍有图路。实际 239.727–312.354 秒无新 route_status/path，首个 120 秒退避到期后约 0.449 秒恢复选点。此前“不是永久障碍”不能代表这种临时区域排除是正确的；本轮主要退化证据是区域筛选清空，并非图边屏蔽切断唯一出口。图路仍不等同汽车可执行路线，这个窗口也不解释全部 216.248 秒静止。

详见 [探索退化审计](EXPLORATION_REGRESSION_AUDIT_20261016.md)。本次仅审计，生产不变；heading 未知硬否定等潜在缺陷与现场已证原因分别记录，完整实验仍未通过。
