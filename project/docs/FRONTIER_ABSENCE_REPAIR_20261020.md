# frontier持续缺失的目标退役与动作交接（2026-10-20）

## 范围和修复前证据

处理OPEN_ISSUES第5类中的F12：`ExplorationTarget::frontier_resolved(nullopt)`一直返回false，Planner继续持有已从图中消失的frontier。F11“多次局部成功但整体搜索没有收益”没有在本批解决。

先读取PROJECT_MEMORY与前次416成员冻结源。本机`test_artifacts/frontier_absence_repair_20261020/before/`保存9个文件改前字节和SHA；服务器`/home/ly/frontier_absence_repair_20261020`从同一416成员父版本逐SHA复制。未改前次通过的Smac、车辆几何、Native或物理。

- 修复前实际C++42项有2失败：连续新鲜缺失无法退出、恢复/采集断档后重新累计也无法退出。
- 新退役原因的桥接反例测试在旧逻辑上122项8失败/1错误：旧桥会按通用reset取消动作。这是新增协议必须联动的反例，不是说旧Planner已经发送过该原因。
- 初次验证脚本因PATH缺colcon未构建；仅修验证脚本环境后复现上述C++反例。失败日志保留，不算生产导航故障。

## 数据流与约定

Mapper完整图快照→评分精确快照配对→Planner持有的exploration_id及目标位置→缺失状态判断→退役消息→GraphPathBridge→当前动作租约/新意图/查询切换。

- 仅接受与目标同frame、比选择及上次证据更新、非未来且未超age的图来源戳。使用来源戳跨度，不用重复回调或本机等候时间累计缺失；源快照间隔超过stale_observation_seconds时重新累计。
- 默认缺失窗口10秒，经只读`missing_frontier_timeout_seconds`显式配置、launch与runtime gate核对。图的默认新鲜度5秒不变；短暂消失容忍，持续没有图不凭墙钟推定目标消失。图中节点重新出现则打断累计；节点存在但属性或位置不可分类时保守持有，不把它当确定缺失。
- 10秒表示在当前地图接口上撤回无持续支持的探索意图，不证明物理世界永久移除了物体，也不证明传感器真实采集新鲜；第6类采集身份/epoch问题仍开放。
- 明确non-frontier沿用原`frontier_resolved`；持续缺失产生`frontier_missing`。均不冒充观察成功，不增加visited，不把区域/边永久标为障碍，不写错误执行学习标签。
- 桥将两种原因共同交给既有有界动作交接：保留健康且满足原距离/时间/车体检查的已接受动作，只撤回旧队列尾部和未下发的后台意图。新障碍/过期输入/坏执行器仍走原停车隔离；更换查询仍撤销旧动作。

## 变更

- `graphnav_planner/include/graphnav_planner/exploration_target.hpp`：连续缺失来源证据、reset/新目标隔离与失效原因。
- `graphnav_planner/src/planner_node.cpp`：只读窗口、缺失原因发布、存在但不可分类的保守保留。
- `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`：缺失退役复用既有动作、恢复和后台交接；不另建取消机制。
- `official_adapter/launch/official_graph_stack.launch.py`及`runtime_contract.py`：默认10秒入口及部署核对。
- C++、动作交接与runtime测试同步更新，改前反例保留。

## 验证结果与限制

- 本机590项既有回调替身全通过；本批123项相关回调/配置测试通过。它们覆盖拒绝、恢复、迟到结果、后台规划、目标切换、障碍、旧动作终态和停车隔离，不是现场证据。
- 服务器实际C++43项全部通过；实际ROS类型与Node下123项回调/配置测试通过，回调测试本身没有动作服务器。
- `dds_20261020_171520`实际C++ Planner→DDS→Bridge→真实ROS FollowPath动作句柄联测6阶段通过。已接受1动作，持续缺失后选新目标而取消请求0；查询切换后取消请求1，旧动作终态5(CANCELED)，迟到旧退役消息没有再次取消。只读窗口服务修改被拒绝。
- 该DDS探针输入为合成图/里程计/健康状态，动作服务器不驱动车辆；为隔离所有权，手工安装了一次合成已接受路径，不能当Mapper→全局/局部规划→真实车辆全流程通过。诊断窗口.6s、stale=.2s只用于时序控制，生产10s/5s不改。
- 首次DDS探针因默认禁用桥没有创建动作客户端，在4阶段、未接受动作时退出；后继显式创建测试ActionClient重跑。该夹具缺陷与失败结果保留，未修改生产来迁就测试。
- `entry_20261020_171927`实际physical入口的图栈启动核验通过：默认缺失窗口10s/default profile，与runtime gate一致；108项安装Python/config及四库身份核对，运行Python与入口来自本轮install。未启动Nav2/Native/车辆，不能当整车通过。
- 整车缺失frontier场景、F11全局无进展、完整目标搜索和20轮成功率仍未验收。不录制视频、不替换正式安装、不commit。

## 封闭证据

服务器与本机`test_artifacts/frontier_absence_repair_20261020/sealed/evidence.zip`：8740523B/480载荷，SHA `21f39276a22e1d0557e6ab04c2e57714807133dea2eea665c2d74faae9614822`。本机`verify_evidence.py`核CRC、全部载荷SHA、416冻结运行源与本机一致、9份改前源、实际C++ XML、DDS/动作及入口结果；`local_verification.json`为通过记录。`verified/`保留紧凑结果，失败构建环境、修复前C++反例和首个DDS夹具失败均在封包中。结束进程清单[]，不能将封包校验当整车搜索成功。
