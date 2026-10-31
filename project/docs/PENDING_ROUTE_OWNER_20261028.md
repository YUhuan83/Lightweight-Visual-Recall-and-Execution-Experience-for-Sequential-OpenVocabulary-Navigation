# 在途规划的图路线归属修复

v61 trial221644精确时间线：约1793197132.99发起(733.133,-68.707)的ComputePath；1793197133.081旧FollowPath完成；on_follow_result在pending检查之前把self.route/self.route_endpoint换成latest_route。1793197133.669仍返回原候选的计算结果，但planned记录graph_goal已是(742.396,-77.229)。因此此前把执行终点(733.326,-68.257)与graph_goal差异直接说成“主动缩短最新图路线”不准确，包含异步路线归属/日志串线。其他真实短候选选择和Smac失败仍可能存在，不能反向声称全部都是日志问题。

v62_pending_route_owner从v61隔离，仅调整on_follow_result顺序：旧动作完成后，在采用latest_route之前检查pending/replacement_pending；有在途规划时保留该规划已持有的route及endpoint，无在途规划时仍采用latest_route并正常接续。没有新增状态/计时器，不更改GraphNav或碰撞检查。

真实ROS新增两个回归：旧动作完成、规划仍pending时不改route/endpoint；无pending正常采用新route。旧版5项中1项失败，修正后验证如下：整组单进程回归再次段错误，日志保留；改为每个既有测试模块独立进程，沿用原套件和断言，全部900项通过，所有模块returncode0。不能将测试隔离视为生产ROS段错误已修，根因未定位。本次运行代码变更仅上述顺序调整，测试进程隔离不进入部署。

两部署包构建/475来源冻结/预检清场通过，正在目标后180秒绿房屋现场验证，未推广主源码或commit。源码test_artifacts/pending_route_owner_20261028；服务器v62_pending_route_owner；证据before.log、real_ros_review.log、isolated_ros_report.json及isolated_ros各模块日志。

## 现场关闭与审计边界

trial222746启动43.093/搜索180.124/收尾22.628秒超时，session errors=[]/console0，后检项目进程/X148空，未到达。odom同口径近停28.910%、最长7.801秒、累计124.463/净62.946米、测得前后方向切换49次，3处接收间隔>0.5秒。净位移增加但近停比v61更差，整体不达标，不推广commit。约query92.8秒首次确认接管至(748.805,-130.083)，之后未记录再次awaiting_graph_route目的地覆盖；保持不等于到达。

用planned事件replay_inputs.route_stamp_ns与对应GraphNav路径终点比较：v61匹配22份/v62匹配18份，均0终点不一致。这项审计不能独立证明运行结果归属已修，因为旧回调会先换self.route，后续候选请求/日志也会随新route切换，记录本身可能自洽。前述旧版(733,-69)请求→旧FollowPath终态→route变化可由实际回调及反例证实，但不能据终点差异断言每个短终点都由此错误造成，更不能声称本轮已验证修复效果。仍须核对“请求发起时的候选位置、请求路线”与回调上下文，而非只看planned最终字段。

本机field/{fluency_odom_audit,arc_trial_audit,route_binding_audit}.json保存；流畅性/完整目标未验收。源码变更仍仅2行顺序调整，没有扩大为图规划策略重写。
