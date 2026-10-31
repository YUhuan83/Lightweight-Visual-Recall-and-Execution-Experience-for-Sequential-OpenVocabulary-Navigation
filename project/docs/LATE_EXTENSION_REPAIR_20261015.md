# 2026-10-15 后台续接完成顺序修复

## 原因和最小修复

上轮session205942出现173秒停车。现有方法反例证明：旧FollowPath先完成，后台续接结果后返回且含换向时，仍缓存prepared_extension，旧完成回调已不再到来；新GraphNav路径又可能被近似重复保护抑制。上轮未记录完整候选路径与prepared内部字段，因此仍不能宣称历史现场状态逐字节复现。

`nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py:execute_checked_path` 现在只在following仍为true时执行缓存/替换。旧动作已终止则进入正常motion_queue/send_motion_segment，重新受实际姿态、完整/局部costmap、新鲜度、执行器健康、动作隔离和换向握手限制。不修改目标判定、单位、坐标、时间戳、代价图、Smac/RPP原生库、速度或车辆物理。

完整diff保存于test_artifacts/late_extension_repair_20261015/production_delta.patch。冻结435成员SHA `262264de3fc9ed131fafd67a7a43901f01bf467c577c0abccdc8af6fd8535377`；相对0d145100仅上述方法所在文件与test_planner_execution_ownership_ros.py变化。旧源码/manifest与部署包按SHA保留，回滚需要停止实验并重建Python包，不能在活动场景直接替换源。

## 回归与限制

- 新增10测试。修复前8项初始扩展中4项失败，证实晚到换向路径无人执行、晚到前进路径误作replacement等；原日志callbacks_before.log保留。另2项补充真实调用派发和保存可行路径后另一个heading失败的顺序。
- 384项全部生产回调替身通过；50PASS/0FAIL/6WARN全局离线通过，范围重叠，不能相加为仿真次数。正常与逆序、局部碰撞、quarantine、验证拒绝、query切换旧回调、同端点候选比较等均有覆盖。
- 实际安装模块SHA与冻结包一致，重建wildos_nav2_adapter。Smac/RPP库SHA分别保持f915a4fc.../bd96eb9d...。
- 首服务器测试入口误将多个专为ROS替身写的整套测试直接投入真实消息环境，109项中7FAIL/9ERROR，原因逐项含mock publish属性、SimpleNamespace代替真实Node/Header/Time、重复init及替身Clock.steady字段。保留build_and_tests.log/test_installed_initial.py；不为通过这些夹具修改生产。
- 适用的真实ROS安装回归：PlannerOwnership25/TrackingHandoff5/ShortEndpoint2合计32项通过（确切各套数量以日志为准），路由118项独立context通过。其余替身套件以384回调结果报告，不冒称全部通过真实ROS/DDS。

## 现场完整复测（进行中）

- 启动前两次ps无相关残留、空间167GiB，部署旧435成员逐一SHA核对、仅2文件delta检查通过。
- session_20261015_213429，field_late_extension_search_20261015_213428；一次Town/真实SBR/green residential house/420秒；原保护保持。模型/ACK/真实光心/costmap/运行来源预检已通过。
- 独立诊断记录器补录/wildos/full_planned_path，关闭后再读SQLite；生产算法与本次最小变更清单不受影响。真实RViz双栏从发送query前录制。
- 最终视觉到达、停车、清理、长停驶统计和录像仍待封闭；不宣称修复完整验收，不创建commit。


## 现场封闭结果与新反例

session213429实际421.525秒超时，Native217.792米、最大离站53.556米、最终离站36.677米。41following/33机动6段，64连接搜索均有候选（32触及时限），1次no_vehicle_path。speed<.05且持续>=5秒共5段，分别约10.00/7.00/9.71/8.90/13.30秒；没有上轮173秒静止。但轨迹不同且本轮full_path_handoff_review的completion_first_cases为空，未捕捉到本次修复针对的准确现场竞态，不能把静止缩短当作受控因果证明或稳定性成功。

补录22份完整候选路径，含63.88米/7 section的候选。它说明短距离端点也可能引出多次机动，应继续审计端点朝向和执行代价；候选发布不等于整条路线均执行，不直接凭此修改曲率或安全约束。

870真实评分、20反馈20写入、759检索命中、GraphObservation最大效用差.190015；经验修改分数计数0。mask/目标确认均0，完整绿色房屋到达失败，20轮成功未增加、不进入多目标。3626控制器tick源龄median.2844/P95.4370/max.5664秒；外部新鲜而内部>.5计数0，模型快照max3.670秒，性能风险保持。

两次执行失败：329.902秒execution_stalled/actuated_no_path_progress，5秒进度-1.5745米；374.554秒FollowPath中止，controller日志确认为bounded command arc碰撞拒绝。两事件前cmd请求倒车、实际gear=R、通信ACK/执行器健康正常，但Native direction·velocity曾为正，即车实际前行；后一个事件还同时存在碰撞保护，不能把两个事件全部归为同一故障。

进一步核实到速度反馈接口缺陷（未在本轮修复）：cmd_vel_to_control.on_state只取hypot(vx,vy)，drive_outputs和积分误差均使用速度模长。当前代码无法区分“正确倒车1.2”和“请求倒车但前行1.2”，两者都可输出throttle=0/brake=0。reproduce_speed_direction_loss.py抽取当前on_state原方法并调用生产drive_outputs，使用相反速度的两状态复现。此契约缺陷确定存在，现场R挡溜动与之吻合；地形坡度/传动和轮胎力对滚动的具体贡献未闭环。下一步应独立修复有符号纵向反馈和反向移动时的制动交接，同时保持用于换挡/超速/静止的速度模长，覆盖前/后行、侧滑、坡道滚动、方向或状态缺失、换挡和恢复，不能仅提高油门或取消保护。尚未修改此速度环。

SDK持续停稳.5秒<.05m/s并退出，session errors=[]，两次独立ps无本轮残留。原ROS退出KeyboardInterrupt仅清理阶段，不归成搜索期崩溃。

### 本机证据

- captures/20261015_late_extension_retest/rviz_live.mp4：真实左2D/右camera，3407帧/425.875秒/8fps，全部解码，SHA4ab562df8aa1c1ce47282e54a8b3f5391f820dc0af4cab1e1470643335a33567与服务器及归档一致；无独立front_camera原始流MP4。
- test_artifacts/late_extension_repair_20261015/core_evidence.zip：193成员SHA/CRC全部通过，SHA124cb3ac9e078063ea3c0c4f2dd9983d3a8f571ca74a4159d30fbe75dd589024。规划/控制/接口bag、传感器时序/TF/完整路径及诊断齐全。
- 大原始点云sensor_interfaces_0.db3未随core下载，完整保存在服务器/home/ly/wildos-console/late_extension_field_20261015_213429.zip，194成员，外层SHAd86fe33b931711afd71080a39b64fb793d3f6ff86a15c157005359f160a19343；外部原始成员SHA988c5384724c0c2e09cca8c5230e2f84c17262db22eabba6af1d8997e6a3a0cc，下载core中的EXTERNAL_RAW_SENSOR_MANIFEST.json可追踪。不得称本机持有该大点云原始库。
- failure_control_review.json保留两次失败前后Native挡位、油门、制动、signed speed与健康；reproduce_speed_direction_loss.json保留方向丢失反例。

交接修复组件回归通过，完整现场任务失败、特定竞态仍无精确现场验收；速度方向缺陷待修，保持候选未commit。
