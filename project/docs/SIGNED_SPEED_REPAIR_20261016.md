# 2026-10-16 有符号纵向速度反馈修复

## 已确认接口缺陷

上轮session213429倒车指令时Native实测沿车头方向前行，实际R挡与控制通信均正常。原cmd_vel_to_control.on_state仅保留hypot(vx,vy)，使速度环不能区分正反运动；同模长反向运动可被判为已达到目标速度。方向丢失反例保存在上轮reproduce_speed_direction_loss.py/json，历史现场失败还包含碰撞保护和可能坡道作用，不能全部归为同一根因。

实际服务器StateSensor消息同时包含世界坐标velocity与车头dir；生产Native发布器同一poll填入state['vel']/state['dir']。新drive_math.longitudinal_ground_speed将两者投影至归一化水平车头方向，单位m/s，符号为车头正向。保留hypot水平速度，用于换挡、静止、超速等安全判断；不通过挡位猜测运动方向，不使用异时刻额外TF。

PID按请求方向上的非负纵向进度计算比例输出，积分使用有方向的速度误差；实际反向滚动超过已有direction_change_speed=.15m/s时先全制动并清积分，停稳后按既有驱动流程恢复。此处复用既有换向停止阈值，未添加固定速度上限、调油门参数或关闭车辆物理。

方向无效/近竖直方向XY退化/非有限输入时制动，不以模长兜底；旧stamp不能覆写新状态。control_status增加measured_longitudinal_speed（无效时null）及invalid_motion_direction/opposite_motion_braking原因。不改变ROS话题、消息类型或时间戳所有权，原新鲜度、Native健康、hold、换挡握手与碰撞保护保持。

## 回归、失败证据与冻结

- 新增9项回调回归：两挡反向滚动、正常前/后行、世界朝向旋转与归一化、横向速度与纵向PID隔离、方向无效、旧消息/新状态恢复、积分重置/制动恢复、模长换挡保护、陈旧状态停车。
- 原版反例日志callbacks_before.log保留。首修复回归检出新增制动分支steering未初始化，已在部署前修正；另恢复测试没有推进20Hz时钟，dt=0按既有积分保护停输出，修正测试时间推进，不放宽生产dt保护。callbacks_after.log保留，失败版本未部署。
- 最终393生产回调替身全部通过；全局离线50PASS/0FAIL/6WARN，通过范围重叠，不作仿真成功数。
- 435成员冻结SHA `9bcaf8700786cbaf2ab40a22405121f20288fae5e8f51d6d777fbf5eb3d87997`，相对262264de只改drive_math.py/cmd_vel_to_control.py及test_cmd_vel_to_control_ros.py。独立deploy/20261016_signed_speed_candidate，production_delta.patch可复查。旧bundle_262264de/manifest按原身份保留。
- 重建wildos_nav2_adapter；实际ROS消息下9项新方向测试与32项原交接测试通过。Smac/RPP库SHA保持f915a4fc.../bd96eb9d...。不把回调+消息测试说成真实动作或物理验收。

## 完整现场复测（待封闭）

- 用户明确连接服务器继续；启动前ps无项目残留，147GiB可用空间。逐文件核对旧冻结源，限制3文件delta；原真实SBR车体、碰撞、半径、规划与视觉目标条件保持。
- session_20261016_085025，field_signed_speed_search_20261016_085024；一次Town/green residential house/420秒，模型与ACK/运行来源/传感器/光心/可视化预检通过。
- 记录实际RViz左2D右camera，补录完整候选路径、Native和控制状态。活动bag不查询SQLite；两终态标记存在后才封闭分析。本轮最终到达/停驶/制动恢复/退出尚待验证，未commit。

## 剩余风险

- 坡道先制动后再驱动可能反复触发，需要现场看制动次数、持续时间、实车移动及恢复，不能只看新分支输出正确。
- 现有最小油门、积分和发动机/传动负载响应仍可能不足；不以增加场景专用推力替代方向反馈修复。
- 宏观候选可能产生长路径/多换向，语义确认和完整目标到达、20轮成功率仍未通过。此次只修速度方向约定，不称所有导航故障已解决。


## 现场封闭结果

- session085025实际421.967秒超时，Native累计225.141米，最大离站40.634米、最终离站32.250米。45following/39机动/5段完成，7次机动后重规划、19次地图等待，FollowPath失败0/no_vehicle_path0。43次连接器搜索都有候选（31次触及连接预算仍保留候选）。4段Native速度<.05且持续>=5秒，最长12.304秒。轨迹不同，不能与前轮2失败直接推导修复效果或成功率。
- 8986份控制诊断均有有限有符号速度，方向/制动契约违约0；opposite_motion_braking触发0、invalid_motion_direction触发0。说明新反馈正常运行且本轮未误触制动，**没有现场验证反向滚动后的制动恢复**；其保证仍限离线/真实ROS回调9项，不把分支未触发当作已验收。
- 678实际WildOS评分，28反馈28写入，628检索命中；GraphObservation候选效用差最大.191248，经验修改分数计数0。mask/确认仍0，完整目标到达失败，20轮成功计数未增加，不进入多目标。
- 3899控制器tick源龄median.2497/P95.4182/max.5315秒；外部新鲜内部>.5计数0。模型图快照max2.584秒。车辆仍在小范围反复机动，短静止与规划效率问题未全部解决；没有仅凭跟踪无报错宣布导航完成。

### 状态源戳异常的核实与边界

控制StateSensor源戳年龄median11.482/P9511.647/max11.770秒，而接收龄约.1秒。独立Native速度序列对齐8184样本，最佳Native时间偏移约-.05秒（MAE .02124m/s），零偏移MAE .04983；若按源戳假设11.5秒数据延迟，MAE .87865m/s。证据支持“源戳时间域偏移”，不支持“控制使用11秒旧速度”的判断，不能把此日志直接当本轮停驶原因。

这仅比较一维速度序列，未证明StateSensor所有字段或精确采集戳均正确；源戳契约仍不一致。下一步核对原ROS桥_make_header时间基准/场景暂停与启动锚点，并与直接/wildos/native_state作来源对照，不能在未验证采集新鲜度时简单改成当前时间。当前生产本轮未改时间域或数据来源。state_clock_alignment_review.json保留完整范围。

### 证据与收尾

- SDK连续停稳、session errors=[]、独立ps无项目残留。
- 本轮真实双栏RViz：captures/20261016_signed_speed_retest/rviz_live.mp4，3392帧/424.0秒/8fps，全部解码；SHA9a7dca290a7886305f30317fc01365dd71d39bfd238163c8d003bc7be0ccfcc1，与归档一致，用户请求后已提供本机可发送路径。右栏camera，没有独立前摄原始流MP4，视频是完整搜索失败证据。
- test_artifacts/signed_speed_repair_20261016/core_evidence.zip：196成员SHA/CRC及视频一致性通过，SHA7c47463a9a3f6d5172c4244a3c1a019c461fea38927aff7751cc778b4bd70191。大原始点云SQLite仅在服务器完整归档，位置及成员SHA见本机core内EXTERNAL_RAW_SENSOR_MANIFEST.json，不称已下载该原库。
- 速度方向修复回归通过、真实正/反向正常执行有证据；反向滚动分支未现场触发，完整绿色房屋任务失败。保留候选与证据，未创建Git commit。

- 服务器完整归档：/home/ly/wildos-console/signed_speed_field_20261016_085025.zip，197成员，SHA0225603aa4840cdbd34127ae780f6f55e31824b53df79c7a5ccb84bf06790955。
