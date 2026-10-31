# 探索目标归属候选现场复测（2026-10-16）

用户要求部署并测试，随后指出加油站附近绕行、频繁静止。本轮完整任务失败；不计成功、不 commit。高代价路径、搜索覆盖和经验分类尚未修复，不能以目标交接通过宣布规划整体通过。

## 部署及测试条件

- 运行候选 436 成员 `11af6c17b0335766c50ce892bf128af5ab88f9a7a82571e0799f3e555e06d9cd`；旧 bundle/source_identity/console 配置备份。实际进程 `/proc/maps` 验证新 GraphNav 库 `cb188e47…`，Smac `f915a4fc…`、RPP `bd96eb9d…` 保持原身份；Bridge 与配置身份门禁和实际消息 17 项回归通过。
- Town 加油站，同轮地图仅加载一次，唯一 query `green residential house`；真实 SBR 723 节点/0 固定/467 车身碰撞/四轮各32碰撞节点/1449.72kg，实际8米、规划12米半径。没有调速或关闭物理/避障。
- 首启 session_20261016_095604 在发目标前关闭控制门时 SetParameters 响应超时，goals=0；已持续停稳和清理。未开始录像，不能提供该次 MP4。随后同版本完整重启，服务握手通过；根因尚未证明，不能归为模型或 GraphNav 算法失败。保留首启日志。

## 完整失败轮 session_20261016_095740

- 420秒搜索预算，result.elapsed_s=423.913（含结束处理）；timeout，没有视觉到达。Native累计196.599米、最大离站60.979米、末58.560米；不能把行驶里程当探索效率。
- 1024条 route_status/10个探索交易，同一交易目标坐标漂移0。39次Following/31次机动完成/4次segment_done，27次观察回传；**4次 observation_completed、5次 vehicle_route_rejected**，共9退役。正常观察退役和局部无路换点在真实车体得到组件证据，不等于绿色房屋成功。
- 首交易从发布至退役171.932秒，Native行驶82.639米；出生点至该目标直线约14米。最长发布候选42.068米/6方向段、位移15.423米；候选发布不等于所有方向段均执行。近点高代价及重复机动依然存在。
- 三次 follow_failed 均是 action_status=6/follow_aborted（elapsed109.755、126.953、362.337）；控制器具体中止原因还要与源码/栈日志逐项关联，不把 action status 单独当作物理碰撞证据。
- 三段 speed<.05 持续>=5秒：8.750、8.214、61.766秒。末段无路更换了5个探索交易、各尝试3/7/10/9/15个候选。Smac日志有最大迭代/未找到路，Bridge统一接到empty_path。**不是目标交接未发生，而是多组候选尝试仍没有可执行路线**；不能据此认定车轮空转、CPU负载或BeamNG接口是原因。
- object mask/定位/确认/visible approach均0；评分1068次、经验反馈27/写入27、检索926命中但memory_modified_scores=0。不能说识别出绿色房屋或学习了失败，也不能用无mask证明图像里不存在房屋。
- SDK持续停稳0.5秒以上、session errors=[]；独立ps无车辆/规划/RViz/观察残留。原始大点云/拓扑SQLite留在服务器原路径，闭包后仅以只读方式分析。

## 录像和证据

- 本机 `captures/20261016_exploration_owner_retest/rviz_live.mp4`：实际 RViz 左图/右camera，3423帧、8fps、427.875秒，137810319字节，全解码通过；SHA `569c7e9ef23b63c96c1bb2df81120cfeace689866793aa9d33eb9695130f937e`。已展示给用户；没有单独前摄像头MP4。
- `test_artifacts/exploration_owner_field_20261016/core_evidence.zip`：182085741字节，117个载荷逐员SHA/ZIP CRC通过，SHA `5e1d744072671f5eb82d06ccc5126a5d235ba1df64f71be7f45b5048546c86ff`。包含两次启动日志、实际部署/映射库、接口包、执行观察、Native、传感器时序及本轮视频。外部1个原始传感器DB的绝对路径、长度、SHA见 EXTERNAL_RAW_SENSOR_MANIFEST.json；不称完整大点云已下载。
- closed_review.json、core_validation.json、video_validation.json、loaded_runtime.json 保留统计与范围。Native位置与 odom参考点不同，目标1米约束以实际使用的odom参考点核验，不能混用车辆中心距离。

## 下一步：处理效率，而非重复整轮

先在同一封闭地图/实际车姿重放首个高代价路径和末段无路请求，比较目标朝向与车辆可行进朝向的路径代价，检查连续起点连接器与局部恢复的退出条件。新方案要同时覆盖评分收益、真实长度/换向代价和有限计算预算；不能简单提高搜索预算、放宽碰撞或转弯约束。分类原因和恢复接口通过后再现场，不靠刷轮次统计稳定性。
