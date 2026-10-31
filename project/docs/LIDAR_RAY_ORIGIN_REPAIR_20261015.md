# 2026-10-15 LiDAR 射线原点接口修复

## 已确认的问题

PointCloud2 的 header frame 表示点坐标使用的坐标系，不一定是 LiDAR 光心。现有 cloud_filter 将点变换到 `spot1/base_link`，Nav2 两张 costmap 未设置 `sensor_frame`，于是以车辆后轴原点作为观测和清障射线原点。

桥接的 `spot1/ouster/front/os_lidar` 也是历史虚拟点云坐标系，位于 StateSensor 原点，不是传感器真实安装位置。只把 costmap 改用这个虚拟 frame 仍不正确。

2026-10-15 对正在运行的原始 SBR rig 做 SDK 只读位置查询，并用前后车姿校核。低位移样本（前后位置差 0.016～0.090 mm）得到实际光心相对后轴约 `(1.252, 0, 1.560) m`。原 WildOS occupancy/terrain 射线使用 StateSensor 原点，XY 同样未与实际光心对齐。

证据：`test_artifacts/sensor_origin_repair_20261015/lidar_origin_observation.json`。早期一次 SDK 查询误用了 ROS custom_frame 名称，读取失败且已断开连接；更正为实际 SDK 名 `front_lidar` 后取得上述数据，没有创建传感器、暂停或移动汽车。

## 修改与全局影响

- `sensor_geometry.py` 增加当前 SBR rig 的可覆盖安装标定 `SBR_LIDAR_OFFSET`；`lidar_offset` 动态变化要求协调重启。
- Adapter 保留旧虚拟点云 frame、点坐标、采集源戳、车体/导航 frame 和相机外参；单独发布 `spot1/base_link -> spot1/lidar_ray_origin` 静态 TF。
- 实车两张 costmap 的 marking 与 clearing sources 都显式指定该光心 frame。Nav2 按点云源戳取得光心姿态，而非改变点坐标。
- WildOS occupancy 从相同的真实光心 XY 发出射线；terrain 将该三维光心经过采集时的实际车体旋转，投到其水平坐标系。原理适用于标定外参的传感器，不修改转弯半径、车速、物理、碰撞或目标到达门槛。
- ideal_point 未切换其既有 ray-origin 标定，也未改变该 profile 的 costmap 配置。真实 SBR profile 的标定不能声称适用于其他传感器安装。
- runtime contract 同时核对两个 costmap 的实际 sensor_frame、Adapter 的 lidar_offset 和 /tf_static 中光心变换；不一致时保持控制关闭。

## 反例与验证

- 新增 `test_lidar_ray_origin.py` 在修改前 1 failure / 2 errors，原始 `before.log` 保留；覆盖 costmap 两源配置、虚拟点云 frame 不变与光心分离、外参修改必须重启。
- 实际 Humble Nav2 ObservationBuffer 隔离探针：不设置 sensor_frame 时 origin=(100,200,50)，设置真实光心后 origin=(101.252,200,51.56)，点的世界坐标始终为 (102,202,49.85)。旧射线会经过 (101,201)，真实射线在同一 y 上约 x=101.626；因此会清除不同单元。此为受控组件反例，不等于首轮停车地图的精确重放。
- 旧冻结版本实际 ROS node_geometry 4 项中有 2 项失败：夹具调用 lidar_cb 后未执行现有延迟队列 flush，尚未产生点云/terrain。仅给夹具补 flush，未改变生产消息配对逻辑。新测试首次又因 mock OccupancyGrid 返回 float 数组失败，改为与真实 mapping 相同的 int8；保留旧基线和失败日志。
- 初次修复版 session203653 预检拒绝：新 lidar_offset 为 DOUBLE_ARRAY，但旧通用读取器只识别 STRING_ARRAY，错误报告 None。原点 TF 实际正确、目标数 0、两门未解锁，无搜索录像。增加严格按 ROS 类型解析数值/字符串数组的 runtime_parameter_value 和回归，不放宽错误类型。
- 最终 435 成员 SHA256 `0d145100656f87d692b6a81fdfa45a97b11c6ee30d98e46929cdf4051ea8328a`，相对 ef3fc7 的变更均限于上述原点契约、参数解析、相关测试/预检。冻结 `deploy/20261015_lidar_origin_candidate`。
- 最终离线 50 PASS / 0 FAIL / 6 WARN，374 接口替身回调包含在全局范围中；服务器 3 原点、5 真实 ROS 几何、24 runtime、32 occupancy、5 terrain 全部通过。实际 Nav2 探针通过，C++生产源码/库未修改。

## 修复前完整诊断轮

- session201924：421.715 秒超时，215.416 米，41 following / 31 maneuver_done / 2 segment_done；7 follow_failed（4 次单方向反向保护、3 次碰撞弧检查）。54 连接搜索均非零候选，25 次触及时限但仍保留候选。没有目标确认，完整搜索失败。
- 控制器 3539 tick 源龄 median 0.2568 / P95 0.4162 / max 0.5968 秒，其中外部 odom 新鲜但控制器 >0.5 秒的 tick 为 1，不能写成 0。
- 同轮首/末失败点云分析没有复现 session163334 的零连接候选状态；不能以本轮射线证据断言过去全部停驶都是此原因。
- `captures/20261015_sensor_origin_baseline/rviz_live.mp4`：424.625 秒 / 3397 帧 / 全解码，SHA256 `f0a38ee458d0c5a39dd8570e0d905671e3fd759a0f223dbb8d43370ecee2c9c8`，失败录像。

## 当前现场范围

最终修复版 session203918 已通过源、参数、光心 TF 和传感器预检，完成一次 Town 地图中的 420 秒绿色房屋搜索，结果仍为失败：

- 421.850 秒超时，Native 176.337 米，48 following / 42 maneuver_done / 4 segment_done，1 次碰撞弧检查中止；本轮反向保护中止与 no_vehicle_path 均为 0。69 次连接搜索都有候选，其中 63 次触及时限但仍保留候选。不能把这 63 个 timed_out 标志当成 63 次导航失败。
- 相比修改前一轮 7 次 FollowPath 失败，本轮为 1 次；两轮实际轨迹不同，没有做同一时序观测的受控行驶消融，不能据此宣布失败率已降低到某个统计水平。
- 6 段 Native speed <0.05 m/s 且持续超过 5 秒的静止，最长 19.793 秒。4 段开始于 replanning_after_maneuver、1 段 waiting_for_costmap_update、1 段 following；开始状态不能证明整个静止窗只有一个原因，也不直接称物理受困。详单独取回的 `spatial_review.json`。
- 最大离站 36.663 米，结束离站 21.645 米；累计路程不代表探索覆盖改善。未产生目标掩码、定位或确认，搜索未通过。
- 1060 次真实评分、17 条反馈全部存储、898 次检索命中；memory_modified_scores=0，仍为如实的正反馈中性先验。模型调用数包含预热，不作为目标期 FPS。
- 控制器 3579 tick 源龄 median 0.2360 / P95 0.4043 / max 0.5339 秒；外部 odom 新鲜而内部 >0.5 秒计数 0。原始 odom max 0.6171 秒、RGB max 0.6743 秒；模型快照 median 1.5240 / P95 2.0703 / max 2.5738 秒。没有解决全部处理延迟。
- SDK 持续停稳、session errors=[]，独立进程检查确认本轮引擎、传感器、导航、模型、RViz、录制器均已退出。

### 受控真实点云重放

对已封闭 session201924 的 8 份相同点云，用相同障碍过滤，只改变 occupancy 的射线起点：每份 82～170 个栅格状态变化，障碍端点逐格完全一致。每份有 41～73 个旧 free 单元在正确原点下为 unknown，也有新的可观测 free 单元。这证实实际扫描受错误原点影响，不代表模拟器地面真值或某次碰撞的唯一原因。详 `recorded_ray_origin_replay.json`。

### 录像与归档

- 修复前完整归档 `baseline_evidence.zip`：116 成员、SHA256 `9838ce807a078fe242ca448a549166c4c205312a0955095dc40d0fdc2c4e49ba`，全部成员 SHA256 / ZIP CRC 已核验。
- 修复版完整归档：本机 `test_artifacts/sensor_origin_repair_20261015/field_evidence.zip`，服务器 `/home/ly/wildos-console/sensor_origin_field_20261015_203918.zip`，148 成员、SHA256 `6b7dd4c8748a7d94726a2558ef8e043f0e0143d2fd625d57b7c60ceb4177bfdd`；全部成员 SHA256 / ZIP CRC 已核验，所取回 MP4 与归档中的视频哈希一致。原始 SQLite 保存在完整 ZIP 中。
- `captures/20261015_lidar_origin_retest/rviz_live.mp4`：424.625 秒 / 3397 帧 / 8 fps / 全部解码，SHA256 `1311939d1b59ddd4bb116e415eadc27815fa559c703ff3c04f1a32a40613693c`。真实双栏 RViz，右侧含实时相机；本轮未启用独立前摄像头原始流 MP4。录像为失败证据。
- 纳秒重放采样初版使用浮点 linspace 导致首个查询时间舍入到第一条消息之前，诊断脚本已改为整数算术。生成分析脚本时另有一次换行转义语法错误，改为直接上传语法已验证文件。两者均发生在只读离线分析，未进入生产或改变现场执行。

射线原点缺陷修正不代表全部 FollowPath、连接器、植被路由或目标识别已解决；没有完整视觉到达验收，不计 20 轮成功、不进入正式多目标、不创建 Git commit。

下一步应沿同轮时间线拆解机动后重新规划/清图等待、当前碰撞弧的实际输入，以及 frontier 选择与真实车体执行代价之间的交接；保留本次射线修复，不用继续改变外参补偿其他问题。
