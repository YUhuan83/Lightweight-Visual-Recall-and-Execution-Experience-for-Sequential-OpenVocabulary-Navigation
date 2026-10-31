# 导航检查流程：先离线，后最小现场验收

本流程针对同一次地图加载内连续目标搜索：先离开 Town 加油站寻找其他目标，再把目标切换为已记忆的加油站。成功条件是利用记忆选取可达观察位置、合理重新寻路；不要求沿原轨迹返回。静态检查通过不等于车辆可行驶。

## 1. 固定版本与入口（Windows，无服务器）

在项目根目录运行：

```powershell
python tools/package_official_bundle.py
python tools/offline_navigation_check.py
```

Town 仿真启用理想点传感器载体时，先在已加载的 BeamNG 场景中安装并启动 `wildosKinematic.lua` 固定节点配置，再运行 `ros2 launch wildos_nav2_adapter kinematic_ideal_point.launch.py`。此节点默认 `enable=false`；链路确认后再设置 `/wildos_kinematic_vehicle_control` 的 `enable=true`。官方适配器默认订阅 `/wildos/kinematic_state`，Nav2 默认只加载 `GridBased`。理想点能原地转向，平移沿当前朝向进行；没有转向半径和车身外形，BeamNG ego 只保留相机、LiDAR 等传感器宿主。

需要本机已有 numpy、PyYAML。脚本不联网、不调用 ROS、不发送车辆控制。输出 `test_artifacts/offline_navigation/report.md`、`report.json` 和每组测试日志。返回码 0 表示已执行检查全部通过；待运行验证项目仍单独列出。修改代码后重建部署包，再运行检查，防止发送旧包。

实际检查代码：`official_adapter`、`nav2_adapter` 和 `wildos_port/repro_check/src/nebula2-wildos/visual_navigation`。不要将另一个 `workspace` 目录的旧源代码混入对比。部署包覆盖两个适配器和原包工具清单；它不包含完整视觉导航源码，视觉导航改动仍需从上述 repro_check 路径单独同步。

当前 `official_wildos_window_demo.sh` 默认使用 `american_road` 的锯木厂出生点，且初始搜索词是 `door`。它不能直接充当 East Coast Town 加油站的验收入口。Town 测试必须使用加载 `east_coast_usa` 区域包的 Town 场景，核对实际 odom 距出生点不超过 3 米，再在同一进程内依次发布 `trash can`、`gas station` 查询。`offline_navigation_check.py` 将这两项列为 WARN，避免把其它场景的 PASS 误报成 Town 通过。

该演示入口还硬编码 `carrier_model=ackermann`、`planner_id=CarLike`、最小转弯半径 8 m，并加载有 SBR 足迹的 Nav2 配置。它只能验证 Ackermann 车模，不得用来验证 `ideal_point`。理想点测试必须分别启动 `kinematic_ideal_point.launch.py`、`nav2_official_control.yaml` 和 `GridBased` GraphPathBridge，状态源接 `/wildos/kinematic_state`，并在放开运动前确认运行状态中的模型为 `ideal_point`、半径为 0。该入口的 Lua 固定节点 profile 和传感器宿主重设步骤尚未合并为一条经过现场验证的自动启动流程。

2026-09-27 的双区域测试报告明确说明车辆保持静止，只验证传感器、图评分和图路径。该报告不能证明 Nav2 costmap、FollowPath 或车辆行驶通过。检查器会给出对应 WARN。

## 2. 检查顺序与判定标准

| 顺序 | 检查内容 | 离线通过标准 | 失败时处理 |
|---|---|---|---|
| 1 | PointCloud2 解码 | 组织化点云、row_step、填充字节、截断输入正确处理 | 修复解码，不能靠扩大障碍容差掩盖坏点 |
| 2 | 点云限量与车体过滤 | 覆盖扫描头尾；去除车体内点；保留车体外低障碍 | 检查采样顺序、车辆尺寸及传感器原点 |
| 3 | 障碍标记/清除 | 地面回波可提供清除端点，但不标记障碍；高处回波被排除 | 核对两路话题、marking/clearing 和高度过滤 |
| 4 | 坐标及时间 | 以同一姿态完成世界→车体→世界变换；输出时间戳对应转换姿态 | 检查原始采样时间、状态时间和 TF；姿态时间统一不等于传感器同步已完成 |
| 5 | TF 所有权 | 官方适配器单独发布 odom→base_link | 去掉重复发布；真实 TF 连续性待 ROS 验证 |
| 6 | costmap 参数 | 坐标系一致；足迹和车体过滤尺寸一致；清除距离≥标记距离；高度范围容纳 Town 海拔 | 检查启动覆盖参数，不仅检查 YAML 默认值 |
| 7 | 路径与控制约束 | 当前理想点载体只加载 `GridBased`，转向半径为0且没有车身足迹；巡航目标1.2 m/s；`max_speed=0`与`overspeed_limit=0`取消额外速度上限；无效输入与超时停止 | 若运行状态报告 `CarLike`，说明服务器仍加载了旧配置；Ackermann 实车配置单独放在 `nav2_ackermann_control.yaml` |
| 8 | 记忆与 heading | 同进程保留多个观察位置/bin；重启后为空；先检查车辆可达性再比较路线成本；原观察朝向失败时可换到达朝向；全部失败释放旧目标 | 修正候选生成、路径校验与不可达反馈 |
| 9 | 地图区域 | 显式报告出生点、失败点相对裁剪区域的位置 | Town 出生点在边界上；约定测试路线留在保留区，或另行扩展裁剪区 |
| 10 | 发布一致性 | ZIP 中每个文件与本地源文件一致，SHA256正确 | 重建部署包，不能沿用旧压缩包 |

现有纯逻辑测试还覆盖目标去重、heading变化、重试条件、记忆检索和前沿历史。ROS action 的取消/接收竞态在 `test_path_planner_bridge_routing.py`，需第二层环境运行。理想点控制的 BeamNG RPC 在独立 worker 中串行执行，ROS 定时器持续发布 `telemetry_age_sec`、`state_age_sec`、`telemetry_ok` 和 `state_ok`；`wall_time` 是主机墙钟，`state_stamp_ns` 保留传感器源时钟，`stamp_ns` 使用 ROS 时钟。不要把这三个时钟直接互相相减。

合成清除测试使用简化的一维栅格，仅证明“删除地面回波会失去清除端点”的机制。它不复制 Nav2 射线算法、膨胀、三维投影或 footprint 碰撞判定。二维射线清除仍可能在坡地、遮挡和不同高度障碍上出错，必须用原始数据回放确认。

## 3. 真正的 Nav2 离线回放（需要 Linux ROS 2，不需要 BeamNG 服务器）

当前 Windows 没有 rclpy/已安装的 WSL Linux/Docker。2026-09-30 已在服务器 ROS Humble 完成 59 项回归和真实规划器候选筛选验证，记录见 `test_artifacts/server_validation_20260930/memory_revision_report.md`；失败 rosbag 回放仍未完成。已有 Linux ROS 2 Humble 环境时，构建两个适配器并先运行：

```bash
colcon test --packages-select beamng_wildos_adapter wildos_nav2_adapter
colcon test-result --verbose
```

源码路径必须是本轮适配器版本。需要本机已具备 ROS/Nav2/消息包依赖；此流程不假设可以联网安装。先跑 ROS action 路由测试，确认目标切换后旧计划/FollowPath 被取消，迟到结果不会恢复旧目标。默认理想点配置只加载 NavFn `GridBased`；若诊断输出中的活动规划器是 `CarLike`，先核对实际参数文件和服务器部署版本。Ackermann 实车测试使用独立配置，并保留实测转向半径约束。

当前缺少 Town 失败时的 rosbag，其他场景 `lidar.npy` 不能替代该证据。有录包后按两个层次回放：

出现“Starting point in lethal space”时，Nav2 已经收到规划请求并进入碰撞检查。优先区分 ROS 2 链路故障与真实/误标的障碍，保存下列只读诊断输出：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 param get /planner_server use_sim_time
ros2 param get /global_costmap/global_costmap use_sim_time
ros2 topic info -v /spot1/ouster/front/points_filtered
ros2 topic info -v /wildos/points
ros2 topic info -v /wildos/clearing_points
timeout 8s ros2 topic hz /spot1/odom
timeout 8s ros2 topic hz /wildos/points
timeout 8s ros2 topic hz /wildos/clearing_points
timeout 8s ros2 run tf2_ros tf2_echo spot1/odom spot1/base_link
```

某路话题缺失或 QoS 不兼容，先查发布/订阅链；TF 外推错误或时间戳持续超前/滞后，先查状态与激光时间基准；planner_server 未激活，先修生命周期。若这些均正常，再看原始点云、清除点云和足迹下的 costmap 代价值。GridBased 成功而 CarLike 起点失败时，优先核对车身轮廓、近车障碍与转向约束；两者都失败且中心附近有代价值时，追踪标记点及清除射线。单独的 `ros2 topic hz` 正常不能证明障碍栅格正确。

1. 输入已有 `/spot1/ouster/front/points_filtered`、`/spot1/odom`、`/tf`、`/tf_static`，验证 cloud_filter→Nav2。这不能覆盖上游世界点云转换。
2. 输入原始 `/vehicles/ego/sensors/front_lidar` 和 `/vehicles/ego/sensors/state`，运行官方适配器重新生成点云和 TF，验证完整感知链。不要同时回放旧派生 TF 和新生成 TF。

所有回放节点使用 `use_sim_time: true`。复制 Nav2 YAML 并把每一处 ros__parameters 下的 use_sim_time 改为 true；cloud_filter、生命周期管理和诊断节点也使用仿真时间。只启动点云处理、planner_server 和必要的生命周期管理；不启动车辆控制适配器。rosbag 仅回放上述感知话题并使用 `--clock`，不能回放 `/control` 或历史速度命令。

逐帧对比原始/适配/障碍/清除点云、中心栅格和足迹周围栅格；先检查时间、TF和障碍是否一致，再调用 ComputePathToPose。必要时用 RViz 同时显示两类点云、足迹和 costmap。

| 回放用例 | 必查结果 |
|---|---|
| 出生点静止 | 位置确实在出生点3m内；没有车身回波标记；区分未知、膨胀和真实障碍 |
| 直行及转弯 | 静态障碍的世界位置不随车辆漂移；TF无重复来源及时间外推错误 |
| 离开旧障碍区域 | 有有效清除射线时旧障碍消失；不能以强制清空地图代替正确清除 |
| 近墙/窄道/坡地 | 真实障碍保留，足迹碰撞仍被拒绝；高射线不能随意擦除低障碍 |
| 同起终点切换规划器 | 理想点默认只记录 GridBased；只有显式使用 Ackermann 配置时才比较 GridBased/CarLike，并记录成功率、失败原因、路径长度及曲率 |
| 切换记忆目标 | 旧 action 取消；多个观察位置按可行路径成本选择；不可达候选回退；全部不可达释放旧目标；到达后仍需视觉确认 |

保存 `/global_costmap/costmap_raw`（若运行版本发布）、可视化 OccupancyGrid 和实际参数 dump。OccupancyGrid 中100/99是可视化映射，精确碰撞判定要结合原始代价和 footprint，不能只看中心格。

## 4. 仅在恢复服务器后补齐的最小采集

按 Town→Sawmill 顺序。Town 先做静止验证，再做低风险短直行/转弯；通过后才做离站搜索和目标切换。地图只加载一次，单轮失败保留数据而不清空 costmap。

在已 source 的 ROS 终端，先录制必要话题（额外话题不存在时记录该缺失）：

```bash
ros2 bag record -o town_costmap_capture /tf /tf_static /vehicles/ego/sensors/state /vehicles/ego/sensors/front_lidar /spot1/odom /spot1/ouster/front/points_filtered /spot1/local_map /wildos/points /wildos/clearing_points /global_costmap/costmap /global_costmap/costmap_raw /local_costmap/costmap /local_costmap/costmap_raw /wildos/planned_path /wildos/plan_status /wildos/memory_selection /wildos/cmd_vel /wildos/native_control /wildos/native_control_status /received_global_plan /lookahead_point /lookahead_collision_arc /rosout /spot1/object_search/query /spot1/remembered_object_goal /spot1/remembered_object_candidates /spot1/remembered_object_feedback
```

录制时另外保存 planner_server、controller_server、两个 costmap、cloud_filter、控制适配器的实际参数，以及 ROS/Nav2版本、地图/车辆/出生姿态和源文件校验值。录制速度话题仅用于分析，回放时不选它。

用 `ros2 topic list` 确认 RPP 的 `received_global_plan`、`lookahead_point` 和 `lookahead_collision_arc` 实际命名空间。默认配置为空命名空间，话题位于根路径；若启动时额外设置命名空间，录制对应路径。
REEDS_SHEPP 路径按换向点分段执行时，必须同时部署 `maneuver_goal_checker` 配置并重启 controller_server。现场核对 `motion_sections_remaining`、`maneuver_done` 与实车姿态，确认到达换向点后才发送下一段。

在 Town 出生点运行只读诊断：

```bash
python3 /home/ly/official_planner_probe.py --require-near-spawn > town_spawn_probe.json
```

失败现场再次运行（此时不要求接近出生点）：

```bash
python3 /home/ly/official_planner_probe.py > town_failure_probe.json
```

该工具只申请路径，不调用 FollowPath。它记录点数、时间戳、中心/足迹代价值以及两个规划器的结果。各话题是最新收到的消息，时间差不是严格同步配对结果；精确归因使用 rosbag 时间序列。

## 5. 视觉记忆功能最终验收

同一地图、同一出生点、同一目标序列，分别完成记忆关闭/开启的对照；每组至少3轮，保留全部失败。每轮只启动一个进程并加载一次地图，完成第一个目标后立即给出记忆过的目标。记录切换到有效计划时间、切换到到达时间、行驶距离、frontier决策次数、重规划次数、失败原因和实际速度。

以有效可行路径和实际到达为前提比较中位数及各轮结果；记忆查询更快不等于车辆搜索更快。无需原路返回，不强行抬高最低速度，也不关闭碰撞检测来制造通过结果。

## 6. 停止条件与当前证据边界

任一配置/纯逻辑检查失败，先修复再进入下一层。理想点模式以固定、无碰撞的 BeamNG ego 作为传感器宿主；相机和 LiDAR 继续挂在宿主上，隐藏车身网格，但仿真里仍存在这个传感器实体。当前点载体允许原地旋转，平移沿当前朝向进行，不包含横向侧移。通过离线检查后仍待确认：ROS运行契约、真实 Nav2 生命周期与动作、BeamNG 网格隐藏效果、Town 在线闭环、记忆对搜索效率的实际提升。
