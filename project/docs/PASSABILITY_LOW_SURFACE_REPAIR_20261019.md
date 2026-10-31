# 低表面障碍与地形台阶契约修复（candidate7）

2026-10-19。接续 `PHYSICAL_EVIDENCE_CONTRACT_REPAIR_20261019.md`。用户授权继续修复并提醒“考虑全局、不要引入新 bug”。本轮先备份 current candidate6，再在独立目录实现 candidate7；正式安装、6153 包、Native、Smac 和碰撞参数均未替换。初始组件阶段未启动BeamNG；后来4.5节实际启动真实车体做制动准入诊断，未下发运动目标。未录像，未commit。

后续复核见 `CANDIDATE7_REVIEW_20261019.md`：曲面误标和亚格距离已有后继组件修复；独立静态表面比Terrain高约12.5cm，局部坡度较大，不能仅据本报告稀疏回波称该处为平滑表面。以下保留原实验记录。

## 1. 备份

本机：

- `deploy/20261019_passability_repair_backup/`：改动前六个文件的副本 + `SHA256SUMS.txt`。
- 备份文件 SHA256 与 candidate6 源一致（pointcloud `0571afbe…`、mapping `a9543dc3…`、terrain `e8a12fd5…`、node `0bef3a3d…`、terrain_metrics `a55dcbfc…`、mapper `47fdab53…`）。

服务器：

- `/home/ly/passability_repair_20261019/backup/candidate6_candidate_20261019.tar.gz`，SHA256 `69dd48bbb005195bd617ba70f7f235f83f5d4a843f92cf70eca77dd6cf54358f`。
- `candidate6_install_20261019.tar.gz`，SHA256 `bb618d7d35250b8458f0da42f3e3aa5e9cd93c9ab8e200118dfd2335275198cb`。
- `SOURCE_SHA256SUMS.txt`、`candidate6_result.json`、`ARCHIVE_SHA256SUMS.txt`；两个归档均通过 `gzip -t`。
- candidate7 独立目录 `/home/ly/passability_repair_20261019/candidate7/`。

## 2. 已确认并修复的故障

### 2.1 低表面在进入地图前被删除（真实录包）

修复前代码顺序：`pointcloud.py::split_costmap_points` 先用 base-frame `min_height=0.1` 裁剪，再按拟合地面删除残差 `< ground_clearance=0.25` 的点。084254 接触邻域约 0.16 m 的低表面在第一步被删除；即使删除第一步，第二步仍会删除。

candidate7 增加 `obstacle_min_height`：

- 提供该参数时，拟合半径内的点按“相对局部地面高度”判定，不再受 base-frame `min_height` 限制；
- 拟合半径外或地面拟合失败时，保留原 base-frame 路径；
- `obstacle_min_height=None` 完全保留旧行为，供 `cloud_filter` 旧调用方兼容；
- 非法/非有限阈值直接拒绝。

接入点：

- `official_adapter/beamng_wildos_adapter/mapping.py`：`occupancy_from_points(..., obstacle_min_height=0.10)`。
- `official_adapter/beamng_wildos_adapter/terrain.py`：`terrain_layers(..., obstacle_min_height=0.10)`，占据图与地形使用同一阈值。
- `official_adapter/beamng_wildos_adapter/node.py`：新增 ROS 参数 `obstacle_min_height` 0.10，传入 occupancy/terrain。
- `nav2_adapter/wildos_nav2_adapter/cloud_filter.py`：Nav2 costmap 输入路径同样传递该参数。
- `nav2_adapter/launch/nav2_official_control.launch.py`：正式载体路径显式设置 `obstacle_min_height: 0.10`（已有 `ground_plane_radius: 8.0`）。

注意：0.10 m 是复用原 `min_height` 数值并改为地面相对基准，**不是**实测底盘净空；实际阈值仍需车辆几何标定。

### 2.2 0.5 m 地形格无法区分台阶与短坡

修复前 `terrain_metrics.hpp` 把“拟合平面后的残差范围”当台阶：17 cm 合成阶跃只报 8.5 cm，通过默认 10 cm 上限。

candidate7 不直接把公式改成相邻格差，而是保留原始点的亚格高度分布：

- `terrain.py::witnessed_step_by_cell`：0.1 m 亚格 min/max，相邻亚格的高差减去 `max_slope=0.35` 可解释的平滑上升，得到“实际观测到的突变”；
- 要求每个亚格至少 2 个回波；只使用局部地面 +0.5 m 包络，避免树枝/悬空回波变成台阶；
- 结果作为 `witnessed_step` 层；`elevation_min/elevation_max/height_span` 同时计算但只留在进程内诊断，不进入 DDS，避免消息负载翻倍；
- `mapper.cpp` 将该层加入 `layer_names`，节点与边的 `step_height/maximum_step_height` 取 `max(原残差指标, witnessed_step)`；无该层时行为完全不变；
- 节点在 `elevation_valid=0` 时也会读取 `witnessed_step`，因此缺高度但仍测到突变的节点可以形成物理违例。

### 2.3 占据图→SDF→Planner 的低表面路径

新增 DDS 阶段 `low_surface_occupied`：平坦高程、无地形坡，仅把接触带标为占据；实际 Mapper 计算 SDF 后，边 `physical_violation_observed=1`、`minimum_clearance=0`、`physical_feasible=0`，实际 Planner `complete=false`。这验证了低表面标记不是只停在地图输入。

## 3. 全局影响审查

- **兼容性**：所有旧调用方不传 `obstacle_min_height` 时走原分支；`cloud_filter` 的 `ground_plane_radius=0` 旧配置不会改变行为。正式 `nav2_official_control` 才启用地面相对阈值。
- **两条地图链一致**：WildOS `local_map/local_terrain` 与 Nav2 costmap 的 `cloud_filter` 现在使用同一 `obstacle_min_height`，避免只修一条链。
- **DDS 负载**：只增加 `witnessed_step` 一个 80×80 float32 层（约 25.6 KB/帧）；`elevation_min/max/span` 不发布。
- **CPU**：在 30k 点、80×80 栅格上，服务器 `terrain_layers` 约 42–48 ms；向量化后的 `witnessed_step_by_cell` 约 7 ms。新增成本约 7–9 ms/次，未改变控制周期或超时参数。
- **旧行为保留**：无 `witnessed_step` 层的旧录包/Mapper 输入行为不变；candidate6 的 15 阶段目标交接、5 阶段记忆决策、7 阶段物理链均保持原版本证据，不覆盖。
- **未改**：`min_height`、`ground_clearance` 的默认数值保留；Native QoS、Smac、RPP、6153 包、正式安装、碰撞/JBeam 均未改。

## 4. 验证

### 4.1 组件

| 项目 | 结果 |
|---|---|
| `nav2_adapter/test/test_pointcloud.py` | 18/18 通过（含 5 项新增地面相对/兼容/非法参数测试） |
| `official_adapter/test/test_terrain.py` | 9/9 通过（含 0.17 m 台阶、平滑坡、突起、稀疏、悬空、envelope） |
| `official_adapter/test` 全部 | 46/46 通过（含实际 ROS 消息/回调） |
| `nav2_adapter/test/test_cloud_filter_ros.py` | 1/1 通过（参数确实传入 `split_costmap_points`） |
| C++ `graphnav_mapper_test_mapper` | 22/22 通过（新增 `WitnessedSubcellStepBlocksEdgeAndNode`） |
| C++ `test_route_mechanism` | 40/40 通过 |

Nav2 全量套件：777 项，6 failures / 298 errors；与未修改的 candidate6 基线逐项名称 diff 为空（304 条完全相同）。这些是当前服务器测试环境/消息版本导致的既有失败，不是本轮新增。新增的 5 项点云和 1 项 cloud_filter 测试均通过。

### 4.2 实际消息链（Mapper CDR → DDS → Planner）

`/home/ly/passability_repair_20261019/candidate7/dds_v2_20261019_183829`：

- `passed=true`，`errors=[]`，9 个阶段全部符合预期。
- 原 7 阶段：partial_steep/unknown_after_steep/complete_steep 拒绝；flat/unmeasured/separated/stale 通过。
- `witnessed_step_flat`：平坦高程 + 0.17 m 亚格突变；边 `maximum_step_height=0.17`、`physical_violation_observed=1`、`physical_feasible=0`；Planner `complete=false`。
- `low_surface_occupied`：仅占据带；边 `minimum_clearance=0`、`physical_violation_observed=1`、`physical_feasible=0`；Planner `complete=false`。
- 实际库身份：Mapper `4a0401b1a99332facc995db717488cb2a1061218ddad14c19f8b53e780eb0b02`，Planner `3f40485009bacbf349fdcd9bd45161409dee0ca75dadd4278ea8460006c557d7`；probe 从 `/proc/<pid>/maps` 核验，不是只读安装路径。

### 4.3 冻结 084254 重放

本机与服务器都运行 `candidate7_replay_validation.py`（22 帧，只读）：

- scan_00–05：旧 `min_height+ground_clearance` 路径 contact cell = 0；candidate7 contact cell = 1（occupied/traversability=1），116 个 ≥0.10 m 残差回波被保留。
- scan_06–21：接触点进入车体 footprint，外部回波为 0，无法再标记；这说明修复必须在接近前生效，不能等压上后再补救。
- 接触邻域 `witnessed_step≈0.029`，没有被误判为台阶；实际低表面是约 0.16 m 的平滑抬升，不是尖锐阶跃。
- 早期每帧新增约 51–61 个 occupied 格；这些新增格是否全部为真实障碍仍需现场确认，不能把数量本身当成功率。

### 4.4 真实数据 Mapper 全链回放（新发现：车体尺寸契约仍不足）

在隔离 ROS 域 200 用 candidate7 adapter 的 `terrain_layers` 从 084254 的 22 帧 `level` 点云重新生成 `local_terrain`（含 `witnessed_step`），发布给实际 candidate7 Mapper；只读原 DB，无车辆。结果 `/home/ly/passability_repair_20261019/candidate7/replay_candidate7_20261019_184118/result.json`：`completed=true`、22 快照、62 个图消息、errors=[]。

- 新标记确实到达 Mapper：scan_00–05 接触栅格 `traversability=1.0`（在 level 帧 cell 48/40，scan_05 47/40）。
- **但最近图节点距接触点约 0.71 m，`minimum_clearance=1.0` > `robot_minimum_clearance=0.3`，`physical_feasible=1`、`physical_violation_observed=0`、`terrain_observed=0`。**
- 原因是**图级**净空仍使用 `robot_minimum_clearance=0.3`，小于 SBR 实测半宽 1.05；而**真实物理栈**的 Nav2 costmap 使用 `nav2_ackermann_control.yaml` 的 SBR footprint `[[3.4,1.05],[3.4,-1.05],[-1.05,-1.05],[-1.05,1.05]]`、`inflation_radius` 1.6/3.75（`nav2_official_control.yaml` 的 `robot_radius=0.15/inflation=0.3` 只属于点载体路径，不是物理车路径）。图节点 feasible 不等于 Smac/costmap 最终允许；但也不能反过来假设 costmap 一定挡住。
- 因此 candidate7 证明“低表面信息已进入 WildOS 和 Nav2 两条地图输入”，但**不能证明整车会避开**：图级选路仍可能把 0.71 m 外的节点当作可行，最终是否安全取决于 Smac 对 SBR footprint/inflation 的检查。下一优先级需要实际 costmap 回放/受控短段确认，并考虑把 graph clearance 与车辆几何统一；不能只调大单个阈值。
- 这不影响“低表面标记已修”的组件结论，但把“整车仍待验收”具体化为可复现的数值反例。

### 4.5 受控短段：地图在接触点拒绝，未动车、无接触（安全失败，不是整车通过）

在 `/home/ly/passability_repair_20261019/candidate7_field` 做隔离叠加（beamng_wildos_adapter / wildos_nav2_adapter / graphnav_mapper / graphnav_planner；只覆盖本轮文件，正式安装未改）。字段叠加使用 candidate7 Mapper + 未改的 6153 Planner/Smac/RPP（源码未变，便于沿用既有运行时身份校验）。受控短段使用 084254 原始场景和录段。

尝试与修正（失败均保留）：

1. `frame_20261019_184534`：Python 环境缺少 `beamngpy`，未加载地图。
2. `frame_20261019_184626`：地图加载，但叠加包把 `wildos_ackermann_smac/rpp` 的 plugin/ament 索引错误指向 graphnav_mapper 前缀，Smac 插件找不到。
3. `frame_20261019_184820`：修正索引后栈和 Smac 启动，但 contact probe `admission_reason=source_protocol`、`goal_sent=false`；录包内 admission 之后的状态实际已有正确的 `control_protocol/source_receipt/source_ack_ready`，判断为初始化/就绪时序问题；verifier 仍按 6153 Planner 校验 candidate7 Planner 而失败。
4. **最终 `frame_20261019_185152`**（candidate7 Mapper + 6153 Planner/Smac/RPP + admission 前等待完整 source ACK）：

- 运行时身份校验通过：Planner `db52f00e…`、Smac `fc149c80…`、RPP `d2fd9cee…`（`runtime_identity.log` `passed=true`）。
- `map_loads=1`，`protocol_closed=true`，`errors=[]`，`component_closed=true`。
- `outcome=admission_rejected`，`admission_reason=collision_cell`，`goal_sent=false`，`motion_goals=0`；车身接触事件 0、wheel 3440、`maximum_native_speed≈0.00284 m/s`；无录像。
- 录段路径 33 点；global/local costmap 在路径末端第 31/32 点（距接触点 0.70/0.53 m）为致命格；接触世界坐标处 global cost 99、local cost 100；接触点 2.5 m 内分别有 161/256 个致命格。**这与历史受困接触位置一致**，说明 candidate7 的低表面标记已进入实际 Nav2 global/local costmap，admission 在动车前拒绝该录段。
- **这是安全拒绝，不是成功穿越**：车辆没有移动、没有接触，也没有证明它能在真实搜索中选择绕行路线。harness 的 `passed=false`（`execution_passed=false`）是正确语义；`component_closed=true/errors=[]` 只表示诊断协议完整关闭。下一步应测试一条不经过该接触点的路线或完整搜索，确认车辆能绕行而不是仅停在起点。

## 5. 证据入口

- 本机：
  - `docs/PASSABILITY_LOW_SURFACE_REPAIR_20261019.md`
  - `deploy/20261019_passability_repair_backup/`
  - `test_artifacts/passability_contract_20261019/candidate7_replay_validation.json`
  - `test_artifacts/passability_contract_20261019/contact_height_analysis.json`
  - `test_artifacts/passability_contract_20261019/ground_relative_marking_sweep.json`
  - `deploy/20261019_passability_repair_candidate7_harness/`
- 服务器 `/home/ly/passability_repair_20261019/candidate7/`：
  - `mapper_build.log`、`planner_build.log`、`official_adapter_tests.log`、`nav2_adapter_tests_final.log`
  - `dds_v2_20261019_183829/result.json`
  - `replay_candidate7_20261019_184118/result.json`
  - `candidate7_replay_validation.json`
  - `result.json`、`harness/`、`fixtures_v2/`
- 服务器 `/home/ly/passability_repair_20261019/candidate7_field/`：叠加包与 `run_field.sh`、`run_frame_candidate7.py`、`field.log`。
- 受控短段最终输出 `/home/ly/contact_frame_audit_20261019/frame_20261019_185152/`：`result.json`、`contact_result.json`、`contact_result.events.jsonl`、`stack.log`、`runtime_identity.log`、`interfaces/`（含 global/local costmap）；失败尝试 `frame_20261019_184534`、`184626`、`184820` 均保留。

## 6. 边界与仍未关闭

- **不是整车验收**：初始组件阶段没有车辆；4.5节启动了BeamNG、真实车体和Native，但未下发FollowPath目标，没有完整视觉搜索。不能把制动准入当行驶成功；20轮至少16轮成功仍未验收。
- **不是生产部署**：正式安装、6153 包、日常入口均未替换；candidate7 只在独立目录。
- **阈值标定**：0.10 m 是原数值的地面相对化，不是实测底盘净空；需要在真实受困短段确认既不漏低障碍也不把草/路缘过度标成障碍。
- **平滑低表面 vs 台阶**：本轮用 `obstacle_min_height` 处理真实平滑低表面，用 `witnessed_step` 处理亚格突变；两者不能互相替代。
- **车体尺寸/净空契约不足**：真实数据回放中最近节点距接触 0.71 m、`minimum_clearance=1.0` 仍判 feasible，因为**图级**阈值 0.3 小于 SBR 半宽 1.05/前伸 3.4。物理栈的 Nav2 costmap 已使用 SBR footprint 和 1.6/3.75 inflation（点载体配置的 0.15/0.3 不适用），但图选路与最终 Smac/costmap 之间的契约仍未用本轮候选验证；不能据此宣布真实车体会避开，也不能假设 costmap 一定挡住。下一优先级是实际 costmap 回放/受控短段。
- **每帧重建/车体遮挡**：占据图仍是当前扫描重建；接近/压上后车体 footprint 会遮挡。需要持续地图或短时保留，未在本轮修改。
- **Nav2 清除/标记顺序**：`clearing_cloud` 仍包含障碍端点（既有行为）；实际 costmap 的 clearing 是否覆盖 marking 尚未用本轮候选重测。
- **其他开放项**：真实采集时刻/epoch、Native 间歇延迟、整体无进展退出、记忆长会话容量、动态换挡/制动/换向、完整视觉到达与记忆增益。
- 测试通过不等于整车通过；未 commit，待受控短段和完整验收后再决定提交。
