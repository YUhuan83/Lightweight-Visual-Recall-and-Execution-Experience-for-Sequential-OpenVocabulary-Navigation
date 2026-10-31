# 2026-10-17 追踪交接修复现场验收：完整搜索未通过

## 冻结与部署

- 用户授权现场部署验收并要求继续。398 成员候选 `f9d1caa748373c8bd9106cde7f44105604d069a3b667b9dd3279b47b4d5defec`，本机 `deploy/20261017_visible_pursuit_candidate`；相对 2e69583c 仅改三个实现及 `test_visible_approach_ros.py`。CRC、逐员 SHA、当前源对应全部通过。旧版本保留。
- 独立部署 `/home/ly/visible_pursuit_repair_20261017`；重建 `wildos_nav2_adapter` 与 `beamng_wildos_adapter`。Linux 54 项协议等检查、74 项真实 ROS 回调（含 20 项本次交接）、68 项交接与代价测试、实际 launch 展开通过。离线回调替身 487 项通过。**这些范围不等于物理到达成功。**
- 运行时 `/proc/maps` 核对 GraphNav `db52f00e`、Smac `f915a4fc`、RPP `bd96eb9d`，参数来自本轮 install。Python 源身份、预检、制动保持的源生命周期探针通过。

## 实验条件

Driver：`field_yaw_left90_search_20261017_155026`；session：`session_20261017_155029`。

Town 加油站位置不变，初始朝向左转 90°。实际 Native 朝向为 -99.936°，门禁通过。只加载一次地图，无传送；真实 SBR 723 节点/467 车身碰撞节点/四轮各 32 碰撞节点，camera、LiDAR 保留。车速、半径、碰撞检查、模型和三维到达判据不变。

目标 `green residential house`，预算 420 秒。沿用上一轮共同搜索链对照条件：三个新增记忆先验关闭。**不是记忆增量实验、不是完全未适配的上游 WildOS，也不以此统计记忆成功率。**

## 封闭结果

**完整验收失败：421.123 秒 timeout，没有找到并到达房屋。**

| 项目 | 结果 |
|---|---:|
| Native 参考点累计行驶 | 108.697m |
| 最大离出生位置 | 13.181m |
| following / maneuver_done | 51 / 45 |
| >=5 秒静止段 | 8 段，最长 20.593 秒 |
| waiting_for_costmap_update | 57 次 |
| follow_failed | 2 次 |
| no_vehicle_path | 0 次（不代表没有空候选） |
| 掩码 / visible hint / 定位 / 确认 | 全部 0 |
| visible_target_reobserving | 0 次 |

两次实际 controller 错误均为 `Ackermann RPP detected collision on its bounded command arc`，随后 FollowPath status 6 中止。相对 query 发送约 234.045 秒和 395.648 秒，不能与上一轮视频 6:36 的目标掩码混为同一轮事件。这证明局部地图/预测弧的安全检查拦截了动作，**不能据此断言车身真实碰撞或地图障碍一定正确。** RViz 对应抽帧能看到路边树木/停车区，尚未完成对私有碰撞代价地图逐格的因果核对。

WildOS 评分调用 1440 次，输入存在 frontier，但本轮目标未检测；不能仅凭零掩码判定模型失效，因为车辆基本在出生点周围机动。新追踪状态没有被触发，**本次修复已部署，现场功能验收仍待完成**。

## 针对绕行的补充审查

最长探索交易 `7e49db29-6bf2-63c5-25de-823a9f800fe2:413751041545867:9` 指向 `(727.68336,-30.04983)`：连续 816 条完整图路线、持续 289.154 秒，其间 Native 行驶 84.863 米，但最近 odom 距离仍 8.133 米，未进入实际 1 米观察范围；没有退役记录。不能把 visits=0 当作观察退役代码失效，它还没有满足观察条件。

三个同 header/节点位置严格匹配的 scored/raw topology 快照：前、中段 UUID 仍存在且是 frontier，末段 UUID 已缺失，最近三个替代节点均非 frontier。三份快照不证明缺失持续了多久，也不证明整段绕行都因 UUID 缺失。该结果与项目记忆中尚未修复的“长期无进展/节点持续缺失缺少有界退出”契约相关，需要继续按真实路线队列、后台延伸和局部地图检查，不能直接宣布唯一根因。

## 控制源与停车

目标时间窗内旁路收到 8457 个 Native 状态，全部 SDK 输入回执匹配已观察的源命令身份、载荷、截止时间；错误匹配、无有效源 ACK 却确认正油门、watchdog、执行器故障、遥测不健康均为 0。源龄 median .03751/P95 .04936/max .19657 秒；回调时源龄 P95 .00798/max .03640 秒；控制 RPC P95 16.53/max 161.35ms。

该范围支持“这轮没有复现速度环→Native 源命令过期或错配”，不证明上游 RPP→速度环无延迟，也不把 ACK 当物理响应。

最终使用 `goal_001/stop.json` 的真实 SDK 五次持续停稳样本，末速度 .000694m/s；`session_result.errors=[]`。注意 session 根目录 `stop.json` 是目标前预检停车记录，不能将它当本次结束位置。独立 `/proc` 检查无仿真/导航/录制残留。最终磁盘空闲 45937938432 字节。未运行额外完整轮次、不计成功、不 commit。

## 视频与证据

本轮真实双栏 RViz：

`C:\Users\Yu\Desktop\拟真\captures\20261017_visible_pursuit_yaw90\rviz_live.mp4`

423.5 秒、3388 帧、8fps、1920×1080，全解码通过；116843072 字节，SHA `596d129a1f0cc664611cd29889a01e7aa0f4065d0684799427d0bfd24f446b7e`，服务器/归档/本机一致。实时 camera 位于右栏，无独立前摄 MP4。session 自带 `video_recording_enabled=false` 仅指 console 内置录像；本轮外部屏幕录像由 driver 管理，实际文件已核实，不能据该 flag 宣称无录像。

`test_artifacts/visible_pursuit_repair_20261017/`：

- `pursuit_closed_review.json`、`baseline_summary.json`、`server_components/`、`server_status_after.json`。
- `visible_pursuit_field_evidence.zip`：176477499 字节/121 成员，SHA `37b76b5265da579781808a5396afda5a1200e60999366ffbdd8de5990b157d9e`，120 有效载荷 CRC/SHA 全通过，`archive_validation.json`。
- 2.69GB 原始 sensor DB 只保留服务器，外部 manifest 含路径/长度/SHA `851e013c4da38b2dad33a3dcdaa1fdfd23f6bcdad552d731bcf9cdc5376ae4ad`。
- 后续只读图审查另封 `target_progress_supplement.zip`，不覆盖原核心包；SHA `391d6eb8f557aa9f3f79d0f811621265b65622024e0318a15b40515e6ac9cbfa`，7 有效载荷/CRC/SHA 通过。

期间一次只读 SSH 状态查询因自动审批服务额度用尽而未执行；用户要求继续后，通过原审批通道恢复查询。driver 自身按预算完成停车和关闭录包，未绕过审批。

下一步应先解决探索路线的整体进展与失败退出，再在能实际进入目标靠近阶段的轮次核验本次追踪续接。不通过关闭碰撞、弱化到达或人工注入目标制造完整成功。

## 停驶程度更正与后续执行审查

用户指出“大部分时间都没动”后，重算全部 8420 个 Native 样本：按接收时间加权，速度 <.05m/s 共 214.337 秒，<.2m/s 共 277.365 秒。原表 8 段只是 >=5 秒连续长停，不能代表总停驶时间。51 个下发段中 33 个 <=2 米；8 次换向后地图拒绝对应计划时可走、交接时不再可走的公开地图反例。局部段成功反复清零重试次数的源码缺陷已复现，修正候选及边界见 `docs/EXECUTION_PROGRESS_REPAIR_20261017.md`。完整搜索依然失败，本轮运行版本不变。
