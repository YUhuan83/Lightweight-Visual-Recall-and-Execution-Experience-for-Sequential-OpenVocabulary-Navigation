# 2026-10-15 评分修复后的完整搜索与连接器诊断

## 版本与条件

- 冻结生产源：`ef3fc7f814683f640e2d4a93dfc5ef601f76f7f1a6e85e56187631f8fec5a54b`。未修改生产算法、车速、车体物理或碰撞保护。
- session：`/home/ly/wildos-console/audit_results/session_20261015_163334`。
- Town 加油站出生，一次地图加载，查询 `green residential house`，420 秒完整搜索；实际 RViz 左侧二维图、右侧相机，独立只读控制器观察器。
- 开始前及结束后独立进程检查均无旧仿真、导航、模型、RViz、录制残留。SDK 持续停稳与拥有者清理正常，session errors 为空。

## 完整试验结果：失败

- 420.9321 秒超时，Native 路程 26.0263 米；6 following、6 maneuver_done、31 no_vehicle_path、11 waiting_for_costmap_update。没有目标掩码、定位、确认或严格到达，不计入成功轮次。
- 真实评分 1041 次，最多 14 个非默认评分节点；2 条执行反馈均存储，检索命中 899 次。正反馈保持中性先验，本轮 memory_modified_scores 为 0，不能宣称执行经验改变路线。
- GraphObservationStore 效用差最大约 0.18999；238 条路线状态中 126 匹配近期真实评分候选、104 匹配回退候选、8 未匹配。此为数据流核对，不是因果消融或任务成功。
- 本轮未出现 FollowPath 反向中止；不能认为旧问题已经修复。535 条控制器内部路径观测的源龄 median 0.1602 / P95 0.2497 / max 0.4613 秒；外部 odom 新鲜但内部超过 0.5 秒的计数为 0。此统计只覆盖控制器实际运行的 tick。
- 外部 odom 源龄 median 0.2446 / P95 0.4053 / max 0.5644 秒；RGB median 0.2557 / P95 0.4121 / max 0.5689 秒。模型图快照年龄 median 1.8239 / P95 2.4272 / max 2.8239 秒，处理延迟仍未闭环。

## 当前最早可见的执行阻塞

- 完成前面机动后，59.825 秒首次 no_vehicle_path；车辆在实际位置持续制动，不是已下发油门但车辆没有动力的证据。
- 531 次连续起点连接搜索中 529 次候选为 0，2 次超时。日志明确 `No safe bounded continuous-start plan`。图规划多次接收路线拒绝并换 frontier，仍无法得到车辆可执行路径。
- 从本轮第一、中间、最后三个 no_vehicle_path，提取最近先前 odom 和已发布全局 OccupancyGrid，使用实际 12 米规划半径、72 朝向、0.01 米 padding 后足迹重放。原地图在 6000 和 100000 预算均 0 候选；自由地图均可生成 6 候选。不能仅归因搜索预算不足。
- 起点 safe=1，但三个样本前后直行 0.2 米均 safe=0；自由图候选放回原图时，约移动 0.13～0.18 米即首次被足迹检查拒绝。附近存在 lethal/inscribed 单元。尚未证明这些单元来自真实障碍、自车点、地面误分或残留观测。
- 重放使用已发布 OccupancyGrid 的近邻快照和反量化 cost，不是插件内部地图的逐字节快照；不能据此断言真实环境绝无可行路径。

## 本机证据

- `test_artifacts/full_score_search_20261015/field_evidence.zip`：72 个原始文件，SHA256 `2ce1b9d4bfbba2ea74d0a93676961c7c5e0c07d6dc882282bdecbdbecf503c93`；ZIP 与所有成员 SHA256 已核验。
- `test_artifacts/full_score_search_20261015/closed_analysis.json`：封闭统计；分析脚本强制 goal 和 session 两个终态存在后才读 SQLite。
- `captures/20261015_full_score_search/rviz_live.mp4`：真实 RViz，423.5 秒 / 3388 帧 / 8 fps / 1920×1080，全部解码；SHA256 `8e4f8e84dce1cd0f80c120b6dcc6572eee21a8225e8210ec28b13f15e10d5a1b`。这是失败录像。
- 连接器及净空重放脚本、`connector_replay.log`、`clearance.log`、`inputs.json` 已单独取回同名 test_artifacts 目录，未混入上述已冻结 72 成员归档。

## 点云来源组件诊断：封闭结果

- 用户继续测试后，首个诊断 session_20261015_200325 在预检阶段拒绝：新增 sensor_bag 被原预检判为旧测试记录器。没有发 query、没有开控制门，也没有搜索录像；这是诊断入口时机错误，不是新增导航算法故障。保留本机 `sensor_startup_rejected_preflight.json`。
- 仅修改独立诊断包装，把 owned sensor_bag 放在未改动的启动预检通过后、目标 begin 前；等待全部订阅就绪，否则拒绝 begin。拥有者原 SIGINT/身份检查清理不变。4 条新增回归覆盖预检前不启动记录器、参数/返回值保留、后续目标复用、记录器退出时不 begin；本机全部通过，`diagnostic_lifecycle_tests.log`。
- 重试 session_20261015_200533，单次 Town、真实车体、相同生产版本，180 秒诊断，额外记录 marking/clearing 输入、适配 LiDAR、TF/静态 TF/odom。查询后 182.564 秒超时，Native 111.7438 米；21 following、17 maneuver_done、3 segment_done，no_vehicle_path=0、follow_failed=0。结束时取消剩余执行并持续停稳，errors=[]，独立进程检查无本轮残留。
- 7 次连接搜索均有候选，其中 5 次触及搜索时限但保留了可用候选；不能把这个 timed_out 标志单独当作本轮路径失败。
- 537 真实评分、15 条反馈均存储、409 次命中，memory_modified_scores=0；没有目标掩码、定位或确认，不能称找到绿色房屋或加入成功计数。评分调用总数含模型预热阶段，不当作目标期 FPS。
- 控制器 1775 tick 源龄 median 0.2432 / P95 0.3643 / max 0.4584 秒，外部新鲜内部超过 0.5 秒计数 0；模型图快照 median 1.5131 / P95 2.1050 / max 2.6479 秒，延迟仍在。
- 本轮没有复现上一轮连接失败，封闭点云来源分析 `cloud_origin_review.json` 的 failure_events=0，不能拿这批点云解释另一轮的障碍单元。额外录制负载、短时限及轨迹不同，也不能把两轮当作受控消融。
- 人工抽帧复核：本轮录像约 150 秒时相机前方已接近树丛（`frame_1200.png`）。无动作异常并不证明路线合理或正确避开植被；这张相机图也不能单独证明车体已经穿入树丛。路线与障碍观测的一致性仍待核验。

### 补录轮证据

- `test_artifacts/full_score_search_20261015/sensor_evidence.zip`：102 原始成员，SHA256 `3b22109e38ae35af92b806c2abf5823f4b393392fa6bb1dd525017ec69782b1f`；ZIP 解压 CRC 与全部成员 SHA256 已核验。
- `captures/20261015_sensor_origin_audit/rviz_live.mp4`：真实双栏 RViz，185.375 秒 / 1483 帧 / 8 fps / 全解码，SHA256 `3991162421f34840896ff8f1abe4982ee2aec4efefbdd420e738636d6fd289d5`。搜索超时录像，不是成功录像。
- 首轮反例的 `connector_replay.log`、`clearance.log`、`inputs.json` 已单独取回本机同名 test_artifacts 目录，另包含在补录轮归档 diagnostics 下；未改动首轮已冻结 72 成员归档。

下一步需要在连接失败实际发生的同一轮记录 costmap 输入点云，或用首轮更低层日志缩小障碍来源；不得以关闭碰撞、调大预算、调低到达门槛替代此证据。

完整视觉到达、20 轮 80% 成功率及多目标仍未通过；未创建 Git commit。
