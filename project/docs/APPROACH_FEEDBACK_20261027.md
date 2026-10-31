# 接近阶段：实际执行点失败被最新候选掩盖（2026-10-27）

## 已确认的最早违约点

用户要求专注修改目标接近阶段。v45约搜索125.35秒，Bridge对实际接受的(736.976,-87.918)报告no_vehicle_path。此时GoalStandoff已发布新的候选(736.716,-90.931)，而Bridge因当前接近动作仍活跃只将其缓存。Producer的on_navigation_plan_status却要求失败点与last_goal（最新发布候选）相距≤.6米；两者相距约3.02米，真实失败反馈被丢弃。该比较混淆“已发布候选”和“车辆实际接受的接近点”。

之后Bridge检查旧缓存新点，当前地图下其车身占位已被拒绝，转而选择建筑附近约24.70米外的径向观察点。此点最终30秒观察路线超时，随后产生11.2秒停车重规划。本次并非通过放宽碰撞或延长观察超时解决。

两遍5项真实类回调改前均4fail/0error，覆盖发布端缺实例身份、旧点失败被新候选掩盖、其他实例失败及重复处理。已关闭v45录包/同帧地图与真实回调两遍复现旧分支：receiver_regenerated=false，deferred点被当前footprint拒绝，恢复点距离24.702米、坐标与现场一致至约1e-6米。不是仅凭状态标签猜原因。

## 隔离v46

基于v45，test_artifacts/approach_feedback_20261027/candidate；服务器v46_approach_feedback。仅2运行文件：confirmed_approach.py在实际接近失败中携带当前选择的instance_id；goal_standoff.py对新鲜、同query、当前实例的失败处理实际失败点并重新生成接近点，不再要求等于最新未接受候选。其他实例明确拒绝；旧无身份反馈仍要求原候选位置匹配。重复/过期反馈和查询切换保护保留。

复用原rejected_observation_poses、当前地图/车辆几何和publish_standoff→approach_refresh→deferred→start_live_observation交接。保留实例、目标坐标、原可靠确认/到达、连续整路、停车隔离与12米Smac/8米控制；不新增直接控制通道。

## 验证与限制

相关125项两遍及828离线通过。首次全量/真实ROS回归中test_confirmed_failure_feedback_enters_existing_same_instance_handoff手工装入active_approach但漏confirmed_instance_anchor，新增身份校验阻止该不完整状态；原失败日志保留，夹具补齐生产retry_confirmation同时建立的实例锚点，断言和运行身份校验未放宽。真实ROS重跑待补，不提前宣称通过。

同一v45录包反事实回调两遍：新失败反馈正常被Producer接受，重新计算(736.015,-92.794)，保留该点自身航向-1.77885rad；接近恢复距离由24.702米改为7.963米，当前车辆地图检查通过。这只证明当前地图下中间接近点选择和消息交接，不是车辆实际完成。真实Smac对照与完整现场待补。

回放重建了定位/车姿/公开地图及生产回调，未完整重建候选比较、执行动作及私有锁定costmap历史；均明确acceptance=false。所有旧版/失败证据保留，主运行源码未覆盖，必要验收前不提交/推广。

## 接近恢复路线实际规划对照

819真实ROS两遍及32保留实际C++两遍、console已通过；828原离线及125相关两遍通过。初次漏锚点夹具导致的真实ROS/离线失败完整保留，未更改运行断言或降低身份门。

使用同一v45失败时刻车姿与公开raw全局图、真实12米Smac，两个版本各两遍：旧径向点可生成路径，但需要7次换向，maneuverCost=99.4252；新中间点可生成零换向路径，maneuverCost=7.9815。两者采样车身碰撞0、曲率契约通过，两遍结果相同。这里99.4是带惩罚的路径代价，不是99.4米距离。旧恢复点直线距离24.702米，新7.963米。

因此该修复确实改变接近失败后的可执行恢复路线，而非只新增状态日志。整车还可能受更新地图、感知和局部控制影响，不将录包规划对照当完整成功。对照报告及原始输入/日志：test_artifacts/approach_feedback_20261027/replay/{before,after}；机器可读validation_summary.json保留full_acceptance=false。

预检空进程/X148锁、冻结SHA后启动v46 trial_20261027_103053，实际query后180秒，地图/模型启动及收尾另计，无录像；当前结果待关闭，不推广/commit。

## v46现场关闭，修复分支尚未整车触发

103053已关闭：启动42.374/搜索180.188/收尾23.035秒超时，session errors=[]、console0、项目进程/X148锁空。全Native181.904秒口径近停28.411%/51.682秒，最长5.095秒；净58.184/路程114.243米，stale18.816其中planning11.706秒、gear6.778秒。不能把最长停顿缩短归因本次修复或宣称完整无回退。

129mask、0可靠三维定位/0确认，新增确认后接近失败反馈分支未触发。本轮仍有13次弱目标靠近、5次visible_target_no_vehicle_path、11次局部fallback、6次观察步结束/再观察。需要继续聚焦这条接近阶段链路；本次没有修改语义、记忆、定位质量或到达阈值。

真实运行Smac/RPP及模型SHA匹配冻结清单；12条退役限制后旧方向越界0，无命令曲率越界/原地旋转。Native/events复算两遍SHA均bc79c6ef...。实际路径30条，完整路径队列保持一条整路动作；不能以路径生成数代替到达验收。

记录范围为“接近失败反馈接口和录包恢复路线已验证，整车确认后接近与完整搜索未通过”。所有候选保留，703主源码/原ZIP再核验未变，不推广/commit；证据field/field_result.json、field_phase_audit.json、full_path_audit.json及validation_summary.json。
