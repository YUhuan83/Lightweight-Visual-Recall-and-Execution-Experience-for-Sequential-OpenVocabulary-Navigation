# 普通探索重复Action中止交接（2026-10-26）

## 已确认的问题

v42复测215959同owner cacb44a1...:4持续约175秒，局部运动155米，整轮净位移5.342米。真实RPP在query约56/107/154秒三次报告bounded command arc collision；第三次follow_failed/action_status=6，恢复延迟已8秒，随后仍同owner following。此非动作稳定崩溃，也不能用累计行走米数替代探索进展。

源码follow_failed中已有重复失败恢复，但替代frontier分支只纳入execution_stalled/tracking_deviation和明确local_footprint_collision；实际Nav2控制器中止统一经on_follow_result传入follow_aborted/status=6，未进入该分支。原credit_recovery_progress已按固定owner目标的新1米进展计信用，不是任意走一米都重置；本次不改该正确规则。

独立复核：关闭现场源日志与实际RPP栈对应；7条受控回调改前两遍均2fail/0error（达到旧重试预算仍不退避frontier，完成不产生新进展的局部prefix也不退避）。另5项正/负对照覆盖首次失败、新实际进展、STATUS_CANCELED、明确目标、已退休动作。不是有两个独立现场样本，不能扩大该证据范围。

## 最小修改v43

从v42冻结复制。唯一运行改动为graph_path_bridge.follow_failed：普通、当前、未退休探索owner、reason=follow_aborted且真正STATUS_ABORTED=6、既有recovery_attempts≥2时，复用reject_exploration_route的原相关route stamp/owner协议；GraphNav据原协议退休并退避该frontier，选替代目标。第一次仍用现有恢复；新实质进展仍按原规则恢复信用。

不把STATUS_CANCELED=5算作abort，不影响目标/记忆策略或新owner；retired_motion及ContinuousGraphPathBridge旧动作分流仍先处理，不拿旧失败惩罚新路线。不变更次数、碰撞、物理、速度、12m规划/8m控制、完整路径RPP、定位或2米车身到达门。临时退避不是证明物体/区域永远不可达。

风险：真正瞬态故障连续三次且未挣得新目标进展时，也会暂时换frontier；沿用既有有界恢复成本，现场仍须核对是否更有效探索。对于尚无三次abort但持续可行局部往复，不能称此一条已全面解决。

## 验证状态

- 169相关回调两遍通过；真实ROS802项两遍通过，原32实际C++插件两遍通过，console通过。RPP仍v42 SHA217609ca...，没有重新修改C++。
- 首次fakeROS因夹具仅暴露STATUS_SUCCEEDED缺STATUS_ABORTED出现6error，日志保存initial_fake_interface_failure.log；仅补ROS标准枚举，未改生产判断或断言。最初本机全量回归因候选复制省略三份原历史夹具出错，分两次补齐；失败日志保留offline_missing_fixtures.log/offline_missing_prefix_fixture.log，服务器继承原夹具的真实ROS回归不受此缺失影响。本机828最终结果待补，不把缺夹具记为通过。
- 已预检空进程和X148锁、源清单与实际库SHA冻结后启动180秒green residential house现场。现场启动时本机全量夹具补齐回归尚未结束，属于验证顺序偏差；相关169和真实ROS802已通过。不能宣称所有验证已在现场前完成。关闭结果待补。
- 主目录703源码及原基线、v42与更早候选保留，无录像/commit、不推广。证据test_artifacts/abort_recovery_20261026；服务器execution_continuity_review_20261022/v43_abort_recovery。

## 221716关闭：进入接近后失败，新增目标绑定证据

启动41.646/搜寻180.144/收尾24.030秒，超时、完整验收false，session errors=[]、console0、全部项目进程/X148锁清空。近停31.896%/最长11.447秒、路程121.764/净87.875米，stale36.376其中planning15.177秒、gear3.527秒。相对v42复测净位移更高但近停更差，不能据此宣称无回退，不推广/commit。

本机828原离线补齐原三份历史夹具后最终通过；169相关两遍、802真实ROS两遍及原32实际C++两遍通过仍有效。运行真实库匹配v42 SHA。13次整路退役消息之后旧方向后缀越界0；正常整路动作23条、8含换向，6内部方向切换；无曲率越界或原地旋转。现场普通探索未出现满足新abort预算分支的三次失败，v43新增分支未现场触发，不把本轮更远路线归因该修复。

145mask/51有效LiDAR/16程序确认，确认帧最小车身间距23.358米，未达2米；4次真实RPP碰撞弧中止、33次instance_mismatch、2次confirmed_approach_failed，后期观察路线超时。程序确认计数仅证明内部质量门通过，不是语义真值或目标到达证明。

### 只读目标绑定投影，两遍一致

失败RGB failure_last_rgb.png中左侧蓝房屋、后方浅绿房屋。使用存储CameraInfo/相机TF/精确同帧odom，把旧确认点和最新定位点投影到该RGB：

- 首个确认点(746.267,-112.280,58.787)投到(430.66,184.07)，蓝房屋区域。
- 旧实例最后确认点(746.291,-111.501,55.306)投到(408.10,306.28)，仍在蓝房屋区域。
- 最新定位点(748.930,-130.086,58.121)投到(591.45,269.87)，后方浅绿房屋区域，并在最新目标mask内；旧两点均已不在该mask内。

RGB与最近mask相差.475秒，车辆实际移动.198m/转.02196rad；不是直接把两个不同相机姿态叠加。由同帧odom+相机TF恢复固定外参，再用RGB精确odom补偿；末5帧外参一致到1.1e-13m/4.4e-16rad。原始mask [1,1,640,960]展平诊断初次错误已修正并保留initial_shape_error.json；最终两遍JSON逐字节相同。

因此本轮不应仅归为“正确目标已经固定，车不会靠近”：旧确认实例实际绑定到前方蓝房屋区域，后来对浅绿房屋的新几何触发旧实例不匹配。还必须区分错误始于语义mask包含错误表面，还是mask→LiDAR表面关联；几何质量支持不等于语义正确。没有放宽实例门或强行合并相距约19米的两表面。现有foreground_depth已拒绝多个有足够支持的深度表面，不再误诊为完全没有歧义检查。相机投影不是初始时刻原RGB的完整逐帧真值标注，具体最早错误仍待原mask/深度关联回放。

下一步同时保留路径执行边界修复和普通探索恢复候选，优先从正确实例绑定到车辆可执行接近点的完整契约定位，不以降低最终2米门或删除碰撞检查制造成功。证据field/{approach_instance_audit_1.json,approach_instance_audit_2.json,closed_core/.../failure_last_rgb.png}及两次Native/events审计。
