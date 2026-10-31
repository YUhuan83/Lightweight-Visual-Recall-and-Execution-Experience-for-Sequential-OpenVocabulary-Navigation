# 确认后继续接近：v31隔离修复

2026-10-25。基于冻结v28，不混入v29定位几何和v30确认窗口改动。候选在`test_artifacts/confirmed_progress_repair_20261025/candidate`；服务器`/home/ly/execution_continuity_review_20261022/v31_confirmed_progress`。主目录用户源码及旧版本保留，未commit。

## 已复现的缺口与改动

v28复测trial_20261025_200844后段26条确认消息全部no_reachable_standoff。旧规则的最终环候选均被直线占用射线截断，唯一截断点再被车身/连通性检查拒绝。pending_confirmation同时暂停原有弱证据观察步，bridge无接近点就返回，形成确认与导航交接空缺。该缺口不是全部近停的唯一根因。

仅两个运行文件变化：

1. `official_adapter/beamng_wildos_adapter/goal_standoff.py`：最终接近点及直线截断点都不可用时，复用现有observation_step寻找最多8米的中间观察点；它仍必须通过新鲜局部图的自由射线、已知自由连通性、真实车身占位及目标安全距离检查。中间点以既有clamped standoff交接给GraphNav/Smac，不直接发送速度，不保证几何可达。确认等待无有效接近点时允许新鲜、方向与确认实例一致的bearing_only观察；已接纳的active_approach仍不被弱提示抢占。
2. `official_adapter/beamng_wildos_adapter/visible_approach.py`：修复网格步长减法让理论2米边界变成1.999999999999999而漏采样。仅在1e-9米浮点误差内补到原2米边界，不改变最小距离或碰撞阈值。

原四条支持、目标/实例及时间戳门、车身外缘2米成功、物理与轮胎、Smac12米/RPP8米、0.3秒换向和持续探索配置保持v28。已有动作取消/终态/停车ACK不变；中间点不是搜索完成或可信记忆写入证据。

## 回归与同输入证据

- 初始8项专测改前3fail/0error；最初两个断言写成直接读取缺席Mock调用产生AttributeError，改为明确断言调用存在，保留失败日志，未放宽成功条件。
- 增补浮点边界反例，改前1fail/0error。最终9项新增测试、118项接近/恢复/到达相关回调和原828项离线回归通过。
- 实际ROS构建与709项组件通过，包含新9项。新建候选回放环境显式使用候选visual_navigation路径，未混载v29几何签名。
- 实际部署的GoalStandoff在相同26个旧失败快照中8个可生成中间点、18个保留no_reachable_standoff；此前两个受控弱观察抑制反例均不再被pending_confirmation压住。回放仍是最近接收地图/odom快照，省略历史拒绝区域，连续性count受控设3，不能当完整时序或车辆执行通过。
- 使用同一记录快照的集成回调验证：producer给出原观测戳/四条支持/中间clamped standoff，bridge接纳为接近任务，object_found仍为false。

## 现场验证状态

709真实ROS通过、源码清单SHA核对及预检无残留后，启动单轮Town绿色房屋180秒测试：trial_20261025_205143。启动/收尾另计，无录像。关闭后需分别检查确认→接近交接是否实际触发、Smac及车身执行是否可行、近停与无新鲜命令占比、完整目标是否通过；组件通过不能替代该验收。

证据目录含`before_clean.log`、`boundary_before.log`、`after.log`、`offline.log`、`real_ros_report.json`、同输入standoff_filters报告、`source_scope.json`、`v28_to_v31.diff`、增量ZIP、真实地图夹具及主目录保护核验。源码范围只有上述两文件，未修改车辆规划或控制器。完整结果待关闭，不提交、不推广为稳定基线。

## v31现场关闭及低净位移调查（覆盖待关闭状态）

trial_20261025_205143：启动47.601秒、搜索180.168秒、收尾21.827秒，超时、完整验收false、session errors=[]、console0、closed。所有项目进程/X148锁清空，无视频/commit。

- 近停16.022%/最长3.695秒，stale近停8.264秒，其中planning6.159秒；净28.163米、采样140.288米。
- 仅6条mask，定位和确认消息均0；没有visible_target_approach/confirmed approach状态。本轮没有验证新确认接近分支，不能将16%直接归为该修复收益，也不能宣称流畅完整搜寻通过。
- 前60秒走52.319米/净48.003米，约68.73秒最远离出生点57.158米；60–120秒走53.570米却向出生点方向折返，最后120–184秒走34.140米/净4.802米。以native velocity·direction估计前进100.002米/倒车38.227米（只计<=.5秒间隔）。主要损失是后程重复运动，不是总近停继续高达40%。
- 两个exploration_id累计约112.78秒、79.99米。6次FollowPath abort全部有RPP bounded command arc collision原始证据，不是人工推断物理碰撞。
- **新增已复现缺陷**：ContinuousGraphPathBridge.on_route以latest_route已保存当作基础层已准许开始的标志；base保存latest_route后仍会因退避、预算或重复路线返回，子类随后无条件start_segment，绕过这些限制。4项受控回调中基础层2项通过、持续探索2项失败（0error），两遍一致。
- 关闭录包对应实例：声明4秒重试，0.794秒后同owner继续，路线相对旧线最大偏差2.336米（不足既有3米“新路线”提前重试条件）；另2秒重试仅0.607秒后开始、路线偏差2.971米。还有一次1.835秒<2秒/偏差1.932米。第一例1.108秒<2秒但路线偏差5.975米符合原规则，已排除，不能把所有提前重试都当错误。

新问题位于持续探索适配层，v28也有相同源码；v31两处producer改动没有引入该调度代码。下一版本应使基础准入继续控制停止后的重试，同时保留健康动作的后台接续。证据`displacement_audit.json`、`abort_retry_audit.json`、`retry_contract_before{,_2}.log`、`field_phase_audit.json`。
