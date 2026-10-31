# 2026-10-15 冻结版本续测与后台续接交接诊断

## 试验与结果

- session_20261015_205942；Town加油站单次加载；green residential house；真实SBR完整物理与碰撞；420秒搜索。生产435成员SHA `0d145100656f87d692b6a81fdfa45a97b11c6ee30d98e46929cdf4051ea8328a`，本轮未改生产源码、车速、到达条件或保护。
- 实际420.768秒超时，Native累计144.555米，最大离站52.732米；29 following、24 maneuver_done、5 segment_done、0 follow_failed、0 no_vehicle_path。不能以没有FollowPath错误称执行稳定。
- Native speed<.05m/s连续173.005秒，开始于首Native样本后248.894秒。状态尾部显示等待costmap，但后续实际请求/结果仍出现，不能把整段停驶都归为地图线程不更新。
- 25连接器搜索均有候选，7次触及连接搜索预算；677真实WildOS评分，17执行反馈全部写入，896检索命中。ExecutionExperienceStore正标签先验保持中性，memory_modified_scores=0；GraphObservationStore候选效用最大差0.191315。只证这些接口参与，不是记忆搜索收益验收。
- object_mask/object_confirmation均0，未完成视觉到达，20轮成功计数不增加，不进入正式多目标。
- 控制器2386 tick内部pose源龄median .2588/P95 .3683/max .4671秒，外部新鲜而内部>.5计数0。模型图快照median1.478/P952.099/max2.639秒。未证明处理延迟是此次停驶根因。
- SDK持续停稳，session errors=[]，独立ps无本轮项目进程残留。退出期间的ROS KeyboardInterrupt traceback不等同搜索期崩溃。

## 现场停驶时间线

- elapsed247.767秒仍following。249.509至256.041秒多个候选失败/等待地图；252.009秒当前FollowPath成功完成，发布segment_done。
- 1792069472.141秒候选5成功51 poses；为了比较同端点另一heading继续候选6；1792069473.793秒候选6失败，按代码应执行normal_best候选5。
- 随后GraphNav约每2秒仍接受相同区域的新route，pending=false/following=false，直到超时却没有新的执行。last_status保留waiting_for_costmap_update；重复路线保护以last_plan_start/endpoint判相近路线直接返回。
- 原接口bag未录/wildos/full_planned_path，未记录prepared_extension内部字段，不能恢复候选5完整形状以证明该现场事件走了哪个分支。保存这些缺口，不凭日志补造状态。

## 已复现的源码交接缺口（未修复）

`graph_path_bridge.py:execute_checked_path` 对extending_path且含换向/多section的计划，将路径写prepared_extension后直接返回。`on_follow_result`只有旧动作完成回调才消费prepared_extension。

当旧动作先完成、后台规划后返回时，following=false/pending=false/extending_path=false，但prepared_extension仍有路径，没有后续回调消费者。旧状态又可触发on_route重复路线保护，静止不会自行恢复。

`test_artifacts/planner_wait_audit_20261015/reproduce_late_extension.py`抽取当前生产方法原AST，使用合成两section路径与mock验证器分别调用：旧动作活跃时合理缓存；旧动作已结束时同样缓存、无dispatch。两状态反例均按预期复现。此为精确方法状态反例，未验证地图/车辆安全；与现场时间线吻合，但不是现场候选路径逐字节重放。

建议修复边界：缓存换向续接只允许存在仍会回调的旧动作；旧动作已结束时将其交还正常执行入口，由现有实际姿态、双costmap、新鲜度、跟踪与换向保护重新验算。须覆盖“完成先于规划结果”“规划先于完成”“替换接受间隙”“取消/超时/旧serial”“验证失败恢复”，不可仅取消去重保护或强制发车。本轮仅测试与诊断，未改生产、未称修复完成。

## 上轮19.8秒停驶重放与更正

session203918日志确认碰撞拒绝后串行尝试多个候选直到较近候选成功；当时不应简单称物理空转。独立ROSdomain89加载最接近已发布地图/odom和端点匹配图路径，实际生产插件0.511秒成功；6连接器中5个原生搜索合并验证可行。未复现现场拒绝。

A*首候选耗尽预算可能压缩其他候选尝试的假设，尚不能解释这次记录。重放使用最近发布地图和推断heading，非实际请求和插件私有地图；运行于完整现场试验期间，会竞争CPU，不是受控性能消融。没有据此调大预算或重排候选。输入范围见input_scope.json，完整输出replay.log。

## 证据与交付边界

- 当前实际RViz双栏录像：`captures/20261015_lidar_origin_repeat/rviz_live.mp4`，1920x1080，8fps，3391帧/423.875秒，全部解码；SHA `b9d5380d37d84d120a8ce4606c18c14e52b7bcae6e6decf28c53ea465af1fb62`与服务器一致。右侧是实时camera，没有独立front_camera原始流MP4。
- `test_artifacts/planner_wait_audit_20261015`保存封闭统计、停驶反例、重放及原始归档。视频是本轮失败证据，不能称目标成功。
- 完整到达失败，新状态缺口待修复/验收；不创建Git commit。

- 本轮完整归档test_artifacts/planner_wait_audit_20261015/field_evidence.zip已取回，163成员SHA/CRC全部校验通过，外层SHA cc684dca9ac6fbdcfe8c81bbf0697937f61c87885518f297cf793dd517f43573；本机MP4与归档视频一致。archive_validation.json保存结果。状态反例脚本与诊断脚本语法检查通过，原方法反例两状态均复现；这是故障检出，不是修复通过。
