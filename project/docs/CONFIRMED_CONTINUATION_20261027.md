# 确认目标后的连续接近（v50，2026-10-27）

当前为隔离候选，尚未通过整车到达，不替换主源码、不提交。主703运行文件和原ZIP保持；v49及更早证据保留。

## 已确认的问题

v49 trial164340中，已生成的同目标接近点因confirmed_route_busy统一等待旧动作结束，形成短点串行执行。按实际/spot1/odom、已接受的/wildos/planned_path、当前controller_path_progress和两张costmap复核两遍：125.425、129.196、172.553秒存在有效前向窗口，剩余1.102/.257/.426m；174.485、176.427秒处于倒车窗口，应沿用既有换向规则。

审计最初错误地用full_planned_path的规划源戳匹配执行动作戳，造成accepted_recorded=false；改用planned_path的实际FollowPath戳后匹配成功。该审计错误未用于修改运行坐标/时间戳。最终开启与现场一致的require_source_control，运输新鲜度与实际几何检查均通过。

## 最小改动

只改confirmed_approach.py、continuous_graph_path_bridge.py：确认目标身份保持；同一目标的新中间点通过已有prepare_intent_change、GraphNav、后台Smac与FollowPath替换接续，不另建控制状态机。正在运行的确认动作不再一律挡住同目标的有效前向路线更新。

沿用现有前向移动窗口、碰撞/实际转弯半径/ACK/恢复约定；规划在途或换向不抢占，最终车身2m与新鲜确认要求不变。不调整识别、记忆评分、模型、控制库、传感器或质量门。

## 验证与边界

- 改前12个新用例两遍3fail/0error，改后13个新用例及相关102用例两遍通过；原828离线通过。
- 853真实ROS用例两遍、原实际C++32用例两遍、console/rig通过。实际RPP仍217609ca...，Smac及模型不变。
- 同一v49录包、控制动作确认状态受控恢复的生产方法回放，两版本各两遍一致。125.425、172.553秒从旧版暂存变为提前发出新路线意图，原Action和epoch均保留，object_found仍false。129.196秒增量不足1m保持等待；两个倒车窗口也不抢占。这不是反事实整车运动或语义到达成功。
- 当前启动v50实际绿房屋搜索，目标下发后180秒，启动和停车另计；不录视频。结束结果待补。不得用组件回放替代整车验收。

证据：test_artifacts/confirmed_continuation_20261027；服务器/home/ly/execution_continuity_review_20261022/v50_confirmed_continuation。

## v50关闭：连续更新触发，整车仍失败

trial171954搜索180.068秒超时，session errors=[]、console0、项目进程和X148锁空。145mask/125LiDAR/41程序确认，最小确认帧车身距57.616m。全Native182.896秒近停27.757%、最长7.197秒、净53.020/累计125.217m，含收尾非精确180秒。未到达，不推广/commit。

63/66/70秒连续接近点确实更新；更早于97.7秒no_vehicle_path的故障是73.712秒旧动作结束，73.814秒新FollowPath提交未ACK，73.820秒on_route把following真和旧metadata当成已接受旧动作，误报retired_controller_progress_unavailable并取消。与源码状态约定及两个受控状态反例复核两遍；后续修复另存v51，详CONFIRMED_ACTION_HANDOFF_20261027.md。
