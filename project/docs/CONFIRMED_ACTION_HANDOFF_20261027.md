# 连续接近的动作接受交接（v51，2026-10-27）

隔离候选，不覆盖主源码和v50。当前尚无整车通过证据，不提交。

## v50现场及最早违约点

trial171954已关闭，startup39.304/search180.068/shutdown16.020秒超时。session errors=[]、console0、项目进程/X148锁空。145掩码、125有效LiDAR定位、41程序确认，确认帧最小车身距57.616m，完整到达失败。全Native182.896秒（含收尾）近停27.757%、最长7.197秒、净53.020/累计125.217m；较v49近停低但路线不同，不能称稳定不回退。

63.024/66.453/70.085秒现场连续更新确认接近点，旧动作不被立即取消；这验证v50新分支实际触发，但不能替代完整接近。73.712秒旧点segment_done；73.814秒新的planned发出FollowPath；73.820秒桥接层报告retired_controller_progress_unavailable并取消新任务。其后停车重试，97.725秒失败交接，最终超时。

代码与受控状态测试各重复确认：following在send_goal_async时就置真；此时follow_active/active_follow_serial仍空，新动作尚未ACK。旧结果回调结束时following已被新提交置真，旧_motion_intent可继续保留。on_route只用following和旧intent判断正在执行旧动作，错误调用restrict_current_direction；没有已接受动作则失败，继而走follow_failed。这是动作接受状态约定错误，不是该时刻物理碰撞或证明整个路径不存在。后面的Smac空路径/过多换向仍需分别核验，不能全归为同一原因。

## 修改

仅在continuous_graph_path_bridge.on_route的旧动作退役判断中，使用既有follow_active、active_follow_serial/_motion_serial及epoch确认实际动作归属。未接受的新提交和旧metadata不属于已接受旧动作，不走旧动作退役失败。没有新增状态、计时器、恢复模块或关闭安全检查。真实仍在执行的旧动作依然必须有当前方向进度才能交接。

## 验证

三个新状态用例初版缺动作戳，令既有退役缓存把两个None视作已退役，造成额外夹具错误；保留初版失败记录，补实际非空动作戳。最终改前两遍2fail/0error；改后84相关用例两遍通过。原828离线及服务器ROS/现场结果待补。

证据：test_artifacts/confirmed_action_handoff_20261027；上一轮失败保存在test_artifacts/confirmed_continuation_20261027/field，未删除。无录像。

原828离线通过，856真实ROS两遍、原实际C++32两遍、console/rig通过；主703/旧ZIP/RPP保持。预检项目进程和X148锁空后启动v51实际query后180秒现场，结果待补，不录视频/推广/commit。

## v51现场关闭未通过

trial173048搜索180.115秒超时，session errors=[]、console0、项目进程/X148锁空。109mask/93LiDAR/46程序确认、确认帧最小车身54.397m。Native180.547秒近停32.633%/最长13.910秒，净91.231/累计105.113m（包含收尾，非精确180秒）；不称无回退/完整成功，不推广/commit。

接近阶段未出现v50新提交未ACK误退役；探索10.896秒仍有同名进度缺失，不能称所有同名错误已消除。后续120.704/123.881秒局部车头碰撞，128.390秒失败交接。按精确录包输入两遍确认：不是旧路径前缀，而是提出的目标车身在局部图已碰撞、全局图却允许，上游没有该局部数据。下步仅把已有局部图接给选点，详APPROACH_LOCAL_MAP_20261027.md。
