# 规划归属修复与部署候选（2026-10-13）

本轮按顺序先修后台规划超时，再更新旧通用包；未连接服务器。

## 原因与改动

旧 recover_timed_out_plan 递增整个导航 epoch，误使当前 FollowPath 的合法终态回调失效，遗留 following/队列。GraphPathBridge 现以独立单调 request serial 区分每次 ComputePath，实际接受和结果回调检查 epoch/index/serial；超时、候选失败和结果消费只退役所属规划请求。重复接受不重复注册，终态仅消费一次，同 index 重试仍拒绝旧响应。

相关交接缺口：动作先完成、扩展规划后失败时，原动作回调因 pending 推迟交接，原 extension_failed 没有恢复。ContinuousPath 现统一在扩展失败且无活动动作/替换时，通过原 start_segment 保护恢复交接；动作仍活动则保持动作归属。

生产改动仅 graph_path_bridge.py 与 continuous_path.py。目标切换、输入/执行器失效、模式切换与真实动作失败仍失效整个导航事务。速度、碰撞/轮胎、8m半径、坐标/时间戳、新鲜度和视觉确认规则未改。serial=None兼容既有直接回调夹具，生产异步 dispatch 全部捕获 serial。

## 验证

证据：test_artifacts/planner_ownership_repair_20261013/。

- before/ownership_tests.log 保存原动作归属和两种交接顺序反例。初次导入夹具类使 unittest 额外发现43项既有测试，合计53；两项错误来自新增serial未实现。随后改模块引用，独立新增集为11项，不把53项称新增测试。
- callback_guard_report.json：最终324项生产回调替身通过，含15项新增；覆盖成功/abort、超时、普通拒绝、迟到/异常/重复、index重用、目标切换；既有替换、输入和执行器失效、记忆交接保护通过。初批320项结果仍保留callback_report.json。
- integration_tests.log：7项历史归属反例回归和4项目标链回归通过。历史测试原要求缺陷存在，现要求恢复；历史JSON/日志未覆盖。
- goal_chain.json：正常及后台超时注入下，Ackermann确认视点station→green house→station、ideal未预置station查询的passive memory链均完成。供应特征/定位、图路线、直线规划及完美运动，需要新观测才确认object_found；非真实RADIO/Smac/RPP/车辆测试。
- package_tests.log：5项通过，资源缺失、路径越界、源中途变化均拒绝发布且保留旧包。首轮误预期plugins.xml，实际CMake用plugin.xml；修正测试后通过，原失败保留package_tests_attempt1.log，没有修改插件。
- full_guard_report.json：最终40 PASS / 0 FAIL / 6 WARN，592次测试执行含重叠；初批588次的full_report.json保留。SQLite/临时目录用项目内TMP/TEMP，未改数据库实现。
- unpacked_callback_guard_final.log：最终包解压后324项回调通过。首包unpacked_goal_chain.log的4条合成链通过，两包间只增加4个保护回归测试，生产和链脚本字节未改。383成员逐字节匹配当前源。
- ownership_guard_tests.log：补测真实start_segment回调的健康交接、过期位姿、执行器故障，以及延迟替换确认。后三者不错误下发新动作；这四项补测不需要新增生产代码改动。

六项警告保留。通用默认ideal/其他地图入口不替代当前真实SBR、Town验收。上述替身和合成检查不等于真实C++/DDS/BeamNG验收。

## 冻结包与回退

deploy/20261013_planner_ownership_candidate/official_wildos_beamng_bundle.zip：383成员，最终SHA256 94f6e301ec17d2b1d884830362c64e034d34bdba4303aac0858c47ca45c9bb42。同目录manifest逐成员hash、sha256文件、CANDIDATE.json记录范围与现场待验收状态。初包SHA48b2f4c64d62f3c948d4d4a34254f6447085d4c7ffc23076d8903b72e4a8ba05随对应rollback包保留；最终包新增4项回归测试。

此前32成员差异及当前Smac/lease缺失文件已刷新。打包器先冻结源hash，生成后核对归档及源字节，全部一致才发布。旧包SHA 1e7af2ec4b63c8c135e62a32bde56dfcb7e2b58d9332a1c38bf0c449390cd1eb，保留本轮before目录及主目录rollback.<sha>.zip。

包包含当前候选的既有改动，不是新commit或服务器部署证据。未本机编译C++；Smac要求nav2_smac_planner 1.1.20 EXACT，目标环境需核对。

## 后续门槛

1. 先核对服务器残留进程、车辆/传感器身份、源码/插件/Lua资源hash，明确physical_vehicle_control.launch.py真实配置；预检后启用。
2. 真实短动作中注入所属后台规划超时，测两种交接顺序、迟到响应、目标切换、执行器失效，保存serial/epoch及各接口时间戳，不录像。
3. 组件通过后完整绿色房屋视觉到达与真实执行；20轮至少16成功后再多目标。

短cusp新路线可执行性、after_maneuver预算、候选实际姿态接入失败归属、复杂曲率/terrain约束、引擎响应空档/崩溃与costmap退出故障继续未解决。现场门槛未通过，按AGENTS不commit。
