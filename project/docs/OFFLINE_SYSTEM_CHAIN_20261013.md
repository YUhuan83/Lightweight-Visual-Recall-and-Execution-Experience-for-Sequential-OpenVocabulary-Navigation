# 整体离线检查与目标链路重放（2026-10-13）

结论：当前Python目标/记忆/导航交接在受控正常输入下可以串联完成；故障分支仍不能完整推进。尚不能宣布整个WildOS/RADIO→GraphNav→Nav2→BeamNG系统离线跑通，更不能宣布现场稳定性达标。本轮只检测，没有修改生产代码或连接服务器。

## 本机实际可运行范围

本机Python、NumPy、PyYAML、PyTorch可导入；`rclpy`、`message_filters`不可导入，`ros2`/Docker命令未发现，WSL列表查询返回1并报告未安装WSL。没有安装或下载运行时。本轮没有可用的Linux ROS 2运行环境，无法启动真实DDS、Nav2生命周期、GraphNav/Smac/RPP C++组件。

因此保留两个不同层级：已有源码/单元/历史证据检查，以及新增生产Python回调的离线集成。真实视觉编码器、语义检测、GraphNav选路、costmap更新、Smac规划、RPP速度命令、车辆控制和物理响应仍未在本机完整运行。不能把接口替身和供应路径称作这些组件已闭环通过。

环境证据：`test_artifacts/offline_system_chain_20261013/environment.json`、`wsl_inventory.txt`。WSL原始文本编码在控制台显示有问题，但退出码与未安装信息已保存，不将乱码当正常可用环境。

## 全局离线检查

运行现有 `tools/offline_navigation_check.py`：首轮36 PASS、3 FAIL、6 WARN。失败包括两组视觉记忆数据库测试与部署包一致性。

两组数据库测试报SQLite无法打开文件/临时目录权限错误，路径位于Windows沙箱Temp。改用本项目证据目录下的scratch作为TMP/TEMP，SQLite实际写入探针通过，然后重跑；未改数据库实现。两组测试通过，最终38 PASS、1 FAIL、6 WARN；通过项包含572项测试执行，范围有重叠，不作为572次仿真或完整成功率。

首轮失败保留在 `attempt1/`，最终检查为 `report.json`、`full_check_workspace_tmp.log`。WARN包括真实mask/LiDAR message_filters同步缺环境、旧Ackermann简化重放不能验收当前系统、旧演示入口地图/目标配置与本任务不同，以及历史静止测试范围。没有靠隐藏WARN或更新断言使全局报告全绿。

## 当前通用部署包不能代表当前候选代码

`official_wildos_beamng_bundle.zip`自身SHA为 `1e7af2ec4b63c8c135e62a32bde56dfcb7e2b58d9332a1c38bf0c449390cd1eb`。归档内容和当前文件比较：32个已包含成员不同，包括RPP、Ackermann配置、Bridge、Native、速度环与部署工具。当前生产源码扫描还发现归档未包含：

- `ackermann_smac/src/planner.cpp`、`connector.cpp`。
- `nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.lua`、`vehicle_control_lease.py`。
- `nav2_adapter/wildos_nav2_adapter/direct_vehicle_control.py`。

这是本机旧通用包的部署一致性失败，不证明服务器此前冻结的独立候选包也缺文件，更不证明服务器正在运行旧包。之前已核验的服务器组件修复范围仍保留。未覆盖旧归档、未重打包或部署。详细清单 `bundle_inventory.json`。

## 新增完整Python目标交接重放

工具 `tools/offline_goal_chain_replay.py` 使用当前生产逻辑：

- 从当前WildOS `nav.py`抽取相关目标查询、确认、历史检索方法；不是启动完整视觉节点。
- 实际SemanticLandmarkMemory、ViewpointMemory、内存VisualMemoryMap。
- 实际GoalStandoff和GraphPathBridge回调；供应匹配时间戳的GraphNav路线状态与路线。
- 实际规划动作接受/结果、候选比较、记忆选择、FollowPath接受/结果与重新观察确认回调。

替身明确供应语义特征/定位、无障碍栅格、直线规划响应、执行器健康和完美位置反馈。物理配置组仅供应保持车体朝向的直线前进/倒车路径，不支持的规划朝向返回失败，不放宽生产半径约束。理想组供应可原地转向的运动反馈。没有真实RPP控制律、速度环、转向/换挡/制动动态；两组使用同进程合成相机几何，不是实际Town传感器数据。

| 受控流程 | 正常结果 | 明确范围 |
|---|---|---|
| Ackermann配置：加油站→绿色房屋→记忆加油站 | 3个供应动作完成，三阶段均在新的定位确认后完成 | 使用已确认ViewpointMemory；不是未指定目标的被动记忆真车验收 |
| ideal_point：先保存未指定加油站的场景特征，再绿色房屋→加油站 | 5个供应动作完成；第二目标经passive_semantic_landmarks检索、候选比较、执行和重新确认 | 被动特征/后续文本向量均合成；没有运行RADIO识别 |

两组确认内存对象身份保持、未重建数据库；仅路径/动作成功时仍不宣告object_found，最后供应新鲜目标观测才完成。第一组明确事先查询过加油站，第二组首个实际查询为绿色房屋、被动采集时也只有绿色房屋活动查询；两种记忆来源不混称。

这证明当前若干关键Python数据流可以连接，不证明真实规划器会生成相同路径、真实控制器能跟踪、绿色/蓝色房屋能正确区分，也没有评估记忆提升的实际搜寻时间。

## 同一链路注入故障后不能跑通

在绿色房屋当前FollowPath已接受、仍在执行时，注入后台延长规划超时，运行真实超时和旧动作结果处理回调。两种配置都得到：

- `epoch`递增，旧动作结果失效。
- 供应运动动作结束，但Bridge仍following=true、队列未清。
- 状态为following/extension_rejected=planner_timeout，绿色房屋阶段未完成，未进入后续记忆目标。

该结果将上一轮独立回调反例放入多组件目标流程重现，仍是合成条件的离线反例，不是新增现场失败。不修改生产代码就无法声称该故障链已修复。真实Ackermann短cusp接续、恢复预算、候选失败归属等历史反例也未因正常直线路径通过而消失。

## 证据、验证与下一步

证据根目录：`test_artifacts/offline_system_chain_20261013/`。

- `goal_chain.json`：2条正常链、2条故障链的供应规划输入和状态轨迹。
- `goal_chain_tests.log`：4项新增正常/故障集成检查通过。故障检查要求反例存在，通过不意味着生产修复。
- `goal_chain_attempt1.log`为最初2条正常链通过输出，保留；最终输出为 `goal_chain_final.log`。
- `summary.json`、`source_manifest.json`与 `verification.json`保存结论、输入版本及验证范围；此前15个生产文件仍与冻结版本一致。

本轮达到“正常Python流程可串联、已知故障可在链路中复现”的检测目标。整体可靠通过仍需先修复规划/执行身份与车辆接续，再以真实C++/DDS和车辆验证相关失败/恢复/交接路径；部署前生成并核验完整冻结候选包。当前未修改生产代码，未commit，未计入20轮/80%或多目标现场成功率。
