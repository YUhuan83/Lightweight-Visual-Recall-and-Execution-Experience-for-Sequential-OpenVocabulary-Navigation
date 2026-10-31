# 2026-10-14 控制器位姿输入与 DDS 对照

## 已复现的最早不一致

生产算法继续使用46ff候选：Smac规划半径12m、实际指令半径8m、固定3m前视、真实SBR车身/轮胎物理和碰撞。没有降低速度或放宽0.2m换向到达门。

component3前两段完成，第三段中止。Native遥测经现有body_rotation/SBR_BASE_OFFSET重建后轴，最近距端点约0.00928m但未停稳；这不是精确控制器位姿回放。随后增加独立只读DDS记录，component5从Town复位后四个换向段完成、最终段失败：外部odom更新，RPP变换后的路径/前视点却重复使用旧位姿时间戳和坐标。在末段约1.8s内保持同一0.46568m/s指令，收到更新时端点已经位于车后，触发方向保护。方向保护是后果，不应移除。

`received_global_plan`的时间戳来源已核对Nav2 1.1.20源码，是用于变换路径的robot_pose戳；不是plan_status重复旧事件的假延迟。同期外部/spot1/odom和/ tf广播保持更新。控制器内部接收/缓存链的新鲜度与上游广播不一致已确定，尚不能断言全部历史停驶由这一原因造成。

## 对照范围

| 组件 | 配置/范围 | 几何执行结果 | 控制器位姿最大源龄 | 源龄>0.5s且外部odom<0.3s的tick |
|---|---|---|---:|---:|
| 4 | 默认DDS，起点已移动 | 通过 | 2.12150s | 39 |
| 5 | 默认DDS，Town复位 | 失败 | 2.43480s | 43 |
| 6 | 默认DDS，父调试器一次中断 | 报告通过，仅诊断、不计验收 | 2.12103s | 47 |
| 8 | XML额外UDP+ROS本机模式 | 通过 | 0.30155s | 0 |
| 9 | 回环UDP、ROS本机自动覆盖关闭 | 通过；观测文件封闭状态须再核验 | 当前已取回运动记录0.21607s | 0 |

不是绿色房屋识别/视觉到达或20轮成功率。不同轮规划拓扑会随连续实际起点变化，不宣称路径逐字相同；完整参数、路径和实际位姿保留。component4即使报告通过也有旧输入，不能把route_done当输入链健康证明。

## 线程与传输检查

- 无权限直接ptrace；服务器用户没有sudo权限。改为本轮controller的父GDB，不更改算法/参数，捕获源龄>.75s且外部odom<.3s时的线程栈。两项诊断launch一致性检查通过。快照中主要线程在DDS/executor等待和正常周期睡眠，没有看到bufferCloud阻塞；该快照不证明所有时刻都没有阻塞。GDB中断影响时序，因此component6不算正常现场验收。
- /dev/shm约63GiB、仅23MiB使用，不支持共享内存总容量耗尽。未删除其他用户/其他任务DDS文件。
- 已安装rmw_fastrtps_cpp/shared_cpp为6.2.10。对应官方participant.cpp确认ROS_LOCALHOST_ONLY=1会在XML用户传输之后自动追加回环UDP和SHM。因此component8虽然改善，不能称作已彻底关闭SHM。
- component9在ROS_LOCALHOST_ONLY=0下使用仅127.0.0.1白名单的UDP XML，保持本机通信边界；运行controller的fd未出现fastrtps共享内存/端口句柄，环境核对正确。改动仅在诊断启动环境，尚未写入正式通用部署入口。
- 证据支持DDS传输配置是重要影响因素；仍需默认配置恢复对照、再次回环UDP复测及完整搜寻，不把一次对照外推为所有故障已修复。

## 本轮确定修正的诊断问题

component7与旧component2一样在原生状态发布者立即查询处失败，未开门/未行动。component7随后独立观察者确实收到该发布者状态，证实预检需要等待异步发现；不能追溯替component2补不存在的身份记录。

`tools/check_ackermann_cusp_execution.py`改成最多5s发现等待、精确节点/命名空间/类型/数量连续稳定0.5s；重复/陌生发布者立即拒绝，持续缺席超时拒绝，门仍关闭。缺席、晚到、消失、重复/错误类型与正常路径14测试在本机和服务器通过；component8/9实际检查通过。工具依赖和测试已补进398成员新包94e37140f6e06d2dc9ad01ea68c300cce4cc61d86fcc70dd1eebf10cddcec19c；安装运行仍是46ff，逐员比较只有5个tools文件变化。全局离线检查通过，未commit。

## 证据与未完成工作

本机：`test_artifacts/tracking_contract_repair_20261014/`。

- physical_cusp_component3、4、5、6、7、8、9：结果、事件、实际Native输入/遥测、失败图片（失败轮）、传感器时序。
- component4/5/6/7/8/9_observer：9类DDS接口；9的当前取回文件仅用于运动阶段比较，封闭后应重新取回并验hash。
- tf_freshness_comparison.json、component5_observer/alignment.json、controller_gdb.log/.decoded.txt、snapshot.request。
- resume_session1..5、source_identity.json、retired_stack.json在服务器保存拥有者/安装/配置身份。新引擎1709748；当前session5 sensor1743484/stack1743566/controller1743587，执行后关闭两门、停稳暂停。

新引擎只加载Town一次；后续组件准备复位共4次，测试执行中没有瞬移/地图重载。每次复位均发生在旧拥有者退出、传感器清理后。

下一步：默认DDS恢复/回环UDP再次对照→统一正式环境/打包/所有运行拥有者核验→完整绿色房屋搜寻。控制器最大位姿源龄保护、真实停稳动态余量、terrain路域和其他历史未闭环问题仍须分别处理。

### 证据封闭补记

component9观察者125s结束，result.json计13417条消息，PID1744310已不存在。重新以服务器完整字节封闭取回，先前运动阶段比较由完整记录复核，最大源龄仍0.21606834s、外部odom新鲜时过龄tick仍0。

证据包`test_artifacts/tracking_contract_repair_20261014/tf_delivery_evidence_20261014.zip`为10,359,177字节、145原始成员，SHA256 `41a32bfae0c8c6859c6e7c2f60c2c4be0702dd7c726300617be132e78fec40c0`；本机全部成员SHA核对、4项证据回归通过。包含未通过原结果、封闭的观察者记录和调试源码；仍活动的session5栈/传感器日志仅为归档时字节快照，明确不称作进程退出后的完整日志。

正式`component_env.sh`仍会导出ROS_LOCALHOST_ONLY=1；仅拷贝UDP XML不足以重现component9。下一轮应先完成默认恢复对照，再统一所有本任务ROS进程的环境入口与身份检查。当前未开始新的完整语义搜索，也未提交。

官方源码核对入口：

- https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp
- https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp
- https://github.com/ros2/rmw_fastrtps/blob/6.2.10/rmw_fastrtps_shared_cpp/src/participant.cpp

获取的原文保存在服务器upstream_audit；把对应版本6.2.9早期核对更正为实际安装6.2.10，旧文件保留。不保存凭据。
