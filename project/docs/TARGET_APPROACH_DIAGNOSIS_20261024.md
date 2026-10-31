# 最新目标靠近链路诊断（2026-10-24，只读）

依据已关闭v25两轮213139/213551，未再发起车辆试验或改运行代码。与换向去抖优化分开解释。

## 已核实事实

- object_mask话题106/30条（不是106/30次人工确认正确识别）；object_localization有效输出3/1条，source全为lidar；object_confirmation均0条；bearing_only可见靠近提议157/36条。提议包含对同一源观测的重发，不能当独立定位数。不能说三维定位未实现或从未有三维点，也不能说最后仅是到达门槛问题。
- 实际冻结goal_standoff.py/official_graph_stack.launch.py设confirmation_frames=5，required_cluster_support=max(3,ceil(.8*frames))=4，需要30秒内同一1.5m簇的至少4条有效定位。两轮总共也只有3/1条，不可能满足该支持数。第一轮首批定位之间还有约61秒间隔，不能跨过30秒窗口累积。frames不是雷达数量、也不是Triangulator的views计数。
- 因没有确认锁，运行停留在bearing_only短观察步。第一轮63.64s开始，73.40/78.72/82.98/88.44s推进临时goal；96.91s换向后续段footprint_collision，位置偏差约.129m/heading .00028rad；114.65s可见目标no_vehicle_path释放；119.98s重新靠近，后续目标位置又改变。障碍格来源/真实不可达性仍未闭合，不能关闭碰撞门或把拒绝一律判作误标。
- 第二轮61.97/84.86s两次临时靠近；114.15s FollowPath aborted。stack.log明确1792849101.763s Ackermann RPP detected collision on its bounded command arc；随后约114.93s靠近30秒期限到期释放。145.07s完成的是观察步，转重新观察并回退探索，155.18s重新观察超时。179.82s另一次旧frontier动作同样被RPP弧碰撞中止（非已到达目标）。此为两次控制器碰撞检查拒绝，不证明BeamNG物理实际发生碰撞，也未判定地图格真假。
- 最大命令空窗已在SHIFT_WAIT_OPTIMIZATION中记录：靠近续段/换向后的路径不可达和串行恢复都可造成停车。这与2秒换挡确认是两条独立链路。

## 原文与当前定位实现区别

官方ObjectMaskTriangulator默认前/左/右3相机、订阅一个LiDAR点云话题；并非“三个雷达定位”。原代码优先将LiDAR点投影进掩码，点够用中位距离点提供深度；不足时跨观察位姿生成粒子，调用Triangulator.triangulate对粒子按视线距离加权得到粗三维目标。依据官方main的obj_mask_triangulation.py及triangulation3d/triangulator.py，不能称其为三个雷达求交。

本项目当前仍LiDAR优先、多视图补充，但几何关联和多视图计算已经修改：foreground_depth筛前景/歧义面，fallback调用localization_geometry.triangulate_bearings做最小二乘射线交会和基线/视差/重投影检查，不能说完整原样使用原文粒子定位。实际安装ObjectMaskTriangulator与v25冻结文件SHA均1241cbc9f5f21f2e3af1d9b659abe0c4f830b08f356bdf970d2cefa15273d5bc。两轮没有有效multiview输出，原因须继续追踪有效掩码、同步、独立视点及几何准入，不能凭此直接归因模型或ROS延迟。

## 诊断结论及后续边界

当前缺口是“稀少三维观测→多帧确认→车辆可达观察点→失败后保持同一目标并选替代视点”的闭环不稳定。掩码仍新鲜时可推进临时观察点，但已接纳路径不是每过2秒就必取消；2秒门影响下一次提示准入，30秒靠近期限/观察步完成/路径失败是另一些切换入口。不能仅降低确认帧数或取消碰撞检查掩盖问题。

后续先追踪为何大量mask只产生稀少定位；并固定相同车姿/两张costmap复现可达观察点和RPP弧拒绝，区分地图、候选几何和跟踪轨迹，再改完整目标意图/恢复交接，不把帧缺席等同目标消失。本轮只读，无运行改动，无commit。

证据本机test_artifacts/continuous_exploration_20261024/{target_approach_audit.json,stack_213551.log,triangulation_213551.log,server_v23_base/goal_standoff_v25.py}；服务器v25_fast_shift/target_approach_audit.json及full/trial_*/sessions/*/goal_001/interfaces。已关闭且prestart进程/显示锁为空后读bag。原文链接：https://github.com/nasa-jpl/nebula2-wildos/blob/main/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py ，https://github.com/nasa-jpl/nebula2-wildos/blob/main/triangulation3d/triangulation3d/triangulator.py 。

## 逐门细查补充（本轮无运行源码修改）

### 多视图观测回放

两轮106/30条掩码都非空，实际mask-LiDAR paired104/30，日志回调数一致；第一轮2条未配对不解释其余定位缺失。配对时源龄.331–1.103/.580–1.890s，未超过5秒准入；不能把同步笼统归作主因。

同源掩码、相机采样位姿、相同1m独立视点选择与5秒滑窗回放两遍一致：第一轮64次视点太近、9次不足两视点、18次无合格基线/视差、6次重投影误差、3次不确定半径过大、1次交点在相机后方；第二轮分别6、8、9、0、3、3次。另有现场LiDAR3/1次分支。视点跳过及不足两视点计数与实际日志完全吻合；多视图均0输出。最终带原始视线与投影诊断SHA4ba4f82d37335e21f4facfc95b68d5fdc7fdb029dd372258af83217ba87e7fa7。

仅诊断把视点历史5改10/30秒，两轮实际视线仍0定位，不能通过简单延长超时解释或修复。保持实际位姿、将视线替换为指向首个LiDAR提议的理想固定点，仅作数学控制，5秒下可有6/5次通过，表明一致视线条件下求解能工作；它不是实际目标真值、不是新成功定位。控制SHA cdc54d2991262fbd410a9500eadd2d86672ef37882943029852283d808237813，两遍一致。

当前add_views每帧以最大连通区中位像素构造一条点视线，无跨帧特征对应。对面状目标/候选切换存在假设风险，但本轮未找到“同一候选点位于所有视图掩码内，而单独因中位射线失败”的实证（首个LiDAR提议只落入7/4帧掩码、共同覆盖视图组0）；故不能将全部失败已确定归为数学规则过严，也不能说已经证明掩码正确或错误。应继续区分物体实例、面上不同区域和语义误检。

### 更正：原始点云实际已保存

先前根据通用wildos_console录制列表写下“没有原始LiDAR”的判断不成立。实际full/console_graph_capture.py追加了EXTRA_POINT_TOPICS，metadata确认两轮保留raw/filtered LiDAR、mark/clear、tf_static及costmap_raw。旧回放caveat缺数据的说法保留为更正历史，不得再将其当阻塞原因；本轮没有为补录而启动新的受控测试。

采用实际几何函数、配对LiDAR源戳、记录TF与相机采样变换的深度回放，两遍一致。第二轮全部30配对可回放：29条在去重后掩码回波不足8像素门被拒，其中20条0、4条1、其余2/3/5/6/7各1；1条输出与现场三维坐标误差0。故第二轮深度稀少的直接门已证实。配置16线、60度垂直视场、40m量程，如何共同造成低投影命中还需区分遮挡、掩码区域与采样几何，不能直接判为模型或硬件错误。

第一轮回放有88条同点数门、7条无合格深度簇、1条表面检查失败；但回放产生6条深度提议，而现场仅3条，且有1条配对cloud未录到、1条TF覆盖缺口。仅其中2个现场输出逐坐标完全匹配；不得将第一轮全部负例归因视为闭合。控制Python及实际vision-cu128解释器重复两遍均有此差异，尚未由运行库差异解释。掩码时间戳无重复，LiDAR只有1组相同数据重复、TF无同源戳冲突；没有发现时间戳重用绑定不同数据的反例。

### 碰撞拒绝的范围

公开地图/先前odom/received_global_plan两遍检查SHA3d40ae03972a3d03d2e863042df161380bdbd307f5c9a97645cbbd0852ce1311。三个拒绝时刻当前位置车身未被公开地图lethal占据；第二轮两次失败前局部路径分别约.85m/.25m处有车身与lethal格冲突。第一轮当前车位空闲不能解释完整续段。此仅公开栅格及路径前缀近似，非控制器私有地图锁与实际被拒命令弧的精确复现；现已确认有costmap_raw，可继续提高回放保真并追mark/clear来源，不据此撤销碰撞门或声称错误停车。

### “独立回波像素”规则的模块职责

该门位于共享localization_geometry.py::foreground_depth，经ObjectMaskTriangulator决定是否发布object_localization，再由GoalStandoff聚类生成object_confirmation、由GraphPathBridge核验到达。它不是ExecutionExperienceStore等记忆存储策略或RecallCandidatePolicy候选打分，首次实时搜索也会经过。原文有掩码内LiDAR点数门；逐像素去重与当前8像素稀疏门是后续定位修改，不能归为原文原样逻辑。仅凭目前记录不能把引入动机准确追溯为专用于视觉记忆。

可信记忆/最终到达需要可靠空间证据，临时接近应允许利用有不确定度的弱证据并受车辆/碰撞约束。当前仍有bearing_only接近，故不足8不是完全不准运动；真正缺口是弱证据→持续观察→可靠确认的交接不稳定。后续不能把记忆筛选规则直接当到达判定，也不能只删几何门来制造成功。

### 误启动记录（排除）

诊断加载环境时误source完整v9/run_full.sh，启动trial_20261024_215808。已按精确PID/目录核验发送SIGINT，正常关闭：startup38.239s/search38.488s/shutdown7.811s、InterruptedError/console0/closed=true；两次prestart确认无残留进程/X148锁。它不是有意对照，不纳入任何性能或诊断统计，不改写原v9历史失败。原主目录703运行文件与备份SHA再次不变，无commit。
