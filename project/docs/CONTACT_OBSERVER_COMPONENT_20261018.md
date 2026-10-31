# 原生接触观测器：全制动组件验证（2026-10-18）

接续PHYSICAL_STALL_HANDOFF_AUDIT_20261018。目标是补齐“SDK已确认驱动，但车辆为什么不前进”的接触证据。没有修改生产模块、规划/控制参数、JBeam或碰撞；Native生产和Smac6153冻结均保持。没有新增搜寻或移动路径目标，没有视觉模型/RViz/录像，不计20轮成功，不commit。

## 原生接口与诊断边界

只读核对当前BeamNG.tech 0.39.4.0的vehicle/main.lua及controller.lua：onNodeCollision向controller.nodeCollision分发id1、位置/法向、法向力、滑动力、穿透深度及材料编号。通过v.data.nodes[id1].wheelID区分body/wheel，缺失节点归unknown。

该事件没有被撞对象ID/名称。诊断器不把材料编号变成物体语义，不把轮胎正常接地当车身阻挡，也不把原始向量未经核对便称为某个ROS坐标系。

观测器仅作为auxiliary controller注册，使用package.preload加载，**未向游戏安装或user mods写文件**。不替换已有回调，不写油门/制动/挡位/轮胎/物理属性。只添加本实例的诊断controller，调用现有分发链，结束按实例身份卸载。

记录有界：每窗最多1024不同节点、输出最多128节点，body优先；记录丢弃/截断数、累计事件数、窗口序号、obj:getSimTime窗口、Python请求/响应wall时间。复制事件的数值/向量，避免引擎反复复用particleData导致旧记录被覆写。

## 观测工具自身的回归

- liblua5.3夹具共11项通过：body/wheel（含wheelID=0）/unknown、事件复用、非法数值、窗口/累计序号、容量界限、body优先、可选字段及注册/卸载所有权。此层不是真实碰撞引擎测试。
- 新增所有权反例先1项失败：安装被同名preload拒绝后，cleanup仍可能删除不属于本实例的preload。保留before代码及before_ownership日志；增加独立注册归属标记，安装失败/半完成只清本实例，另一实例拒绝清理，最终11项通过。是诊断候选自身缺口，未部署到生产或当作历史故障。
- Python语法检查通过；实际BeamNG运行后验证真实注册、采样、卸载。测试数量不替代下述真实分支边界。

## 全制动现场组件

远端`/home/ly/contact_observer_20261018/braked_20261018_211038`。

- 测前prestart拒绝相关残留/display占用；一次Town地图，原位置左90°、原SBR配置。使用GPU0，不操作GPU1外部负载。
- 仅传感器与导航底层栈；速度环enable=false、Bridge follow_path=false，ROS参数核对两个门关闭后才安装诊断器。原Native拥有制动。诊断器未发布控制，只发布/wildos/diagnostic/contact_snapshot。
- actual /proc/maps核GraphNav db52f00e、Smac fc149c80、RPP d2fd9cee；全部404候选成员SHA一致。诊断源码在启动前单独冻结哈希。Native保持原可靠深度1，未合BEST_EFFORT候选。
- 15窗口，sim时间4.078～7.312s；真实轮胎接触事件6300，body=0、unknown=0，输出180份节点窗口样本，轮胎节点峰值法向力1729.4N。
- 原始事件位置约x733.725～735.865、y-12.501～-9.697、z51.930～51.962，法向z约0.99978～0.99999，与出生点地面接触一致。但未做移动/旋转坐标对照，向量完整坐标契约仍保留待核验标签。
- 全程正油门0，最大Native速度0.00256m/s；原质量1449.72kg、723节点/467车身碰撞节点、四轮碰撞保持。观测器卸载ACK通过，结束停车、errors=[]、独立域进程清单[]。

**这验证了真实事件分发、轮胎支路和诊断生命周期，尚未触发真实车身接触支路。** 出生点本轮未观测到车身接触不代表全部位置/动态状态安全；没有复现旧失速或完成目标搜索。

### 唯一watchdog帧

组件记录到1份watchdog，未隐藏。该帧源command_age约0.01242s、接收龄0.01153s，已有源receipt，actuator/SDK ACK正常；telemetry_ok=false且telemetry_age=null。它是首次遥测尚未建立的关闭路径，不能称为源已过期或>200ms延迟。

135份直接Native回调观测源龄P95约5.993ms/max10.064ms。不能把本轮正常当作历史间歇延迟已修复；本轮没有内核UDP诊断链。

## 记录覆盖与尚缺的时钟观测

封闭bag实际包含raw LiDAR62份、adapter cloud59份、Nav2 marking/clearing各55份、接触快照15份，另有TF/odom、terrain/local map、两张costmap及Native/源命令/速度环状态。启动发现/处理阶段数量不同不能直接当掉包bug。

本组件未录`/spot1/measurement_timing`和raw StateSensor，**不能声称完整验证了sim时间到ROS时间的跨流映射**。当前先验证观测器，下一次受困片段采集必须增加这两项，结合源stamp/frame和TF核对，不以最近接收时间强行匹配不同时间域。

还须区分公开costmap与控制器私有缓存；本轮没有新增私有地图版本指纹观测。wheel/body事件不能单独决定障碍物高度或模型类别。

## 原短段证据已冻结，尚未运动复现

本机`test_artifacts/contact_observer_20261018/recorded_section_fixture.json`从原封闭164222记录保存33点/4.564m实际段、原planned状态和邻近Native状态。原goal checker为cusp_goal_checker，后轴起点约(738.526,-75.721,-1.14897rad)，邻近Native速度0.07858m/s。

尚未验证从车辆实际dir/up还原引擎出生四元数，也未下发该路径。新车出生不会带回此前累计变形、原挡位/速度状态，后续只能称受控执行组件，不能称精确全流程重放。不得省掉实际姿态、碰撞/输入新鲜度、源ACK和持续停车门禁。

下一步顺序：校验出生姿态变换及原路径接入→受控短段记录真实body contact、原始/各级点云和时钟映射→定位物理阻挡与地图遗漏→与Bridge/视觉恢复/GraphNav clear事务共同修复和回归→完整搜索。单独短段通过不替代整体模块验收。

## 证据

本机目录`test_artifacts/contact_observer_20261018`。核心contact_component_evidence.zip为122856B/30载荷，SHA256 `1c3193eecf13b4d95d78e7e9c36e50668537e6af52550514123420f4c9649a74`；本机CRC/逐载荷SHA、真实接触/关闭门/卸载、已知未验证边界断言通过。

原始传感器库仅服务器：`.../braked_20261018_211038/interfaces/interfaces_0.db3`，102608896B，SHA256 `202f590531c571136b416569127d56554673d3781fc35486f77911450220968b`。外部清单在归档中，不称原库已下载。旧6153包、旧失速证据和上轮封包均保留。
