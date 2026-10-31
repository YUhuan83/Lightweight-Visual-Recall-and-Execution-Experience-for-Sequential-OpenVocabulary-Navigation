# 接触参考系、桥接时钟与控制启用边界审查

2026-10-19。接续 `RECORDED_STALL_ROOT_AUDIT_20261019.md`，按用户“先查底层、检查整体连通性”的要求继续独立核验。生产6153、Native、规划/控制参数和真实碰撞均未改变；仅独立诊断/测试/证据变化，没有整轮搜索、MP4或commit。

## 1. 本次新增的确定证据

### 1.1 车身接触位置的world坐标假设得到独立核验

原生auxiliary观察器在同一次碰撞回调中另读：

- `obj:getPosition()`：车辆原点。
- `obj:getNodePosition(id)`：节点相对坐标；按安装引擎其它模块的用法，world节点位置为两者相加。
- `obj:getDirectionVector()`、`obj:getDirectionVectorUp()`、`obj:getSimTime()`：同时刻车姿和引擎时刻。

这些读取放在pcall中，不覆盖原回调、不改力/输入/车辆。完整复制向量，不持有引擎复用对象。参考数据缺失会阻止诊断准入或使诊断关闭，不放宽生产保护。

最终轮 `frame_20261019_084254`：656份节点窗口记录、其中24份车身。车身原始接触坐标与独立world节点坐标误差中位约0.0209毫米、P95约1.34毫米、最大约4.63毫米；相对坐标假设误差则数百米。同一回调的两份sim时间完全相同。运动轮胎节点误差较大，全部节点最大约6.35厘米，不将滚动接触点与动态节点位置声明为数学上完全相等。

因此，上轮保留的**车身接触位置是否可与世界点云比较**这一不确定性已由独立重测显著收紧。这里核验的是位置；法向量与世界几何/同时刻车头方向一致，但没有额外的独立引擎力向量API核验。不将此扩写为所有引擎向量/力语义都已证明。回调仍不给碰撞对象ID或名称。

### 1.2 独立重测再次受困，低表面回波仍被过滤

同一原33点/4.564米路径、原真实SBR，只发一次FollowPath，Bridge follow_path=false，无视觉切换。

- 5.005秒沿路径进展0.041米，目标速度约0.71345m/s。
- 对应101份Native记录全部精确源/载荷/有效ACK匹配；D101、制动0、油门中位0.2140、watchdog0。
- body929事件、wheel18111、unknown0；node161/f3l/sbr_body接触峰值85504.5N。
- 连续实测停稳末速0.01916m/s，自己的动作取消、观察器卸载ACK，父子errors=[]，独立退出进程清单[]。

按独立节点world坐标选取接触邻域，早期两帧均有368份raw点、adapter后116份；车体范围剔除后仍116份，body z>=0.1后0份，marking0，clearing各7份。五个时刻的正式marking/clearing重放逐点误差0。

这支持“**车身受阻位置邻近的低表面已被传感器看到，却没有进入障碍输入**”。仍不指定它属于哪一个场景资产，也不声称邻域每个点都是碰撞面。地面残差最高约0.17～0.18米，低于现有0.25米余量；单改body高度门槛仍会被后一道过滤删除。需要统一实际底盘通过能力、地面/台阶/坡折、障碍与clearing契约。

## 2. 时钟链进一步核验：header对齐不等于真实采集同步

### 2.1 已建立的显式时间关系

独立复制传感器启动器，只读取已初始化的VehicleNode.start_time和DELAY_OFFSET_SEC，不重新设时钟。最终轮真实anchor为1792370573193935406ns，安装桥接DELAY_OFFSET_SEC=10.0。

实际安装代码：

1. VehicleNode._compute_time_offset建立桥接anchor。
2. publisher_callback先poll车辆状态，再用`float_to_time(anchor,state["time"])`建立本轮time。
3. 将这个time传给各sensor.publish。
4. LidarPublisher.get_data使用`sensor.poll()`取得最新缓存，用`_make_header(time,"map")`赋header。
5. adapter再做source→output ROS映射。

由此能把已分配的StateSensor/LiDAR header还原为对应桥接轮询的引擎秒数。独立碰撞峰值8.199500389秒，与所选state轮询8.186000388秒相差约13.5毫秒；这一差值明确保留，不再以Native邻近wall时间猜测时钟原点。

**10秒DELAY_OFFSET不是本次测得10秒通信延迟。** 它是桥接锚定规则的一部分，后续adapter映射另有offset。不能删除补偿或把所有header直接覆盖为接收时间。

### 2.2 已发现的采集身份缺口

本次136份raw LiDAR消息中，有72对相邻消息data字节完全相同但source header不同；其中21对的StateSensor位置已移动超过1厘米。各重复对的payload SHA、header推进量和车姿变化保存在frame_review.json。

结合实际发布源码，能确认桥接在轮询最新传感器缓存时赋予新的车辆轮询时间。**因此，adapter测得state/LiDAR header skew=0，不能单独作为同一次采集或精确采集时刻的证明。** 这不是ROS时钟算术本身失败，也不是已经证明本次受困由缓存时差引起。

限制与全链影响：

- 原始雷达是world点，adapter用同一指定姿态转换并以同姿态TF变回world，静态障碍坐标可抵消这次换基；不能直接推导21次都把障碍平移了。
- 射线起点、近场车体过滤、动态物体、camera像素到3D投影及记忆空间关联依赖真实采集姿态，仍需携带采集身份/时间或可证明的时间区间。
- 本轮未录完整原始camera流，不能宣称已在camera上复现相同缓存问题，也不能用它解释所有历史3D定位失败。
- 不可仅凭payload hash相同就丢帧：静止场景的真正新帧也可能相同。应先核SDK/引擎可提供的采集ID、真实reading time、采集区间和接收时间，再定义下游匹配/拒绝策略。

实际导入的bridge vehicle/utils/publisher base/publisher sensors及SDK lidar五份源码已保存路径与SHA快照，避免只审阅未运行副本。

## 3. 首轮保护退出：诊断准入不能复用启用前的旧代际

首轮 `frame_20261019_083934`：动作接受后约20毫秒触发source_owner_unconfirmed，关门取消，无实质运动，只有轮胎接触。这个结果保留，没有将其算作完成受困采集。

录包显示enable附近从gen0/disabled/ACK-ready切到gen1/enabled/ACK-not-ready。诊断客户端在set enable返回后，只调用通用execution_transport_issue；若其两份缓存仍同时为gen0，就可能暂时通过，随后新gen1控制状态与旧Native回执不一致，触发保护。

- 当前代码及输入反例确认“旧代际一致缓存不能证明本次enable已建立所有权”；首轮录包时序与此相符。
- 首轮没有逐回调保存探针私有缓存，故不把旁路录包顺序当作私有回调顺序的精确证明。
- 只强化独立诊断客户端：保存enable前sender/gen，等待**本次预期新代际**、post-request发出时间、相同Native已应用回执、ACK-ready与原新鲜度全部成立，才发动作。超时仍3秒，源期限和运行中保护不变。
- 最终轮记录enable_requested、enable_current_owner_confirmed、follow_accepted及异常时完整缓存。确认新gen1后才发动作，并完成了受困采集。
- 不能据此宣称生产Bridge历史所有source_owner_unconfirmed均已定位或修复；本次没有修改生产准入或取消逻辑。

## 4. 全系统当前边界

| 层/接口 | 新证据 | 仍未解决 |
|---|---|---|
| SDK/真实车体 | 持续车身接触、驱动已应用；位置world参考独立核验 | 实际台阶/净空/坡折通过包络、具体场景碰撞资产 |
| LiDAR→障碍输入 | 独立接触区域的早期低表面回波被高度门槛删除 | 与真实车体一致的地形/障碍/clearing定义 |
| 传感器→时间/TF | bridge轮询header时钟原点已直接读取 | 真正采集时刻/缓存身份、camera与LiDAR采集匹配 |
| 启用→执行所有权 | 独立诊断器旧缓存准入缺口已加强并验过 | 生产全链动作/控制代际身份审查，历史其它owner异常 |
| 视觉释放→物理恢复→Graph clear | 沿用既有源码反例 | 视觉超时清恢复、成功clear耗预算、同query清退避仍未生产修复 |
| Native DDS | 本次运行中不因源回执失败终止，101有效驱动回执 | 历史间歇迟滞仍待故障帧完整捕获 |
| 目标搜索与记忆反馈 | 本轮刻意隔离，不参与短段 | 完整视觉到达、有效探索、记忆增益、20轮16成功均未通过 |

不能只降低一个阈值、删除停车保护或仅修reset计数后宣布整体解决。下一步应把上述观测与原恢复事务反例一起作为约束，先确定公共接口，再选择最小生产修复。

## 5. 证据与验证

- 本机：`test_artifacts/contact_frame_audit_20261019/closed_evidence/`；主要运行 `runs/frame_20261019_084254/`，前轮保护退出 `runs/frame_20261019_083934/`。
- 18真实Lua接口夹具、最终13真实ROS输入/时钟/启用代际检查通过；不是物理或导航成功。
- 11项证据一致性检查通过，覆盖独立位置、同时刻、显式anchor、过滤逐点重放、重复缓冲、实际源回执、启用次序、停车/卸载；完整导航始终标false。
- 两轮诊断源版本逐SHA保存。旧10项测试源码从后续文件重建后，与首次运行记录57f0864a…逐SHA相同才保存；没有用近似代码冒充旧版本。
- 6153本机404源/归档再核一致；运行时实际GraphNav db52f00e、Smac fc149c80、RPP d2fd9cee，Native原QoS保持。没有merge Native或改日常部署。
- 封包2919861B/129载荷，SHA256 `6a82c23299347ca85c52bfc46a8beba3353e7e81e414f9fef007505bb9bfe2c0`，本机CRC/逐载荷SHA通过。
- 主原始DB268894208B仅服务器保留，SHA256 `0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc`；前轮82583552B原DB清单同包保留。未将核心包称为原始录包全量下载。

本轮未修生产或完成整轮验收；调查继续。
