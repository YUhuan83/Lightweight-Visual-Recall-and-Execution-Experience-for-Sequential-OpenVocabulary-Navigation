# 车辆几何与GraphNav接口修复（组件通过，短段待验收）

2026-10-19。接续 `CANDIDATE7_REVIEW_20261019.md`。主证据 `test_artifacts/vehicle_profile_repair_20261019/`；9份改前源码/测试及逐SHA保留。正式安装/6153不修改，不录视频、不commit。

## 最早违约点

- `official_graph_stack.launch.py` 未接收真实载体几何，Mapper沿用默认横向净空0.3m。真实接触附近节点距障碍约0.71m、SDF约1m，仍feasible；但SBR半宽1.05m。Nav2检查正确footprint不能使上游目的地变得可达，可能导致反复拒绝。
- `Mapper::get_graph_msg` 把横向SDF净空填进 `RobotMotionConstraints.ground_clearance`（竖向离地间隙）；未指定body/动态字段以0发布，而消息约定负值才是未知。
- `navigation_graph.cpp` typed边坡度直接发布rise/run，typed节点/约束却是atan后的弧度；消息要求弧度。内部地形阈值仍应保持rise/run，不混用。

## 当前候选

- `graph_geometry_profile(config)` 从实际local/global Nav2矩形footprint和padding推导必要的方向无关内切圆净空，当前 `min(3.4,1.05,1.05)+.01=1.06m`。拒绝两图几何/padding不一致；经实际physical→official_graph_stack的launch参数传递到Mapper。
- Mapper新增通用几何参数和校验：指定平面尺寸须完整，横向最小净空不能小于配置的内切圆半径。发布4.45×2.10m平面尺寸、非对称footprint顶点、padding与单独horizontal_clearance属性。高度/底盘离地间隙/未标定动态上限仍为-1，不能捏造数值。
- 保留全链当前 `robot_profile_id=default`：视觉评分两处和执行反馈也用该ID，只在Mapper/Planner改名会使实际物理评估被忽略。运行门禁同时核Mapper/Planner ID和地图几何。
- typed边坡度发布atan(rise/run)，内部指标和阈值不变；profile最小转弯半径为现有控制器8m，不改变Smac规划12m/RPP8m的余量或物理参数。
- **内切圆只是必要条件**，不能证明前伸3.4m的矩形转向扫掠可行；仍须Smac完整footprint校验。未知空间/原5s证据TTL、地形3D净空等尚未解决，不得把本次改动称作车辆规划完成。

## 当前验证状态

- 本机runtime_contract27项、既有接口替身570项通过。
- 已添加5项C++回归：未知值语义、后轴非对称profile、尺寸/净空矛盾拒绝、已通行历史不绕过1.06m必要条件、typed边坡度单位。
- 初期构建记录：415成员父源逐SHA核对后叠加9文件，随后因只读配置及安装器遗漏扩展为14文件；构建与后继真实ROS/DDS结果见下节，原SSH62011已结束。
- 原连通地面完整冻结源 `connected_ground_candidate.zip`：415文件/5817434B，SHA `49cd7731e124beca49a24e78c69fe738d443c1ba67e88f05b4282859004c3750`。该包是本次几何改动前父版本，不是本次几何最终候选。
- Mapper C++类布局已变化，后继fixture/exporter必须用新header重新编译；不得将旧分配大小的fixture二进制直接链接新库。现有Mapper节点与测试同包重新编译，正式安装不替换。

后续：读构建/实际C++测试，核physical入口真实launch参数与DDS profile、完整运行源及旧交接回归，再决定是否允许真实车体短段。完整单目标、记忆收益、20轮至少16成功仍未完成。

## 后继验证与补充修复

- v1独立构建四包通过，实际Mapper27/Planner40 C++通过；`profile_dds_20261019_220517`实际physical入口只启动图栈，参数与DDS均为1.06m净空、4.45×2.10m平面body、非对称footprint、8m配置转向半径，高度/底盘净空/动态上限-1。源profile速度0原先是未知量编码缺陷，当前没有证据说明某消费者据它直接令车辆停下，不能将其当历史停驶根因。
- 再发现参数状态缺口：`profile_dds_20261019_220727`实际ROS服务接受把最小净空改成.3，但内部仍发1.06。该轮`passed=false/reproduced_mutable_profile_defect=true/errors=[]`是反例，不是通过。启动时只读一次的几何/物理字段和Mapper/Planner profile ID现以只读descriptor声明；occupancy输入订阅选择同样不可在运行期虚改。
- v2重编及27/40 C++通过；`profile_dds_20261019_221108`真实参数服务拒绝修改净空及两个profile ID，参数与内部DDS一致，passed=true/errors=[]。CLI启动时参数覆盖仍正常；不冻结查询/动作服务或控制enable。
- 继续发现自助安装器漏选`graphnav_mapper`、库门禁未包含Mapper、Python路径硬编码旧工作区。已将Mapper纳入构建/四库身份、依赖路径绑定本次bundle，更新安装器fixture和28项console测试；不为通过旧hash换回旧库。修改范围因此从9文件扩为14文件（含PlannerNode及四个部署/测试脚本），改前文件均保留。
- v3完整415成员逐SHA一致，视觉包另行安装；安装后的Python/config和四库共108项核对通过。真实ROS runtime27/official51/pointcloud23/cloud_filter1/console28、安装器夹具、原60真实ROS事务/51记忆测试通过。初次记忆DDS探针因缺少本轮构建身份`result.json`在0阶段退出，失败保留；补齐由实际XML/库SHA生成的组件报告后，5阶段记忆决策通过。
- 最终15阶段真实Bridge/Planner/DDS目标交接、5阶段真实记忆决策、9阶段实际Mapper CDR→DDS→Planner均通过。9阶段fixture按新Mapper头/库重新编译，避免类布局ABI混用；2个旧风险阶段仍保留原TTL语义，不宣称旧危险已消失。
- 连续地面服务器证据另见candidate7_review目录：88成员/3411918B，SHA `4f4e63457ae593a0cc974402545d7e5873dfc32aa40c1cdc54d98cbf821ac5b3`，本机87载荷核验通过。

## 短段结果与后继证据（2026-10-20更正）

- `short_20261019_224809`诊断脚本SDK解释器缺lark，启动失败，未执行；已分开ROS launch与SDK解释器并在加载地图前预检。
- `short_20261019_225630`启动通过、四库核对通过，但逆向3m生成倒→前→倒三段，单挡测试准入拒绝，motion_goals=0、SDK平移0；不能称车体FollowPath失败。
- 后继隔离Smac新增连续目标候选，见`CONTINUOUS_GOAL_REPAIR_20261020.md`。实际`short_20261020_164936`一个FollowPath status4，SDK实际平移2.178889m，距目标.818844m满足当前1m容差，停车门确认，原真实车身/轮胎碰撞保持，父子errors[]、后检进程[]。仅直线倒车，不验证GraphPathBridge动作所有者、转弯/换向/多段或完整搜索。
- 前述几何/C++/实际ROS/DDS及旧失败轮次封包已取回：`test_artifacts/continuous_goal_repair_20261020/profile_component_evidence.zip`，3168431B/625载荷，SHA `e234841f6f5305a7c64f8ca4591642be0f9c493d0ac7f1d796a776c27a58a2ff8`；本机所有载荷校验通过。原始bag留服务器并记录SHA，不重复下载或录制视频。没有正式替换或commit。
