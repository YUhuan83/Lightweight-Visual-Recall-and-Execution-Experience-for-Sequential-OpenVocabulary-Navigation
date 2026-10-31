# 连续目标与栅格锚点衔接修复（2026-10-20）

## 反例与失败阶段

证据目录：`test_artifacts/continuous_goal_repair_20261020`；服务器同名目录`/home/ly/continuous_goal_repair_20261020`。

- `vehicle_profile_repair_20261019/short_20261019_224809`是诊断解释器缺少lark导致启动失败。工具已分开SDK与ROS launch解释器，并在加载地图前检查依赖；不是FollowPath执行失败。
- 后继`225630`启动/四库身份核对通过，逆向3m规划status=4，返回52点/4.3368m/倒→前→倒三段。单挡短段准入拒绝，运动目标0、SDK平移0；不能称作车辆无法执行，也不能计完整成功。
- 只读打开225630的已关闭bag：直接倒车3m在6张公共全局图、7张公共局部图上都通过既有全车footprint/路径/局部前缀检查。公共图不能证明规划时私有costmap逐格一致；保留原DB SHA及各消息bag/source时间戳。
- 实际Smac插件在自由地图中对同起点、前/后3m及旋转90°控制复现不必要换向/终点偏移；旧源码新增回归失败。地形、车辆、Native、模型均未参与该反例。

## 最早违约及数据流

当前连续起点包装层只枚举`真实后轴位姿→栅格锚点→原生Smac→目标附近栅格`，短目标也必须进入中间栅格状态。位置/朝向离散产生额外换向，即便真实起终点之间已有简单的曲率可行连接。既有完整路径比较只能比较已枚举的候选，无法找回缺失候选。

这证明该包装层有短任务候选缺口，不证明所有历史停驶来自此处，更不能据此关闭7类系统问题。动态换挡、清除/3D净空、语义整路约束、任务级无进展、采集epoch及Native历史迟滞仍分别待查。

## 最小联动改动

文件：`ackermann_smac/src/planner.cpp`，相关测试`ackermann_smac/test/test_planner.cpp`及冻结公共图`test/fixtures/reverse_goal_public.txt`。

1. 复用已有Reeds-Shepp连接器，增加`真实起点→真实目标`的完整候选。沿用既有radius/min_section/采样/8192点上限，不放宽车体、曲率、未知空间或总时限。
2. 保留原Smac栅格搜索；长任务、连接受阻、直接候选不成立时仍走既有候选。原平滑失败保留同次raw路线、参数冻结、非连续起点门禁不删除。
3. 最终在同一锁定地图版本重验两类路线，计算沿后轴路径积分的inflation代价；未知计最大可通行风险，不奖励未知。直接候选仅在几何换挡成本（原length+3m/cusp）和风险均不劣时替换已选原路线。原路线的容差终点到请求终点距离仅作完成同一任务的成本下界，未更改目标容差。
4. 原路线均不安全或搜索失败时，可采用独立通过完整检查的直接候选；任何返回仍受总deadline约束。风险值不是三维净空证明，也没有给物体新增语义适配。

## 验证与边界

- v1实际C++：14 planner、4 connector、7 path_contract全部通过；冻结公共图逆向3m变为一段3m精确目标路径。
- 补充风险对照初稿假定原生analytic扩展会绕过非致命高代价条带，实际原生本身返回穿越条带的路线，故该测试失败；生产比较保留了native，没有绕过低风险替代路线的证据。保留`risk_fixture_v1_failed.*`，不以放宽断言处理。
- 对照实验仅将该测试的analytic_expansion_max_length设置为.5以形成真正低风险的栅格替代路线；生产配置50m不变。补充后15 planner、4 connector、7 path_contract全通过，覆盖该低风险绕行、真实公共图、自由地图多方向、障碍拒绝与恢复，以及原有平滑/预算/参数生命周期回归。
- 该实验同时提示原生analytic扩展不是非致命代价避让的完整保证，不能把非致命高代价当不可通行语义硬约束；第4类问题仍开放。
- 真实物理短段`physical/short_20261020_164936`通过：416源成员冻结，实际/proc映射核Mapper/Planner/新Smac/RPP四库；新Smac SHA `a0b4453f728e27944dac68587524b51ff59370d102afdd71a57ce14cad286cc6`。使用原物理launch仅隔离GraphPathBridge动作所有者、无模型/搜索/视频；新source ACK后下发一个FollowPath，status=4，返回约3.000003m/单挡路线，SDK实际平移2.178889m、odom位移2.181171m，距目标.818844m（当前goal_checker容差1m）。不能说实际完整走了3m。
- 13次SDK停车样本、stop_gate_closed=true，父子errors均空，停车后读速.03335m/s；原723节点/467车身碰撞节点/四轮各32碰撞节点/1449.72kg，未启用kinematic或collision override。结束预检项目进程[]，无display148锁。证明这一条直线倒车闭环，不证明弧线、换向、多段交接、20轮或完整视觉搜索；历史Native间歇延迟仍未关闭。

无正式部署替换、无commit；完整视觉到达、记忆收益及20轮16成功仍未验收。源父版本及失败反例保留，待相关必需验收通过后才按AGENTS规则提交。

## 本机封存核验

- `continuous_goal_evidence.zip`：6006928B/551载荷，SHA `e96756302a923e7761811ee14fdb49a98dbe6dc60b4be8cdab5fe24126a87cff5`。
- `profile_component_evidence.zip`：3168431B/625载荷，SHA `e234841f6f5305a7c64f8ca4591642be0f9c493d0ac7f1d796a776c27a58a2ff8`。
- `verify_evidence.py`核两个ZIP的CRC、全部载荷SHA、实际26项C++ XML，以及416个冻结源与本机一致；复制真实公共图fixture及紧凑现场结果到`verified/`，详`local_verification.json`。未将编译/路径单独当成功。
- 现场运动期间42个姿态采样相对初始直线的最大横向偏差约.004257m；该轮Native RPC P95约8.46ms，仅此短时窗口，不能关闭历史间歇延迟。
