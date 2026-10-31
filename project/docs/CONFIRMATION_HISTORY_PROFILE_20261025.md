# v30 稀疏可靠定位的确认历史配置

2026-10-25。活动候选 `test_artifacts/confirmation_window_repair_20261025/candidate`；服务器 `/home/ly/execution_continuity_review_20261022/v30_confirmation_history`。保留v28/v29/r2，未覆盖主目录用户改动，未commit。

## 已复现的配置问题

v29r2现场4条可靠LiDAR空间位置接近，跨37.921秒，原30秒确认历史在第四条到来时淘汰第一条。实际ROS类对相同原消息回放：30秒 retained=3/confirmation=0；60秒 retained=4/inliers=4/radius=0.033695米，最新观测和query校验仍走原代码。此为几何确认组件通过，无地图动作、无语义真值、无到达。

`official_adapter/launch/official_graph_stack.launch.py`仅新增显式 `confirmation_window_sec=60.0`。这是历史支持样本的保留时间，**不是**把最新检测/三维位置的新鲜度放宽到60秒。通用GoalStandoff类默认值仍为30秒，原最新3D≤5秒、新鲜掩码、四条支持、1.5米簇半径、query/实例、2米车身外缘到达及旧几何禁止完成规则不变。60秒属于原接口允许的5–120秒范围，不增加目标名称、坐标或特殊场景分支。

## 验证

- 新增8项配置/实际消息回调回归：启动配置60、原4条消息能确认、30秒反例、只有3条、历史超过60秒、最新观测超过5秒、重复帧、切换query。修改前仅配置项失败，其余明确覆盖对照；修改后通过。另连同到达、歧义、可见接近及连续探索共120项回归通过；原828项离线回调通过。
- 实际ROS验证708项、实际导入定位/几何/同步59项通过，随后才进入现场。
- 构建验证入口初次漏加候选visual_navigation路径，导致新check_lidar与旧foreground_depth混载，回归报reference_position签名不匹配及旧中位数期望。此时未启动整车。保留失败日志，修复构建/验证PYTHONPATH显式候选视觉根路径后重跑通过；未修改生产实现或放宽断言。后续复制build脚本必须保留该候选视觉路径。
- 现场只读 `ros2 param get /beamng_vehicle_goal_standoff confirmation_window_sec` 确认 `Double value is: 60.0`，记录在服务器 `actual_confirmation_window.txt`。不能仅凭本机配置宣称部署生效。

## 现场边界

按原Town真实车体/绿色房屋/目标下发后180秒预算启动，结果待关闭。需要分别核实几何确认、路线执行、车身2米距离和实际绿色房屋图像，不以确认日志替代完整成功。未录视频。

证据根包含 `confirmation_window_replay.json`、`localization_messages.json`、before/after回归日志、部署增量和候选源。v29/r2几何修复细节及失败记录见 `RANGE_CONSISTENCY_REPAIR_20261025.md`。

## 现场关闭结果（覆盖上文待关闭状态）

`trial_20261025_173253`：启动49.990秒、搜索180.189秒、收尾22.227秒，超时、完整验收false、session errors=[]、console0、closed。后检项目进程/X148锁清空，无录像/commit。

- Native覆盖180.234秒，近停40.916%/73.744秒，最长12.643秒；净38.857米、采样90.915米。stale48.484秒，其中planning44.542秒；换挡4.820秒。66次planning、32次planned、29次following、3次no_vehicle_path、6次short_route_probe。整体规划执行仍未稳定，不把此轮当修复成功，不用v28的15.668%单轮替代当前失败。
- 26条掩码、有效3D=0、confirmation=0。60秒参数实际生效，但这一轮没有定位证据，不能据此评价它在现场能否完成四帧确认。相同v29r2消息的30/60回放结论仍仅为受控组件证据，不能替代此次现场失败。
- 4个最长近停约12.64/10.97/10.41/9.05秒。`long_stop_candidate_audit.json` 显示对应请求均在车前（投影为正），多数单候选间隔约1.4–1.9秒，反复在多个位置/朝向请求后才退到约4米短探针。不能将这些停顿归因为D/R换挡或倒车方向问题。失败缓存、变化地图、frontier身份切换与重复候选之间的关系仍待查，尚未实施此方面修改。
- 代码也显示未达到四帧支持前，`GoalStandoff.on_localization` 不发布物体standoff；可靠但未确认的三维位置对接近指导的利用仍是后续应审查的交接缺口。此项为源码事实与设计缺口，不能直接当本轮零定位或全部停顿的根因，也未据此临时增加分支。

本次候选对照v28累计运行改动仅两个定位文件与一个launch参数，范围及SHA见 `source_scope.json`，可审查差异/增量为 `v28_to_v30.diff`、`v30_delta_from_v28.zip`。原主目录703运行文件和原ZIP SHA仍保持，未提升为已验收基线。当前没有本项目仿真在后台运行。
