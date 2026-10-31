# WildOS评分与记忆参与度核查（2026-10-15）

## 结论与边界

用户要求优先核实评分与反馈是否真正参与决策，暂不叠加停车恢复修改。本轮没有修改生产导航/模型算法或阈值。
服务器仍使用431成员候选751218fde4ef9575a606b53e440719c6b737bb2840d54fa2ebaebe0738527f8c。
实际GraphNav来源tracking_contract_candidate的9个C++源码/头文件与当前bundle逐字节一致。
此检查不证明论文原系统有错，也不是视觉记忆方法原理失败的证明；暴露的是当前部署范围约定及经验采集耦合。

**现场没有正常用上预期的视觉frontier评分与ExecutionExperienceStore学习闭环。**
网络仍计算图像特征/前沿图，但真正把图像转成几何frontier分数的score_geofrontiers没有被调用。

## 封闭证据

- 上轮session_151859：17条执行反馈，16次无近期样本、1次节点/时间匹配失败、0次Stored visual-memory feedback。
  原120/420秒日志缺真实评分来源和数据库行数，不能仅凭进程存活或is_default_scored标志确认生效。
- 新诊断session_155034，启动期+120秒green residential house；349次处理/经验检索。
  110份约1Hz详细评分记录（15份door初始化、95份green目标），真实scorer调用0、可见评分帧0、经验检索命中0。
  记录到最近frontier距离最小10.871489m，配置frontiers_range=9.0m；记录中within_config_range始终0。
  最终3条反馈、0条经验、0待匹配样本；3条反馈均在pending_before=0时到达。
  没有把稀疏1Hz记录冒充349次全量范围统计；0 scorer调用计数覆盖全部349次处理。
- GraphObservationStore最终8条node_observations、3条traversal_history、5条search_coverage，说明确有写入。
  所观测候选效用相对去除对象/覆盖记忆项的计算差为0；本次不能宣称该存储已改善选点。
- PassiveLandmarkIndex最终520条、持续采集；ConfirmedViewpointCache无确认对象，query候选没有命中。
  单目标短测不验证后续新目标检索/记忆导航能力，不能将所有记忆模块一概判定失效。
- GraphNav 255条route_status的目标均匹配近期评分图中的节点，7个不同目标。
  这与源码评分图订阅/精确拓扑合并相符，说明数据流在工作；不是评分开关的控制变量因果对照。
  因本轮真实scorer调用0，使用的是几何默认分数与其方向/风险等后处理，不能称视觉评分生效。

## 三个明确问题

1. official_adapter/config/beamng_wildos.yaml:20限定frontiers_range=9.0。
   GeoFrontierToImage先按节点与车辆距离截断，再进行投影/FOV筛选。
   本次几何前沿在该范围之外，真实scorer的输入为空。不是通过降低目标识别阈值解决的问题。
2. nav.py:994只有raw_score_rows非空才加入pending_memory_samples。
   因视觉特征采集与“恰好存在可评分frontier”绑定，模型即便持续处理图像也不保存执行监督所需样本；
   反馈能到达，但学习库始终为空。尚未更改为虚构分数/空样本，也未放宽节点/时间匹配保护。
3. nav.py:1194/1205缓存命中分支一律写is_default_scored=0；默认分数也进入同一缓存。
   下一帧缓存默认分数因此被误标为非默认。本轮出现nondefault_nodes>0但scorer调用0的直接反例。
   不得继续用该字段单独判断真实模型评分或学习是否生效。

## 诊断方法与回归

- test_artifacts/search_mechanism_audit_20261015/audit_vision.py为显式诊断包装入口。
  观察实际函数调用、原返回值、反馈前后数据库行数；不修改输入/返回或导航参数。
  三项包装测试验证返回/参数身份保持、观察异常不改原结果、原异常保留，均通过。
- 诊断读取的是模型自身进程的私有SQLite连接；没有跨进程读取活动rosbag数据库。
  只在试验关闭后读取bag，避免重复151045的数据库锁干扰。
- 首次诊断154833因预检只认识原模块入口、不认识包装脚本而拒绝；没有发送目标/解锁。
  为独立诊断preflight显式登记audit_vision.py绝对路径，保留唯一进程/DDS/源输入保护，另启用RViz/图像流检查。
  第二次155034预检通过，120秒后正常超时、SDK持续停稳、errors=[]，独立ps无本轮残留。
  此项是短时机制诊断，不是绿色房屋到达验收，不计20轮成功；未commit。

## 视频与证据路径

- 真实RViz录屏：captures/20261015_score_memory_audit/rviz_live.mp4，1920×1080，992帧/8fps=124秒。
  已全部解码，实际截图核实左侧2D节点/frontier/路线/代价图，右侧实时camera；未确认物体故无名称。
  不是前摄像头单独视频，也不是合成布局；SHA1ec7a6a85986f47822c2e3982c4396f7caa64a8e1fff470516eb0d9dbbd64043。
- test_artifacts/search_mechanism_audit_20261015/score_memory_audit.zip，80原始成员，
  SHA75e8088db178a67b94f7470f8b63aaafe6b9a8eb75710865a9271a4a000c60da，ZIP与逐成员SHA通过。
  解包evidence/audit_results/session_20261015_155034/audit_summary.json、vision_participation.jsonl及接口bag为主证据。
- 另对151859停止现场使用真实安装连接器重放最近发布代价图：6000和100000预算均0候选；
  同姿态自由图可找到候选。输入不是planner私有地图精确快照（OccupancyGrid有量化），
  不能把“6000预算不足”当作该轮停车的已证根因；本轮未修改连接器。

## 后续修复顺序（尚未实施）

1. 对齐几何frontier、可观测地形范围和视觉投影的契约；保留视场/空间支持门，避免仅任意调大常量。
2. 将执行监督样本采集与是否存在可评分frontier解耦；显式保留未观测方向的不确定性，不用默认分数冒充模型证据。
3. 为评分缓存保存真实来源与新鲜度，修正默认分数被标为非默认；补失败反例、恢复和目标切换回归。
4. 相同输入比较视觉分数/默认分数、记忆有无的候选排序，再做真实车体与完整目标验收。
5. 停车后的无可执行路线/局部受困恢复另行修复，不能以本次评分诊断称已解决。
