# 评分与执行经验契约修复候选（2026-10-15）

## 变更前设计

依据SCORE_MEMORY_PARTICIPATION_AUDIT_20261015的真实反例，先修观测与学习数据流，不混入停车/连接器恢复算法。
输入契约为：传感器可用距离→几何frontier投影/FOV→真实模型分数；场景描述子+相同执行锚点/采集时间→实际执行反馈。
输出契约为：默认分数不能冒充模型观测，未监督方向不产生惩罚；历史信息只能调节当前分数，不能宣告安全或到达。

## 实施范围

1. official_adapter/config/beamng_wildos.yaml用YAML anchor共享observation_range_m=40m与frontiers_range。
   40m来自现有点云过滤范围；nav2_official_control.launch.py读取同一配置，点云距离保持40m。
   相机正深度、FOV、frontier heading筛选、局部/全局障碍、真实车体/半径/速度/目标确认门保持。
   模型暴露只读visual_frontier_range_m；现场preflight核对实际模型与cloud_filter参数，拒绝不一致。
2. execution_scene_samples.py在每次有有效描述子时采样，不依赖可评分frontier。
   锚点采用执行端同样的最近有效节点/6m界限，保持同frame、采集时间、UUID严格匹配。
   相同节点至少.25s才新增，重复/乱序戳拒绝，TTL60s、上限512条，避免连续推理挤掉稍早动作起点。
   1152维float32描述子满512条约2.25MiB（不含小量元数据），不保存图片/全特征图；不跨进程。
   query切换保留目标无关场景经验；旧动作归属/执行源仍由原反馈保护，无放宽迟到旧动作学习。
3. ExecutionExperienceStore增加supervised_mask列，旧NULL掩码保持历史兼容；新记录只有实际执行bin有监督。
   无监督方向检索返回1（乘法中性），各方向只对支持自己的记录加权，避免未见方向被打低分。
   Nav不把目标偏置后的frontier分数作为执行标签，样本heading_scores=None，反馈单独监督执行方向。
   API保留已有显式profile调用以兼容旧调用，生产新样本使用中性起点。
   正面反馈可能写入并命中但保持中性，不以“必须改变排序”为验收来伪造负面经验。
4. frontier_score_state.py保存默认/视觉来源、真实raw/mean/std、观测计数与更新时间。
   默认分数计数0；首次视觉观测重置默认统计；视觉缓存5s过期退回默认；读取不会刷新真实观测时间。
   缓存时保留同次语义traversability；重复使用topology snapshot先替换派生properties，避免旧query字段堆积。
   本次cache age为处理更新后的缓存年龄；传感器源龄仍由原采集戳/运行时证据独立核验。

## 回归与部署

- 6项新增机制回归：9m反例/14m正确投影、背后/超距拒绝、共享配置、无frontier样本、相同执行锚点、
  重复/乱序/过期/延迟反馈、真实生产反馈回调、重复反馈不重复学习、512边界、默认/视觉/过期恢复/重复properties。
- ExecutionExperienceStore另增独立方向监督合并测试；旧数据库迁移/重启遗忘/地图隔离和非有限保护通过。
- 新runtime契约测试拒绝9/40错配与NaN/无穷/0。最终全局49PASS/0FAIL/6WARN；回调替身/合成链含重叠，不能相加成现场次数。
- 服务器重建adapter/official/visual三Python包；6机制、5经验存储、23runtime回归通过。
  C++库、规划器/控制器/车辆物理与参数未改。
- 初版bf3ee70d通过软件回归，之后为进一步隔离目标偏置改3个相关文件，最终434成员
  SHA042ee7af47bd9e24d57cf9cee38081de6fe3122ebe9bb0effdbfb2052d876a94。
  相对751218限定14个文件；各版本逐员核验，旧bundle按SHA保留。
  deploy/20261015_score_memory_contract_candidate冻结，必要现场验收待完成，不commit。

## 现场验证（进行中）

继续使用同一显式诊断包装，120秒真实车体、先预检后解锁，录真实RViz双栏。
验收关注真实scorer调用、写入/检索、默认来源、模型和控制输入源龄；不以120秒组件试验代替目标到达或20轮稳定性。
完整目标搜寻、停车后的局部受困、后续多目标召回仍待验收。

## 组件反例与追加时序修正

- 042ee7候选session_161453已真实运行：评分调用从0恢复，执行反馈写入从0恢复；但真实评分刚写入即变默认。
  原因是update_navgraph先取得now，_update_frontier_score稍后再取wall_time写observed_at，
  fresh_visual按旧now看到微小负年龄，立即回退覆盖。
  该问题由本次缓存修复引入，不能归咎BeamNG/原论文或旧模型。
- 保留cache_clock_counterexample.log：把测试时钟改为每次递增1ms后原逻辑失败，证明原常量时钟夹具漏覆盖。
  同一评分更新事务改用同一个now，未来时间保护保持；6机制回归/全局49PASS及服务器6/5/23回归通过。
- 新434成员ef3fc7f814683f640e2d4a93dfc5ef601f76f7f1a6e85e56187631f8fec5a54b，
  仅相对042ee改Nav时钟与对应测试2文件，服务器停稳并清理161453后部署。
  新冻结deploy/20261015_score_memory_contract_clock_candidate，042ee回退保留。
- 新session_161910正在120秒组件复测：中途90次评分、7个真实视觉节点/26默认，
  说明同帧回退已停止复现；完整封闭统计另补，不提前宣布现场验收成功。

## 最终组件封闭结果（161910）

- 最终运行ef3fc7候选，配置/安装/原C++库身份及模型40m/cloud_filter40m实际参数预检通过。
  启动期+120秒目标期243处理/243真实评分，记录最多10个真实视觉评分节点；无诊断错误。
  202条route_status中160条目标匹配近期真实视觉评分节点，42条匹配回退节点。
  这是数据流与来源证据，不是同输入memory开关的因果成功率对照。
- 6条反馈全部存储，6条experiences、146次检索命中；先前组件042ee的8条也全部写入，
  但该版真实分数被时序错误覆盖，不能混算其评分采用验收。
  新经验先验本轮保持乘法中性（memory_modified_scores=0），没有伪造负面标签。
  GraphObservationStore覆盖/对象效用项改变了候选值（所记录最大差约.1893），不等同证明选路更好。
- Native37.3512m；18following/14maneuver_done，3follow_failed，均为控制器
  Ackermann lookahead would reverse a single-direction path，发生在elapsed8.349/12.735/18.839秒。
  这组真实评分/经验修复未解决实际车辆执行故障，未放宽反向保护。120秒到达仍超时，无确认对象。
- 外部odom源龄median.14275/P95.33350/max.79914秒；RGB median.15454/P95.33969/max.83609秒。
  模型所用图快照年龄median1.28459/P95 1.81605/max2.15204秒，评分计算增加的延迟需继续优化/核查。
  外部odom新鲜不能替代控制器内部TF位姿证据，本轮没有独立内部TF观察者；不宣称所有时间戳故障排除。
- 两轮均SDK持续停稳/原723节点467车身碰撞及轮胎保留、session errors=[]，独立ps无仿真/RViz/录制残留。
  20轮成功数未增加，完整绿色房屋/后续多目标未验收，不commit。
- 两轮104原始成员封闭证据test_artifacts/score_memory_repair_20261015/field_evidence.zip，
  SHAc78c7f62b9a1aa1e6d47c5a3c17bde57575b545f74a9935c6c05a27266464fd1，ZIP与所有成员SHA核对。
  首次校验在scp尚未结束时被文件占用拒绝；等待scp退出0后完整校验通过，不当作机器人故障。
- 真实RViz录像已取回captures/20261015_score_memory_repair/：
  rviz_clock_failure.mp4（124.5s/996帧，SHAcbcba42d8f20879d10b50355a08b54e136426cc9349a80a5995ccb2d570fbb50）；
  rviz_component_retest.mp4（123.5s/988帧，SHAf4c4db4ac808d6c1aed97aed75d5dc07cb6e0cb87363d22ee14cc899fe4c174b）。
  两者1920×1080/8fps，逐帧可解码、真实左二维图右camera；均不是成功到达视频。

## 当前后续事项

本次修复范围的评分调用/来源/反馈写入/检索组件链已验证，完整系统未通过。
下一步沿3个实际单方向反向中止对齐路径端点、控制器内部位姿/源龄与刹停/换向交接，
然后做相同输入评分/经验消融及完整搜寻。保留模型快照约1.3秒延迟问题，不只调阈值或依据里程宣称成功。
