# Visual Experience and Recall：当前组件命名

2026-10-15起，整体称 **Visual Experience and Recall（视觉经验与检索系统）**。按存储、检索与决策职责命名，不再统称“四种memory”。本次仅更名与兼容导出，不改变算法、容量、数据库结构、配置键或ROS消息协议。

| 原名称 | 当前名称 | 职责 |
|---|---|---|
| VisualMemory | **ExecutionExperienceStore** | 保存场景描述子、执行反馈与方向评分，检索相似场景的方向先验。 |
| VisualMemoryMap | **GraphObservationStore** | 在已有导航图节点/heading bin上保存观测、物体、覆盖与遍历记录；不额外建立几何地图。 |
| ViewpointMemory | **ConfirmedViewpointCache** | 缓存已确认物体的观测视点、物体区域和失败冷却；为同进程后续查询提供候选。 |
| SemanticLandmarkMemory | **PassiveLandmarkIndex** | 被动保存有LiDAR空间支持的轻量视觉描述子，收到新目标后检索可能相关的历史区域；结果仍须重新观察确认。 |
| MemorySelection | **RecallCandidatePolicy** | 比较历史候选的图路径、车辆可行性和代价，选择/退避/恢复；是决策策略，不是第五种记忆。 |

## “Semantic”中哪些是规则

原SemanticLandmarkMemory确实有历史数据：受字节预算限制的float16特征、位置、观测者姿态、时间戳和独立支持。因此把它叫纯判断规则也不准确。现在用Index突出其历史特征检索职责。

相似度门槛、空间支持、实例隔离、语言关系约束和候选比较属于policy/rules。它们读取历史数据并做判断，不因为名称含semantic就构成另一份记忆。关系规则仍由relation_query实现，导航候选策略由RecallCandidatePolicy实现。

## 数据流

当前相机/位姿/LiDAR与模型特征
→ PassiveLandmarkIndex收集历史特征，GraphObservationStore记录已有节点观测
→ 收到新query后检索历史区域/ConfirmedViewpointCache历史视点
→ RecallCandidatePolicy比较候选
→ GraphPathBridge负责规划与执行交接
→ 当前观测重新定位、确认与接近
→ ConfirmedViewpointCache更新确认视点；ExecutionExperienceStore依据执行反馈更新方向先验。

这条链中，索引命中、路径生成和动作成功都不能替代视觉到达。历史语义线索只引导搜寻，不直接宣布物体找到。

## 兼容与生命周期

- 生产nav.py已使用四个新模块/类名；GraphPathBridge使用RecallCandidatePolicy。
- 旧Python模块保留薄兼容导出，旧类名是同一个类对象的alias，不能产生第二套存储或复制已有状态。
- 配置键visual_memory_*、ROS话题/evidence字符串、SQL表及导航内部状态字段保留兼容。它们不是额外的记忆类别。
- 当前实验维持同进程模式；SQLite配置`:memory:`不跨进程保存，PassiveLandmarkIndex和ConfirmedViewpointCache也是进程内有界存储。本次没有启用持久化。
- 历史故障报告/旧图保留当时名称，以本表映射到当前源码，避免覆盖历史证据。

## 源码

视觉组件：`wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/`下的execution_experience_store.py、graph_observation_store.py、confirmed_viewpoint_cache.py、passive_landmark_index.py。

候选策略：`nav2_adapter/wildos_nav2_adapter/recall_candidate_policy.py`。

更名等价性证据：test_artifacts/offline_recall_viz_20261015/rename_equivalence.json。7个生产模块经名称/import规范化后AST与冻结71f9版本一致；这只证明更名没有修改其逻辑，导航正确性另由相关回归/现场验收判断。
