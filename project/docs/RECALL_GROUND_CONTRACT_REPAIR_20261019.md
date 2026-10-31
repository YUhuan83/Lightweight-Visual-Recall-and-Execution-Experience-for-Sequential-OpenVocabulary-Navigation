# 记忆到规划的契约修复与地面拟合修复

2026-10-19。用户授权修复已列问题，并恢复 SSH。此次完成第一批候选与隔离验证；没有宣布所有问题修复，没有启动整车搜索、录视频或 commit。主代码其余工作区改动保留，原6153部署包和正式安装保持。

## 修改和上下游约定

### GraphObservationStore → heading candidates → GraphNav

- 原 search_coverage 把节点各方向的最大负证据共享，导致 heading0=1、heading8=.1 后，heading8 也按1扣分。新表 search_heading_coverage 按 map/query/node/heading_count/heading_bin 保存证据；实际 `_append_heading_candidates` 读取对应方向，整体 penalty 为各方向之和除以分箱数。
- 分箱分辨率显式传入；分辨率改变不会把旧 bin 当新方向。重复 bin 去重、无效值在写入前拒绝、更新与历史汇总同一 SQLite 事务。旧汇总继续保留作诊断，无法恢复方向的旧最大值不猜配到各方向。
- 兼容掩码只保存低63位，实际计分表支持1～1024 bins；空候选不查零分辨率。每地图默认最多4096个 query/node 覆盖记录，驱逐时通过外键连同精确方向记录删除，改变分辨率清掉该记录旧分辨率明细。不代表其他历史表的容量问题也已解决。
- `_record_negative_search` 不再把 NaN/空输入当强负证据，也不将 `1-raw cosine` 当概率。按当前 mask_threshold 的相对差距产生有限偏好；相似度已过阈值但被竞争/连通域支持拒绝时，记观察但不添加负惩罚。这仍是当前视图的弱搜索证据，不是证明整个区域不存在目标；可视覆盖语义仍有后续工作。

### ExecutionExperienceStore → frontier scores → physical constraints

- 旧兼容 `retrieve()` 的0～1惩罚语义保留；新增生产入口 `retrieve_execution_prior()` 使用明确的0～2乘子，1中性。实测成功奖励1、label_strength=.8时，执行过的方向乘子1.8；未监督方向保持1。旧数据库新列为NULL时沿用原惩罚语义，不把旧数值重新解释成新成功证据。
- `append_scene_sample` 保存当时 yaw/frame；回调把它与执行反馈一起写入。检索时将执行方向按历史视角到当前视角的 yaw 差旋转到当前方向分箱；缺少有效方向、frame不一致时不应用新乘子。避免在新成功偏好中复制旧全局方向到旋转后的视角。
- `fuse_execution_prior` 仅改变已有视觉支持的方向，维持0及显式抑制值，不改变输入数组，将输出限制在0～1；既有物理 feasible、障碍、曲率、动态风险和动作归属保护不改变。动态/取消等不可归因结果仍不强行学习为地形失败。
- 该批解决成功-only反馈恒等的问题，不证明跨场景迁移正确率或实际记忆搜索提速。

### 地面拟合 → 两种占据图及 terrain

- local_ground_plane 保留每格最小Z对应的真实XY，修复只挪XY到格中心造成的坡面高度偏差。已知平面 z=.24x+.24y-.45 的原截距 -.5556；修复后 -.45。正负坡向、物体返回、打乱顺序及缺少支持均有回归。
- 未降低原高度/地面余量，不把修正地面拟合宣称为低表面遗漏修复。22帧真实084254输入重放：旧实现四层地图逐格一致；新实现总 marking 29573→29566，四层累计差异为 traversability 1格、unknown 0、elevation 129816格、elevation_valid 1格。大量高度变化主要是未知高度的平面填充值，不提高其有效标记。接触邻域早期 marking 仍为0。

## 修复前反例与验证

本机 `test_artifacts/recall_contract_repair_20261019`，地面/真实点云 `test_artifacts/passability_contract_20261019`。

- 保留5份改前源码/测试及SHA清单；其余补充基线从不变的6153包恢复，位于 baseline_additional，明确其归档来源。首轮Windows临时目录权限/夹具问题、float32严格比较失败、修复前接口失败及后续通过日志分别保留。
- 修复后本机51项记忆/实际方法检查、19项点云/terrain检查、570项既有ROS接口替身回调通过。这些不是完整车辆测试，数量不作稳定性证明。
- SSH恢复后首先检查服务器：无相关仿真/导航/RViz残留，约147GB可用。只读提取关闭的084254原DB（SHA0a123472…），22帧点云/地形本机归档14782757B，SHA02ab9c2f2ef2d7fbf5c3535780d9dabfce6f655e4cd24e929b0c74420244681c0；没有新开车辆。
- 服务器从404成员6153包加31增量文件构建独立候选，共414个当前成员逐SHA核验。新目录 `/home/ly/recall_contract_repair_20261019/v2`，ROS域196；51项记忆、19项地面与60项真实ROS接口回调通过。
- 新5阶段实际DDS使用当前存储、实际 `_append_heading_candidates` 方法、真实 NavigationGraph 序列化和C++ Planner：基线A→按方向覆盖改选B→新query恢复A→实测成功先验改选B→B物理不可行时仍选A。库SHA86fecc86核验；没有视觉模型、FollowPath动作服务器、BeamNG或到达实验。
- 首次诊断以错误库文件名检查进程映射，提前失败，未进入选路阶段；保留原目录/包/日志。仅改诊断为实际 libgraphnav_planner_component.so 并保持原库SHA要求，v2通过，生产候选不因该诊断错误而放松。
- 本机 `verify_and_seal.py` 核验归档CRC/源SHA/414成员/日志/决策结果及22帧回放通过。服务器证据 server_evidence.tgz 106419B，SHAfeca3489f5c323a6ec5842c50dd18fcf97eaf50c6da934a2c39f8ead6224abe3。退出前项目进程清单为空，SSH正常退出。

## 未通过的进一步候选，不部署

surface_step_prototype.py 根据相邻实际返回的高度差是否超过 `max_step + max_slope*XY距离` 检查已观测突变。合成平面/凸面/坡顶均0标记，17cm阶跃60标记；但实际受困记录早期接触邻域仍为0，故不能修复本案例，不写入生产或宣称通过。更大高度残差不能自动当垂直台阶；表面形状、视野遮挡、车体底面净空与清除证据仍须联动处理。

## 下一批及验收边界

低表面障碍/地形/3D净空与射线清除、已知局部危险被整体低置信度冲淡、完整路径语义约束、整体无进展退出、真实采集身份与时钟epoch、Native历史延迟及真实车辆动态仍未关闭。先解决这些共同链路再整车搜索；完整视觉搜索/记忆增益/20轮至少16成功均未验收。不commit。
