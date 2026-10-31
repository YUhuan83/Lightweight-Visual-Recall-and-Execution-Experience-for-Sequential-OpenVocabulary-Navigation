# 到达判断专项复核（2026-10-25）

用户纠正范围：当前检查到达判断，不推进观察点/转弯规划改造。本轮仅只读现场版本、回放已关闭录包、运行隔离回调反例；未修改运行源码、未部署、未发起新整车试验、未 commit。

## 实际运行版本的门槛

检查对象是服务器 `execution_continuity_review_20261022/v25_fast_shift` 的实际安装模块，不用本机未部署候选代替。完整文件路径和 SHA 见 `test_artifacts/target_arrival_audit_20261025/arrival_contract.json`。

1. `ObjectMaskTriangulator.check_lidar_in_mask` 调用共享 `foreground_depth`：投影到目标掩码、按像素去重，至少8个像素后再检查深度簇/表面一致性。这是空间定位质量门，既不是直接的到达距离判断，也不是视觉记忆存储专用规则。
2. `GoalStandoff.on_localization`：默认5帧配置要求30秒内至少4条在1.5米簇内的一致三维定位；不是4条普通RGB检测。该层输出 `position_confirmed`。
3. `GoalStandoff.retry_confirmation`：v25将 `search_confirmed` 与 `approach_goal_ready` 同时设为 `publish_standoff` 成功。这使“目标已经可靠定位”仍依赖“能生成接近点”。
4. `GraphPathBridge.on_object_confirmation`：要求新鲜确认、正确query/实例、有效空间质量，以及一个有效standoff；需距standoff不超过0.6米，同时满足目标表面距离门。普通目标默认4.5米，地图截断时可扩至至多6米。这里并非统一的“距目标≤5米”。
5. 验收器还按同一观测戳配对 `object_found`、确认与新鲜掩码；旧记忆、旧定位刷新和仅完成观察步均不能冒充成功。

## 两遍反例复核

`audit_arrival_contract.py` 直接调用服务器实际安装的 GoalStandoff / GraphPathBridge；隔离ROS域89、构造输入、模拟发布器及路线请求，不驱动车辆，不代表DDS或整车验收。6个检查两遍结果一致，SHA `36c04eacf7025a25565a73667758eda18ef4ee163e7617f70a6ca83af2133007`。这里检查通过表示旧行为被复现，不表示旧行为满足用户要求。

| 输入 | v25结果 |
|---|---|
| 距目标4m、3条可靠三维观测 | 没有确认输出 |
| 距目标4m、4条可靠一致观测，但不能生成standoff | `position_confirmed=true`、`search_confirmed=false` |
| 距目标5.01m、4条可靠一致观测，不能生成standoff | 同上，不应算到达 |
| 新鲜可信确认，距目标4m，距指定standoff 3m | `confirmation_before_standoff`，不完成 |
| 新鲜可信确认，距目标4.8m，已在standoff | `object_visible_not_arrived`，不完成 |
| 新鲜可信确认，距目标4m，已在standoff | `object_found` |

因此，相对用户“五米内算到达”的要求，**指定停车点依赖和旧4.5m阈值确有不一致**。本机已有 `CONFIRMED_OBJECT_ARRIVAL_RADIUS_M=5.0` 和相应回调/验收候选，先前为隔离控制变量没有并入v23→v25；项目记忆已保留这个待部署状态。本次不能把旧版已复现的问题称为新发现，也不能声称五米候选已经现场验证。

## 最新两轮为什么没触发到达

两轮实际有效三维定位分别3条/1条、最终确认0条/0条。导航参考点到当帧定位目标的平面距离分别为9.921、12.053、11.272米，以及33.402米。这些测距样本本身均不满足五米。

所以这两轮首先卡在“可靠三维观测不能组成确认”，没有执行到最后的距离判定；它们不能证明“车已进入五米范围却被拒绝”。只改最终距离不能使这两轮自动成功。本文距离只对应已有有效定位帧，不是整条轨迹对真实目标的最小距离，也不证明检测语义正确。

## 上一轮定位回放差异已闭合

只使用录包TF时，第一轮跨缺帧插值产生6条定位，与现场3条不符。录包 `/spot1/odom` 比TF更完整；两轮1093/1182个同源戳对照，odom与base_link TF的平移/四元数逐元素差为0。用这些已核对一致的odom补动态变换后，实际生产定位方法两遍准确复现3条/1条现场输出，四个正例坐标误差均为0；第一轮仍有1条配对点云未录到，明确保留缺口。

`depth_tf_coverage.json` 两遍SHA `54d3cd7fe3bbb5132c4d8ebe67d2843a0e6366f0531d696b09328aaa0feffdde`。这是修正回放保真，不是定位节点漏发或新增运行修复；不能用不完整TF插值生成的额外候选当现场漏检证据。

## 后续范围

只推进到达职责的拆分与五米候选的验证：可靠目标证据、实际目标距离、接近点可达性分别表达；已达五米不再要求精确停车点。定位可信度和新鲜实例门不能直接删除，记忆写入门与最终到达要求需分别核验。需先确认现有五米候选上下游完整兼容，再单独部署/验收；不混入观察弧、换向或其他规划调整。

本轮曾扩查观察点几何和失败反馈，用户纠正后停止。对应诊断文件保留，不作为本次到达修复的实现或验收依据。意图消息朝向不等于最终所有Smac候选的硬航向约束，不能沿用过度简化的描述。

## 原公开代码的到达处理（用户追问后核对）

已读取本机原始参照 `C:/Users/Yu/nebula2-wildos` 并在线核对官方仓库对应文件，原始文件SHA保存于 `upstream_arrival_sources.json`。

- 原 `ObjectMaskTriangulator` 用LiDAR掩码深度或多视图粒子定位，`publish_navigation_goal_and_marker` 只要已有非空三维位置就发布 `/spot1/imgnav_waypoint`。原来仍有深度/多视图定位前置条件，不能说“看见掩码就算到达”。没有当前GoalStandoff的4条3D同簇确认和search_confirmed事务。
- 官方 `graphnav_planner.launch.yml` 将该目标接到planner的goal_pose，明确 `goal_radius=3.0`。`planner_node.cpp` 的 `plan_to_goal` 将目标变换到odom坐标系，计算 `(goal_vec-odom_vec).norm()`，小于3米就 `goal_pose_.reset()` 清除目标。这里是含z的三维欧式距离，不是目前平面五米规则，也不是另一个通用GoalNavigator的2米默认值。
- 该段是规划器清除目标的完成处理，公开代码没有当前项目的独立 `object_found + confirmation + mask` 历史配对验收；不能把清目标等同论文全部实验成功的自动判据，或保证下层执行器已经停车。
- 当前定位质量、多帧确认、指定standoff精度和独立验收链均有后续新增，修改范围超出记忆检索/存储。当前这些门不以启用记忆为条件，关闭记忆不会恢复原完成规则。没有逐提交历史证据时，不将每一门的引入动机都归为记忆模块。

官方依据：
https://github.com/nasa-jpl/nebula2-wildos/blob/main/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py
https://github.com/nasa-jpl/nebula2-wildos/blob/main/graphnav_planner/src/planner_node.cpp
https://github.com/nasa-jpl/nebula2-wildos/blob/main/graphnav_planner/launch/graphnav_planner.launch.yml
