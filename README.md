# WildOS 视觉记忆与 BeamNG 导航研究项目

本项目在 WildOS 开放词汇视觉导航基础上，研究如何加入轻量的、同进程运行的视觉记忆，并在 BeamNG.tech 中验证阿克曼车辆的目标搜寻与路径执行。目标是在同一张地图、同一进程内，利用先前观察到的物体与视点帮助后续目标搜索；记忆不要求跨进程保存，也不替代探索或车辆规划。

## 系统工作流

```text
RGB 相机 + LiDAR + 里程计
        ↓
WildOS 视觉语义、可通行性和 frontier 探索
        ↓
目标掩码与三维观测 ── 视觉记忆检索与候选评分
        ↓
GraphNav 全局拓扑引导
        ↓
Nav2 Smac 阿克曼路径规划 + RPP 连续路径跟踪
        ↓
BeamNG 车辆、传感器与物理仿真接口
```

WildOS 负责视觉判断、地形可通行性和 frontier 探索；视觉记忆为已观察地标和执行经验提供检索线索，并参与候选评分。GraphNav 选择全局拓扑方向，Smac/RPP 负责符合车辆运动学和代价地图约束的路径生成与执行。记忆不会替换 frontier 或 traversability 地图，也不会单独保证目标到达。

项目保留四种轻量记忆存储：`ExecutionExperienceStore`、`GraphObservationStore`、`ConfirmedViewpointCache`、`PassiveLandmarkIndex`；`RecallCandidatePolicy` 用于比较召回候选。

## 当前源码

[`cleanroom_ws/`](cleanroom_ws/) 是当前独立的单一 colcon 工作区，包含运行源码、配置、测试、架构说明和所需的第三方源码依赖。工作区结构与服务器运行版本对应。项目历史源码快照、旧候选和一次性辅助脚本已从当前分支目录中清理。

净室工作区面向 ROS 2 Humble。完整现场运行还需要本地 BeamNG.tech、地图及 RADIO/SigLIP 模型权重；这些运行资产、模型和录像不包含在源码中。具体依赖说明见 [`cleanroom_ws/README.md`](cleanroom_ws/README.md)。

## 验证状态

2026-11-02 完成的三目标现场任务按“红绿灯 → 带顶棚候车亭 → 加油站”运行，目标下发后总搜索时限为 300 秒。红绿灯阶段在约 109.7 秒通过，车辆与目标的水平距离为 1.85 米；候车亭阶段未产生目标确认，任务截止前没有开始加油站阶段。因此，**完整多目标流程仍未通过**。测试范围与数据见 [`cleanroom_ws/VALIDATION_20261102.md`](cleanroom_ws/VALIDATION_20261102.md)。

组件测试不能替代整车验收。请区分离线验证、单阶段成功与完整任务通过。

上游 WildOS 项目为 [NASA JPL Nebula2 WildOS](https://github.com/nasa-jpl/nebula2-wildos)。仓库保留各组件原有许可和归属，不对第三方代码或资源重新授予许可。