# WildOS 视觉记忆与 BeamNG 导航研究项目

本项目在 WildOS 开放词汇视觉导航基础上，研究如何加入轻量的、同进程运行的视觉记忆，并在 BeamNG.tech 中验证阿克曼车辆的目标搜寻与路径执行。目标是让车辆在同一张地图、同一进程内，能利用先前观察到的物体与视点更快地定位后续目标；记忆不要求跨进程保存，也不替代探索或车辆规划。

## 系统工作流

```text
RGB 相机 + LiDAR + 里程计
        ↓
WildOS 视觉语义、可通行区域和 frontier 探索
        ↓
目标掩码与三维观测 ── 视觉记忆检索与候选评分
        ↓
GraphNav 全局拓扑引导
        ↓
Nav2 Smac 阿克曼路径规划 + RPP 连续路径跟踪
        ↓
BeamNG 车辆、传感器与物理仿真接口
```

WildOS 负责视觉判断、地形可通行性和 frontier 探索；视觉记忆为已观察地标和执行经验提供检索线索，并参与候选评分。GraphNav 选择全局拓扑方向，Smac/RPP 负责符合车辆运动学和代价地图约束的路径生成与执行。记忆不会把 frontier 或 traversability（可通行性） 替换成另一套地图，也不会单独保证目标到达。

项目保留四种轻量记忆存储：`ExecutionExperienceStore`、`GraphObservationStore`、`ConfirmedViewpointCache`、`PassiveLandmarkIndex`；`RecallCandidatePolicy` 用于比较召回候选。运行机制和数据结构以当前净室源码为准。

## 仓库目录

| 路径 | 内容 |
| --- | --- |
| [`cleanroom_ws/`](cleanroom_ws/) | 当前独立的单一 colcon 工作区，包含运行源码、配置、测试与架构说明；对应服务器实测版本。 |
| [`candidates/v74_document_handoff/`](candidates/v74_document_handoff/) | 先前隔离验收候选。它与净室工作区是不同快照，不要混用同名文件。 |
| [`project/`](project/) | 主项目源码、配置、测试和项目记录的历史工作区快照。 |
| [`auxiliary/`](auxiliary/) | 离线检查、日志分析与回放工具；其中脚本不一定属于当前运行流程。 |


## 运行与许可

净室工作区面向 ROS 2 Humble。完整现场运行还需要本地 BeamNG.tech、地图及 RADIO/SigLIP 模型权重；这些运行资产和录制视频不包含在源码快照中。运行入口与依赖说明见 [`cleanroom_ws/README.md`](cleanroom_ws/README.md)。上游 WildOS 项目为 [NASA JPL Nebula2 WildOS](https://github.com/nasa-jpl/nebula2-wildos)；仓库保留各组件原有许可和归属，不对第三方代码或资源重新授予许可。
