# WildOS + Visual Recall + BeamNG — code review snapshot

这是用户授权于 **2026-10-31** 发布的公开源码快照，便于 Gemini 等工具审阅完整数据流与故障。包含未提交工作，**不是已通过整车验收的发布版本**。

## 从哪里开始读

| 目录 | 含义 |
| --- | --- |
| [`project/`](project/) | 本机主项目当前工作区源码、配置、测试、文档；包括用户未提交改动，保持原目录结构。 |
| [`candidates/v74_document_handoff/`](candidates/v74_document_handoff/) | 最新已实际测试的服务器隔离候选完整源码、配置、测试和运行入口；它没有合并到本机主源码。分析最新现场失败请优先阅读这份。 |
| [`auxiliary/`](auxiliary/) | 辅助工作区的诊断、回放、实验与修复脚本；历史实验脚本不等于当前运行模块。 |
| [`publication/`](publication/) | 文件哈希清单、公开快照脱敏记录与发布验证结果。 |

不要把 `project/` 与 `candidates/v74_document_handoff/` 中的同名文件混成一个运行版本；`test_artifacts/` 内也保留了历史候选源码。

## 给 Gemini 的推荐阅读顺序

1. [`project/docs/PROJECT_MEMORY.md`](project/docs/PROJECT_MEMORY.md)：历史证据、已验证修复、最新状态与尚未解决的问题。以末尾最新记录为准，历史假设不可当成当前结论。
2. [`project/docs/DOCUMENT_PLAN_REPAIR_20261030.md`](project/docs/DOCUMENT_PLAN_REPAIR_20261030.md)：v73/v74 修改与验证范围。
3. [`candidates/v74_document_handoff/candidate_scope.json`](candidates/v74_document_handoff/candidate_scope.json)：候选来源和完整验收失败状态。
4. [`project/docs/VISUAL_RECALL_COMPONENTS.md`](project/docs/VISUAL_RECALL_COMPONENTS.md)：视觉记忆组件术语及旧名映射。
5. 下列当前候选接口，顺着消息、坐标、时间戳、目标身份和动作所有权检查。

## 当前候选关键代码

目录前缀均为 [`candidates/v74_document_handoff/`](candidates/v74_document_handoff/)：

- `wildos_port/repro_check/src/nebula2-wildos/`：WildOS 源码及本项目修改，重点 `graphnav_mapper`、`graphnav_planner`、`visual_navigation`。
- `nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py`：图路线、目标与 Nav2 规划/执行交接。
- `nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py`：连续路径更新、控制器进度交接与目标接近。
- `nav2_adapter/wildos_nav2_adapter/graph_route.py`：路径采样、地图/轮廓检查。
- `nav2_adapter/wildos_nav2_adapter/arrival_criteria.py`：独立到达证据核对。
- `nav2_adapter/wildos_nav2_adapter/pointcloud.py`：LiDAR 地面/障碍数据处理。
- `ackermann_smac/`、`ackermann_rpp/`：车辆运动学相关全局规划及局部控制修改。
- `official_adapter/`、`tools/`、`config/`：仿真传感器/车辆控制接口、启动和配置。
- 各包的 `test/` 目录：接口和回归测试。

主项目中还包含 `target_localization/`、`ros2_adapter/`、`vision_offline/`、`evaluation/` 等辅助包。精确启动来源以候选 `run_full.sh` 与配置为准，不能认为仓库内每个工具都同时运行。

## 最新已知验证边界

- 用户当前到达标准：车辆里程计到视觉定位目标的**水平 XY 距离严格小于 3 米**；原始定位 XYZ 保留用于确认和记忆。这是用户授权的新平面标准，不称为原文的三维距离。
- 每轮目标下发后搜索 120 秒，地图/模型启动与停车收尾另计；默认不录像。
- v74：914 项相关 ROS、28 项运行契约验证通过；与 v73 相同的 GraphNav 源码/库通过 30 项相关原生测试。组件通过不代表完整流程通过。
- v73、v74 的两次 120 秒整车试验均超时未到达目标。近停率分别约 19.7%、32.9%；v74 约 93.6% 的近停时间发生在首个三维目标出现之前。
- 待查：探索阶段 `retired_controller_progress_unavailable` 的实际控制器进度/路径所有权交接来源，以及目标出现后的低效图出口。状态码本身还不是已证明的根因。
- 最新 v74 为隔离候选，没有推广为成功版本；本仓库的快照 commit 仅用于代码分享，不是整车修复验收 commit。

## 快照范围与保密处理

包含源码、配置、测试、文档和历史实验代码；不上传 Git 历史、密码/API 密钥/私钥文件、聊天导出、虚拟环境、依赖缓存、构建安装目录、录像、原始 ROS 录包、模型权重或大型地图二进制。

公开副本中检测到的字面凭据会替换为占位符，原项目文件保持不变；路径及类别见 `publication/redactions.json`，不记录凭据值。需要的本地资源应通过环境或本地配置提供。该仓库用于代码审阅，不是包含 BeamNG、模型和地图数据的即开即用镜像。

原始 WildOS 项目：<https://github.com/nasa-jpl/nebula2-wildos>。保留已包含的上游许可与归属；本快照不为第三方代码或资源重新授予许可。

Windows 克隆时若遇深层历史目录路径过长，可用 `git -c core.longpaths=true clone <仓库地址>`。最新两轮的少量结果与统计保存在 [`evidence/document_plan_20261030/`](evidence/document_plan_20261030/)。
