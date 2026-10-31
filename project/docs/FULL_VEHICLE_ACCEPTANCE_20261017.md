# 合并修复后的整车验收：未通过（2026-10-17）

## 条件与运行版本

用户授权部署整车验收。Town 加油站原出生位置，左转 90°，一次地图加载；目标 `green residential house`。真实 SBR 车体、车身碰撞 467 节点、四轮各 32 碰撞节点，非理想载体。沿用原速度与转向参数。

本轮沿用共同执行链基线，三个新增记忆先验关闭；不是视觉记忆增量实验，也不是未经适配的上游 WildOS。目标未通过前不进入多目标或 20 轮稳定性统计。

- 冻结 400 成员：`932ac9bf8cbe6444076f2ad702c560e0ab2eed781fdc749d7687998e75b38e44`。
- 本机 `deploy/20261017_full_vehicle_candidate`；服务器 `/home/ly/full_vehicle_acceptance_20261017`。
- 相对 f9d 仅 10 个文件变化：完整路线选择 C++/头/测试/fixture、Bridge 执行进展与重试准入、两个回调测试和测试套件入口。
- driver `field_yaw_left90_search_20261017_172049`，session `session_20261017_172053`。
- 实际进程库：GraphNav `db52f00e…`，RPP `bd96eb9d…`，新 Smac `3cd81492caf0df265fd9e635e528d1e876efa1e540e5fa94b9e05f09d2dfefd5`。新目录重新构建导致二进制 SHA 与隔离回放构建不同，源码逐成员身份一致。
- 部署前 /proc 无相关残留，GPU0 空闲、磁盘约 43GiB；没有使用 GPU1。新安装 C++ 21 项及真实 ROS 回调 26 项通过，实际安装导入路径/源 SHA、候选 400 成员、原日常 e6、基线开关和 harness 校验通过。

## 结果

**423.841 秒超时，未找到并到达绿色住宅。**

- 目标记录中的 Native 累计轨迹约 175.699m，最大离站 33.864m；这些为记录采样口径，不作为目标进展或成功依据。
- 650 条 GraphNav 路线决策均完整，未见超过 5 秒的发布间隔；670 次实际视觉评分，说明决策和评分在运行，不能据此认为探索有效。
- 42 次 following、28 次 maneuver_done、3 次换向后重规划、1 次 FollowPath 失败、1 次 no_vehicle_path。尾部日志包含退出期间事件，逐事件采集时间保留。
- 2 次目标掩码，配对源戳差均 0；visible hint、三维定位、目标确认均 0。没有形成被验收的目标接近/到达。两次掩码不足以单独诊断模型正确或错误。
- 记录到 frontier_resolved 28、vehicle_cost_selection 5、vehicle_route_rejected 1 条退役消息；尚未逐条证明真实观察覆盖或退役是否过早。无 observed_route_endpoint ACK，不能把退役当作实际到达观察点。
- 全部旁路 Native 接收采样在目标窗口内积分，8474 个样本、覆盖 423.752 秒：速度 <0.05m/s 共 **194.997 秒**，<0.2 共 241.648 秒；长停 ≥5 秒 10 段，最长约 23.799 秒。不能用“无持续卡死”或累计里程宣称稳定性已解决。
- 按最后一条公开状态近似归属，静止发生在 following 约 79.17 秒、waiting_for_costmap_update 约 43.43 秒、exploration_retired 约 17.71 秒等。状态在计算/交接期间可能保持，不等于精确内部等待时间。

本轮与旧轮轨迹和地图演变不同，不能从 214 秒→195 秒静止或 109m→176m 行程直接推导修复效果、成功率或统计稳定性。

## 已定位的失败证据

1. query 相对约 356.49 秒，RPP 明确报 `Ackermann RPP detected collision on its bounded command arc`，FollowPath status 6。随后约 359.95 秒，原目标一次规划返回 empty_path，之后系统继续探索。该证据表示控制器碰撞检查拒绝，不证明真实车身发生碰撞，也未证明当前代价地图标注正确。
2. 约 296.15–298.93 秒出现 4 次 execution_unavailable，原因 source_protocol / source_expired 各 2 次。全部旁路核验中，8468 条 applied 回执匹配有效源载荷，错误匹配/无有效回执却正油门均 0；watchdog 消息 58，执行器故障和遥测不健康 0。源命令年龄最大约 .846s、回调源龄最大约 .452s。具体调度/延迟来源待继续关联；不能据此把所有静止归为网络或车辆物理。
3. 规划比较、普通 frontier 退役和短段交接仍频繁。必须继续检查接受运动段期间的连续性、frontier 状态与观察契约、公开图及控制器私有图障碍来源。没有因为这次验收修改安全检查或调低到达标准。

## 停车与证据

goal_001/stop.json 核验真实车体持续停稳，末样本速度 .001026m/s；session errors=[]。结束后独立 /proc 无仿真、导航、RViz、录包相关残留。日常 e6 入口保留，候选独立安装保留；验收失败，不 commit。

本机录像：`C:\Users\Yu\Desktop\拟真\captures\20261017_full_vehicle_acceptance\rviz_live.mp4`。

- 427.75 秒、3422 帧、8fps、1920×1080，完整解码通过。
- 133474920 字节，SHA256 `88ce3ba9b941413d82ed94fc30efa0dc154484ce0cb6fdba08ebf8524cbb7cca`。
- 左侧二维地图、右侧实时 camera。没有独立 front_camera.mp4，右栏保留前摄画面。最后 RGB 图为失败时图像，不是到达证据。

本机证据目录 `test_artifacts/full_vehicle_acceptance_20261017/`，`closed_evidence/` 为封闭解压证据。

- 核心 ZIP 195421240 字节、128 成员（127 载荷），SHA256 `bcd9ed33a77f3c23e969f0f0b5f63d0303c46959e7a8fcd0d5832a3fe4998afd`；本机整体 SHA、CRC、逐载荷 SHA 和本机录像 SHA 均通过。
- 3.246GB 原始 sensor DB 保留在服务器，SHA256 `3752c0ecba23da71bd506b2d5552ac9466f14428e9acf8b1ae3cd9159bcafcb4`，外部 manifest 含路径/长度；仅在 driver、goal、session 三个关闭屏障通过后分析 SQLite。
- 本轮没有运行新的多轮/记忆目标搜索；不能作为绿色房屋成功或记忆功能成功证据。
