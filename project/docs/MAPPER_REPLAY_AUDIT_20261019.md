# 实际Mapper物理信息回放

本次仅在独立ROS域191回放关闭的084254传感器录包，没有启动BeamNG、规划器、控制器或车辆。结果位于 `test_artifacts/mapper_replay_20261019/replay_20261019_090817/`。服务器同目录另存22份graph CDR及Mapper日志，原始DB仍仅服务器保存。

## 实际输入与版本

- 源DB SHA：`0a1234726e95c0369bf11446701e2bb1ac16e3a92c4e397feea54d3419177dbc`，只读。
- 128个去重odom时间戳、22份terrain、254组TF，保持原header，通过独立/clock回放。
- 实际Mapper库SHA：`50fe10c4a5b2e796decba50068febdb9120a3fabf23a5a9e7c2bd6aa58c955c9`；运行路径、/proc/maps及消息库SHA见runtime_identity.json。
- 实际参数：slope=.35、step=.1m、roughness=.08m、minimum_clearance=.3m、use_occupancy_input=false，诊断use_sim_time=true。
- 使用确定ID及单位姿态的单节点PoseGraph；Mapper仍随机采样，不能称历史原图逐节点重建。

## 结果与限制

回放完成、errors=[]，收到61份图，保存22份terrain处理后的快照。接触邻域3米内节点每图3～5个，均feasible=true，部分confidence=.8，部分为0；中心线距接触位置1.05米内图边每图8～34条，其物理评估confidence全为0。

当前Planner源码确实读取物理字段：需confidence>=.7且新鲜，才用于物理硬拒绝；confidence=0作为不确定性代价，不能提供同样的硬屏障。没有运行实际Planner选路，不宣称本次复现了它选择某一具体历史图边。

实际图profile包含全零body_size、转弯半径和速度/加减速字段；不能把这些零解释成控制器限速零，真实短段仍有约.713m/s目标速度。图层车体约束完整性与Nav2实际车辆参数的契约尚需处理。

结合此前真实地图重放与C++台阶反例，地形链不能作为占据图漏障碍后的可靠兜底。此回放收窄了“实际Mapper输出未知”的缺口，没有解决过滤、台阶指标或恢复机制，也不是完整搜索成功。
