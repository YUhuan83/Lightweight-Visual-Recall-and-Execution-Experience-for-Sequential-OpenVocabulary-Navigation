# 就绪检查表

## 已完成

- [x] 原仓库只读检查 HEAD/status/diff，记录两个未提交文件，不宣称 clean。
- [x] 独立工作区源码导出，排除 `.git`、虚拟环境、模型权重和缓存。
- [x] 原工作区修改保留，214 个选择源文件记录 SHA-256，10 个补丁/新增文件可复现。
- [x] 新配置与原默认配置分离，raw 图像、非倒置相机、显式本地模型路径。
- [x] frame strip 修复、triangulation config 参数、mapper C++17。
- [x] stdlib 测试，不导入 ROS/CUDA，不加载权重；二次独立导出哈希比较。
- [x] 依赖审计、中文文档、原源码与导出哈希复核。

## 仍未就绪

- [ ] Ubuntu 22.04 / ROS 2 Humble 上真实 colcon 构建；Python 3.10 与项目 Python 依赖版本验证。
- [ ] 检查 `find_packages` 对缺少 `__init__.py` 的子目录是否漏装，验证安装后的 console scripts 导入。
- [ ] Graaf v1.1.1 真实头文件来源审核及离线构建准备；grid_map、Eigen、OpenCV C++、nanoflann、ROS 消息/TF 依赖齐全。
- [ ] CUDA 驱动、torch/torchvision、FP16、RADIO/head/SigLIP2 版本匹配及真实推理测试。
- [ ] 实际 RADIO 文件名和只读权重挂载配置，离线模式实际断网验证。
- [ ] BeamNG 相机 K/D/R/P、分辨率、畸变、光学轴和外参实测；当前没有 BeamNG 标定交付。
- [ ] 三相机/CameraInfo/LiDAR/IMU 的时间戳、编码、QoS、帧率和同步实测。
- [ ] 真实 TF 发布者及 odom/base/camera/lidar 链路验证，无假静态外参。
- [ ] 真实 LIO / pose graph / elevation+unknown+traversability 地图生产者接入，明确地图值方向。
- [ ] mapper/planner 图输出及连通性验收；不得以 dummy pose graph 通过。
- [ ] 独立 ExploRFM triangulator 的机器人枚举/数量限制和输入配置现场适配。
- [ ] 对象定位多视角/点云投影误差验证，候选目标仍与车辆控制隔离。
- [ ] 车辆控制、急停、速度限制和闭环安全测试。

结论：源码准备可用；Humble 构建、BeamNG 标定、真实地图链路与导航运行均未验收。
