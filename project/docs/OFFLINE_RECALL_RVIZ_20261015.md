# 2026-10-15 离线检查、组件更名与RViz双栏布局

## 范围与现场边界

本轮按用户要求离线修改，不连接服务器。Windows本机无ROS2/Nav2/BeamNG完整运行环境，离线完成代码、配置、Python回调替身、合成目标链及Qt布局验证；实际RViz/Ogre、DDS实时流、车体执行和目标到达仍须现场验收。没有录制MP4，没有新增完整搜索成功数。

上轮71f9候选的真实换向组件13已取回passed=true，4个方向段实际完成、前进/倒车均有运动，观察者125s结束共13278条消息。本轮不是新实测。随后上传清理脚本被自动审批额度错误拦截、未执行；不能声称旧栈已清理，服务器当前状态须下次连接时重新核查。

## 组件更名

详见VISUAL_RECALL_COMPONENTS.md。新名称区分ExecutionExperienceStore、GraphObservationStore、ConfirmedViewpointCache、PassiveLandmarkIndex和RecallCandidatePolicy；旧类是同对象alias，配置、数据库、topic和evidence协议兼容。更名7生产模块AST等价检查通过，15个C++源码/头文件/CMake与已验证71f9版本一致。

## RViz布局与数据来源

- 左半：TopDownOrtho二维XY视图，导航图节点/边、frontier节点/边界点、GraphNav路线、车辆规划路径、执行段、轨迹、轮廓和代价地图。
- 右半：原生RGB + Target图像面板，相机每帧独立回调发布，新的模型结果不是刷新前提。保留原采集时间戳，仅叠加与图像尺寸/时间相符的掩码；图像QoS为depth1/best effort。
- `/spot1/nav_graph`直接提供当前拓扑，转换到`/wildos/viz/topology`显示副本。新图替换旧图，空图清旧节点；保持原XY，仅将显示高度投影到车辆附近，不改导航图。
- `/spot1/object_localization`提供临时物体名（localized，5s过期）；`/spot1/object_confirmation`提供确认名称（confirmed，同显示进程/坐标系保留）。没有空间定位的掩码不放在观察者位置上。不同实例ID分开；帧不一致、旧query、围栏前/过期/未来消息、NaN拒绝；确认不能被临时定位降级。缓存最多128项，切换query保留已确认名称，切换坐标系清空。
- 名称来自目标查询及其定位/确认结果，并不是对场景所有物品运行一个额外的通用检测器。显示进程需要在测试开始前启动；当前没有“任意时刻新开RViz就恢复此前所有名称”的历史快照接口。
- 局部车体系执行路径按其源戳匹配历史里程计（最大差150ms），不再用最新车姿转换旧路径；无匹配时跳过该副本。此修正只影响显示。
- 显示节点只发布可视化话题，不发送控制、目标或确认。不自动开启录像，后续需要录像时使用此默认布局。

使用现有入口：`ros2 launch wildos_nav2_adapter beamng_rviz.launch.py`。默认读取新版beamng_navigation.rviz；若传入rviz_config覆盖文件，则使用其自定义布局。

Qt离线restoreState验证：1920×1080，地图957px、相机957px，各50%。layout_fixture.png为合成地图与历史失败相机帧的布局样张，已明确标注fixture，不能作为实时RViz/目标到达证据。Qt5/RViz端实际恢复、文本与刷新率待现场验收。

## 离线发现及处理

1. DDS配置8测试在受限Windows进程中4处Path.resolve(strict=True)报WinError5，XML读取本身可用；相同源码在正常本机权限下8项全部通过。未修改生产DDS路径校验来迎合沙箱。
2. 合成目标链两条Ackermann测试在末次返站失败。更名前冻结71f9包同样2失败，排除本次更名引入。诊断是测试供应的倒车路径只有相距20m的两端点，8m局部窗口只剩起点，触发tracking_preview_would_reverse。测试供应规划器改为按既有0.25m分辨率采样直线，保留生产几何/到达/反馈保护；补最大间距和长倒车样本检查后4链全部通过。不是新增物理搜寻成功，也不证明任意路径都能执行。
3. 合成目标链与tracking_preview、物理试验策略纳入全局离线入口，避免以后仅执行旧子集而漏掉这些失败。

## 验证入口

全局：tools/offline_navigation_check.py；回调替身：tools/offline_callback_check.py；双目标合成链：tools/test_offline_goal_chain_replay.py；名称兼容：test_recall_component_names.py；显示数据/配置：test_scene_display.py、test_rviz_configuration.py和test_visualization_bridge_ros.py。

Qt布局：`python tools/configure_rviz_layout.py --check`。Qt只用于离线工具，服务器RViz读取保存的配置不依赖PySide6。

本轮证据：test_artifacts/offline_recall_viz_20261015/。首次失败日志、71f9基线失败、修正后合成链日志、AST等价报告、Qt布局报告和预览保留。最终全局报告/冻结身份以该目录summary.json为准。必要现场未通过，不commit。
