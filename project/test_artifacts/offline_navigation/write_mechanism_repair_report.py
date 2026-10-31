"""Write the offline mechanism repair report from current check results."""
from pathlib import Path
import ast
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "test_artifacts/offline_navigation"
BASE = "wildos_port/repro_check/src/nebula2-wildos/"
source_paths = [
    BASE + "graphnav_mapper/include/graphnav_mapper/mapper.hpp",
    BASE + "graphnav_mapper/include/graphnav_mapper/navigation_graph.hpp",
    BASE + "graphnav_mapper/src/mapper.cpp",
    BASE + "graphnav_mapper/src/mapper_node.cpp",
    BASE + "graphnav_mapper/src/navigation_graph.cpp",
    BASE + "graphnav_planner/include/graphnav_planner/planner.hpp",
    BASE + "graphnav_planner/src/planner.cpp",
    BASE + "graphnav_planner/src/planner_node.cpp",
    BASE + "graphnav_planner/src/path_follower_node.cpp",
    BASE + "graphnav_planner/CMakeLists.txt",
    BASE + "graphnav_planner/package.xml",
    BASE + "visual_navigation/visual_navigation/wildos/nav.py",
    BASE + "visual_navigation/visual_navigation/wildos/viewpoint_memory.py",
    BASE + "visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py",
    BASE + "visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py",
    BASE + "visual_navigation/configs/triangulation3d_objsearch_conf.yaml",
    "official_adapter/beamng_wildos_adapter/goal_standoff.py",
    "official_adapter/config/beamng_triangulation.yaml",
    "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py",
    "nav2_adapter/wildos_nav2_adapter/memory_selection.py",
    "nav2_adapter/wildos_nav2_adapter/memory_candidates.py",
    "nav2_adapter/wildos_nav2_adapter/graph_route.py",
    "nav2_adapter/wildos_nav2_adapter/input_quality.py",
    "nav2_adapter/wildos_nav2_adapter/target_observation.py",
    "nav2_adapter/wildos_nav2_adapter/execution_feedback.py",
    "nav2_adapter/wildos_nav2_adapter/continuous_path.py",
    "nav2_adapter/wildos_nav2_adapter/drive_math.py",
    "nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py",
    "nav2_adapter/wildos_nav2_adapter/model_visualization.py",
    "nav2_adapter/config/nav2_official_control.yaml",
    "nav2_adapter/config/beamng_navigation.rviz",
    "nav2_adapter/package.xml",
    "tools/patch_beamng_sensor_lifecycle.py",
    "tools/offline_callback_check.py",
    "tools/offline_navigation_check.py",
    "tools/package_official_bundle.py",
    "tools/BEAMNG_RVIZ_BRIDGE.md",
    "test_artifacts/server_validation_20261002/full_town_memory_test.py",
    BASE + "graphnav_planner/test/test_route_mechanism.cpp",
    BASE + "graphnav_mapper/test/test_mapper.cpp",
]
test_paths = [
    p.relative_to(ROOT).as_posix()
    for directory in ("nav2_adapter/test", BASE + "visual_navigation/test")
    for p in (ROOT / directory).glob("*.py")
]
files = sorted(set(source_paths + test_paths))
for name in files:
    path = ROOT / name
    if not path.is_file():
        raise FileNotFoundError(path)
    if path.suffix == ".py":
        ast.parse(path.read_text(encoding="utf8"), filename=name)
report = json.loads((OUT / "report.json").read_text(encoding="utf8"))
if any(row["status"] == "FAIL" for row in report["checks"]):
    raise RuntimeError("Cannot publish a passing repair report while an offline check failed")
callbacks = json.loads((OUT / "callback_report.json").read_text(encoding="utf8"))
if not callbacks["passed"]:
    raise RuntimeError("Callback regressions failed")
count = sum(row["detail"]["tests"] for row in report["checks"] if row["status"] == "PASS"
            and isinstance(row.get("detail"), dict) and "tests" in row["detail"])
planner_source = (ROOT / BASE / "graphnav_planner/test/test_route_mechanism.cpp").read_text(encoding="utf8")
mapper_source = (ROOT / BASE / "graphnav_mapper/test/test_mapper.cpp").read_text(encoding="utf8")
mapper_new = mapper_source[mapper_source.index("TEST_F(MapperTest, PhysicalEvidenceAgeAndTraversalSurviveGraphPublication)"):]
cpp_count = len(re.findall(r"^TEST(?:_F)?\(", planner_source, re.M)) + len(re.findall(r"^TEST_F\(", mapper_new, re.M))
bundle = ROOT / "official_wildos_beamng_bundle.zip"
digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
manifest = {
    "date": "2026-10-03",
    "scope": "Local offline modifications and regressions; no deployment or new server driving test",
    "python_tests_passed": count,
    "production_callback_tests_passed": callbacks["tests"],
    "cpp_tests_added_but_not_run": cpp_count,
    "cpp_build_verified": False,
    "real_ros_dds_verified": False,
    "beamng_closed_loop_verified": False,
    "memory_search_speed_improvement_measured": False,
    "server_deployment_performed": False,
    "bundle_sha256": digest,
    "files": [{"path": name, "sha256": hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}
              for name in files],
}
(OUT / "mechanism_repair_20261003.json").write_text(
    json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf8")

def link(title, relative):
    return f"[{title}]({(ROOT / relative).as_posix()})"

document = f"""# Town 视觉记忆机制离线修复 — 2026-10-03

## 结果与边界

本地功能源码和部署包已更新，{count} 项 Python 单元/回调测试通过，0 项失败。其中 {callbacks['tests']} 项使用接口替身调用实际生产回调。没有运行 DDS、Nav2 action 服务、真实视觉模型或 BeamNG 物理仿真。新增 {cpp_count} 项 C++ 回归测试尚未编译、运行；当前 Windows 未配置 ROS 2 C++ 工具链。

本轮未连接服务器、部署源码或启动车辆。本报告确认离线修改和已运行回归的结果；完整驾驶任务与记忆加速效果尚未验证。

## 对照此前修复顺序

| 顺序 | 已完成的代码修改 | 离线证据与运行验证边界 |
|---:|---|---|
| 1 | 查询切换清空当前目标的定位/规划状态；保留进程内记忆；用 query、查询开始时间、原始观测时间和 action generation 拒绝迟到消息 | 加油站 → 房屋 → 加油站回调回归；旧加油站结果不能完成新加油站查询。JSON 话题没有新增共享 generation 字段；时间屏障承担感知链路隔离 |
| 2 | 图消息保留 visited / traversed、真实物理观测时间；零高度占据图标记 elevation_valid=0；瞬移与重复 odom 不建立已遍历连接；过期或不充分观测不等同永久断路 | C++ 回归已加入，尚未运行。真实局部路线仍须通过当前 costmap 和车辆 footprint 校验 |
| 3 | 已知目标先在现有图上选路，再计算 frontier 启发式；前缀不追加未验证的约 51 米虚拟尾段；Path 与 route_status 按 stamp 关联 | 前缀/完整路线、错目标元数据和过期路线回调回归通过；C++ 路由需编译验证 |
| 4 | 选中记忆候选后保存图路线、局部路径和端点，最新地图复核后直接交接；局部失败保留目标和原因，进行有限重试或替代观察接近 | 回放历史 9 位姿、约 3.01 米前缀：原地图可进入 FollowPath 发送入口；后来的地图以 footprint_collision 拒绝，并保留物体目标 |
| 5 | controller odom=/spot1/odom，规划/控制最小转弯半径对齐；联合约束速度和角速度保持可实现曲率；前向片段结束前准备下一段，接受替换 action 后才更新执行状态 | 指令曲率、持续规划、旧 action 结果隔离、延续失败保留当前动作回归通过；Nav2 的实际抢占行为和驾驶稳定性待测 |
| 6 | LiDAR 掩膜内做最近深度面/空间一致性检查，稀疏支持需质量门槛；多视角需独立基线、视差、前向射线和重投影一致性；原时间戳同步；5 帧/4 个一致观测确认；末端有限替代观察位置 | 9 项定位几何回归、历史房屋定位共识和生产确认回调通过。真实目标类别、LiDAR 精度与重新进入视野的成功率待测 |
| 7 | 修复 optional、NaN、越界索引、非法权重、重复点、地图格式、缓存容量；Nav2 已接受动作 + 控制输出 + 实际 odom 位移才生成成功监督；取消/禁用/过期输入不产生几何失败训练 | Python 回归通过；C++ 验证待运行。控制输出状态不是 BeamNG 收到指令的回执 |
| 8 | 更新同图同进程 Town 多目标测试脚本和 RViz 配置/录制话题 | 测试入口已准备；本轮没有现场执行，不能填写成功率、路程或停车次数 |

## 关键机制

- 记忆存实际物体区域、位置不确定半径、观察位姿/视场及附近图节点。候选观察点帮助规划，但旧观察位置不是强制终点。
- 已遍历拓扑是带风险代价的先验，不能绕过当前局部地图。只有新鲜、充分观测的未遍历阻断会过滤连接；未知/过期证据增加不确定性代价。
- 新鲜、同查询且空间匹配的视觉确认，可以在不同于旧观察点的位置完成找回。到达接近点后等待新观测；未确认时尝试有限个可规划接近位置，耗尽后报告 memory_observation_exhausted 并保留目标。
- 同一地图/进程内切换目标保留记忆；ViewpointMemory 最多 64 个查询、每查询 8 个观察点，文本特征缓存最多 64 个查询，目标三角定位默认用 12 个视角的滑动窗口。
- 期望速度保持 1.2 m/s，max_speed / overspeed_limit 为 0，没有新增 20 米离站要求。车辆转向、换挡、输入过期和真实地图校验仍会影响实际速度。
- 前向短段可提前规划延续；方向切换处保留到达及换挡检查。不能用取消旧动作后重新发送来宣称连续执行。
- 倒车反馈把运动方向用于 traversability 监督，车身起始朝向用于视角记忆；两者不能混用。

## 地图时间与输入修复

Mapper 原先用最新 TF 变换车辆坐标系下的历史局部地图，车辆移动后可能造成障碍位置偏移。现在按地图原采集时间查 TF，查不到对应变换则拒绝该次更新。GoalStandoff 将局部地图与采集时间附近的 odom 配对，不能用当前车辆位置解释旧地图。

OccupancyGrid 的尺寸、数据长度、分辨率、取值和轴对齐原点先验证；Generic GridMap 缺层或转换失败则拒绝。NaN traversability 不再被当作自由空间。重复、过期、未来消息不能刷新输入存活时间或替换执行路线。

## 感知与传感器生命周期

{link('目标定位代码', BASE + 'visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py')} 使用掩膜内每像素最近 LiDAR 返回，深度分组与鲁棒离散度检查剔除背景/零散遮挡。稀疏模式默认至少 8 个投影像素，不能只靠降低原先 150 点阈值宣布定位可靠。多视角必须满足基线、视差、条件数、射线深度及残差检查；采集/处理都检查原时间戳和查询。

{link('传感器清理补丁工具', 'tools/patch_beamng_sensor_lifecycle.py')} 可离线修补既有 BeamNG ROS bridge：停止计时器，逐个清理自己持有的传感器，单个失败继续清理；重连复用已有 publisher，不重复创建 Camera / LiDAR。已在保存的真实 vehicle.py 源码上验证补丁幂等、清理异常和实际重连方法。第三方 bridge 源码当前不在本地工作区，工具尚未应用服务器；进程被强杀遗留的无主传感器不由此补丁自动清除。

## RViz

保留 Frontier and Traversability 主图配置：上排 frontier / traversability 热图，下排 RGB + 目标 mask / frontier 路径，左侧 costmap 和车辆概览。图片继承模型输入采集 header，由新模型输出驱动，不能用重播旧图冒充实时推理。当前热图排布在此前服务器轮次已有截图；本轮没有启动新的 RViz 或录像。

## 已运行的验证

- {count} 项 Python 回归全部通过；其中 {callbacks['tests']} 项实际生产回调使用 ROS 接口替身。替身不验证 DDS、真实消息序列化、TF、lifecycle 或物理车辆。
- 同一进程的加油站 → 房屋 → 加油站状态回归保留同一个记忆实例，返回时交接给定的合法局部路径，并在新的观察位置完成确认。测试提供定位与路径数据，没有运行真实视觉识别或导航仿真。
- 历史房屋前 5 个定位中 4 个形成 1.5 米共识；发散、重复、过期和错查询结果不通过。该回放不是视觉分类准确率测试。
- 地图变化、路径交接、倒车拐点、旧 action 结果、禁用/取消监督、曲率不一致、消息格式和传感器清理异常回归通过。
- Python 语法、配置与 ZIP/源码一致性检查通过。报告中的三项 WARN 为原通用 demo 默认值、目标切换验收和旧理想运动学适用范围提示。
- 8 组旧理想运动学对比只用于历史问题定位，不能代表本次 Nav2 / BeamNG 全任务结果。

## 后续运行验证

同批更新 mapper、planner、vision、triangulator、GoalStandoff、GraphPathBridge 和 Nav2 配置。新 bridge 默认需要 route_status，不能只更新一端。

先在实际 ROS 2 工作区编译并运行已有及新增 C++ 测试：

~~~bash
colcon build --packages-select graphnav_mapper graphnav_planner beamng_wildos_adapter wildos_nav2_adapter visual_navigation --symlink-install
colcon test --packages-select graphnav_mapper graphnav_planner --event-handlers console_direct+
colcon test-result --verbose
~~~

为避免故障中途创建重复传感器，检查第三方 bridge 补丁兼容性，再应用到其源码目录：

~~~bash
python tools/patch_beamng_sensor_lifecycle.py --workspace /path/to/beamng-ros2-integration --check
python tools/patch_beamng_sensor_lifecycle.py --workspace /path/to/beamng-ros2-integration
~~~

部署修复时可以在任务开始前重启进程、建立新图；一次多目标任务内部不能重载地图或重启 vision。Town 专用入口为加油站观察 → 房屋 → 加油站。脚本要求初始加油站记忆与新鲜确认、房屋确认后才切换，单独记录 memory_navigation_exercised，避免把“立即又看见目标”解释为规划加速。后续需测首条可执行路径耗时、真实成功率、路程、起停及控制/传感器状态，并对比记忆开关；C++ 编译、真实 TF/QoS/action、视觉质量和完整闭环均为待验证项。

## 文件

- {link('完整离线检查', 'test_artifacts/offline_navigation/report.md')}
- {link('修复范围与源码 SHA256', 'test_artifacts/offline_navigation/mechanism_repair_20261003.json')}
- {link('Town 完整测试脚本', 'test_artifacts/server_validation_20261002/full_town_memory_test.py')}
- {link('本地部署包', 'official_wildos_beamng_bundle.zip')}

部署包 SHA256：{digest}
"""
(OUT / "mechanism_repair_20261003.md").write_text(document, encoding="utf8")
print(json.dumps({"python_tests": count, "callback_tests": callbacks["tests"],
                  "cpp_tests_pending": cpp_count, "source_files": len(files),
                  "report": str(OUT / "mechanism_repair_20261003.md")}, ensure_ascii=False))
