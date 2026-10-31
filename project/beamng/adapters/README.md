# 时间与传感器适配器

本目录实现纯 Python 标准库离线核心，不依赖 numpy、BeamNGpy、模拟器或服务器。依据 `../interface_contract.json` 和 `../接口核查.md` 的源码核查约束实现；未改写其原有“提案/未运行验证”状态。所有新增文件均位于本目录。

## 实际范围

- `core.py`：秒转纳秒、采集时间检查、epoch/重复/回退策略、刚体变换及逆/组合、协方差旋转、相机/点云/IMU 归一化适配。
- `calibration.schema.json`：运行标定的 JSON Schema。`validate_calibration()` 不依赖第三方 schema 库，执行必要结构与数值语义检查，包括 JSON Schema 不能表达的单位四元数、帧匹配、共址和投影约束。核心只使用明确字段，额外字段不作为别名或自动映射使用；需要严格拒绝额外字段时可另用 schema 验证器。
- `calibration.draft.json`：未标定 IMU 草稿，证据未接受，未知值为 `null`，不能运行。不得将 `accepted` 改成 true 就声称测量完成。
- `replay.py`：JSONL 回放 CLI，支持传感器数据、时钟及显式 epoch 事件；坏记录立即返回退出码 2，不继续输出后续记录。前面已输出的有效记录不会撤回。
- `ros_clock.py`：可选薄 ROS 2 `/clock` 发布器。除此之外没有 Image、CameraInfo、PointCloud2、Imu 的 ROS 发布/二进制解码节点，也没有 TF 查询/发布、里程计、LIO、控制器或模拟器连接。
- `test_adapters.py` 与两个 `*.example.jsonl`：合成测试。`lidar.synthetic.calibration.json` 的 accepted 仅表示解析几何合成例的测试认可，不是可部署 BeamNG 标定。

## 离线运行

在项目目录 `C:\Users\Yu\Desktop\拟真` 执行，Python 3.8 或更新版本，无需安装依赖。`-B` 防止在包父目录创建字节码缓存。

```powershell
python -B -m unittest beamng.adapters.test_adapters -v
python -B -m beamng.adapters.replay beamng/adapters/clock.example.jsonl --epoch run-1 --clock-source offline_fixture
python -B -m beamng.adapters.replay beamng/adapters/lidar.example.jsonl --epoch run-1 --calibration beamng/adapters/lidar.synthetic.calibration.json
```

时钟例应输出 `1.25 s = sec:1, nanosec:250000000`，丢弃重复时间，显式切换 run-2 后允许 0。点云例将 `[1,0,0]` 转为约 `[10,21,30]`，法线转为约 `[0,1,0]`。这些是解析预期，不代表模拟器的实际坐标方向。

传感器 CLI 每行格式为 `{"sample": {...}, "matched_transform": {...}}`，非点云省略 matched_transform。完整点云记录见示例；相机/IMU 完整合成字段见测试文件的 `calibration(kind)` 和 `sample(kind)`。核心直接调用：

```python
from beamng.adapters.core import SensorAdapter, load_calibration

adapter = SensorAdapter(load_calibration("validated_sensor.json"), "session-unique-id")
# sample 必须包含明确的身份、帧、采集时间和测量数据。
normalized = adapter.adapt(sample, matched_transform=external_transform)
# normalized is None 表示该传感器重复采集时间，不能再次发布。
```

## 时间约束

`seconds_to_ns()` 接受非负、有限的 int/float/Decimal 秒，拒绝 bool、字符串、null、NaN、Infinity 和超 int64 纳秒范围。以最短十进制 float 表示转换，使用 half-even 纳秒舍入并正确进位，不使用错误的 `1e6` 系数。float 已丢失的精度无法恢复；需要精确子纳秒测试时直接向核心传 Decimal。JSONL 使用标准 JSON 数值解析。

每个采样必须明确携带 `timing = {seconds, source, epoch, time_kind:"acquisition"}`。不接受轮询时刻、墙钟、上游 header 或缺失时间，不补戳、不插值、不根据数组索引推断时间。调用方必须保证时间确实来自采集时刻，字段声明不是真实性证明。

`AcquisitionClock` 每条有序流检查原始秒是否回退，包括舍入后同纳秒的微小回退；回退锁存，后续即使更大也拒绝。相同纳秒丢弃，暂停不产生新时钟，恢复只发布新的采集时间。单个无效输入抛错、不发布；ROS 外壳对任何无效时间均永久停止发布。传感器先验证完整负载再提交时钟，坏负载不会消耗时间戳。

核心仅通过 `begin_epoch("新的唯一ID")` 清除回退锁存和历史；拒绝复用 epoch。CLI 对应事件 `{"event":"begin_epoch","epoch":"run-2"}`。不加人为时间偏移，新 epoch 可以回到零，因此保证的是 **epoch 内** 单调，不是跨 epoch 连续。

调用方必须是唯一 epoch 管理者，在切换时停止旧数据、清空队列/TF 缓存/估计器、同步重置所有传感器和时钟；本库没有跨进程 epoch 广播协议。每个传感器独立检查顺序，同一源可共享 epoch，不能把异步传感器到达顺序误当成全局采集顺序。延迟乱序记录会拒绝而非偷偷重排。归一化源是共同模拟时间域，不能用每车不同 offset。

## 标定和传感器

所有运行适配器必须先通过显式标定验证。`evidence.accepted` 必须严格为布尔 true，`reference` 必须非空，应指向实际标定数据、配置版本和验收报告。身份、输入/输出帧和时间源没有默认值，不读取猜测的上游 topic/TF 别名。证据标志不是签名认证；提供者对证据真实性负责。

刚体约定为 `p_target = R(q_xyzw) p_source + translation_m`，四元数要求单位长度，容差 `1e-9`，不自动修正上游 `(0,0,0,0.1)`。q 和 -q 等价；逆变换和组合均执行数值计算并检查中间帧。变换不能表达反射/左手坐标转换，实际轴系必须先验证。

**相机**

仅支持已验证的无畸变 rectilinear pinhole、packed rgb8。标定必须明确宽高、水平和垂直 FOV（度），不猜方形像素：`fx=W/(2*tan(hfov/2))`，`fy=H/(2*tan(vfov/2))`。像素中心为整数坐标，图像边界为 `[-0.5,W-0.5]` 和 `[-0.5,H-0.5]`，主点为 `((W-1)/2,(H-1)/2)`。这是一种受支持且必须验证的投影，不是对上游真实投影的断言。

必须明确光学轴 x右/y下/z前、验证外参并确认像素已对应目标光学帧。不会旋转/重采样图像来弥补错误帧，仅在这种验证前提下产生匹配的 Image 数据与 CameraInfo 字典。D 必须显式给出五个已验证的零，不能把 null 当作零；尺寸与时间戳严格匹配。不支持裁剪/行填充、鱼眼、非零畸变或语义图像，绝不把上游 8UC1 深度当米。

**点云**

输入为显式 xyz 数组，不读取 PointCloud2 二进制。所有点必须有限；遇坏点拒绝整条记录，不静默删点。每条记录必须独立提供匹配 sensor_id、frame、epoch、time_source、stamp_ns 的变换和已接受证据，真正对 xyz 做 R 与 t。可选法线必须单位长度且数量匹配，仅做 R，不加平移。可选 rgba 保持 uint32 颜色语义，不改成强度。不合成 ring、time、intensity 或 deskew 信息。

世界点到运动 lidar 的姿态不能用静态外参冒充。`provenance` 必须为 `independent_estimate` 或显式 `ground_truth`；`static_calibration` 仅适用于已验证的局部坐标输入。默认 navigation 模式拒绝 GT 变换；仅 `--purpose debug`/`purpose="debug"` 接受 GT 变换，输出 `navigation_allowed:false`，且所有点云始终 `lio_validated:false`、`deskew_supported:false`。本库不读取 GT state，不发布真值里程计，不证明独立估计没有循环依赖。实际 transform 来源隔离及独立性仍需系统验收。

**IMU**

角速度和加速度仅旋转，单位分别 rad/s 和 m/s²。要求外参平移严格为零且 `colocated_origin_validated:true`；哪怕很小的非零平移也拒绝，不发明杆臂、角加速度或向心补偿。gravity included/excluded 与 smoothing 必须明确、输入一致，核心不增减重力或取消平滑。

协方差执行 `R C Rᵀ`；已知矩阵要求有限、对称、半正定（归一化主子式数值容差 `1e-12`）。ROS 约定的首元素 -1 表示不可用，全零表示未知，均原样保留，不能提升为已知测量置信度。即使首元素 -1，其余元素也必须有限。上游九个 -1 不会被替换为猜测的噪声。姿态只接受不可用协方差，输出 `orientation:null` 和原标记，不把上游单位四元数当测得姿态；本适配器不支持有效姿态输入。

## 可选 ROS 2 时钟

前提：用户自行提供可用 ROS 2 Python 环境、`rclpy`、`std_msgs`、`rosgraph_msgs`，以及经过验证的、单一发布者的 **归一化采集模拟秒** Float64 topic。该 topic 是新输入协议，不是官方 BeamNG 端点，也不能将上游 TimeSensor 改名/转成 Float64 就视为修复。上游纳秒字段系数错误，且 header 为轮询时间；本目录没有可信原始采集路径的获取实现。

```powershell
python -B -m beamng.adapters.ros_clock --input-topic /normalized/acquisition_sim_seconds --source-id validated_common_sim_clock --epoch session-001 --confirm-normalized-acquisition
```

输入 topic 必须显式绝对名称，拒绝 `/vehicles/*` 与 `/clock` 回环，不自动发现、不接受 ROS remap 猜测接线。输入/输出 QoS 为 reliable、volatile、depth 10，需要运行环境确认匹配。无墙钟 timer，不外推、不发布初始假零。ROS sec 另检查 int32 上限；任何坏时间/回退后停止发布并记错误，节点可保持在线但不自行恢复。

Float64 不携带 epoch/source 元数据，CLI 配置是对整条输入流的人工协议承诺；无法从消息内容鉴别真实采集时间或多个发布者。重置必须停止旧生产者，排空旧队列，以新 epoch 重启节点并清空全部消费者。不要只重启时钟绕过回退保护。必须确保单一 `/clock` 权威；本节点不订阅自己的 `/clock`、不依赖 `use_sim_time` 驱动回调，消费者仅在时间链验收后启用模拟时间。

## 验证边界

离线测试覆盖有限性、量化/进位、重复/暂停/epoch、回退锁存、四元数符号与半周旋转、逆/组合、协方差已知/未知/非法值、点云数值转换与匹配、GT 隔离、法线、IMU 共址与语义、相机投影/像素中心/时间一致性、CLI 和 ROS 独立输入门控。

未安装或连接专有模拟器，未启动服务器，未执行 ROS graph 联调。真实传感器采集时间、轴系/重力、投影重投影误差、运动时点云坐标、外参证据、QoS/掉帧、跨节点 reset 协调与 GT 隔离都仍是部署前置验收项。单测通过不等于 BeamNG/ROS 实时可用或 LIO 有效。
