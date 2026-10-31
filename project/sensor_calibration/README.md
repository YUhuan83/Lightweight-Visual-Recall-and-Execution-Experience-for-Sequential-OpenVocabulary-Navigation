# BeamNG 传感器标定夹具

日常周期预览与摘要请先看 [操作入口](README_操作入口.md)：默认使用实际 `rural-user` profile，单命令显式授权场景替换。以下保留底层诊断参数与历史示例；示例中的 `D:\BeamNGUser` 需换成自己的实际 profile。

本目录独立使用，提供夹具诊断与显式实时预览。代码更新及离线测试不启动游戏、不连接服务器，不代表运行时标定通过。是否通过以本次 `captures/<唯一ID>/runtime_report.json` 为准，不保证 `calibrated=true`。

## 夹具

- 基于已读取核验的 BeamNG.tech 0.39.4.0 原生 `smallgrid`，可见 `GroundPlane` 位于 z=0；不是 AmericanRoad，也不覆盖 smallgrid 的原有关卡文件。
- 自制 24×30 m 浅灰白中性漫反射平台，顶面 z=0.1 m，金属度 0、粗糙度 1。采用非纯白值 205，避免把曝光饱和当作白色材质正确。
- 红、蓝、绿实心盒和 5×3.5 m 棋盘。棋盘 10×7 格、格长 0.5 m、内部角点 9×6。PNG、DAE、法线、UV、材质绑定全部自制。
- `TSStatic` 明确设置 `collisionType="Visible Mesh"`，碰撞与渲染使用同一网格；平台和盒为闭合实体，棋盘有实体背板。运行时碰撞加载与车辆稳定性仍须实测。
- ZIP 只添加 `levels/smallgrid/art/shapes/nz_sensor_calibration/` 下的资源。运行器经授权后生成独立原生 scenario/prefab，不在原关卡中永久放置物体。
- 车辆为原生 D-Series `pickup` / `d15_4wd_A.pc`，AI disabled、`realistic_automatic`（1.36 的实际 API 名称）、油门 0、脚刹和手刹 1，等待 5 秒并连续检查速度、位移、位置与直立方向。

## 离线准备

在本工作区 PowerShell 中运行，以下操作不接触游戏进程：

```powershell
$py = '.\beamng_scene\.beamng-venv\Scripts\python.exe'
& $py .\sensor_calibration\inspect_local.py --home 'D:\BEAMNG\BeamNG.tech.v0.39.4.0'
& $py .\sensor_calibration\build.py --home 'D:\BEAMNG\BeamNG.tech.v0.39.4.0'
& $py .\sensor_calibration\test_fixture.py
& $py .\sensor_calibration\test_preview.py
```

产物为 `dist/nz_sensor_calibration.zip`（可安装的真实 mod）与 `dist/sensor_calibration_source.zip`（仅源码迁移包）。后者不包含游戏文件、原生 Lua/材质源码、许可证密钥、虚拟环境或采集数据；目标机须有自己的合法 BeamNG.tech 安装，解压后重新 build。`generated/fixture.json` 中的已知几何只用于本目录的校准诊断，禁止作为导航算法输入或目标定位真值捷径。

## 安装

先由用户决定关闭或保留已有游戏。安装器不会关闭进程、清理缓存或修改 `mods/db.json`。已有相同 ZIP 则无操作；已有不同内容则拒绝覆盖，需要人工审查并仅移走这个夹具自己的 ZIP。

```powershell
& $py .\sensor_calibration\install.py --user 'D:\BeamNGUser'
```

要求 `D:\BeamNGUser\current` 已由游戏创建。`--user` 可传父目录或 `current`；安装位置统一为 `父目录/current/mods/nz_sensor_calibration.zip`，启动时 `-userpath` **必须传父目录**，不传 `current`，避免 `current/current`。运行器连接后只读查询 `FS:getUserPath()`，必须与预期有效 `current` 完全相符，并检查夹具 DAE 已挂载，否则在场景替换前中止。安装后如资源未挂载，由用户保存工作并重启对应 profile，不自动重启或改数据库。

## 显式运行

这些命令会替换当前运行世界，请先保存编辑器工作。此任务中没有执行这些命令。无 `--connect`/`--launch` 或无 `--replace-current` 会直接退出，不会默认连接，更不会默认启动。

已有本机 Tech/BeamNGpy 服务监听 64256 时，优先连接：

```powershell
& $py .\sensor_calibration\run.py --home 'D:\BEAMNG\BeamNG.tech.v0.39.4.0' --user 'D:\BeamNGUser' --connect --replace-current --opencv
```

仅在明确要新启动且无 BeamNG 进程时，将 `--connect` 改为 `--launch`。启动器遇到已有 BeamNG 进程或端口占用会拒绝第二次启动。默认只连接 localhost；在服务器上安装源码包、创建独立 Python 环境并在服务器本机执行即可，不开放远端监听端口。

```powershell
python -m venv .\sensor_calibration\.venv
.\sensor_calibration\.venv\Scripts\python.exe -m pip install -r .\sensor_calibration\requirements-visual.txt
```

Linux 服务器使用相应 `python3`、`.venv/bin/python` 和游戏 Linux 路径。只准备或做 API 诊断可安装 `requirements.txt`，不需要 OpenCV。VirtualBox 仅用于源码构建、ZIP/DAE/prefab 和合成数学离线测试，不在此承诺游戏传感器运行支持。

追加 `--mounted` 会在静态世界相机之外测试车载相机；追加 `--lidar` 会要求静态 LiDAR 几何诊断也通过。静态相机位置 `(0,0,2.35)`，朝 `(0,-1,0)`，上 `(0,0,1)`，960×640、垂直 FOV 60°，是排除车辆局部坐标变换影响的对照。车载相机使用车辆局部位置与方向；记录实际 `get_position/get_direction` 和车辆 up，API 没有 `get_up`，因此不会假称完整传感器姿态已直接读回。

## 实时预览

连接用户已经运行的 64256 服务，不启动第二个游戏；仍会替换当前场景，请先保存工作：

```powershell
$py = '.\beamng_scene\.beamng-venv\Scripts\python.exe'
& $py .\sensor_calibration\run.py --home 'D:\BEAMNG\BeamNG.tech.v0.39.4.0' --user 'D:\BeamNGUser' --connect --port 64256 --replace-current --opencv --mounted --lidar --preview
```

- 不传 `--preview` 仍是一次性诊断并清理。预览只显示所选传感器：静态彩色相机必选，`--mounted` 加车载彩色窗口，`--lidar` 加静态 LiDAR XY 鸟瞰窗口，不会暗中添加选项。
- 先创建全部传感器，再执行诊断，然后保持传感器存活进入预览。数值拟合/测量诊断失败可继续预览，并明确显示 `NOT CALIBRATED`；创建失败、通信故障等仍中止，不放宽判定阈值。诊断过程可能较慢，控制台分阶段即时输出。
- 传感器设置 `is_visualised=False`，不显示游戏内传感器标记；不改地图 mod，不要求重新安装已有相同夹具 ZIP。
- 需要桌面版 OpenCV GUI 和可用交互桌面。连接前检查 GUI；Linux 无 DISPLAY/Wayland 或 headless OpenCV 会明确拒绝，不自动安装软件或修改环境。当前工作区虚拟环境检查到 OpenCV 5.0.0 / WIN32UI，游戏实时显示仍须用户执行验证。
- `q` / `Esc` / 关闭任意预览窗口退出全部窗口，支持 SIGINT（Ctrl+C）和 SIGTERM。`--preview-seconds 30` 限时预览；默认 `0` 等待退出操作，计时从诊断完成、窗口开启后开始。
- `requested_update_time=-1` 保持 ad-hoc 采样；唯一采集线程顺序请求传感器，主线程只显示最新实际样本，不调用 BeamNG socket。低采集帧率不会阻塞窗口事件处理。等首帧时只显示等待文字，不生成假图像或假点云。显示的 FPS 是同一传感器相邻成功采集的主机时间间隔倒数，age 是主机样本年龄，不是配置帧率或同步模拟时间。
- LiDAR 鸟瞰以本次查询的传感器位置为中心，世界 +X 向右、+Y 向上；仅显示有限且距离 0.05–60.1 m 的真实点。诊断未唯一判定坐标系时，并列展示 `raw_world` 与 `translated_local_world_axes` 两种假设，不能当成已验证外参。显示保留/原始点数，滤除数据不改变原始点云。
- `S` 保存各传感器最新原始 PNG/NPY 到当前采集目录的 `preview/<唯一ID>/`，附请求 pose、主机时间、未同步声明、坐标假设和过滤计数；样本不是同时采集。磁盘保存可能短暂占用 UI，不调用游戏 API。
- 退出立即关闭窗口并取消后续请求，再等采集线程结束。单次 socket 超时为 `--timeout`（默认 60 秒），ad-hoc 在各调用之间检查截止时间和取消状态；并非整个请求严格 60 秒内完成。等待线程最多 `timeout+1` 秒，随后才安全执行 AI off、油门 0、脚刹/手刹和传感器移除。若线程仍阻塞，跳过所有 socket 清理并在报告标记失败，避免并发访问同一连接；此异常情况下传感器可能留在游戏中。不会结束游戏进程。

## 车载 LiDAR 与 IMU 周期预览

以下命令仍会替换当前场景，须由用户自行执行；不切换已有实例的渲染器：

```powershell
& $py .\sensor_calibration\run.py --home 'D:\BEAMNG\BeamNG.tech.v0.39.4.0' --user 'D:\BeamNGUser' --connect --replace-current --preview --preview-only --mounted --lidar --mounted-lidar --imu
```

- `--imu` 必须同时指定 `--preview --preview-only`；`--mounted-lidar` 还必须显式指定 `--lidar`，不会隐式添加。无需车载相机时可省略 `--mounted`。不传新选项时静态 LiDAR 诊断行为不变。
- 周期模式跳过几何标定，始终 `NOT CALIBRATED`，退出码为 2。相机/LiDAR 请求更新间隔 0.2 秒；AdvancedIMU 附着同一车辆，位置 `(0,0,1)`、局部方向 `(0,-1,0)`、重力开启、物理采样 0.01 秒、批量回传 0.2 秒，无游戏内可视化。唯一工作线程顺序轮询，无并发 socket。
- IMU 独立文字窗口显示最新有效记录的实际 `accRaw`（m/s²）、`angVel`（rad/s）、传感器原始 `time` 及本次批量有效/收到样本数，不将样本数或配置周期伪装成实测频率。缺失时间显示 unavailable；空帧沿用等待/保留旧帧及超时机制，不补零。
- `S` 将 IMU 保存为 `imu.json` 测量列表，仅保留有限的原始测量字段（含原有 time），排除世界位置、轴方向及畸形/非有限字段，不推导位姿或时间戳。`imu_metadata.json` 单列主机采集时间与过滤计数；各传感器不同步。
- 车载 LiDAR 使用车辆局部方向、`is_static=False`，预览中心来自实际查询的世界位置而非安装偏移。未验证点云坐标系时仍显示两个明确标记的假设面板；移动中的前后位置查询与缓存点云不保证同一时刻，不能视为运动补偿或已验证外参。
- 仅希望尝试键盘驾驶时，单独追加 `--manual-drive`（要求 `--mounted`）。它只沿用现有释放刹车行为，不自动改挡位或修复此前无法移动的问题；需聚焦游戏窗口并按自己的键位操作。默认不加此选项。

离线回归：`& $py -m unittest discover -s .\sensor_calibration -p 'test_*.py'`，不连接游戏。

## 判定与输出

- 每次显式运行只在 `captures/<唯一ID>/` 写真实采集及报告，保留原始 PNG / NPY。退出仅移除自己的传感器并断开客户端，不关闭游戏、不恢复或改写用户旧场景；新夹具场景和刹车车辆会留在游戏中。
- 相机通过 ad-hoc 请求采样，记录前后实际位置和方向。使用已知棋盘角点调用实际 `world_point_to_pixel`，按相机 right/down/forward 基底拟合 fx/fy/cx/cy；36 点拟合、18 点 holdout，不用 holdout 调参。API 是整数像素，留出最大误差限 2 px，焦距与名义值偏差限 3%，主点限 3 px。
- `--opencv` 检测真正保存的 RGB 图像棋盘，再对照拟合投影，最大误差限 3 px；允许棋盘排序方向翻转。未请求、未安装、遮挡、白屏、未检测到或误差过大均为视觉校准未通过，API 通过绝不能代替图像证据。它不是多视图镜头畸变标定，也不是生产级内外参证书。
- 白平面诊断报告包含 RGB 标准差、近白比例、饱和比例。静态通过但车载失败时重点查看实际 pose、车辆姿态和局部轴；两者图像都白或棋盘缺失时先排查资源挂载、材质、曝光和可见性，而不是更改算法目标坐标。
- LiDAR 保存原始点云，拒绝空、非有限或非 Nx3 数据；保存实际前后 pose、请求 ID、读取 API 来源及返回键。比较原始世界坐标和仅平移局部坐标两个显式假设，用平台高度、棋盘平面命中和 0.05–60.1 m 范围判别；只在唯一假设满足范围比例 >99%、平台 >30 点、棋盘 >10 点时通过。不能由此声称任意车载 LiDAR 外参或 camera/LiDAR 同步已验证。
- API 转换结果不提供同步模拟时间，因此 `simulation_timestamp=null`；host monotonic 只标记请求区间，不伪装成传感器时间。
- `runtime_report.json` 默认 `calibrated=false`。只有车辆稳定、静态相机 API 与真实视觉，以及用户选中的车载相机/LiDAR 全部通过才可为 true，作用域仍仅是本次夹具诊断。失败保留 false 并返回非零退出码。
- 离线测试明确使用 **synthetic** 投影，测试通过不代表游戏加载、图像、碰撞或传感器已运行。
