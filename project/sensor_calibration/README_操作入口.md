# 操作入口

本页命令从 `C:\Users\Yu\Desktop\拟真` 执行。只做传感器准备和人工预览，不是机器人自主驾驶，也没有启动 WildOS 推理。

## 最短命令

先保存当前游戏场景。已有本机 BeamNGpy 服务监听 64256 时：

```powershell
.\sensor_calibration\start_preview.ps1 -ConfirmReplace -Sensors all
```

仅检查路径、进程名/PID、TCP 监听情况，不连接模拟器、不调用运行器、不替换场景：

```powershell
.\sensor_calibration\start_preview.ps1 -WhatIf -ConfirmReplace -Sensors all
```

没有 API 服务时，GUI 开着不等于可连接。用户自行保存工作并关闭原实例，然后明确新启动：

```powershell
.\sensor_calibration\start_preview.ps1 -Mode launch -ConfirmReplace -Sensors all -Dx11
```

`connect` 是默认模式，不自动转 `launch`。已有进程或端口监听时拒绝第二次启动，不杀进程、不抢前台。`-Dx11` 仅 launch 可用；不传时沿用运行器默认渲染设置。TCP 监听不代表 BeamNG 协议或 profile 正确，运行器还会核验实际 userpath 和资源挂载。

默认 `Home=D:\BEAMNG\BeamNG.tech.v0.39.4.0`，`User=D:\BEAMNG\rural-user`，Python 相对脚本定位到 `..\beamng_scene\.beamng-venv\Scripts\python.exe`，与当前 shell 目录无关。需要时用 `-Home`、`-User`、`-Python`、`-Port` 覆盖。User 推荐传父目录，实际 profile 为其 `current`。入口不安装依赖或 mod、不修改执行策略；若组织策略禁止脚本，请遵守管理员批准的运行方式，不使用强制 bypass。

缺少夹具时先按主 README 离线 build，再由用户明确执行安装（不启动游戏）：

```powershell
$py = '.\beamng_scene\.beamng-venv\Scripts\python.exe'
& $py .\sensor_calibration\install.py --user 'D:\BEAMNG\rural-user'
```

安装器拒绝覆盖不同 ZIP。运行器也会核对已安装 ZIP；资源未挂载时须由用户重启正确 profile。入口的 WhatIf 只检查必要路径存在，不证明依赖、GUI、ZIP 内容、许可证或运行时可用；不读取 `tech.key`。

## 预览含义

- `-Sensors camera` 默认只增加车载相机；现有运行器不可避免的静态对照相机仍在。`all` 再加车载 LiDAR 和同车 AdvancedIMU，不另加静态 LiDAR。
- 此入口固定使用 `--preview --preview-only`，跳过标定，不请求 OpenCV 棋盘诊断。`NOT CALIBRATED` 和正常预览退出码 2 是预期行为，但仍须查看报告里的 error/cleanup_errors；退出码 2 不是成功标定。
- “实时”仅指持续顺序轮询、显示最近有效结果。相机/LiDAR 请求周期 0.2 秒；IMU 物理周期 0.01 秒、批量回传 0.2 秒。`poll()` 可返回缓存，主机采集 FPS、age 不是模拟时间、真实传感器刷新率或同步证明。
- `S` 在预览窗口保存最新各路数据到 `captures/<id>/preview/<snapshot>/`。PNG/NPY 为相机/点云；IMU 是 `imu.json` 测量列表与 `imu_metadata.json`。空轮询不补零，窗口可保留旧帧，保存数量不是累计空轮询统计。
- 静态 LiDAR 几何诊断需另用主 README 的 `run.py --lidar` 诊断模式。本入口车载 LiDAR 的 `raw_world`、`translated_local_world_axes` 只是坐标假设；移动时 pose 查询和缓存点云未必同刻，不是已验证外参或运动补偿。

## 驾驶安全

默认 `ManualDrive=false`，AI 禁用、油门 0、脚刹和驻车制动保持，车辆不会被本入口自动驾驶。确需低速人工试车才加 `-ManualDrive`；它只释放制动，不自动挂前进挡、不保证车辆能移动。

手动聚焦游戏，按自己的键位检查驻车制动按钮/指示灯及挡位。空挡 N 不会驱动车辆；确认 D/前进挡后再小油门，先验证脚刹有效。不要把预览窗口的按键当作游戏控制，不自动抢焦点。

`q` / `Esc` / 关闭任意预览窗口退出全部预览；Ctrl+C 也请求退出。退出先等待采集线程，再尝试 AI off、油门 0、脚刹/驻车制动、移除传感器并断开客户端；不关闭游戏、不恢复旧场景。若线程阻塞，代码为避免 socket 竞争会跳过清理并写入 `cleanup_errors`，不能保证车辆已停，须立即在游戏中人工制动/暂停。

## 采集摘要

跨平台、仅 Python 标准库，只读已保存 JSON，不加载图像/点云、不调用模拟器：

```powershell
$py = '.\beamng_scene\.beamng-venv\Scripts\python.exe'
& $py .\sensor_calibration\summarize_capture.py --latest
& $py .\sensor_calibration\summarize_capture.py .\sensor_calibration\captures\<id>
& $py .\sensor_calibration\summarize_capture.py --latest --output .\sensor_calibration\summary.json
```

`<id>` 替换成实际采集目录名。默认只输出 stdout；`--output` 只允许现有目录中的 `summary.json`，已存在则拒绝，明确加 `--overwrite` 才覆盖。`--latest` 按报告修改时间（缺报告则目录修改时间）选最新目录，不按随机 UUID 排序。Linux 可用 `python3 sensor_calibration/summarize_capture.py --latest`。

摘要包含本次 calibrated/runtime_observed、检查和质量诊断、运行/清理错误、各快照来源/主机时间/过滤计数、实际 IMU 记录和有效 accel/gyro 数量。缺失为 null，不等同 0；实际空列表才是 0。没有按 S 保存则无快照，不能据此断言传感器不存在。预览元数据没有 RGB 质量统计，摘要不会伪造；标定运行报告若含 RGB 标准差、饱和比例、重投影误差等则原样展示。

## 服务器边界

RTX 4090 服务器只说明硬件型号，尚未验证本项目兼容性。仍须核验合法 BeamNG.tech 0.39.4.0、BeamNGpy 1.36、目标 OS/驱动/渲染器、OpenCV GUI 和可用交互桌面；无头环境不能直接运行当前预览。PowerShell 入口面向 Windows；Linux 按主 README 使用 `run.py` 的显式参数。不要向公网暴露 BeamNG API。

GPU1 推理部署是另一项独立验证：当前入口不选 CUDA 设备、不调度推理进程；CUDA 的 GPU 编号不等同游戏图形适配器编号，设置推理设备不保证游戏使用同一张卡。4090 本机/服务器推理、模型权重与环境、完整 WildOS 接入仍待验证。

完整 WildOS 还需要实际推理集成、相机内外参与 IMU/车载 LiDAR 坐标和时序标定、受控低速运动与安全停止验证。本次交付不声称同步、导航就绪或自主控制可用。
