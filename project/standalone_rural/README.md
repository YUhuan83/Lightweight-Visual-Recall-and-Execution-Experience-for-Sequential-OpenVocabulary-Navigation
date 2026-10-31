# Nizhen Countryside

全新独立 BeamNG 自由驾驶地图，地图 ID：`nizhen_countryside`。
不是旧地图叠加物，不依赖其他关卡，不使用 Gazebo 文件。

## 现在进入地图

1. 先自行保存当前 World Editor 工作，然后退出并重新启动 BeamNG。
2. 使用用户目录 `D:\BEAMNG\rural-user\current` 对应的游戏实例。其他用户目录不会看到这里安装的模组。
3. 主菜单进入 **Freeroam / 自由驾驶**，地图列表搜索 **Nizhen Countryside**。
4. 选择 **Village Road** 出生点；车辆可选 **Gavril D-Series / D15 4WD**，然后开始。
5. 从村庄南侧水泥路向东行驶，前方是 48 米实体石块路段；环路可绕行一圈，中央支路通往标靶和回车场。

原生自由驾驶流程会在出生点生成所选车辆；地图不硬塞额外车辆，不修改全局默认车型。BeamNG.tech 的默认车型可能为 ETK，菜单中可自行改选 D-Series。脚本模式固定一辆 D-Series。

已安装的新模组位置：`D:\BEAMNG\rural-user\current\mods\nizhen_countryside_standalone.zip`。
如果菜单没有出现，先确认用户目录和模组管理器中的启用状态，退出安全模式，再完整重启；不要覆盖游戏 `content/levels` 或删除旧工作。

## 地图内容

- 256×256 米连续原创网格地表，64 个 32 米分块，完整可碰撞，北部平滑缓坡、南部村庄平地。
- 约 599 米闭环、8 米宽路面、村内支路与回车场。
- 道路是地表本身的材质分区，没有隐藏在地形下面的叠加路面。
- 48 米石块区，1536 块合并实体石块，正向起伏不超过 25 毫米，路段端部逐渐减小。
- 8 栋简化农宅、木栅栏、176 棵树、岩石、棋盘标靶和桶，合计 81 个 TSStatic、170424 个三角形。
- 原创 PNG 材质与带法线的 COLLADA；菜单图是明确标注的俯视布局示意，不是假称的游戏截图。
- 只引用安装游戏已有的公共天空、云、环境立方体贴图；车辆资产由游戏提供，不随包分发。

这是低速自由驾驶基线，不承诺照片级画质。建议先以 15–25 km/h 验收。边界有低矮实体围挡，地图外没有无限地面。没有 AI 路网、交通、任务、室内空间或可变形建筑；地表为 TSStatic，不支持 TerrainBlock 笔刷编辑。树冠为简化实体几何。

## 验证边界

`generated/verification.json` 和 `generated/tests.txt` 记录离线结果。测试包含整个地表三角形拓扑、分块接缝、坡度、道路连通、出生点关系、石块正向起伏、全部 DAE 法线和材质绑定、ZIP 完整性、安装保护及实际 BeamNGpy 1.36 完整 prefab 序列化。

本次没有启动、连接或切换游戏，没有进行游戏内碰撞、落车、材质显示、帧率或完整驾驶验收，也没有完成缩略图目视验收。原生注册结构已对照安装版本的 `smallgrid.zip`、`template.zip` 和 `core/levels.lua`；菜单实际显示仍须重启该用户实例确认。

## 可移植运行

菜单进入不需要 Python。迁移包是 `dist/nizhen_countryside_portable.zip`，包含源码、原生模组与校验结果，不包含虚拟环境、游戏资产或许可证。仅在自己的许可允许的设备上运行；不访问服务器、不迁移或复制 `tech.key`。

需要脚本时，使用独立 Python 环境安装 `requirements.txt`。当前机器可使用已有的 `beamng_scene/.beamng-venv/Scripts/python.exe`。以下命令从工作区根目录执行：

```powershell
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\standalone_rural\run.py" --home "D:\BEAMNG\BeamNG.tech.v0.39.4.0" --user "D:\BEAMNG\rural-user\current" --launch --replace-current --no-sensors
```

必须先保存并关闭已有 BeamNG 进程。脚本拒绝重复启动；只连接 `127.0.0.1`，不连接任何服务器。默认没有传感器。`--replace-current` 是明确的地图替换授权，缺少它不会连接游戏。

若已经主动启动带 TCOM 的目标实例，可把 `--launch` 换成 `--connect`，但它仍会替换当前世界，先保存编辑器内容。默认端口 64256，可用 `--port` 指定。

首次无传感器验收后，将 `--no-sensors` 换为 `--sensors` 启用前 RGB 和 IMU，可追加 `--side-cameras --lidar`。默认采集 10 秒并保留地图与车辆打开，释放驻车制动后手动驾驶。传感器数据写在本目录 `captures/`；IMU 过滤世界位姿，RGB 不导出语义标签，点云保存 NPY。脚本的 `runtime_report.json` 是静止状态记录，不等于驾驶通过报告。

## 构建与安装

```powershell
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\standalone_rural\build.py"
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\standalone_rural\verify.py" --home "D:\BEAMNG\BeamNG.tech.v0.39.4.0"
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\standalone_rural\install.py" --user "D:\BEAMNG\rural-user\current"
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\standalone_rural\package.py"
```

安装器也接受用户目录的父目录并解析已有 `current/`。默认安装 ZIP；新安装可用 `--unpacked`。两种形式不得同时存在；安装器拒绝会遮蔽本地图的其他 ZIP、解包关卡或编辑器保存的原生地图。

安装所有权凭据保存在该用户目录 `settings/nizhen-countryside-standalone-v1/install.json`。同一包重复安装只验证；更新已拥有且未被修改的旧包需要 `--replace`，会先备份到凭据旁 `backups/`。无凭据或已经修改的内容即使指定 `--replace` 也拒绝覆盖。不改 `mods/db.json`、游戏设置、旧模组或旧场景。

源码和生成结果均在本 `standalone_rural/` 内。`evidence/native_formats.json` 仅供本机审计原生小型格式示例，不包含在迁移包或地图模组中。
