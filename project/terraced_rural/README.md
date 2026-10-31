# Terraced Countryside

独立原生自由驾驶地图：`terraced_countryside`，菜单名称 **Terraced Countryside**。不是 Italy 叠加地图。所有源码、生成资产和测试位于本目录；旧 `standalone_rural/` 未修改。

## 地图内容

- 256×128 米连续可碰撞地表；主路沿 +X，窄路段 180 米、宽 3.4 米。左侧 +Y 为高约 1.37–1.73 米田埂及不规则阶地，右侧 -Y 为低田和最深约 1.05 米旱沟。
- 西、东端各约 26×24 米回车场，距实体地图边界仍有余量；中段 28 米长会车区，上侧拓宽至 Y=6 米，可回退和多次倒车转向，不是封闭环路。
- 11 栋不等间距的二至三层农宅，屋顶、山墙、两层以上窗框、门、台阶、烟囱、排水管、院栏及路侧电杆。无室内和可破坏建筑。
- 自制旧水泥裂纹、骨料斑点、砖瓦和抹灰纹理；毫米级实体横缝、肩部碎石和高差共同构成路缘。不是位移贴图模拟整条路面破损，横缝为微小凸起而非深凹槽。
- 大量双面实体尖叶草丛，12 个合并草补丁、约 4.9 万三角形；无需 alphaTest 或透明卡片，不使用绿色球体。草不碰撞；其余障碍物使用可见网格静态碰撞。
- 默认种子 20260911，`passable` 模式有 12 个随机路面侵入障碍和 44 个路边障碍，混合麻袋、桶、岩石与木箱。它们是刚性静态几何，不是可推动或可变形物体。

这是有明确场景布局的程序化乡村测试资产，不宣称照片级真实感、完整 WildOS 集成或算法通关。地面为 **TSStatic，不是可用地形笔刷编辑的 TerrainBlock**。没有 AI 路网、自动交通、田沟水动力或草随风动画。

## 进入与运行

已安装位置：`D:\BEAMNG\rural-user\current\mods\terraced_countryside.zip`。本次没有启动、连接、替换或关闭游戏。

自行保存编辑器内容并在合适时候重启使用上述用户目录的实例，在自由驾驶菜单搜索 **Terraced Countryside**，选择 **West Turning Apron** 和 Gavril D-Series。地图不修改游戏全局默认车型；脚本固定原生 `pickup` / `d15_4wd_A.pc`。初次人工验收从 **0.5–1 m/s** 开始，先检查落车、接地、碰撞和刹停。

从工作区根目录执行以下命令。运行命令会替换当前世界，必须自己先保存并关闭现有实例；脚本拒绝重复启动。默认不启用传感器，车辆保持驻车，不自动驾驶。

```powershell
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\terraced_rural\run.py" --home "D:\BEAMNG\BeamNG.tech.v0.39.4.0" --user "D:\BEAMNG\rural-user\current" --launch --replace-current --no-sensors
```

主动连接已配置 TCOM 的本机实例时将 `--launch` 换为 `--connect`，仍需 `--replace-current`。RGB+IMU 使用 `--sensors`，点云额外使用 `--lidar`；侧相机额外使用 `--side-cameras`。不导出语义标签给算法。输出为本目录 `captures/` 中 PNG、IMU JSON 和可选点云 NPY。静止 runtime report 含状态供验收，不能当算法输入或驾驶成功证据。

Linux 迁移先安装 `requirements.txt`，设置自己的有效安装和用户路径：

```bash
python terraced_rural/run.py --home /opt/beamng --user /srv/beamng-user --launch --replace-current --headless --no-sensors
python terraced_rural/run.py --home /opt/beamng --user /srv/beamng-user --launch --replace-current --headless --sensors --lidar
```

`--headless` 使用 BeamNGpy 原生启动参数，不等于无 GPU。Linux 二进制与授权必须由自己的安装提供；GPU1 的 Vulkan/显示设备选择需在服务器驱动和运行环境另行配置，本脚本不保证 `CUDA_VISIBLE_DEVICES` 能选择渲染 GPU，也不会访问服务器。只连接 localhost。

## 固定随机场景

随机性只发生于离线构建，不在每次加载时改变。`generated/evaluator/ground_truth.json` 仅用于独立评估，不在游戏地图 ZIP 内，运行器也不读取它。里面记录种子、模式、类型、尺寸、实际网格 AABB、净宽、保留区和阻断恢复状态。

`passable`：侵入集中在左侧，连续保留 Y=[-1.7,1.05] 的 2.75 米直线净宽。采用 2.2 米车辆宽度包络、每侧 0.25 米余量，要求 2.7 米；测试同时检查声明尺寸与实际几何。包络不是运行中变形车辆的测量，不覆盖所有改装车型和后视镜配置。几何可容纳不等于控制器能通过。

`blocked`：X=35 米处四个木箱横向封路，间隙约 2 厘米。没有现场横向绕行保证；应停车，沿原路倒回中段会车区或西回车场再转向。不会宣称窄沟可安全绕过。

**改变种子或模式前必须卸载地图/退出实例**，然后重新构建、验证、更新；不要热替换已挂载 ZIP。两种模式使用同一个地图 ID，同一时间只安装一个。示例：

```powershell
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\terraced_rural\build.py" --seed 20260912 --mode blocked
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\terraced_rural\verify.py" --home "D:\BEAMNG\BeamNG.tech.v0.39.4.0"
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\terraced_rural\install.py" --user "D:\BEAMNG\rural-user\current" --replace
& ".\beamng_scene\.beamng-venv\Scripts\python.exe" ".\terraced_rural\package.py"
```

恢复默认使用 `--seed 20260911 --mode passable`。安装器强制验证报告与当前 ZIP 哈希一致，拒绝未知所有权、修改后的资产、重复 packed/unpacked 或其他同 ID 地图覆盖。首次安装可选 `--unpacked`，已有另一种安装时需要自行保留/迁移后再安装。更新只允许本安装器拥有且未修改的内容，并先备份；不修改 modmanager 数据库。

## 验证和交付

`generated/verification.json`、`generated/tests.txt`：离线地表面积/接缝/法线、左右高差、道路与回车场、草双面几何、不同种子障碍物、净宽和无重叠、全部 DAE/XML/PNG、真实本地 BeamNGpy 1.36 完整 prefab 序列化及安全安装测试。原生注册和天空字段对照本机 smallgrid 结构，公共天空和原生 pickup 引用已查到安装文件并记录哈希，不分发这些资源。

`generated/reproducibility.json` 记录完整 ZIP 重复构建、不同种子变化与 blocked 模式验证。菜单图是布局示意，明确不是游戏渲染。**未进行游戏内材质、碰撞、落车、菜单显示、帧率或车辆实测通行验收。**

迁移包 `dist/terraced_countryside_portable.zip` 含原生模组、运行/生成/测试脚本、评估元数据和报告，不含游戏资产、Python 环境或任何密钥。菜单使用不需要 Python；生成需要 Pillow，脚本运行需要 BeamNGpy 1.36。

## 来源与授权

全部模型及 PNG 为本工作区原创程序化资产，按 CC0 1.0 提供；本次未使用 `source_assets/` 下载素材，故不存在外部混凝土贴图依赖。`helper_provenance.json` 记录按用户许可复制的 `standalone_rural/` 原创 helper 的初始哈希，复制后的代码仅在本目录修改。`base.py` 保留原始构建 helper，新布局由 `build.py` 替换，旧 helper 中的旧尺寸/预览描述不是本图最终元数据。

CC0 法律文本链接：https://creativecommons.org/publicdomain/zero/1.0/legalcode 。单独许可的 BeamNG 游戏与其公共天空、车辆不属于原创资产授权；不复制、读取或分发游戏许可证文件。
