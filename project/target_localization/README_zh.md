# 开放词汇目标粗定位准备包

仅使用 Python 标准库和 NumPy，连接现有 `vocabulary_query` 的二维查询输出与真实同步点云，生成不可执行的粗三维观测。只属于 `target_localization/`，不修改模型、传感器采集、ROS 或其他目录。未启动本地仿真，也未连接服务器。

## 当前状态

**实际 BeamNG 标定和同步观测尚不具备，本包默认运行只能等待。** `manifest.draft.json` 与 `calibration.draft.json` 均为 DRAFT，未知实测值保持 `null`，不填假内外参、假位姿或假时间戳。

已检查的证据来源：

- `vocabulary_query/query.py:75-96,172-218`：余弦图与区域 PNG 是 EXIF 校正后的原图像素；推理输入虽缩小，输出不能再按缩放比例转换。区域列表按最大余弦排序，掩码不是区域 ID 图。
- `algorithm_mode/contract.py:68-70,122-161`：oracle 仅允许 ego truth；严格图像、点云、位姿采集时刻与 epoch 一致。独立估计 API 不冒充该文件特定的 LIO profile。
- `beamng/adapters/core.py:162-180,234-275`：整数像素中心、光学轴向、显式数值变换及变换来源约束。这里不直接导入这些库，以保持独立可移交。
- `tools/american_road_smoke.py:338-353,381-421`：图像记录 `elapsed_wall_s` / `timestamp_utc`；LiDAR 记录 `host_poll_unix_ns` / `host_receive_unix_ns`，坐标系未固定地标验证，无逐点时间，代码明确称其为顺序快照而非同步采集。主机时间不能补成采集时间。未把该 smoke 记录当作已标定定位输入，也未从它生成三帧观测。

## 使用

在工作区根目录或解压后包的父目录运行。服务器已有 NumPy 的 BeamNGpy venv 可直接使用其 Python，不需 torch、PIL、OpenCV、SciPy 或 ROS；本次只在本机 NumPy 2.2.6 环境验证，未进行服务器执行验证。

```powershell
python -B -m target_localization
python -B -m unittest discover -s target_localization/tests -v
python -B target_localization/package.py
```

默认 CLI 输出 JSON：`status="waiting"`、`state="SEARCHING"`、`unresolved` 原因清单、`target_estimate=null`、`approach_pose=null`，退出码 2 表示等待/输入未满足。有效已采集数据的 replay 使用 `--manifest <路径>`；退出码 0 仅表示观测处理完成，不表示找到了目标或允许行驶。CLI 不写外部目录，不提供强制跳过、时间容差、假位姿、重复静态图片生成多帧等选项。

完整字段、坐标公式、来源要求、API 接入方法见 `CONTRACT.md`。库入口 `load_query()` 原样接入现有 `result.json`、`query_000_cosine.npy` 与 `query_000_region_mask.png`；`Localizer.process()` 处理每个真实采集帧，停帧时用真实规范化采集时钟调用 `tick()` 释放过期目标。不要用系统墙钟替代。

## 定位与证据

- 重建 8 连通分量，按几何、面积、原始余弦统计匹配区域，防止区域列表排序造成掩码错配；验证完整组件、原始阈值与全分辨率尺寸。
- 明确世界/传感器局部点云、米单位、源帧/目标帧、世界到相机光学 SE3、局部传感器外参、内参 K 与无畸变证据。图像、点云、pose 的采集纳秒时间、epoch 和 clock source 必须完全一致；拒绝过期数据、未知标定、非有限或错误长度数值行。
- 正 Z 且 near/far 范围内投影，每像素仅保留最前回波，避免重复点和大量物体后方背景回波主导统计。沿深度间隙分组，只使用最近稳定且支持充足的簇；最近簇太稀疏时拒绝，而不是跳到背景。
- 必须达到有效点数、独立像素数、掩码覆盖率、深度跨度与物理尺寸门限；对外围环带验证深度分离、同深度比例和 `alpha_silhouette`。没有边界深度证据也拒绝。Silhouette 是确定性的深度分组证据分数，不是物体置信概率；原始 cosine 同样不是概率。
- 输出表面三维中位数、每轴 MAD、范围与点数，不声称是物体真实中心、完整形状或已校准协方差。
- SEARCHING/TENTATIVE/CONFIRMED/LOST 状态机需要三个不同采集时刻、不同图像摘要和采集 ID，保持同 query/epoch/mode/frame/clock，并通过固定世界三维锚点距离门限。同一像素随车运动不代表同一物体；时间倒退、类别变化、TTL、异常输入都会阻止旧证据误确认。

默认门限只是可配置的工程初值，未在真实场景标定：near/far 0.3/60 m，至少 6 点和 6 像素，覆盖率 0.1，深度间隙 0.6 m，深度跨度 1.5 m，包围盒对角线不超过 4 m，silhouette 至少 0.5，外围同深度比例不超过 0.25，三维关联半径 1 m，TTL 1 s，数据最大年龄 0.5 s。详见 `Policy`。稀疏 LiDAR、遮挡、无外围回波、贴墙物体可能导致保守漏检；巨大或深度不连续目标会被拒绝。

## 边界与移交

有深度分离的错误语义候选仍可能误报；此包无法证明模型类别正确、外部证据真实或独立估计无循环依赖。严格元数据检查不能替代现场标定与独立验收。扫描运动畸变未解决，调用方需要提供满足同步语义的真实点云，不能只贴一个 scan-end 时间戳。

`estimated_pose` 接受明确独立来源的已知估计；`oracle_pose` 必须显式声明，仅允许 ego pose 真值。任何模式都不允许 target GT、隐藏语义或伪造传感器点云。

无论是否 CONFIRMED，`target_estimate` 都是 **non-executable** 观测；`approach_pose` 永远 `null`，不计算安全停车点，不执行随机驾驶，不发布 ROS goal。只有未来独立完成可通行性、车辆尺寸、安全接近和路径可达性验证的下游模块才可能构造新的可执行导航命令，本包没有解除该限制的开关。

`tests/` 中所有图像、摘要、时间、标定和点云均明确为 SYNTHETIC，运行时临时文件也只在该目录下创建并清理，不将静态真实图片包装成多个时刻。测试不代表实际模型推理、传感器同步或 BeamNG 定位成功。

## 可复现打包

`package.py` 仅打包本目录显式允许的 12 个源码/文档/草稿/测试文件，生成 `source_manifest.json`（路径、大小、SHA256）与 `target_localization_kit.zip`。ZIP 固定条目顺序、时间戳、权限与无压缩格式，13 个条目；不包含模型权重、原始上游源码、真实采集、依赖目录或缓存。同样源码重复执行得到同样 ZIP 哈希。解压即可用上面的模块命令，不需原工程源码。
