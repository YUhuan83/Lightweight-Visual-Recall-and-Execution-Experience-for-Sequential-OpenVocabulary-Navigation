# WildOS 隔离准备

## 当前边界

这是独立源码准备，不是可用导航系统，也不是 BeamNG 标定结果。所有创建和修改都在 `wildos_port/` 内；项目根目录的 `README.md`、`run.sh` 和原仓库未修改。没有安装依赖、下载模型、加载 CUDA/ROS、复制或改写权重。

原仓库：`C:\Users\Yu\nebula2-wildos`。
原 HEAD：`ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572`。
原仓库为 **dirty**，不是 clean：`explorfm/explorfm_model.py` 和 `nvidia_radio/radio/siglip2_adaptor.py` 有未提交修改。导出读取工作区文件而非 `git archive HEAD`，保留 ONNX 示例跳过修改及 adaptor 的既有本地 `_ms` 回退。

## 交付物

| 路径 | 内容 |
| --- | --- |
| `prepare.py` | stdlib 导出、封装 overlay、源/输出哈希验证 |
| `workspace/src/nebula2-wildos/` | 214 个原始选择文件，加 3 个新增文件；不依赖原源码路径 |
| `workspace/src/nebula2-wildos.provenance.json` | HEAD、原 git status、dirty 标志、选择源文件 SHA-256/权限、输出哈希、排除清单 |
| `patches/manifest.json` | 10 个补丁/新增文件的 before/after SHA-256 |
| `patches/isolated.patch` | 相对原 dirty 工作区的可读 unified diff |
| `patches/overlay/` | 经过哈希校验的补丁后源码，用于重复导出 |
| `repro_check/src/nebula2-wildos/` | 第二次导出的独立副本，用于逐文件可重复性测试 |
| `audit.py`, `dependency_audit.json` | 无 ROS/CUDA 导入的依赖审计及本机结果 |
| `test_port.py` | stdlib 单元、语法和静态回归测试 |
| `READINESS_zh.md` | 未完成的验收门槛 |

## 可重复导出

在当前项目目录执行，目标必须是 `wildos_port/` 下不存在的新目录：

```powershell
python -B wildos_port/prepare.py --source "C:\Users\Yu\nebula2-wildos" --dest "C:\Users\Yu\Desktop\拟真\wildos_port\another_export\src\nebula2-wildos"
python -B wildos_port/prepare.py --source "C:\Users\Yu\nebula2-wildos" --action verify
python -B wildos_port/test_port.py
python -B wildos_port/audit.py
```

导出器无删除/覆盖选项。补丁基线不匹配、输出存在、越界路径、源/目标重叠都会失败。新目标失败后可能留下不完整目录，但不会自动清理或冒充成功；改用新的目标目录。维护补丁时先向新的目录 `--pristine` 导出，然后仅用 `apply_patch` 手动修改，人工审核后再封装；已有 `patches/` 不会自动覆盖。

选择范围：`explorfm`、`nvidia_radio`、9 个 ROS 包及 vendor 包所在 `thirdparty`，以及顶层 LICENSE/README/requirements 等；不包含训练器、图片演示资产、notebook、`.git`、虚拟环境、权重、缓存、build/install/log。选择范围中的普通未跟踪源码也会保留。脚本采用源码扩展名白名单及单文件 8 MiB 上限，拒绝/跳过 symlink 和 Windows reparse point，不跟随它们。文件使用字节复制，不是硬链接。

源文件哈希验证覆盖**选择的源码**和 git 状态/HEAD，并非全盘或权重哈希。权重目录完全排除，不遍历或读取权重；不能把源码校验描述为“所有权重做过哈希验证”。并发恶意替换目录不属于此普通本地导出器的安全模型，执行时不要同时修改源或目标目录。

## 最小适配

- `utils/local_models.py` 与 WildOS `nav.py`：可配置 `ckpt_root`、`adaptor_ckpt_path`、`local_files_only`。未提供新配置时保留原 home/ckpts 和 model ID 行为；仅显式离线配置要求本地 RADIO 文件存在，阻止回退到 URL。
- SigLIP2 adaptor：增加直接本地目录加载，model 和 processor 均传 `local_files_only=True`；原 `_ms` 回退和远程默认仍保留。显式离线解析按直接目录、`base/siglip2_ms`、`base_ms` 查找，解决把 `~/ckpts` 错拼成 `~/ckpts_ms` 的问题。
- `explorfm_model.py` 推理 wrapper 不做行为重写，逐字保留原未提交内容。离线是新节点配置路径的约束，不声称整个第三方模型栈全局断网；保留的 `trust_remote_code=True` 仍要求权重/本地代码来源可信，部署应使用只读权重挂载和网络隔离。
- `obj_mask_triangulation.py`：frame 比较改为 `lstrip('/')`；新增 `--config`，默认仍读取安装目录里的 `triangulation3d_objsearch_conf.yaml`，绝对配置路径也可用。
- `wildos_launch.py`：新增 `config`、`triangulation_config` 参数，默认原值不变；对象 mask topic 可配置，默认 `/spot1/object_mask` 不变。
- 独立 ExploRFM triangulator：复用路径解析，显式 `cams_inverted` 可覆盖倒置列表；其 launch 新增 `config` 参数，原默认不变。既有 robot enum/相机数量限制未广泛重构，此独立入口还没有 BeamNG 专用配置，不应靠任意 namespace 假装支持新机器人。
- mapper 明确 C++17，因为已使用 `std::optional` 和 structured bindings；planner 原 C++20 不变。
- 新配置 `isolated_wildos.yaml`、`isolated_objsearch.yaml` 不替换原 YAML；原压缩图像/倒置默认不变。新配置使用 raw 图像、`cams_inverted: false`，相机 front/left/right 顺序固定。

## 配置与运行门槛

`isolated_wildos.yaml` 的 `/rover/camera/{}/image_raw`、`/rover/wheel_odom`、`/rover/lidar/points` 基于当前项目桥接接口；`wheel_odom` 不是 LIO。`camera_{}_optical_frame`、`lidar_link`、CameraInfo topic 是拟议接口，**尚未从 BeamNG 标定或实测确认**。配置中没有编造内外参或 TF。图像/CameraInfo/点云/odom 的时钟、坐标系、QoS 必须现场核验。

`/mnt/wildos-weights` 是未来 Linux 只读挂载占位符，不是自动挂载操作。必须手动在隔离配置中填入实际 RADIO 文件名并审核版本与 head/adaptor 兼容性。保持占位符会明确报错，而不会下载模型。

只有在依赖构建、传感器和真实图验证后，才考虑下列启动形式（本次没有执行）：

```bash
ros2 launch visual_navigation wildos_launch.py config:=isolated_wildos.yaml triangulation_config:=isolated_objsearch.yaml use_sim_time:=true ns:='' do_object_search:=false
```

新 YAML 在包内，沿用原 setup.py 的 configs 安装规则。launch namespace 不重写配置中绝对 topic 或 frame；不要仅改 `ns` 就期待迁移成功。对象搜索启用时使用同一组显式配置，候选目标发布到 `/wildos_port/candidate_goal`，这里没有连接运动控制器。

另一个构建验收风险：原 `visual_navigation/setup.py` 使用 `find_packages`，而多个 Python 子目录没有 `__init__.py`；普通安装可能遗漏入口对应子模块。当前没有扩大补丁去重构打包，Humble 上必须检查安装结果，不能仅凭源码语法通过认为 console scripts 可运行。

禁止把原 `graphnav_mapper.launch.yml` 作为真实建图验收：它启动 dummy pose graph，并给 mapper 加 gdb。原文件保留供比对，没有自动启动。mapper 需要真实 pose graph 和带 `elevation`、`unknown`、traversability 层的 grid map；其障碍代价约定不能直接拿模型“高值可通行”概率替代，需实测并正确转换，不能填充假地图通过验证。

## 依赖结论

本机 Python 3.13.9/Windows 无 `ros2`、`colcon`、`g++`。审计只看 metadata、源码 XML/CMake 和 AMENT 索引，不导入库；“未找到”不等于对目标 Ubuntu 系统做过完整探测。

`graaf_vendor` 包装源码已复制，不能称它整个包缺失；它的实际 Graaf v1.1.1 头文件没有随仓库提供，原 FetchContent 会在构建时联网。`grid_map_core`、`grid_map_ros`、`grid_map_msgs`、Eigen3、OpenCV C++、nanoflann、ROS/TF/cv_bridge 等需要真实依赖，未通过 dummy/vendor 空壳替代。

当前解释器缺少 timm、open-clip-torch、transformers、fairscale、OmegaConf、ruamel.yaml 等。torch/torchaudio 为 2.11.0+cu128、torchvision 为 0.26.0+cu128，与 requirements 的 2.7.0/0.22.0 不一致；einops 也不一致。审计包括未必用于部署的 ONNX、GPS、示例导入，不能据此盲目全量安装。Humble/Python 3.10/CUDA 支持仍待真实构建与运行验证。
