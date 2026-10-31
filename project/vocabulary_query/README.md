# 本地开放词汇图像查询

这是实际 ExploRFM 稠密文本查询 CLI，不是全图 CLIP 分数定位，也不是三维搜索或导航。所有新增代码、依赖覆盖目录和运行报告均在 `vocabulary_query/` 内。原源码和权重只读，不连接服务器。

## 本次验证状态

- 系统 Python 的 PyTorch 为 `2.11.0+cu128`，CUDA 可用。
- 缺少依赖已安装在 `.deps/`，仅显式传入 `--deps` 的进程使用；没有安装另一份 torch，也没有修改原项目虚拟环境。
- `check_local/result.json`：`selfcheck_passed`，仅文件、静态源码接口与依赖可发现性检查，不代表模型加载成功。
- `attempt_local/result.json`：真实入口运行到 `memory_guard`，`inference_ran: false`。ExploRFM 导入成功，但可用 RAM 为 `4129058816` 字节，低于保守加载下限 6 GiB；空闲显存 `7385120768` 字节已满足 5 GiB 下限。
- 没有执行完整模型加载或前向，没有真实热图。没有用假模型替代。试运行输入是已有砖纹理文件，不是目标检测评测图。
- 5 个单元测试全部通过，仅使用合成稠密数组验证后处理，不能证明识别效果。

## Windows 运行

在工作区根目录执行，`--output` 必须是本工具目录内的新目录。下面命令会执行真正模型加载与前向，而不是测试桩；先确保有足够空闲内存。

```powershell
python -B vocabulary_query/query.py --image assets/models/lane_materials/brick.jpg --query "brick wall" --query "砖墙" --checkpoint-root C:/Users/Yu/ckpts --source-root wildos_port/workspace/src/nebula2-wildos --deps vocabulary_query/.deps --output vocabulary_query/run_brick
```

可将图像换为任意本地图像，将查询换为目标描述。重复 `--query` 支持多个查询，UTF-8 JSON 保留原始中文。官方 tokenizer 会小写化、去 ASCII 标点并截断到 64 tokens；支持输入中文不意味着中文准确率已经验证。

```powershell
python -B vocabulary_query/query.py --selfcheck --checkpoint-root C:/Users/Yu/ckpts --source-root wildos_port/workspace/src/nebula2-wildos --deps vocabulary_query/.deps --output vocabulary_query/check_again
python -B -m unittest discover -s vocabulary_query -v
```

`--selfcheck` 不导入 torch、不分配模型；它无法验证完整依赖导入、权重内容、显存或前向兼容性。失败报告包含阶段、错误及 traceback，退出码为 1。输出目录不能复用，防止旧结果被误认为新结果。

## 真实计算路径

1. 从传入的完整补丁源码导入 `ExploRFMInference`，构造 CUDA FP16 模型，启用官方 NA-CLIP 和 `use_summary_for_spatial`。
2. `forward_on_numpy(RGB)` 返回 `(traversability, frontiers, ad_spatial_features)`。仅查询时不加载前两个预测头，使用第三项 `[B,D,H,W]` 稠密特征。
3. `forward_on_text` 使用官方 SigLIP2 tokenizer 和归一化文本编码器，输出 `[Q,D]`。
4. 从 `visual_navigation/visual_navigation/utils/object_search_utils.py` 用 AST 提取并执行官方 `localize_query` 函数本身，不执行同文件的 ROS 消息导入。空间特征按通道归一化，再 `einsum('qc,bchw->bqhw')`，按官方配置插值到原图尺寸。
5. 默认阈值和插值设置直接读取 `wildos/nav.py` 的 `object_search_config`；当前值为 `0.09`、`pixel_level_seg=False`，即 nearest。可用 `--threshold` 明确覆盖原始余弦阈值。

默认最长边 640，可用 `--max-side 32..640` 降低分辨率；模型内部按 patch 步长取整。输入缩放与 FP16 属于资源折中，尚未做精度对照。RAM 下限保护是保守策略，不是经过峰值实测的保证；模型构造时官方适配器会先加载完整 SigLIP2，再释放视觉模型。GPU 分配上限同时限制为总显存的 80% 和预留 1 GiB 后的可用量。

## 输出语义

每条查询保存以下文件，以编号命名而非用户文本，避免 Unicode 文件名和路径注入问题：

- `query_000_cosine.npy`：原图大小的原始 float32 稠密余弦分数。
- `query_000_threshold_mask.png`：严格 `score > threshold` 的原始二值掩码。
- `query_000_region_mask.png`：过滤小区域后的掩码，8 连通，默认最小面积 64 原图像素，可用 `--min-area` 修改。
- `query_000_heatmap.png`：固定 `[-1,1]` 色标的 TURBO 热图，没有逐图或逐查询 min-max 拉伸。
- `query_000_blend.png`：45% 热图叠加原图与候选区域框，不在图上使用可能缺字的默认字体绘制中文。
- `result.json`：原始查询、每区域均值/最大余弦、面积、框、阈值来源、源码 SHA256、检查点大小、执行状态。

框为 EXIF 方向校正后的原图坐标，`[x1,y1,x2,y2]` 右下边界不包含。区域可能合并多个物体，不是实例检测。原始余弦处于统一数学量纲，没有跨查询 softmax，但跨词语分布仍可能不同，**不是校准后的概率或可靠跨类别置信度**。阈值继承自原始特定目标配置，统一标记 `uncalibrated`；需要真实任务图像校准，不能将亮点解释为目标一定存在。

## 离线与依赖

在任何 ML 导入前强制 `HF_HUB_OFFLINE=1`、`TRANSFORMERS_OFFLINE=1`。推理进程还拒绝 socket 连接，阻止 torch.hub 等绕过 HF 标志下载。明确传入存在的 RADIO 文件和 `siglip2_ms/` 目录，检查源码包含 `AutoModel` / `AutoProcessor` 的 `local_files_only=True` 补丁。该检查是静态验证，不等于运行时验证完整适配器；不接受未补丁原版源码。仅运行可信源码和可信权重，官方 RADIO loader 使用 `torch.load(weights_only=False)`。

`requirements.txt` 用于已装匹配 CUDA torch/torchvision 的隔离环境。不要把原仓库全部 requirements 覆盖到当前 CUDA 环境。本机增量安装命令如下，需要下载依赖时联网，但模型推理始终离线：

```powershell
python -m pip install --no-deps --target vocabulary_query/.deps -r vocabulary_query/requirements-overlay.txt
```

overlay 清单只针对本机已存在的其余依赖，不是通用完整安装清单。Linux 不应上传或复用 Windows `.deps/`。

## 上传包与 Linux 入口

```powershell
python -B vocabulary_query/export_kit.py
```

生成 `vocabulary_query/vocabulary_query_kit.zip`，只含入口、测试、依赖清单、文档、共享 `manifest.json` 和 Linux 启动脚本；不包含巨大权重、Windows 依赖或本机绝对路径报告。

`manifest.json` 按检查点根目录列出所需全部文件名。RADIO 和 SigLIP2 必需；`frontier_head.ckpt`、`trav_head.ckpt` 为共享导航包可选文件，本查询不使用。源码必须传完整的已补丁 `nebula2-wildos` 树，manifest 的 `source_files` 只是关键接口证据，不是完整源码依赖清单。只核对权重存在和大小，不重复计算巨大权重校验和，也不宣称权重内容完整性已验证。

服务器环境和 `~/ckpts` 尚未验证，也未安装或连接。之后在 Linux 本地准备好源码、清单所列权重和独立 CUDA 环境后，可以执行：

```bash
bash vocabulary_query/run_linux.sh /opt/nebula2-wildos "$HOME/ckpts" /data/frame.png "orange flag" vocabulary_query/run_flag
```

脚本接收 Linux 路径，没有 Windows 盘符或自动服务器操作。输出路径以当前目录解析，必须落在解压后的 `vocabulary_query/` 下。`PYTHON` 环境变量可指定隔离环境解释器。
