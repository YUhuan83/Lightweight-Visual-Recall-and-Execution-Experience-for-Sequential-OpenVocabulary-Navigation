# Linux CUDA 视觉离线包

目标：Ubuntu 22.04 x86_64、CPython 3.10、RTX 4090、驱动 580。PyTorch 2.7.0+cu128 / torchvision 0.22.0+cu128；不含 torchaudio，不安装系统 CUDA、不使用 sudo。驱动由服务器提供，本机不探测 GPU、不连接服务器、不安装到任何已有环境。

依据 `vocabulary_query/requirements.txt`、overlay 和 `query.py`；用户的 transforms 按实际代码解释为 transformers 4.51.3。numpy 固定 1.26.4，OpenCV headless 固定 4.11.0.86，timm 1.0.15，einops 0.8.1。uvloop 是显式补充项，不是该查询代码直接需要的包。模型权重和完整补丁源码需另外上传，本目录不包含它们。

## 文件与复验

- `wheelhouse/`：真实 Linux / 通用 wheel，不含 Windows wheel。
- `requirements.lock`：全部传递依赖精确版本及实际 SHA256。
- `manifest.json`：每个 wheel 的来源 URL、官方哈希、实际哈希、字节数、标签、Requires-Python 和 Requires-Dist，以及完整目标标签集。
- `status.json`：是否完整、总字节数、数量、缺失与冲突。
- `logs/`：各包 pip 下载输出；`cache/`、`tmp/` 不需上传。

Windows 工作区执行：

```powershell
python -B vision_offline/kit.py fetch
python -B vision_offline/kit.py verify
python -B -m unittest discover -s vision_offline -v
tar -cf vision_offline/vision-wheels.tar --exclude=vision_offline/cache --exclude=vision_offline/tmp --exclude=vision_offline/vision-wheels.tar vision_offline
scp vision_offline/vision-wheels.tar USER@SERVER:~/
```

`fetch` 只调用 pip 跨平台 binary-only/no-deps 下载，然后递归按 Linux 3.10 环境计算 markers 和 extras；因此不会沿用 Windows marker 漏掉 NVIDIA/Triton。已存在 wheel 不重复下载，pip 缓存保留；最终校验官方 SHA256。若出现约束冲突则停止，不伪造完整锁。首次解析未固定的间接依赖会选择索引上支持 Python 3.10 的版本，生成的 lock 才是精确可复现版本集。校验器借用 pip 自带 packaging，不安装构建工具。

## 服务器手动执行

服务器需已具备 Python 3.10 和可用的 `venv`/`ensurepip`，且用户目录有至少 15 GiB 空间。不自动安装 OS 包。不要上传 Windows `.deps/`。

```bash
tar -xf ~/vision-wheels.tar -C ~/
bash ~/vision_offline/install_offline.sh "$HOME/vision-cu128"
# 可选：仅在服务器显式运行 GPU 检查
"$HOME/vision-cu128/bin/python" -B ~/vision_offline/verify_runtime.py --cuda
```

安装脚本拒绝 root、已有 venv、用户 home 外路径；使用全新用户 venv，`--no-index --require-hashes --only-binary` 安装，再执行 pip check 和 CPU 合成 smoke test。默认测试隐藏 GPU；仅 `--cuda` 检测服务器 GPU 并测试 CUDA 矩阵运算和 torchvision NMS。脚本不下载模型。

本机只验证 wheel 静态完整性及合成依赖测试，不代表 Linux 动态链接、GPU 或完整 ExploRFM 推理已实测。源代码和权重准备好后，用该 venv 的 Python 运行原 `vocabulary_query` 入口，勿使用 Windows overlay。
