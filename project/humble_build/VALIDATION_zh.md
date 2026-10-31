# 本机验证记录

本次在 Windows、Python 3.13.9 上执行，未访问虚拟机或服务器，未执行 ROS 构建。

- `python -B validate.py`：8 项测试，7 项通过，1 项跳过。符号链接测试因为 Windows 创建权限不足跳过，拒绝符号链接的 Linux 路径仍需目标机验证。
- 实际源码复制测试核对所有原文件 SHA-256 不变，副本只新增七个 `__init__.py`；setuptools 实测发现补丁后的七个子包，原始 `wildos` 子包确实未被发现。
- 覆盖九包依赖图、缓存/权重排除、哈希变化检测、拒绝已存在目标、拒绝 home 之外目标、缺 ROS 明确阻塞和必需的 no-server 标志。
- 使用 `C:\Program Files\Git\bin\bash.exe -n` 分别验证 `run.sh` 和 `build.sh`，语法通过。系统 PATH 没有 Bash，实际使用 Git for Windows 提供的 Bash。
- 用 Git Bash 执行 `build.sh unused-workspace --no-server-precheck`，输出 `Missing ROS 2 Humble; no build performed`，在创建工作区之前停止。
- `host_preflight.json` 如实记录 Windows 缺少 ROS/colcon/rosdep、Python ABI 不符等阻塞，不声明编译成功。

可机器读取的 Python 测试结果在 `validation.json`；离线图在 `dependency_graph.json`。这些证据文件由 `validate.py` 生成，不是伪造的 Linux 日志。真实 Ubuntu 22.04/Humble 的 rosdep、CMake、接口生成、C++ 编译链接及 Python 安装仍需按照 README_zh.md 在虚拟机完成。
