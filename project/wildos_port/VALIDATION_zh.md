# 本次验证记录

环境：Windows，Python 3.13.9。未启动 ROS/CUDA，没有导入模型库，没有安装依赖或加载权重。

| 命令 | 结果 |
| --- | --- |
| `python -B wildos_port/test_port.py` | 12/12 通过，包含所有导出 Python 源码的 compile 语法检查 |
| `python -B wildos_port/prepare.py --source C:\Users\Yu\nebula2-wildos --action verify` | 选择源文件与主导出哈希全部匹配；source_dirty=True |
| 对 `repro_check/src/nebula2-wildos` 执行同样 verify | 源与第二份导出哈希全部匹配 |
| `python -B wildos_port/audit.py` | 审计 9 个 ROS 源码包，输出 dependency_audit.json |
| 原仓库 `git --no-optional-locks status --short` | 仍只有原来的两个 M 文件 |

12 项测试：默认模型路径；显式本地路径及缺失失败；SigLIP2 本地参数静态检查；实际 frame AST 表达式；launch 默认及 raw 配置；导出选择/权重排除/未跟踪源码；Windows junction 安全；拒绝覆盖/路径重叠；dirty provenance 与原 wrapper 字节保留；全 Python 语法；二次导出逐文件哈希一致；补丁基线不匹配在写入前拒绝。

哈希范围是选择的 214 个原源码文件和 217 个输出文件，不包含权重。原 HEAD 为 `ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572`，原未提交修改仍在 `explorfm/explorfm_model.py`、`nvidia_radio/radio/siglip2_adaptor.py`。

未验证：Humble/colcon/C++ 构建、安装后 Python 子模块发现、YAML 的 OmegaConf 实际加载、ROS launch 实际执行、GPU 推理、联网隔离运行、BeamNG 标定、真实 LIO/地图/TF 与车辆闭环。详见 READINESS_zh.md。
