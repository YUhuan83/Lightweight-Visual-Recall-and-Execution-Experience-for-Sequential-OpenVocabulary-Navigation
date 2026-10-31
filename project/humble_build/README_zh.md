# Humble 算法构建隔离流程

范围：Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10 的源码编译。与车辆无关，不检查服务器，不需要 SSH、GUI、GPU、CUDA 或模型权重。不会启动节点、原来的 gdb launch、dummy pose graph 或根目录 run.sh。Windows 验证不等于 Linux 编译通过。

## 一个终端

假设共享目录权限已恢复，`/media/sf_wildos_sim` 可读、只读挂载保持不变，且本目录文件已经同步到共享目录。只用虚拟机内一个 Bash 终端，不需要第二个终端。下面第一段只做检查；第二段是明确可选的复制与编译，仍在同一个终端执行。

```bash
HB=/media/sf_wildos_sim/humble_build
SRC=/media/sf_wildos_sim/wildos_port/workspace/src/nebula2-wildos
LOG=$(mktemp -d "$HOME/humble-check-XXXXXXXX")
export PYTHONDONTWRITEBYTECODE=1
printf '本次日志目录：%s\n' "$LOG"
bash "$HB/run.sh" --preflight --source "$SRC" --no-server-precheck > "$LOG/preflight.json" 2> "$LOG/preflight.stderr"
PF=$?
cat "$LOG/preflight.json"
cat "$LOG/preflight.stderr"
printf '预检退出码：%s；0=仅预检通过，2=环境阻塞，1=脚本/路径错误\n' "$PF"
```

预检不会向共享目录或源码写入内容，不运行 sudo、apt、pip、rosdep install/update，不访问网络。报告由终端重定向保存到家目录。`run.sh` 在存在时自动 source `/opt/ros/humble/setup.bash`，缺失时继续生成明确的缺 ROS 报告。`--no-server-precheck` 是必需参数，不存在任何服务器探测分支。

若退出码不是 0，先停止构建并查看报告。`rosdep_check` 包含实际检查输出；数据库未初始化、键无法解析和软件未安装都原样呈现，不自动修复。`Eigen3`、`OpenCV`、`nanoflann` 是实际 CMake 查找名，不是假定的 apt 包名；仅定位配置文件，不保证可链接。`ament_presence` 为空也可能是系统库，不应直接转换成 ROS 安装包名。需由用户另行审核并安装依赖后重跑第一段。

确认预检通过，并且允许从公开 GitHub 获取 Graaf 后，在同一终端执行下面的可选段。它不是预检的一部分，会写入新的家目录并编译；不会安装系统软件。每次生成新 UUID 目录，目标已存在即失败，失败现场保留，绝不覆盖或清理旧工作区。

```bash
if [ "$PF" -eq 0 ]; then
  WS="$HOME/humble-wildos-$(python3 -B -c 'import uuid; print(uuid.uuid4().hex)')"
  bash "$HB/run.sh" --prepare --source "$SRC" --destination "$WS" --no-server-precheck > "$LOG/prepare.json" 2> "$LOG/prepare.stderr"
  PREP=$?
  if [ "$PREP" -eq 0 ]; then
    cp "$LOG/"* "$WS/"
    printf '隔离工作区：%s\n' "$WS"
    (set -o pipefail; bash "$HB/build.sh" "$WS" --no-server-precheck --fetch-graaf 2>&1 | tee "$LOG/build.log")
    BUILD=$?
    cp "$LOG/build.log" "$WS/reports/terminal-build.log"
    printf '构建退出码：%s；日志目录：%s 和 %s/reports\n' "$BUILD" "$LOG" "$WS"
  else
    cat "$LOG/prepare.stderr" "$LOG/prepare.json"
    printf '复制失败，保留现场：%s\n' "$WS"
  fi
else
  printf '预检未通过，不复制、不下载、不编译。报告：%s\n' "$LOG"
fi
```

`build.sh` 按顺序执行以下真实包名的 `colcon build --executor sequential --packages-up-to`，并限制 CMake 并行度为 2。任何阶段失败立即停止，保留首个错误；不是一次性运行所有节点。

1. `pose_graph_msgs graphnav_msgs object_search_msgs gps_visualization`：消息接口，最后一个包也包含 C++ 可执行程序。
2. `graphnav_mapper graphnav_planner`：图算法流水线，包含源依赖 `graaf_vendor`。
3. `triangulation3d visual_navigation`：Python 打包及相关源依赖。

构建始终 source Humble；第一阶段后 source 本次 `install/local_setup.bash`。所有 `src/build/install/log/vendor/reports` 都在新工作区。禁用 BUILD_TESTING 是为了先暴露编译错误，不代表测试通过。Python 构建成功也不代表 console scripts 的 ROS/机器学习运行时导入通过。不要 source 其他旧 overlay 后运行；建议使用未加载旧项目环境的新终端开始本流程。

## 实际修复

已读取 `wildos_port/dependency_audit.json`、`READINESS_zh.md`、实际 CMake 和两个 setup.py。`visual_navigation/setup.py` 使用 `find_packages(exclude=['test'])`，七个源码子目录没有 `__init__.py`，其中包括 `wildos.nav` 等 console entry points 以及 utils。`namespace_patch.json` 是明确列出的新增文件 payload；`--prepare` 只对新副本补齐这七个包标记，已有标记不覆盖。没有修改 setup.py、原隔离源码、C++ API 或算法行为。`triangulation3d` 的包标记已存在，无需修改。

`original_snapshot.json` 包含复制前每个纳入复制文件的 SHA-256 及整体摘要。复制后验证新副本与原快照相等，并再次核验源目录未变化，然后应用补丁。`prepared.json` 记录新增标记、新副本完整快照、来源和排除规则。摘要针对纳入复制的文件字节与相对路径，不包含权限、时间或被排除缓存。拒绝纳入复制范围内的符号链接，排除 .git、虚拟环境、构建/安装/日志、缓存、权重等；复制用独立文件，不用硬链接或 symlink install。构建前核验副本快照，变更后要求重新审核。

## Graaf 来源

实际 `thirdparty/graaf_vendor/CMakeLists.txt` 的 FetchContent 是 `https://github.com/bobluppes/graaf.git`、`GIT_TAG v1.1.1`。已在线读取该标签的上游 [CMakeLists.txt](https://github.com/bobluppes/graaf/blob/v1.1.1/CMakeLists.txt) 和 [include/graaflib/graph.h](https://github.com/bobluppes/graaf/blob/v1.1.1/include/graaflib/graph.h)。该上游启用 C++20，支持 `SKIP_TESTS/SKIP_BENCHMARKS/SKIP_EXAMPLES`，现有 vendor 已全部置 ON。上游 project VERSION 为 0.1.0，不应拿它替代实际获取标签 v1.1.1。

`--fetch-graaf` 执行 `git clone --depth 1 --branch v1.1.1 https://github.com/bobluppes/graaf.git "$WS/vendor/graaf"`。不猜测 ROS Graaf 二进制包名。随后验证 origin、HEAD 与标签指向相等、工作树干净及实际头文件存在，把解析后的 commit 保存到 `reports/graaf-commit.txt`。标签不是独立签名/固定可信哈希的供应链保证，首次获取的实际 commit 应随报告归档。

没有网络时，先在可联网机器获取相同标签的完整 git clone 并传入新工作区的 `vendor/graaf`，保留 `.git`，之后调用 `build.sh` 时省略 `--fetch-graaf`。脚本仍验证来源和标签，不在缺失时偷偷联网。CMake 通过 `FETCHCONTENT_SOURCE_DIR_GRAAF_SRC` 使用本地目录，并设置 `FETCHCONTENT_FULLY_DISCONNECTED=ON`。预检永远不会获取 Graaf。

## 证据与限制

- `dependency_graph.json`：可离线重建的九包依赖图，区分 manifest、CMake 查找和源包边。`cmake_not_literal_manifest` 是审查线索，Eigen3/eigen 等别名不是自动修补依据；test 条件也未作 CMake 求值。
- `host_preflight.json`：当前 Windows 主机的真实报告，不可用作虚拟机环境报告。
- `validation.json`：当前主机可移植测试结果，包含跳过原因和明确的 `linux_build_performed: false`。
- 虚拟机应收集 `$LOG`、`$WS/reports`、`$WS/log`、`original_snapshot.json` 和 `prepared.json`。编译错误请保留完整首个失败阶段日志，不只截图最后一行。
- `requirements.txt` 的 Python 版本清单是运行时库存，不会为编译自动安装 torch/CUDA。依赖 manifest 不完整，预检通过不能证明运行时所有导入可用。
- 未进行 Linux ROS 编译、运行时入口导入、算法数据回放或精度验收。未修复未经实际编译证实的 C++/ROS API 问题。

维护者可在本目录运行 `python -B validate.py` 重建主机证据文件；它会在本目录创建并清理测试副本，不需要 ROS。只读检查入口是 `preflight.py --preflight --no-server-precheck`，两者副作用范围不同。
