# 自助实验入口

## 当前完成范围

新增入口 `tools/wildos_console.py`、Windows启动器 `tools/wildos-console.ps1`。
它们已做本机离线回归；服务器安装、真实ROS发现、地图加载、连续目标和退出清理仍待现场验收。
不能把本文件当作已完成20轮稳定性验收。当前服务器的旧组件进程是否残留尚未重新确认。

## 一次性准备（在服务器SSH终端）

将新的源码包解压到 `/home/ly/wildos-console/bundle`。
不要直接使用旧组件测试目录启动，它曾为组件探测停掉GraphNav planner。
下面命令创建独立安装，继承已有基础依赖，重建六个包（包含 GraphNav planner）；不启动车辆。

2026-10-16 的探索目标协议要求 Bridge 与 GraphNav 配套更新。入口会核对
`libgraphnav_planner_component.so` 的构建身份；旧安装和旧 `console.json` 不能用于
新候选验收。当前候选的组件验证与现场待验收边界见 `EXPLORATION_OWNER_REPAIR_20261016.md`。
已有install或console.json时拒绝覆盖，需要使用另一个新目录。

```bash
bash /home/ly/wildos-console/bundle/tools/setup_wildos_console.sh \
  /home/ly/wildos-console \
  /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
```

生成 `/home/ly/wildos-console/console.json`。若安装路径不同，修改此文件中的解释器、
BeamNG home/user和setup列表。`library_sha256`由独立编译产物生成，不手工填写示例哈希。
服务器需要已有BeamNG、ROS2、基础WildOS包、模型权重、CUDA视觉环境与SDK；此脚本不下载依赖。

默认不启动RViz、不录像。需要实时界面时先确认图形会话可用，再将 `rviz` 改为 `true`，
并设置正确的 `display`。采用已准备的左2D地图/右RGB+Target布局，真实RViz仍待验收。

## 每次开始（任选一种）

本机PowerShell：

```powershell
& 'C:\Users\Yu\Desktop\拟真\tools\wildos-console.ps1'
```

或在服务器SSH终端：

```bash
python3 /home/ly/wildos-console/bundle/tools/wildos_console.py \
  --config /home/ly/wildos-console/console.json
```

先完成setup/DDS引导、源码与安装一致性检查、服务器残留检查、新引擎与单次Town地图加载、
真实SBR车体与传感器、导航栈、三角定位、视觉模型与严格预检；两控制门保持关闭。
初次模型加载可能需要数分钟。看到“输入目标”提示后操作。
启动器会显示本轮日志目录。默认目标超时420秒；每会话最多100个目标/8小时。

## 输入目标、查看状态、停车、退出

直接输入一行目标后回车，建议用模型熟悉的英文表达：

```text
green residential house
/status
```

目标结束后输入下一目标，例如 `gas station`。地图、图、视觉进程及其同进程记忆保持；
新目标从车辆当前位置开始，没有出生点复位或原路返回要求。
正在搜索时不能直接覆盖活动目标，先 `/stop`，看到停车提示后再输入新目标。
输入会按模型协议统一大小写和空格。现有模型/Bridge把相同query视为重复命令，
不会创建新一轮：因此本入口拒绝连续两轮相同目标；重复相同目标的独立试验请退出重开。
模型配置的初始query目前是 `door`，首个目标也须与它不同（入口会打印实际初始query）。
这些是现有接口边界，本入口不会通过发送虚假中间目标来强制重置记忆。

| 输入 | 行为 |
|---|---|
| 一行目标 | 日志记录器就绪→发送一次query→视觉ACK→新鲜Bridge profile→打开控制门 |
| `/status` | 显示当前目标、阶段和证据目录 |
| `/stop` | 关执行门、持续验证SDK停稳、暂停仿真；保留模型和地图，允许下一目标 |
| `/quit` | 停车并清理本入口启动的进程，包括本入口的新引擎 |
| `/help` | 显示帮助 |

不要预先粘贴整串目标：只接受一个活动目标，停车时会丢弃排队目标，防止取消后重新开车。
等待ACK期间也能请求停止；参数RPC和SDK连接有各自超时，停止不是硬实时操作。
EOF、SSH断开/SIGHUP、Ctrl+C、SIGTERM均走退出路径。不能捕获断电或SIGKILL；
底层控制租约保护仍保持。停止失败会记录错误，禁止再次解锁，并尝试暂停/清理。
`/stop`是实验停车指令，不能替代真车独立急停设施。

若原终端无响应，在另一PowerShell中请求该会话退出（用启动时打印的真实目录）：

```powershell
& 'C:\Users\Yu\Desktop\拟真\tools\wildos-console.ps1' -Action Stop `
  -Session /home/ly/wildos-console/results/session_YYYYMMDD_HHMMSS
```

或服务器上：

```bash
python3 /home/ly/wildos-console/bundle/tools/wildos_console.py \
  --stop /home/ly/wildos-console/results/session_YYYYMMDD_HHMMSS
```

这只向经过boot ID、PID起始时间和脚本身份核实的console发送退出请求；
请求发出不等于清理完成，检查 `session_result.json` 中 `errors` 应为空。
不使用通配pkill，不会主动清掉不属于此入口的旧进程。发现旧进程时保存清单并拒绝启动。

## 证据与结果

每个 `goal_001` 等目录保存：

- `result.json`：目标、结束原因、到达证据/失败原因；`passed`始终不会自动置true。
- `events.jsonl`、`native_samples.jsonl`、`sensor_timing.jsonl`：事件及源时间戳。
- `interfaces/`：ROS2接口bag，不包含原始RGB大视频。
- `stop.json`：实际SDK连续0.5秒速度小于0.05m/s的停稳记录及物理状态。
- 成对到达RGB/mask，或带源戳和年龄说明的最后失败RGB。

到达只采用既有严格跨消息匹配，自动触发后记录 `automatic_arrival_pending_visual_review`，
须人工核对物体身份和接近情况。超时、取消、异常均不计成功。示例目标顺序是人工使用方式，
不代表已进入正式多目标成功率验收。

本入口不调速度、不关闭碰撞、不改规划器、不绕过曲率/倒车/租约/新鲜度保护。
日志中的“自动到达”与真正视觉验收必须分开。不要调用历史宽泛停止脚本混合管理此会话。
