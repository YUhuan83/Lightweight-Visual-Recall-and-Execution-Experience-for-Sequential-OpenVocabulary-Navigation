# 整路退役执行边界（2026-10-26）

## 定位两次及范围

原分段执行下motion_queue[:1]确实只保留当前方向段；v37改为整路一个FollowPath后，这一项包含全部后续换向段。ContinuousGraphPathBridge.prepare_intent_change/on_route及共享finish_current_exploration_section仍仅处理外层队列/retired_motion，没有限制控制器内部段进度。控制器到cusp会继续旧意图后续段，甚至在车辆已经看见目标而上层换意图之后仍沿旧段行驶。这是整路接口交接错误，不是视觉记忆原理必然失败。

两个独立现场反例：v37复测170640在query99.094s声明探索段仅剩.526m，旧path仍于101.172/105.772s两次换向；v41在150.141s声明剩1.251m，旧path于152.643/164.144s两次换向，165.2s才耗尽租约，随后15秒串行恢复。其余四轮也有反例。按原accepted path stamp匹配独立控制器进度，排除退休消息后.3秒内的边界时间差；不是仅凭队列或状态名推断。

只读审计两遍：test_artifacts/general_observation_20261026/retired_full_path_audit.json。三个Python入口反例改前两遍均3fail/0error；新C++测试在真实旧v37插件两遍均3fail（继续倒车/迟到权限/重复延长），另3项对照通过。

## 隔离候选v42

从v40_observation_heading复制，不合入v41增加一般规划的实验，以隔离共同执行缺口。主目录703运行文件、原基线ZIP、全部旧候选保留，未commit。只改ContinuousGraphPathBridge和RPP Controller两个运行文件，另两个C++测试文件及一个Python测试文件。

- Bridge在前沿退休、显式意图切换或收到新意图路线时，向/wildos/controller_path_retirement发布当前剩余方向Path；header stamp为实际FollowPath接受的唯一path身份，保持原frame与精确终点pose。按path身份只限制一次，不能用后续进度扩大原权限。
- RPP在与setPlan/compute相同的mutex内接收限制，核对活动path身份、frame及现有方向段精确终点。重复/乱序只能缩短权限；旧path不能影响新动作。路径重复端点取最早匹配边界，属于限制权限，不授权额外旧运动；重叠端点可能更早停车，仍须现场检查效率。
- 当前健康方向仍持续运行，经过验证的新路径仍可滚动接替。到退役边界先发新鲜刹车，真实平移/角速度均≤.05后终止旧action，走既有归属/取消/停稳/ACK与新路线准入。延迟限制若到达时已越过该段，同样先刹停，不继续旧后缀。不伪造目标成功、不把原始全路径终点当作已到达。
- 正常非退役路径的内部必要换向保持整路，不恢复外层分段；12m规划/8m控制、碰撞、四条可靠支持及车身2m最终到达不变。新Bridge和新C++库成组部署并冻结SHA，禁止只换一端。

## 验证与剩余风险

改后50项相关回调两遍通过、原828离线通过；实际新RPP32项两遍通过、795真实ROS组件两遍通过、console测试通过。新RPP SHA217609ca8ff18a5ed3294789301cb16bc9102eeded0efbff744b9c6df101650c。新路径接替、旧身份、错frame、延迟及重复限制均有真实插件对照；真实物理到达/连续探索仍只能由现场证明。

已预检无残留/X148锁并启动180秒green residential house现场。需要关闭后检查新边界是否触发、旧path是否再跨cusp、停车时间、新鲜命令、可靠定位/最终车身间距。当前待关闭，不宣称已跑通或无回退。受控action终止仍可能带来停车与后台计划重入开销；可靠几何稀疏、探索候选串行失败及局部碰撞恢复仍独立待验。

证据test_artifacts/retirement_boundary_20261026；服务器execution_continuity_review_20261022/v42_retirement_boundary。当前未录视频。

## 两轮关闭：边界修复生效，完整目标未通过

两轮215243/215959均搜索180.022/180.171秒超时，startup40.313/40.990、收尾23.031/22.647秒；session errors=[]、console0、全部项目进程/X148锁清空。实际新库SHA匹配。近停23.910%/16.737%，最长4.360/10.248秒；路程132.971/158.675、净27.905/5.342米；stale11.375/14.892秒，其中planning6.419/12.013秒，gear4.561/3.196秒。历史Native采样口径含约数秒query前记录，不冒充精确query裁剪指标。

12/1次实际退休Path消息分别与已接受的动作身份精确匹配，300ms传输容差之后旧方向终点越界0/0；第一轮两次边界处真实停车后受控终止，非崩溃。两轮普通动作仍整路一个FollowPath，实际源命令无8m曲率越界或原地旋转。该具体接口修复已有两轮现场支持，不等于整体无回退。两次Native/events复算均各自完全一致。

第一轮0mask，目标query已切换，322有效相机TF处理帧、143评分输出；第二轮4mask、220有效TF处理帧、124评分输出，均0可靠定位/确认。目标图像未充分形成可接近证据；不归因视觉节点停止，也不能据末帧证明整轮没有可见目标。score/graph_pair诊断观察器的转换异常发生在其最后记录仍已覆盖搜寻后阶段；不把其栈自动记为主决策运行崩溃，具体退出时间需另审。

第二轮同一个探索owner占约175秒、155米路径；三个实际RPP弧碰撞abort后，第三次已使用8秒档恢复，仍继续同owner局部路径，没有切换frontier。源码中重复执行失败只纳入execution_stalled/tracking_deviation或局部预检碰撞，通用FollowPath STATUS_ABORTED不进入该已有替代frontier分支；属于下一项恢复策略缺口，不能以本轮退役边界已好而宣称探索已正常。详repeated_abort_budget_audit.json。未进一步修改此逻辑；仍不推广或commit。
