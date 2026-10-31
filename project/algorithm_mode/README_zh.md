# WildOS 算法优先模式准备

本目录只准备算法输入边界，不关注车辆型号的高保真复刻。仅使用 Python 3.9+ 标准库；没有 BeamNG 坐标假设、路线、地图坐标、仿真文件、服务器操作、ROS 发布器或在线适配器。测试全部是合成代数夹具，不是实际标定或算法性能实验。其他目录不属于本次修改范围。

## 两个阶段

| 配置名 | 算法定位输入 | 可以声称什么 |
|---|---|---|
| `oracle_pose` | 仅自车真值运动状态转换成局部里程计；相机和 LiDAR 仍为真实采集传感器输入 | 定位理想化条件下的**实验性上界**，用于隔离感知、语义搜索、规划问题；不是完整复现，也不保证性能必然优于 LIO |
| `estimated_pose` | LIO 输出的自车里程计；不得使用 oracle 输出、真值初始化或真值纠偏 | 单独验证含估计定位误差的算法链路；本目录没有实现 LIO，不宣称阶段已跑通 |

oracle 中唯一允许越过真值边界的是自车位姿及可选自车线/角速度。目标 GT 坐标、到目标真值距离、隐藏目标身份标签、全地图、障碍连通图、隐藏语义、评分端审核结论均不进入算法。正常相机像素、LiDAR 几何点和算法自身从传感器建立的地图不属于隐藏地图。任务的语言描述需走独立冻结任务协议，不通过本适配器夹带目标位置。评分真值在独立通道中保存。

## 文件与验证

- `config_template.json`：精确包含两个固定 profile 的配置模板，未知标定为 `null`，默认 `verified:false`，结构合法但不可用于适配。
- `contract.py`：严格配置与观测校验、纯字典 `OraclePoseAdapter`。
- `validate_config.py`：只读配置校验入口；成功退出 0，拒绝退出 2。
- `test_contract.py`：SE(3)、信息隔离、时间、epoch、标定门禁和模式边界的单元测试。

项目根目录执行：

```powershell
python -B -m algorithm_mode.validate_config
python -B -m algorithm_mode.validate_config --require-ready
python -B -m unittest discover -s algorithm_mode -p "test_*.py" -v
```

第二条对默认模板**应当失败**，不是安装问题。第一条输出 `ready:false` 和未解决字段。`verified:true` 必须同时具备全部字段；声明和引用只能进行结构检查，无法自动证明实际标定。测试中的 `synthetic-*` 引用绝不可用于实采验收。

## 标定与变换

先验证源世界坐标轴、源车体坐标轴、姿态矩阵含义、长度/时间/角度单位、速度表达坐标系，以及相机/LiDAR 内外参与采集时间定义，再填写有可追溯证据的配置。`source_world_axes`、`source_body_axes` 是已验证的轴声明，不是由程序猜测的 BeamNG 默认值。`evidence_ref` 指向源姿态/轴/速度校准证据，`sensor_evidence_ref` 指向传感器内外参、RGB 通道顺序、相机光学轴、LiDAR 扫描时刻和去畸变证据。此处只保存引用，既不执行标定，也不转换原始引擎 API。

所有旋转为右手正交 3×3 矩阵，使用列向量。`R_A_B` 将 B 坐标向量变换到 A。导航车体 N 即 `base_link`，约定 x 前、y 左、z 上；这不是对源车体 S 的假定。声明 `R_source_nav = R_S_N`，原始自车输入 `R_world_source = R_W_S`。反射、非正交和非有限矩阵拒绝。

每个连续 epoch 创建一次适配器，显式输入固定初始原点 `origin`：

```text
epoch: 非空字符串，例如记录器生成的回合/坐标系实例 ID
initial_sim_time_s: 原点建立时的采集仿真时间，允许为 0
position_world_m: odom 原点在源世界 W 中的位置 p_W_O
R_world_odom: odom 轴在源世界 W 中的旋转 R_W_O
```

origin 由调用端提供并核实；可取回合初始自车参考点和完整初始导航车体朝向，此时首帧局部位姿为零平移/单位旋转。也允许已核实的其他初始局部轴定义，不强迫首帧输出为零。程序不从后续运动重算原点，不只减 x/y，不只扣 yaw。构造时深拷贝 origin 和配置，后续修改调用端对象不会改变已有适配器。

```text
R_W_N = R_W_S R_S_N
p_O_N = R_W_O^T (p_W_N - p_W_O)
R_O_N = R_W_O^T R_W_N
v_N = R_W_N^T v_W      （velocity_frame=source_world）
v_N = R_S_N^T v_S      （velocity_frame=source_body）
```

角速度按同样的旋转规则处理，单位 rad/s；输入必须是角速度向量而非 Euler 角导数。线速度单位 m/s，必须是同一车体参考点相对世界的实际线速度，只是表达基底不同。两种速度必须使用同一声明的源表达坐标系。平移不施加到速度向量。`reference_points_coincident:true` 必须有证据：源自车位置/速度参考点与导航车体原点重合；不支持未补偿杆臂，不靠猜测加入叉乘项。不满足这些前提就拒绝接入，不能靠开关绕过。

`estimated_pose` 的局部原点由估计器建立，不能用这里的真值 origin 初始化 LIO。评分端仍使用独立连续世界系计算真实路程。

## 规范观测

`adapt(ego, camera, lidar)` 返回 `wildos-observation/1` 字典。字段集合严格校验，不是 ROS 消息，也不是现有评估日志。

| 字段 | 含义 |
|---|---|
| `localization_mode` | `oracle_pose` 或 `estimated_pose`，必须与调用端期待的模式一致 |
| `acquisition_sim_time_s` | 采集仿真秒，非墙钟/接收时间；允许有效的零，不把零解释为缺失 |
| `epoch` | 非空字符串，标识连续坐标/时间实例；数字 0、空值拒绝 |
| `odom` | `frame_id=odom`、`child_frame_id=base_link`、`position_m`、`R_odom_nav`、可选两种速度、`velocity_frame=base_link`、`covariance`、`provenance` |
| `camera` | 同时间/epoch，已标定 `frame_id`，正整数 `width/height`，长度恰为宽×高×3的 packed RGB8 `bytes`，不带引擎语义标签 |
| `lidar` | 同时间/epoch，已标定 `frame_id`，`points_xyz_m` 为有限三维数值列表；空列表表示确实取得空扫描，缺帧不可用空列表掩盖 |

原始 ego 仅读取 `acquisition_sim_time_s, epoch, position_world_m, R_world_source, linear_velocity, angular_velocity`。其他原始字段不会复制到输出，允许调用端先用有额外字段的离线记录验证隔离。相机/LiDAR 对象则拒绝所有额外字段，点只接受三个有限数字；绝不透传任意 payload 字典或可暗含真值的引用。输出不包含世界原点、原始世界位姿或原始 ego 对象。

观测 v1 刻意只支持已同步快照：相机、LiDAR 与位姿采集时间及 epoch 必须完全一致。异步采集需要以后单独实现并验证有时间溯源的对齐/插值/去畸变层，不可简单改写旧帧时间冒充同步。LiDAR 时间应对应已核实的扫描参考时刻。本目录无 IMU 接入；LIO 所需 IMU、同步与估计器生命周期属于下一阶段，不应误认为两种传感器就能完成 LIO。

首帧时间必须等于 `initial_sim_time_s`，之后严格递增；NaN、Inf、布尔数值、重复/倒退时间、跨 epoch 均拒绝。验证失败不推进适配器时间状态。暂停后重复帧需由记录器丢弃；传送、重启、重定位或时间重置必须终止回合，新建 epoch/适配器，不能继续拼接。规范观测保留实际采集仿真时间；评分日志另按评估协议转换为回合相对时间，不混用这两个时间定义。

协方差未知明确为 Python `None`（JSON 语义为 `null`），速度未观测也为 `None`，不是零。`provenance=ego_truth_covariance_unknown` 明示 oracle 来源。**ROS Odometry 的 covariance 没有通用 `-1` 未知约定**，本目录不发布 ROS、不填假零协方差、不照搬 IMU 的 `-1` 哨兵。当前规范只接受 null 协方差，未来保留真实 LIO 协方差需扩展版本和测试。相机含 bytes，整个字典不能直接当 JSON 日志写入。

`validate_observation(obs, "estimated_pose")` 拒绝 oracle 模式及 oracle provenance；LIO 输入需标为 `lio_covariance_unknown`。这只是结构边界，不能阻止恶意调用端将真值重新标成 LIO，或在像素中编码秘密。实际接入必须由独立生产者、权限隔离与数据流审计保证来源；本目录未声称完成此类安全认证。

## 最小控制闭环

只需建立可重复的低速路径/速度指令跟踪、限幅、停止、超时停车与干预记录。控制器不是本次 WildOS 创新或车辆动力学复现目标，不在此编造详细车辆标定参数。后续固定车辆、控制器版本与限幅、安全约束，不在比较算法时同时更换这些条件。先在 oracle 阶段验证感知/语义搜索/规划，再接 LIO；不因定位可用就宣称语义搜索已成功。

## 场景与评分隔离

沿用 `evaluation/scenario_template.json` 的待验收场景，不新增猜测路线/坐标。场景家族与 `task_mode`（搜索/点目标）、先验条件、定位模式是不同维度：不得把 oracle 等同于 point-goal，不得向 object-search 提供目标 GT。两定位阶段冻结同一任务、场景布局、传感器、算法、控制器、预算与 trial/seed；跨定位模式只作为显式受控消融分别报告，不池化为同输入基线。

现有 `evaluation/README.md` 第 36、54、78、82 行及 `evaluation/evaluator.py` 第 114、127–128、220–224 行定义严格元数据、`input_contract=sensors_only` 等任务信息边界和比较分组，当前说明禁止 GT 位姿流向搜索算法，且还没有 `localization_mode` 字段。**不能把 oracle 日志标成现有 sensors_only 直接评分，不能借 baseline_name 标签掩盖输入差异。** 简单添加字段也会被现有严格 schema 拒绝。

在评分这些日志之前，主线维护者需要正式扩展/版本化 evaluator 的输入契约：显式允许 oracle 中仅自车真值的例外，同时继续禁止目标 GT/隐藏地图语义；增加受控枚举 `localization_mode`，校验定位来源与模式一致，并将该字段纳入 `comparison_group` 的哈希与汇总隔离。`input_contract` 仍说明任务传感器/先验条件，不能代替定位模式。配对消融须单独保留模式标签及相同 trial/seed，不能通过删标签把两模式放进同组。独立评分位姿 `position_source=scorer_world_pose` 保持不变，不用 LIO 漂移轨迹计算真实路程。本次不修改 evaluator，也不输出伪装成兼容格式的评分日志。

完成范围是可执行离线代码、默认不可运行的已校验模板及合成测试；真实标定、在线采集、LIO 接入、控制闭环与扩展后的评估验收均尚未完成。
