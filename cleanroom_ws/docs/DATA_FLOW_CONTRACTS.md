# 当前系统数据流与数据类型

2026-11-01：按当前源码核对，仅说明接口，不修改运行实现。
方法面向真实机器人；下列ROS话题名是当前实现实例，spot1仅为命名空间。
定位输入可由真实里程计/定位系统提供，当前仿真用状态适配器提供，不据此宣称已验证真车定位。

## 0. 统一记号与约定

- C：相机数量；Q：当前计算的文本query数量；H,W：输入图像尺寸；h,w：低分辨率特征图尺寸。
- D：语言对齐特征维度，取决于实际adaptor/checkpoint；不能用backbone的768直接代替D。
- N：点云点数；V,E：导航图节点/边数；K：方向bin数量，当前配置16；L：路径位姿数。
- ROS Header：stamp.sec:int32、stamp.nanosec:uint32、frame_id:string。
- ROS位置/平移通常float64，单位m；yaw、角度rad；速度m/s；角速度rad/s；四元数顺序x,y,z,w。
- 世界坐标W当前为spot1/odom；物理体坐标B为spot1/base_link；平面导航坐标Bnav为spot1/nav_base_link。
- 光学相机坐标：x向右、y向下、z向前。B/Bnav采用机器人前、左、上方向约定。
- TF给出T_WB、T_WC、T_WL等刚体关系，按数据采集时刻查询。frame名称是string，不携带变换数值。
- 模型分数、几何支持分数、余弦相似度不能不加区分地都称为“目标存在概率”。

## 1. 用户query进入系统

生产者：用户/任务程序；传输：std_msgs/msg/String；话题：/spot1/object_search/query。
有效载荷data:string，例如green house。当前先casefold并合并空白，不生成新任务UUID。
消费者：WildOS视觉节点、目标定位器、GraphPathBridge。
进程内部：query:str；query_started_ns:int，表示本次query开始的ROS时钟时间。
输出：文本编码器产生归一化torch.Tensor[Q,D]。当前主要活动query为1个，Q维度仍保留。
切换query清理本任务的目标/证据，保留同一地图内仍有效的历史Store。

## 2. 传感器、位姿与TF

| 数据 | 传输类型/当前话题 | 核心字段和内存形式 | 消费者 |
|---|---|---|---|
| RGB | sensor_msgs/msg/Image；/spot1/realsense/front/color/image_raw | header；height/width:uint32；encoding:string；step:uint32；data:uint8[]。视觉节点转换为RGB uint8[H,W,3] | WildOS |
| 内参 | sensor_msgs/msg/CameraInfo；.../color/camera_info | K:float64[9]，D:float64[]，R:float64[9]，P:float64[12]，图像尺寸与畸变模型 | 投影、定位、被动采样 |
| LiDAR | sensor_msgs/msg/PointCloud2；/spot1/ouster/front/points_filtered及points_for_localization | header、fields[]、point_step/row_step、data:uint8[]；当前读取x/y/z三个FLOAT32字段，解码float32[N,3]，m | 地形、避障、定位、记忆 |
| 里程计 | nav_msgs/msg/Odometry；/spot1/odom | header、child_frame_id:string；pose.position:float64[3]、orientation:float64[4]、pose.covariance:float64[36]；twist线/角速度各float64[3]及covariance | 全系统 |
| 外参/位姿变换 | tf2_msgs/msg/TFMessage；/tf、/tf_static | transforms:TransformStamped[]；每项有header、child_frame_id、translation float64[3]、rotation float64[4] | 坐标变换 |

Odometry.pose在header.frame_id中，twist按child_frame_id理解。消息存在协方差字段，不代表当前
适配器已经估计了真实测量不确定度。输入PointCloud2可能含其他字段与填充，不能一概按每点12字节读取；
当前过滤器输出才明确为x/y/z三个float32、每点12字节。
近场40m与定位80m是当前配置，不能作为方法的固定理论量程。

## 3. 视觉推理：像素到语义特征

RGB uint8[H,W,3] -> ToTensor -> torch.float32[C,3,H,W]，数值[0,1]。
模型根据配置使用FP16/FP32，内部可缩放输入；输出如下。

| 输出 | 类型、形状 | 含义/去向 |
|---|---|---|
| 可通行性T | Tensor[C,1,H,W]，sigmoid值[0,1]；评分前CPU numpy.float32 | 图像路径约束、frontier评分 |
| 视觉frontier F | Tensor[C,1,H,W]，sigmoid值[0,1]；CPU numpy.float32 | 未知区域方向评分 |
| 语言对齐空间特征Z | Tensor[C,D,h,w]，当前FP16配置；来自adaptor | 当前query匹配、池化描述子、被动空间采样 |
| 文本特征z_q | Tensor[Q,D]，归一化浮点向量 | 与Z同空间计算相似度 |
| 相似度A | numpy浮点[C,Q,H,W]；由特征格相似度上采样 | 目标区域判断/可视化；余弦值并非概率 |
| 二值掩码M | numpy.uint8[C,Q,H,W]，值0或1；当前发布Q=1 | 定位器 |
| 场景描述子e | numpy.float32[D]，对C与空间维池化后L2归一化 | 执行记忆、节点观测 |

骨干中间特征与语言adaptor特征不同；当前对外返回的spatial_feats是adaptor特征。
当前query相似度采用einsum(qc,bchw->bqhw)，在特征格上阈值/连通支持检查后上采样。
主要视觉特征在进程内传递，不作为每帧ROS图消息广播。短时last_scene_features可能缓存完整特征图；
长期Store保存压缩/稀疏描述子，不能把两种缓存混为一谈。

## 4. 掩码进入目标定位

话题/类型：/spot1/object_mask，object_search_msgs/msg/ObjectMaskWithTf。

    header: std_msgs/Header
    query: string
    odom: nav_msgs/Odometry
    cam_infos: sensor_msgs/CameraInfo[]
    object_mask: std_msgs/UInt8MultiArray
    cam_transforms: tf2_msgs/TFMessage

object_mask.data是uint8[]，layout.dim记录[C,1,H,W]，不可丢弃layout后随意reshape。
当前该消息的header复制配对odom.header，不保证它就是原始相机的精确曝光时间；
相机通过近似同步配对，其CameraInfo和TF也必须与该观测对应。

定位器同时接收80m定位点云。把点云坐标变换到光学帧后投影：

    p_C = R_CW * p_W + t_CW
    [u,v,1] ~ K * p_C，且p_C.z>0

在掩码覆盖的像素中提取几何支持，得到三维表面候选；结合已有多视几何与空间一致性判断，
经过滑窗共识得到世界坐标目标。相机像素单位是pixel，三维位置单位是m，两者不能直接混用。

输出分两路：

1. /spot1/raw_object_waypoint：geometry_msgs/msg/PoseStamped。
   header.frame_id=W；pose.position是float64 XYZ；四元数通常设中性w=1。
   这是对象位置，不是已经证明可行的机器人停靠姿态。
2. /spot1/object_confirmation：std_msgs/msg/String，data是JSON，包含：

    query:str, frame:str, observation_stamp_ns:int,
    search_confirmed:bool, object_position:list[float](3), radius:float(m),
    inliers:int, observations:int, observer_position:list[float](3),
    observer_orientation:list[float](4，XYZW), quality:dict

quality包含point支持、关联状态及可用的depth_sigma、covariance等；字段依定位结果而定。
当前PoseStamped.header.stamp在发布时取now，而confirmation.observation_stamp_ns保留观测时刻。
因此“目标消息刚发布”不等于“目标观测刚采集”；两个时间必须分别理解。
消费者：GraphPathBridge更新当前目标/证据；WildOS据确认更新已确认对象和观察视点。

## 5. 几何数据进入导航图和代价地图

### 5.1 几何地图与碰撞地图

- /spot1/local_terrain：grid_map_msgs/msg/GridMap。header、分辨率m/cell、地图位姿与范围、
  layers:string[]、每层Float32MultiArray。当前发布traversability、unknown、elevation、
  elevation_valid、witnessed_step；高度为m，未知性通过层/有效性表达。当前80x80/0.5m不是固定方法要求。
- /wildos/points与/wildos/clearing_points：PointCloud2，float32[N,3]，当前输出B帧，
  保留原点云stamp。前者是障碍marking观测，后者是射线清除端点；端点坐标不等于射线起点。
- /global_costmap/costmap和/local_costmap/costmap：nav_msgs/msg/OccupancyGrid。
  info包含resolution:float32、width/height:uint32、origin:Pose；data:int8[width*height]。
  ROS占据表示为-1未知、0自由、最高100；Nav2发布映射中99对应内切膨胀、100对应致命障碍。
- *_costmap/costmap_raw：nav2_msgs/msg/Costmap；metadata含origin/resolution/size，data:uint8[]。
  Nav2内部0自由、1..252渐变代价、253内切膨胀、254致命、255未知。不能直接与OccupancyGrid数值比较。

### 5.2 图与视觉评分

Mapper输出/spot1/nav_graph及nav_graph_perception，类型graphnav_msgs/msg/NavigationGraph。
WildOS读取感知图并输出/spot1/scored_nav_graph，同类型，不是向规划器发送RGB或完整特征张量。

    Header header
    string[] trav_classes
    Node[] nodes
    Edge[] edges
    uint32 current_node_idx
    RobotMotionConstraints[] robot_profiles

Node主要字段：uuid:uint8[16]，pose:geometry_msgs/Pose；trav_properties[]包含
is_frontier:bool、frontier_points:Point[]、explored_radius/free_radius:float32(m)。
robot_traversability[]保存平台几何/风险评估；heading_candidates[]保存方向候选；properties是KeyValue[]。
Edge用from_idx/to_idx:uint32连接本消息的节点数组，包含traversability及平台评估。
节点UUID表示地点身份，数组索引只用于本图引用，不应跨不同图版本直接替代UUID。

KeyValue为key:string、value:float32[]。当前节点properties可含frontier_scores等长度K数组及统计量。
HeadingCandidate包含heading_frame:string、heading:float32(rad)，以及float32字段
frontier_score、semantic_traversability、physical_traversability、minimum_clearance(m)、dynamic_risk、
motion_cost、predicted_utility、target_utility；另有selected/has_supervision/traversable:bool。
字段被定义不代表每个字段都有独立实测值；保留来源和有效性含义。

视觉评分主要内部形式为numpy.float32[n_frontier,K]；图像可达性/路径属于视觉启发信息，
不是机器人已经可执行的几何路径。图规划融合最新拓扑和评分后输出nav_msgs/msg/Path。

## 6. 记忆内部保存的数据

四Store位于WildOS进程内，通过函数参数/返回值交换；SQLite :memory:并不是额外ROS节点。

### 6.1 PassiveLandmarkIndex：查询无关的稀疏空间特征

| 数组 | dtype/shape | 内容 |
|---|---|---|
| features | float16[Nm,D] | 归一化视觉描述子 |
| position | float32[Nm,3] | LiDAR支持的世界位置，m |
| observer | float32[Nm,4] | 观察者x,y,z,yaw |
| key | int32[Nm,4] | 空间体素键与观察方向键 |
| stamp | int64[Nm] | 原始关联观测时间，ns |
| last_seen | float64[Nm] | 进程使用的wall time秒 |
| count/id/support/fov | uint32/uint64/uint16/float32 | 计数、身份、支持数量、视场角rad |

Nm受当前容量/字节预算约束。它存空间采样特征，不把每项都称为已分割且已命名的对象实例。
查询输入为query:str和归一化text_feature:float32[D]；输出为候选list[dict]，非目标确认。

### 6.2 GraphObservationStore：SQLite内存数据库

- node_observations主键：(map_id TEXT,node_uuid TEXT,heading_bin INTEGER)。
  heading/XYZ及各评分统计是REAL，embedding是float32[D]编码的BLOB，计数INTEGER，时间REAL秒。
- semantic_objects：object_id/map_id/label为TEXT，XYZ/confidence/时间为REAL，
  embedding与可选covariance为float32编码BLOB，保存关联节点、历史视点和状态。
- search_coverage/search_heading_coverage：query、节点、方向、负证据、观测次数及时间。
- traversal_history：起止节点、方向bin、结果类别、时长、位移及可用执行观测。

这些是数据表职责，不是四条并行任务状态机。标签可能来自活动query的确认，
不能据表里有label列就宣称系统对所有经过物体进行了自动命名。

### 6.3 ConfirmedViewpointCache：Python字典和列表

query:str -> list[dict]；每项包含id:str、frame:str、position:list[float](3，观察者位置)、
heading:float(rad)、bin:int、observed_at:float(wall秒)、count:int、blocked_until:float；
可带object_position:list[float](3)、object_radius:float、object_id/anchor_node_uuid及视场角。
它当前使用observed_at，不应声称每个缓存项都保存了原始传感器时间戳。

### 6.4 ExecutionExperienceStore：SQLite内存数据库

embedding:float32[D] BLOB；heading_scores:float32[K] BLOB；supervised_mask:uint8[K] BLOB；
outcome/heading_count等INTEGER；scene_heading、execution_multiplier、时间等REAL；
encoder_id/map_id/scene_frame为TEXT。记录只监督实际执行过的方向。
检索输出prior:numpy.float32[K]和similarity:float；无有效证据的方向prior=1。
先验通过有界乘性融合影响视觉评分，不直接成为速度、转角或障碍代价。

## 7. 历史候选如何跨进程传输

话题/类型：/spot1/remembered_object_candidates，std_msgs/msg/String；data为JSON：

    query:str, views:list[dict], issued_at_ns:int, query_started_ns:int

views共有/可选字段：id、frame、position(历史观察者XYZ)、heading、object_position(对象区域XYZ)、
object_radius、observed_at、object_id/anchor_node_uuid等。被动检索还明确携带
observation_stamp_ns:int、semantic_similarity:float、support_landmarks:int、
evidence="passive_semantic_landmarks"、search_confirmed=false。
不同来源的候选字段不是完全一致的固定ROS消息schema；不要假定所有项都有confidence或sensor stamp。

RecallCandidatePolicy接收list[dict]并按位置/转向/可用confidence启发式排序。
GraphPathBridge当前仅在没有当前三维目标时采用历史候选，发布graphnav_destination:PoseStamped。
该position可能是历史观察视点，不是对象表面坐标；重新观测后以当前定位对象更新目标。
候选反馈/spot1/remembered_object_feedback也为String(JSON)，包含query、id、outcome。

## 8. 拓扑路线、车辆路径与控制

- /spot1/graphnav_destination：geometry_msgs/msg/PoseStamped，W帧，导航意图位置。
- /spot1/graphnav_planner/path：nav_msgs/msg/Path，header + PoseStamped[L]。
  这是拓扑引导，不保证满足车体完整运动学/碰撞条件。
- /compute_path_to_pose：nav2_msgs/action/ComputePathToPose。
  当前Goal设置goal/start:PoseStamped、use_start:bool、planner_id:string；
  start投影到Z=0/yaw-only。Result包含path:Path，外层Action结果另有status。
- /follow_path：nav2_msgs/action/FollowPath；Goal包含path:Path、controller_id:string、goal_checker_id:string。
  背景规划成功后发新Goal标准抢占；不是给每个路径点发一次新任务。
- /wildos/cmd_vel_stamped：geometry_msgs/msg/TwistStamped。
  header.frame_id=Bnav；twist.linear.x:float64(m/s)、angular.z:float64(rad/s)；其余向量分量仍存在。
  真车底盘可接该速度接口，或经适配转换为底盘所需接口，需核对参考点和运动学。
- 当前执行后端另将速度转beamng_msgs/msg/VehicleControl：steering/throttle/brake/
  parkingbrake/clutch为float64，gear:int8。这些是该后端控制量，转向量不是直接以rad解释。
  /wildos/native_state为beamng_msgs/msg/StateSensor，含Header、position:Point、dir/up/velocity/front:Vector3。
  它们是平台适配数据，不是视觉记忆算法的通用输入schema。

## 9. 执行结果、任务状态与完成判断

/spot1/traversal_feedback：graphnav_msgs/msg/TraversalFeedback，包含：

    Header header; UUID from_node,to_node; string traversability_class,robot_profile_id
    Pose start_pose,end_pose
    float32 commanded_heading,observed_heading       # rad
    float32 commanded_distance,observed_distance     # m
    float32 duration                                # s
    float32 energy_cost,slip_ratio
    uint8 outcome; bool retryable; KeyValue[] observations

outcome枚举：0未知、1成功、2几何受阻、3动态受阻、4受困、5打滑、6碰撞、7超时、8中止。
当前桥接主要发布成功进展，commanded_distance被设为observed_distance，且observed_distance为
起止XY位移，不是曲线积分里程；energy_cost/slip_ratio未赋值时的0不等于真实测量为0，
energy_cost当前没有明确物理单位标定。消息支持多结果，不证明全部类别已经在线发布和学习。

/wildos/plan_status：std_msgs/msg/String(JSON)，state:str事件名、task_state:str任务状态、query:str，
以及按事件附带的goal、retained_action、action_status、arrival_basis等。它是诊断事件，不是传感器证据。
当前完成检查使用当前query的目标位置、里程计、有效确认及其观测时间。目标位置、对象位置、
机器人导航位姿要分别保留；FollowPath成功也不自动等于语义目标成功。

## 10. 必须分别理解的四类时间

1. 采集时间：图像/点云header和confirmation.observation_stamp_ns，用于对齐及新鲜度。
2. 发布/任务时间：例如raw waypoint发布时刻、query_started_ns、issued_at_ns。
3. wall time秒：time.time()，当前部分记忆recency与冷却使用；不能直接与ROS ns数值比较。
4. monotonic秒：规划节流、超时、执行间隔；不表示世界坐标系或传感器采集时间。

上线真车时，定位/TF/图像/点云必须有可解释的时间对应。节点UUID、query与map/encoder身份也要对应。
坐标系相同的字符串不保证位姿没有漂移或回环修正；历史锚点在地图校正后的空间一致性仍需验收。

## 11. 代码核对入口

src/object_search_msgs/msg/ObjectMaskWithTf.msg；src/graphnav_msgs/msg/*.msg；src/beamng_msgs/msg/*.msg；
vendor/explorfm/explorfm_model.py；visual_navigation/wildos/nav.py及四Store；
visual_navigation/utils/object_search_utils.py；explorfm_triangulation/obj_mask_triangulation.py；
wildos_nav2_adapter/{graph_path_bridge,navigation,cloud_filter,cmd_vel_to_control,native_vehicle_control}.py。
以上是当前实现说明，未增加消息、模块或运行协议；没有据此宣称跨平台或整车已验收。
