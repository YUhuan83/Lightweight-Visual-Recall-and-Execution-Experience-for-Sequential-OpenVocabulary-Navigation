# 2026-10-14 完整绿色房屋实验与剩余故障

## 冻结与恢复对照

默认DDS恢复组件10虽然几何执行报告通过，但控制器位姿最大源龄2.152028968s、40个tick在外部odom新鲜时仍用过龄位姿。再次回环UDP组件11最大0.210167225s、此类tick为0；统一入口部署后的组件12最大0.21554457s、此类tick为0，真实换向/停车通过。三者参数、车体和8m执行/12m名义规划半径保持；不是目标到达成功数。

源候选06bb47afd4d32df40afd85fef7aa0f11c97e89efff0c9dcf1e120916da4b64d0，402成员。新Python覆盖层在dds_transport_candidate独立安装；C++实际插件沿用明确冻结的tracking_contract_candidate/install，库SHA和算法源码逐文件核对未变。DDS profile作为安装资源，local_dds_run在ROS初始化前应用环境，传感器/导航/模型/记录器继承同一策略。完整实验预检要求每个ROS拥有者及传感器配置匹配；8环境单测、6初版策略测试、本机全局离线/服务器安装与策略回归通过。

## 准备中的配置反例

首个视觉进程1768661因visual_navigation安装目录没有beamng_wildos.yaml而退出；当时未发送目标、未启动车辆搜索，不能写成模型识别失败。对应配置实际在冻结beamng_wildos_adapter/config中。先核对源字节SHA7bfb027ebb4a02e913eda1e726afab078677089497cff93a06454364c9eddb2f，再以绝对路径重启唯一已退出模型1771495，保留vision.log及vision_config_retry1.log。原preparation.passed只反映车体/传感器和进程启动，不能证明模型就绪；此反例单独记录。后续准备脚本已改显式配置路径，全栈就绪/命令门由正式preflight核查。

## 完整现场结果：失败

Town出生点、同一次地图加载；执行中0瞬移/0重载，不预先给加油站记忆指令；真实723节点、0固定、467车身碰撞、四轮各32碰撞，原始质量/转向/物理保留。唯一目标green residential house，ACK/profile预检通过，无MP4。

- 搜索420.084583s后超时；里程计累计222.58127m，Native参考点225.12660m，最远离出生点50.93823m，峰速1.77484m/s。没有修改速度限制或到达判据。
- 46个following、36个maneuver_done、4个segment_done、4次follow_failed、4次no_vehicle_path、4次after_maneuver重规划。出现两个>=5s静止区间，最长21.337s；不能直接称为物理受困。
- 控制器日志没有本轮单方向反向保护失败；两个动作中止明确是bounded command arc碰撞保护。不能沿用旧文本collision ahead=0就说无碰撞拒绝。其他失败包含motion_section_invalid/excess_curvature，原事件逐条保留。
- 后半程28条目标掩码、18条bearing_only接近消息，实际执行一个观测步；3条localization_valid=true，其中一个multiview位置与后两个LiDAR位置相差约18m，后两者彼此接近。object_confirmation为0，稳定实例确认/严格视觉到达未通过。不能说完全没有识别/定位，也不能将该3条定位或route_done当作找到并到达。
- 摄像头信息与掩码戳差均0，mask layout为[1,1,640,960]；没有证据支持本轮mask/CameraInfo尺寸或时间戳错位。首个掩码约在搜索后段，有效一致定位来得很晚；暂不放宽确认帧数或修改模型能力。
- 完整模型负载下3775个控制器tick：位姿源龄median0.24827/P95 0.49938/max0.62627s；外部odom<0.3s而控制器位姿>0.5s的tick为0。旧的独立控制器TF积压未复现，但原始状态采样/渲染延迟仍有尾部（odom max0.84197s），不称作所有时间戳问题消失。

## 已抽取的下一项反例

`green_house_dds_trial1/sliced_path_counterexamples.json`在本机自由图重放记录的原始几何：4个案例整条validate_vehicle_path返回允许，切成实际motion_sections后部分片段返回excess_curvature。首次现场接续拒绝约55.068s，早于173.512s的首个follow_failed。地图碰撞、线路走廊和实际车姿从几何复现隔离；相同点/heading/8m限值下判定不一致已确定。

这证明候选整条准入不能保证随后分段的同一曲率约定；尚未确定是Smac输出/平滑本身不可执行，还是量化/窗口/切片边界造成错误拒绝。不能直接关闭曲率保护或把所有问题归于新视觉记忆。下一步保留反例，检查生产者运动原语/平滑与消费者按方向分段的约定，覆盖探索与记忆路径。

## 停车验收缺口与恢复

旧完整试验末尾只要求一次SDK读数<0.05便暂停：末0.042975m/s，随后独立暂停快照0.051303m/s，位置差约0.0007m。清理保护正确拒绝；不能据单次差异断言未授权移动或车体故障。未放宽阈值。

恢复时保持两门关闭、同一位置/地图/挡位，普通全制动后连续0.5s SDK速度<0.05，末0.010074m/s、暂停。第二次清理通过：stack1768659、vision1771495、triangulation1768660、sensor1768568全部停止，物理保持、传感器空，只有引擎1709748/Town暂停保留。首次round_shutdown.json失败和attempt2通过均保存，未覆盖。

tools/town_physical_single_target_trial.py已统一使用已有settle连续停稳函数，记录样本；9策略测试/全局离线通过。新402成员包40796ca9b1f0be850de25ed40d8beca28d4c33d2d9bcf08e8874a13885d95124只在本机冻结，服务器完整实验运行的是06bb版本，新完整退出路径尚待部署/现场核验。相同settle函数的独立真实制动恢复已通过，不替代新完整调用路径验收。未commit。

## 证据

本机根：test_artifacts/tracking_contract_repair_20261014。

- full_dds_trial1_evidence.zip：39,378,675字节、62原始成员278,475,497字节，SHA bfc0d4f14e40a94cb3ddba3932837e7def142c5b3eaf1dcda96bc8e862a70e07；本机ZIP及全部成员SHA核对通过。包含约230MiB闭包接口SQLite、地形755条、失败图片、原模型配置失败/恢复、停止反例和安装身份。
- green_house_dds_trial1/failure_last_rgb.png：本轮失败图像，source age0.3519s，画面中可见对街浅绿色建筑；不是到达证据。
- typed_interface_audit.json、sliced_path_counterexamples.json、analysis_summary.json，以及组件10/11/12真实记录与tf_freshness_deployed.log。
- 额外单帧诊断未取到新鲜输入：脚本上传时间19:38:23，而完整试验已于19:36:48结束并暂停。后续可靠QoS版本同样在暂停后失败；没有证据把这两次失败归因于RGB网络/QoS。不得据此盲改传输配置或相机刷新。

完整实验仍未通过，20轮80%及多目标记忆流程未开始。目标继续活动；下一步先从这批封闭真实反例修复分段曲率契约，再验证接近/定位/确认链和完整流程。
