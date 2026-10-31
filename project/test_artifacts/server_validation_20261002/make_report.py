from pathlib import Path
import json
import zipfile
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

root=Path(__file__).resolve().parent
dest=root/'server_evidence'
dest.mkdir(exist_ok=True)
with zipfile.ZipFile(root/'evidence_20261002.zip') as z:
    for name in z.namelist():
        if not (dest/name).resolve().is_relative_to(dest.resolve()):
            raise RuntimeError('Unsafe archive path')
    z.extractall(dest)
rounds=json.loads((dest/'round_results.json').read_text())
geom=json.loads((dest/'vehicle_geometry_check.json').read_text())
state=json.loads((dest/'final_state.json').read_text())
assert state['paused'] and state['native']['sent']['throttle']==0 and state['native']['sent']['brake']==1
x0,x1=geom['body_extents']['x']; y0,y1=geom['body_extents']['y']
fig,ax=plt.subplots(figsize=(9,4.6))
ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor='#58ae8b',edgecolor='#247151',alpha=.45,label='Measured body bounding box'))
ax.add_patch(Rectangle((-2.3,-1),4.6,2,fill=False,edgecolor='#ce5353',linestyle='--',linewidth=2,label='Configured Nav2 footprint'))
ax.scatter([0],[0],color='#1760b0',s=65,zorder=5,label='BeamNG StateSensor reference')
ax.scatter([(x0+x1)/2],[(y0+y1)/2],color='#247151',s=35,zorder=5,label='Bounding box centre')
ax.annotate('Uncovered rear body',xy=(x0+.25,.65),xytext=(-3.35,1.8),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.annotate('Extra front region',xy=(1.75,.55),xytext=(1.1,1.8),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.axhline(0,color='#bbb',lw=.7); ax.axvline(0,color='#bbb',lw=.7)
ax.set(xlim=(-3.6,2.8),ylim=(-1.65,2.2),xlabel='Body forward x (m)',ylabel='Body left y (m)',
       title='Actual vehicle body and configured footprint use different origins')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='lower left',fontsize=8,ncol=2)
fig.tight_layout(); fig.savefig(root/'vehicle_geometry.png',dpi=160); plt.close(fig)

rows=[]
for r in rounds:
    dep=r.get('maximum_departure',r.get('measured_max_departure'))
    desc={'house_attempt1':'主动中断：RPP 碰撞中止、时间基准异常',
          'house_attempt2':'加油站初始记忆未建立', 'house_attempt3':'目标转换节点重启环境错误，恢复太晚',
          'house_attempt4':'真实离站、记忆规划返站；未确认房屋/未达20米',
          'house_attempt5':'换向段接近拐点停滞、执行失败触发记忆候选拒绝',
          'house_attempt6':'完成第一段跟踪，后续换向偏离并反复前后移动',
          'house_attempt7':'新偏离保护生效，离站不足；返站布尔标志不是有效完整验收'}[r['attempt']]
    rows.append(f"| {r['attempt'].removeprefix('house_attempt')} | {dep:.2f} 米 | {desc} | 未通过 |" if dep is not None else
                f"| {r['attempt'].removeprefix('house_attempt')} | — | {desc} | 未通过 |")
event_latency=[]
for r in rounds:
    events=json.loads((dest/r['attempt']/'full_test.json').read_text()).get('events',[])
    switch=next((e['elapsed'] for e in events if e['kind']=='query_sent' and e['phase']=='station_return'),None)
    if switch is None: continue
    selected=next((e['elapsed']-switch for e in events if e['kind']=='memory_selection' and
        e.get('payload',{}).get('status')=='memory_candidate_selected' and e['phase']=='station_return'),None)
    preview=next((e['elapsed']-switch for e in events if e['kind']=='plan_status' and
        e.get('source')=='memory_via_graphnav' and e.get('status')=='planned' and e['phase']=='station_return'),None)
    following=next((e['elapsed']-switch for e in events if e['kind']=='plan_status' and
        e.get('source')=='memory_via_graphnav' and e.get('status')=='following' and e['phase']=='station_return'),None)
    event_latency.append({'attempt':r['attempt'],'selection_s':selected,'preview_s':preview,'following_s':following})
(root/'event_latency.json').write_text(json.dumps(event_latency,indent=2),encoding='utf-8')
source=Path(__file__).resolve().parents[2]
report=f"""# 2026-10-02 Town 服务器测试

## 验收结果

**完整流程未通过。** 按用户确认的“加油站出生点 → 寻找房屋 → 记忆返回加油站”测试，要求第一阶段至少离开出生点20米。7轮调试/实测中，最大离站距离12.96米，尚未确认房屋，未完成要求的完整返回验收。

地图一直保留。各轮正式序列内不重新加载地图、不传送车辆、不重启视觉进程；下一轮准备时才复位到出生点并清空视觉内存。此次验证的是同进程记忆，没有采用跨进程记忆保存。

| 轮次 | 最大离站距离 | 结果/停止原因 | 完整验收 |
|---|---:|---|---|
{chr(10).join(rows)}

第4轮切回加油站后命中同一出生点记忆，生成并执行了返站路径，原生位姿回到出生点约0.77米范围内。但第一目标未完成，不计作完整通过。事件时序详见[event_latency.json]({(root/'event_latency.json').as_posix()})：测试脚本先固定等待3秒再启用执行，旧 JSON 中约3.26秒的延迟包含该等待，不能作为纯算法耗时。

**第7轮原始 JSON 的 returned=true 不是有效返程结果**：切换时车辆本来就在记忆目标附近，实际没有足够返程位移，且离站不足20米。原始文件保持不改，汇总中 meaningful_return_motion=false、return_flag_is_valid_full_sequence=false，整轮 passed=false。

## 已修复并部署

1. 整个车身不能重复按99膨胀格判碰撞。车身采样现拒绝100真实障碍，路径中心仍拒绝99；没有删除护栏或关闭Nav2碰撞检测。[Nav2官方值映射](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp)。
2. 长暂停后传感器时间仍沿用旧基准。非use_sim_time模式按同一测量偏移对齐官方ROS输出，原始图像TF查询和位姿/点云配对仍使用原始时间。重复暂停帧不会被刷新成新帧，恢复后的旧帧会被拒绝。现场抽样：图像年龄约−0.024～0.077秒、点云约0.036～0.142秒。
3. FollowPath 执行失败不再立即将记忆视点标为不可达。保留记忆目的地并对当前路线退避；后续完整规划检查仍可拒绝确实不可达的视点。
4. 当前路径段跟踪偏离超过1米时取消执行，保留记忆，避免无限前后换向；测试以净离站距离/接近出生点的改善判定进展。
5. 目标转换节点重启时使用完整ROS构建环境，并检查进程启动结果。

现场还将旧测试的油门限制由最大8%/最小3%恢复到执行器最大12%/速度桥最小6%的原有默认范围，用于基本起步及低速行驶；目标车速1.2m/s未变。本轮没有完成车速性能验收。

## 主要尚未修复的问题：车辆几何参考点

真实包围盒通过BeamNG车辆API现场读取，测量基准为当前StateSensor位姿。测得：

| 项目 | 实测相对范围 | 当前Nav2假设 |
|---|---|---|
| 纵向x | {x0:.3f}～{x1:.3f}米 | −2.3～+2.3米 |
| 横向y | {y0:.3f}～{y1:.3f}米 | −1.0～+1.0米 |
| 车体中心 | x={(x0+x1)/2:.3f}、y={(y0+y1)/2:.3f}米 | 原点 |

![实测车体与代价地图足迹]({(root/'vehicle_geometry.png').as_posix()})

**原点不在车体中心这一点已经确认**：当前配置多覆盖前方约1.17米，同时漏掉后方约0.78米和一侧约0.34米，足迹与实车不一致。

原生饱和转向数据的46个运动窗口得到参考点轨迹半径中位数8.049米。由侧向速度/角速度推算的纵向参考点偏移中位数约2.35米，提示RPP后轴参考点也需统一；这是运动学推断，尚未用车轮节点位置核验，不能把8.049米直接当作后轴最小转弯半径。

因此，不能把当前全部失败归因于“选错记忆bin”。现场存在真实路径跟踪偏离及几何接口不一致。下一步需要确认后轴和车体中心位置，再统一odom、TF、点云坐标、车身自点过滤、costmap footprint、路径验证和记忆观察位姿；不应只放宽偏离阈值。

另有规划耗时与代价地图新鲜度门限的冲突风险：现场仍间歇出现stale_costmap。Smac在规划期间持有全局costmap锁，而桥接器门限3秒、规划时间上限6秒；这条时序需进一步核验和修复。[Smac官方实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_hybrid.cpp)。

## 验证与材料

- 本地118项纯逻辑测试＋40项接口模拟回调测试全部通过；84个Python文件语法检查通过。接口模拟不等于真实DDS或车辆测试。
- 服务器最终38项真实ROS图桥测试通过；控制回调2项在先前部署时通过。动作/服务端点使用模拟对象，在隔离ROS域49中运行，未驱动车辆。
- 8组离线运动学比较仍通过既定模型检查；其理想后轴模型未包含本次测出的实车参考点偏移，不能代替现场验收。
- 190文件部署包已更新，SHA256：`2C43F281586325CB4544B24373C1B737F415364E4425DA56BABEB9C31F6CB98A`。
- [轮次汇总]({(dest/'round_results.json').as_posix()})、[车辆几何原始记录]({(dest/'vehicle_geometry_check.json').as_posix()})、[最终停机状态]({(dest/'final_state.json').as_posix()})。
- [第4轮视频：离站及记忆返站调试]({(dest/'house_attempt4/front_camera.mp4').as_posix()})、[第6轮视频：完成第一段后换向偏离]({(dest/'house_attempt6/front_camera.mp4').as_posix()})、[第7轮视频：偏离保护]({(dest/'house_attempt7/front_camera.mp4').as_posix()})。
- 原始ROS bags保留于服务器 `/home/ly/full_town_memory_20261002/house_attempt*/rosbag`；已下载的归档含元数据、报告、原生遥测和关键视频，没有复制全部大型db3。

结束状态：仿真已暂停，follow_path=false、控制enable=false；确认原生发送油门0、制动1。**尚不能宣称记忆提高了20米以上目标搜寻的成功率或速度，缺少完整通过轮次及无记忆对照。**
"""
(root/'server_test_report.md').write_text(report,encoding='utf-8')
print(json.dumps({'rounds':len(rounds),'full_passes':sum(bool(r['passed']) for r in rounds),
                  'max_departure':max(r.get('maximum_departure',0) for r in rounds if r.get('maximum_departure') is not None),
                  'state_paused':state['paused'],'report':str(root/'server_test_report.md')},ensure_ascii=False))
