from pathlib import Path
import json,hashlib,shutil
repo=Path(r'C:\Users\Yu\Desktop\拟真');base=repo/'test_artifacts/depth_geometry_20261027';field=base/'field'
assert (field/'confirmation_audit_1.json').read_bytes()==(field/'confirmation_audit_2.json').read_bytes()
report=[]
for i in (1,2):
 x=json.loads((base/f'real_ros_report_{i}.json').read_text(encoding='utf8'));assert all(r['passed'] for r in x);report.append(sum(r['tests'] for r in x))
assert report==[826,826],report
r=json.loads((field/'field_result.json').read_text(encoding='utf8'));assert r['result']['closed'] and not r['postcheck']['processes'] and not r['postcheck']['display148_lock']
summary=dict(runtime='v47_depth_geometry',runtime_changes=1,related_tests_twice=82,offline_tests=828,real_ros_tests=report,cpp_tests_twice=32,
 regression_scope='48 old recorded points unchanged; 2 now rejected as supported double-depth ambiguity. Additional measured scanline positions accepted.',
 same_recording_depth_output=dict(before=4,after=6),field_masks=88,field_valid_lidar=6,field_confirmations=0,full_acceptance=False,commit_created=False,
 confirmation_audit_sha256=hashlib.sha256((field/'confirmation_audit_1.json').read_bytes()).hexdigest(),
 remaining='Sparse supported ranges; valid observations do not form four inliers within the existing 60 s/1.5 m confirmation contract. Semantic target correctness not established.')
(base/'validation_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
diag=base/'diagnostics';diag.mkdir(exist_ok=True)
aux=Path(r'C:\Users\Yu\Desktop\Beamng')
for name in ('replay_field_depth_geometry.py','replay_field_depth_geometry.sh','replay_field_old_geometry.py','replay_field_old_geometry.sh','audit_field_confirmation_geometry.py'):
 shutil.copyfile(aux/name,diag/name)
doc=repo/'docs/DEPTH_GEOMETRY_20261027.md';s=doc.read_text(encoding='utf8');old='''## 现场待补

冻结版本、预检无残留后启动v47 trial_20261027_110900，真实query后180秒，启动/收尾另计，无录像。当前结果待关闭；完整定位/接近/到达与流畅验收仍未通过，不推广/commit。证据test_artifacts/depth_geometry_20261027。
'''
assert old in s
new='''## 现场关闭及同录包对照

v47 trial_20261027_110900已关闭：启动39.603、目标下发后搜索180.200、收尾14.818秒；结果timeout，session errors=[]、console0、项目进程/X148锁空。88个mask、85配对、6条有效LiDAR定位、0确认，完整目标搜寻没有通过。实际运行geometry SHA826e3040...、RPP仍217609ca...。无录像，不推广/commit。

不能拿不同路线的v46零定位与本轮6条直接当成增益。对同一v47关闭录包，两遍旧/新真实方法回放结果分别4/6条LiDAR，旧有4条保留；新增两帧1793070705329753881/1793070705843753905分别10/9个独立像素、投影跨度[15,1]/[13,1]，旧版被二维展开门拒绝。新回放6条坐标与实际现场逐条完全相同。旧/新报告SHA分别9e33c2a0.../7484c5f7...。

未确认原因已用实际on_localization方法再复核两遍：运行launch覆盖confirmation_window_sec=60秒，半径1.5米、所需支持4帧。6条观测分属约(738.3,-79.5)和(743.7,-99.9)两个相距约21米的位置簇；在60秒窗口内同一簇最多3帧，不能跨位置/超时拼凑4帧。回放把接收年龄设为0以给出最大可能支持，仍0确认；四个新鲜同位置观测的正对照正常确认。此为确认数据不足，未发现该处确认状态机异常。没有证明这些位置的绿色房屋语义真值，不能称已找到目标。confirmation_audit两遍SHA981bdfd9...。

当前85份配对回放：76帧不足8像素、另3帧无合格深度簇；多视图仍无足够可观测性。扫描线误拒绝已获组件/同录包及现场增量证据，但整体可靠3D/接近/到达仍未验收。先过滤后限点/增加同帧点密度/延长历史的旧探针均未解决，不能据此继续无证据改门限或地图。后续应针对同一物体、同一视点验证实际回波覆盖与掩码稳定性，再决定传感器配置或观察策略，不能直接宣布增加雷达线数即根治。

全Native181.352秒口径近停21.844%、最长3.956秒、净位移47.196米/全采样累计128.925米；v46近停28.411%，但路线不同，不能把改善全部归因本次定位修改。Native/events两遍复算一致，实际退役边界19条/旧方向后缀越界0，未发现命令曲率或原地旋转违约。全流程流畅性不宣称无回退。

证据test_artifacts/depth_geometry_20261027/field及diagnostics，validation_summary.json记录通过范围与未完成项。主703运行文件/基线ZIP再次核验保留。
'''
doc.write_text(s.replace(old,new),encoding='utf8')
mem=repo/'docs/PROJECT_MEMORY.md'
with mem.open('a',encoding='utf8') as f:f.write('''
### 2026-10-27 v47三维定位现场关闭：测距误拒绝修复，确认仍受输入限制（最新）

- trial110900已关闭：启动39.603/search180.200/shutdown14.818秒超时，errors=[]、console0、项目进程/X148锁空；88mask/85配对/6LiDAR/0确认。近停21.844%/最长3.956秒、净47.196/累计128.925m。路线不同，不将近停变化全归因定位修复；完整绿房屋到达未通过。
- 同一v47关闭录包旧/新真实几何两遍4→6有效定位，新增10/9独立像素条带跨度[15,1]/[13,1]原被平面展开门误拒；6新输出与现场坐标完全相同，报告7484c5f7.../旧9e33c2a0...。v46旧录包0→1仍是独立组件范围，不能用跨路线0→6虚报增益。
- 实际on_localization两遍复核：运行launch窗口60秒（非构造默认30秒）、半径1.5m/4支持，6点分约21m的两个空间簇，同簇窗口最多3帧；接收年龄设0的上界仍不确认，四帧同点正对照可确认。未见此处状态机异常；不合并两个位置、不降支持门、不宣称语义正确或已找到绿房屋。
- 85配对中76不足8像素、3无合格深度簇，可靠观测覆盖仍是独立限制，尚不能称整体三维定位完成；不盲目增加点密度/延长历史（旧探针均无效）。826真实ROS两遍/82相关两遍/828离线/32C++两遍通过，703主源码/ZIP保持，未推广/commit。19条退休限制后旧方向越界0。详docs/DEPTH_GEOMETRY_20261027.md关闭节，证据depth_geometry_20261027/field/diagnostics。
''')
print(json.dumps(summary,ensure_ascii=False,indent=2))
