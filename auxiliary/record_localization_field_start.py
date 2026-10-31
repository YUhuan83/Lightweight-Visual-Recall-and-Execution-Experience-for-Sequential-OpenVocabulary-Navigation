from pathlib import Path
import json
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/localization_range_20261027'
for run in (1,2):
 r=json.loads((root/f'real_ros_report_{run}.json').read_text(encoding='utf8'));assert all(x['passed'] for x in r) and sum(x['tests'] for x in r)==840
p=repo/'docs/LOCALIZATION_RANGE_20261027.md'
with p.open('a',encoding='utf8') as f:f.write('''
## v49真实ROS通过，现场进行中

840真实ROS两遍/保留实际C++32两遍/console/物理rig4最终通过，主703源码和原ZIP再次核验未改，v47/v48不覆盖，实际RPP仍217609ca...。本候选没有再次出现v48首次批量回归段错误；旧异常保留，未当作查明根因。

冻结源/配置/库并预检项目进程、X148锁空后，启动实际query后180秒绿色房屋现场。当前控制、地图、传感器和定位日志记录，无视频。现场关闭结果待补，不推广/commit。
''')
with (repo/'docs/PROJECT_MEMORY.md').open('a',encoding='utf8') as f:f.write('''
### 2026-10-27 v49双范围回归通过，现场进行中

- 840真实ROS两遍、原实际C++32两遍、console/rig4、61相关两遍/828离线全部通过，主703/ZIP及旧版保留；RPP仍217609ca...，node247b3c26.../pointcloud41e4a935.../tri configef5e554d.../rig6bd41bd4...冻结。
- 预检项目进程/X148锁空后启动v49实际query后180秒green residential house，无录像，关闭待补。须验证新80m定位DDS有实际远点、40m地图隔离/更新频率和定位确认接近整链；不得用静态12/12宣称整车成功。不推广/commit，详LOCALIZATION_RANGE_20261027.md。
''')
print('840 x2 source reports verified; field pending')
