import ast,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024')
name='nav2_adapter/launch/physical_vehicle_control.launch.py'
with zipfile.ZipFile(root/'continuous_r2_delta.zip') as z:raw=z.read(name)
s=raw.decode()
needle="                 'max_speed': 0.0, 'overspeed_limit': 0.0}], output='screen'),"
assert s.count(needle)==1
s=s.replace(needle,"                 'gear_change_confirmation_time': 0.3,\n"+needle)
assert ast.parse(s)
test='nav2_adapter/test/test_physical_shift_profile_ros.py'
with zipfile.ZipFile(root/'fast_shift_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({name:hashlib.sha256(raw).hexdigest()}))
    z.writestr(name,s)
    z.write(root/'candidate'/test,test)
print(hashlib.sha256(s.encode()).hexdigest())
